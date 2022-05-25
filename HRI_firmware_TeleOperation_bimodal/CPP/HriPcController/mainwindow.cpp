/*
 * Copyright (C) 2017 EPFL-LSRO (Laboratoire de Systemes Robotiques).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QMessageBox>
#include <QDebug>
#include <QInputDialog>
#include <QFileDialog>
#include <QList>
#include <QDateTime>
#include <QStyle>
#include <limits>
#include <cmath>

#define GRAPH_UPDATE_PERIOD 50 ///< Plot window refresh period [ms].
#define LOG_CHECKBOX_BASE_LABEL QString("Log to CSV file")

#define SETTING_WINDOW_SIZE "window_size"
#define SETTING_LIST_FRAME_HEIGHT "list_frame_height"
#define SETTING_PLOT_FRAME_HEIGHT "plot_frame_height"
#define SETTING_LOGS_DIR "logs_dir"

/**
 * @brief Constructor
 * Setups the GUI, and try to connect to the HRI board.
 * @param parent parent of this widget.
 */
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    //
    ui->setupUi(this);

    ui->clearButton->setIcon(QIcon(qApp->style()->standardPixmap(QStyle::SP_DialogResetButton)));
    ui->pausePlotButton->setIcon(QIcon(qApp->style()->standardPixmap(QStyle::SP_MediaPause)));
    ui->setLogLocationButton->setIcon(qApp->style()->standardPixmap(QStyle::SP_DirOpenIcon));

    variablesListLayout = new QGridLayout();
    variablesListLayout->setAlignment(Qt::AlignTop);
    delete ui->scrollWidget->layout();
    ui->scrollWidget->setLayout(variablesListLayout);

    chart = new QtCharts::QChart();
    ui->graphicsView->setChart(chart);
    ui->graphicsView->setRenderHint(QPainter::Antialiasing);

    connect(ui->nPointsSpinbox, SIGNAL(valueChanged(int)),
            this, SLOT(onShownPointsCountChanged()));

    graphUpdateTimer.setInterval(GRAPH_UPDATE_PERIOD);
    graphUpdateTimer.setSingleShot(false);
    connect(&graphUpdateTimer, SIGNAL(timeout()), this, SLOT(updateGraph()));

    connect(ui->pausePlotButton, SIGNAL(toggled(bool)),
            this, SLOT(onPauseToggled(bool)));

    //
    syncVars = nullptr;

    // Recover the previous logs save location, or define if this is the first
    // time the app is started.
    if(settings.contains(SETTING_LOGS_DIR))
        logsDirectory = settings.value(SETTING_LOGS_DIR).toString();
    else
    {
        logsDirectory = qApp->applicationDirPath();

#if defined(Q_OS_MACX) || defined(Q_OS_MAC64)
        // For MacOSX, go to out of the app bundle.
        logsDirectory.remove(QRegExp("/[^/]*.app/Contents/MacOS/?$"));
#endif
    }

    ui->logToFileCheckbox->setText(LOG_CHECKBOX_BASE_LABEL +
                                   " (in " + logsDirectory + ")");

    // Resize the window and the panes to the previous size.
    if(settings.contains(SETTING_WINDOW_SIZE))
        resize(settings.value(SETTING_WINDOW_SIZE).toSize());

    if(settings.contains(SETTING_LIST_FRAME_HEIGHT) &&
       settings.contains(SETTING_PLOT_FRAME_HEIGHT))
    {
        int plotGroupWidth = settings.value(SETTING_LIST_FRAME_HEIGHT).toInt();
        int plotGroupHeight = settings.value(SETTING_PLOT_FRAME_HEIGHT).toInt();
        ui->splitter->setSizes({ plotGroupWidth, plotGroupHeight });
    }

    // Ask for the COM port.
    QString comPortName;
    QStringList ports = HriBoard::getComPorts();

    if(ports.size() == 0)
    {
        QMessageBox::critical(this, qApp->applicationName(), "No COM port.");
        exit(0);
    }
    else if(ports.size() == 1)
        comPortName = ports.first();
    else
    {
        bool ok;

        QString portChoice = QInputDialog::getItem(this,
                                                   qApp->applicationName(),
                                                   "Serial port:", ports, 0,
                                                   false, &ok);

        int portIndex = ports.indexOf(portChoice);

        if(portIndex == -1 || !ok)
            exit(0);

        comPortName = ports[portIndex];
    }

    //
    connect(ui->clearButton, SIGNAL(clicked(bool)), this, SLOT(clearPlot()));
    connect(ui->logToFileCheckbox, SIGNAL(toggled(bool)),
            this, SLOT(onLogToFileCheckboxToggled()));
    connect(ui->setLogLocationButton, SIGNAL(clicked(bool)),
            this, SLOT(setLogfilesDirectory()));

    // Establish the link with the HRI board.
    try
    {
        hriBoard.openLink(comPortName);
    }
    catch(std::runtime_error&)
    {
        QMessageBox::critical(nullptr, qApp->applicationName(),
                              "Could not open the serial port " + comPortName);
        exit(0);
    }

    connect(&hriBoard, SIGNAL(syncVarsListReceived(const QList<SyncVarBase*>&)),
            this, SLOT(onVarsListReceived(const QList<SyncVarBase*>&)));
    connect(&hriBoard, SIGNAL(syncVarUpdated(SyncVarBase*)),
            this, SLOT(onVarUpdated(SyncVarBase*)));
}

/**
 * @brief Destructor.
 */
MainWindow::~MainWindow()
{
    // Store the settings.
    settings.setValue(SETTING_WINDOW_SIZE, size());
    settings.setValue(SETTING_LIST_FRAME_HEIGHT, ui->splitter->sizes()[0]);
    settings.setValue(SETTING_PLOT_FRAME_HEIGHT, ui->splitter->sizes()[1]);
    settings.setValue(SETTING_LOGS_DIR, logsDirectory);

    //
    delete ui;
}

/**
 * @brief Displays the SyncVars list, and starts the streaming.
 * @param syncVars SyncVars list.
 * @remark This slot function is called automatically by the HriBoard object, as
 * soon as the list is received from the board.
 */
void MainWindow::onVarsListReceived(const QList<SyncVarBase *> &syncVars)
{
    this->syncVars = &syncVars;

    // Reset the plot frame.
    clearPlot();
    chart->removeAllSeries();
    linesSeries.clear();

    // Create the widgets to display the SyncVars list.
    QLayoutItem *li;
    while((li = variablesListLayout->takeAt(0)) != nullptr)
        li->widget()->deleteLater();

    syncVarsWidgets.clear();

    for(int i=0; i<syncVars.size(); i++)
    {
        SyncVarWidgetsLine varsWidgets;

        // Name label.
        varsWidgets.name = new QLabel(syncVars[i]->getName());
        variablesListLayout->addWidget(varsWidgets.name, i, 0);

        // Get button.
        if(syncVars[i]->getAccess() != WRITEONLY)
        {
            varsWidgets.getButton = new QPushButton("Get");
            variablesListLayout->addWidget(varsWidgets.getButton, i, 1);
            connect(varsWidgets.getButton, SIGNAL(clicked(bool)),
                    this, SLOT(onGetButtonPressed()));
        }

        // Value field.
        varsWidgets.valueLineEdit = new QLineEdit();
        variablesListLayout->addWidget(varsWidgets.valueLineEdit, i, 2);

        if(syncVars[i]->getAccess() == READONLY)
            varsWidgets.valueLineEdit->setReadOnly(true);
        else
        {
            connect(varsWidgets.valueLineEdit, SIGNAL(editingFinished()),
                    this, SLOT(onSetButtonPressed()));
        }

        // Set button.
        if(syncVars[i]->getAccess() != READONLY)
        {
            varsWidgets.setButton = new QPushButton("Set");
            variablesListLayout->addWidget(varsWidgets.setButton, i, 3);
            connect(varsWidgets.setButton, SIGNAL(clicked(bool)),
                    this, SLOT(onSetButtonPressed()));
        }

        // Stream checkbox.
        if(syncVars[i]->getAccess() != WRITEONLY)
        {
            varsWidgets.streamCheckbox = new QCheckBox("Stream");
            variablesListLayout->addWidget(varsWidgets.streamCheckbox, i, 4);
            connect(varsWidgets.streamCheckbox, SIGNAL(toggled(bool)),
                    this, SLOT(onStreamCheckboxToggled()));
        }

        if(syncVars[i]->getAccess() != WRITEONLY)
        {
            varsWidgets.scaleSpinbox = new QDoubleSpinBox();
            variablesListLayout->addWidget(varsWidgets.scaleSpinbox, i, 5);
            varsWidgets.scaleSpinbox->setPrefix("x");
            varsWidgets.scaleSpinbox->setValue(1.0);
            varsWidgets.scaleSpinbox->setRange(-1000000.0, 1000000.0);
        }

        syncVarsWidgets.append(varsWidgets);
    }

    // Acquire the value of all the readable SyncVars.
    for(SyncVarBase *sv : syncVars)
    {
        if(sv->getAccess() != WRITEONLY)
            hriBoard.readRemoteVar(sv);
    }
}

/**
 * @brief Updates the "value" field of a SyncVar.
 * @param var the SyncVar corresponding to the "value" field to update.
 */
void MainWindow::onVarUpdated(SyncVarBase *var)
{
    int varIndex = syncVars->indexOf(var);

    if(varIndex >= 0 && varIndex < syncVarsWidgets.size())
    {
        QString text = var->toString();
        syncVarsWidgets[varIndex].valueLineEdit->setText(text);
        syncVarsWidgets[varIndex].valueLineEdit->setStyleSheet("");
    }
}

/**
 * @brief Gets the value of a SyncVar whose the "Get" button was pressed.
 * @warning This function should only be called by the "Get" button signal.
 */
void MainWindow::onGetButtonPressed()
{
    int varIndex = getVarIndexFromWidget((QWidget*)sender());

    hriBoard.readRemoteVar(syncVars->at(varIndex));
}

/**
 * @brief Sets the value of a SyncVar whose the "Set" button was pressed.
 * @warning This function should only be called by the "Set" button signal.
 */
void MainWindow::onSetButtonPressed()
{
    // Get the text field widget and the associated SyncVar.
    int varIndex = getVarIndexFromWidget((QWidget*)sender());
    QLineEdit *lineEdit = syncVarsWidgets[varIndex].valueLineEdit;

    SyncVarBase *sv = syncVars->at(varIndex);

    // If the number is correct, send it to the board. Otherwise, color the
    // text field background in red.
    if(sv->fromString(lineEdit->text()))
    {
        lineEdit->setStyleSheet("");
        hriBoard.writeRemoteVar(sv);
    }
    else
        lineEdit->setStyleSheet("background-color: #ffaaaa;");
}

/**
 * @brief Updates the list of SyncVars to stream from the checkboxes' states.
 */
void MainWindow::onStreamCheckboxToggled()
{
    // Setup the streaming.
    QList<SyncVarBase*> varsToStream;

    for(int i=0; i<syncVarsWidgets.size(); i++)
    {
        if(syncVarsWidgets[i].streamCheckbox->isChecked())
            varsToStream.append(syncVars->at(i));
    }

    hriBoard.setStreamedVars(varsToStream, &streamedVarsValuesBuffer);

    // Setup the graph.
    streamedVarsValuesBuffer.clear();

    chart->removeAllSeries();
    linesSeries.clear();

    for(SyncVarBase *sv : varsToStream)
    {
        QtCharts::QLineSeries *series = new QtCharts::QLineSeries();
        series->setName(sv->getName());
        chart->addSeries(series);
        linesSeries.append(series);
    }

    chart->createDefaultAxes();
    chart->axes(Qt::Horizontal).first()->setTitleText("Time [s]");

    // Setup the update timer.
    if(varsToStream.isEmpty())
        graphUpdateTimer.stop();
    else
        graphUpdateTimer.start();
}

/**
 * @brief Sets the max size of the streamed samples queue.
 */
void MainWindow::onShownPointsCountChanged()
{
    hriBoard.setStreamingBufferSize(ui->nPointsSpinbox->value());
}

/**
 * @brief Clears the plot window.
 */
void MainWindow::clearPlot()
{
    for(QtCharts::QLineSeries *ls : linesSeries)
        ls->clear();

    QSignalBlocker sb(ui->pausePlotButton);
    ui->pausePlotButton->setChecked(false);
}

/**
 * @brief Pauses or resume the live plotting of the streamed variables.
 * Clears the plot window if the pause is stopped.
 * @param paused true to pause the live plot, false to resume it.
 */
void MainWindow::onPauseToggled(bool paused)
{
    if(!paused)
        clearPlot();
}

/**
 * @brief Starts or stops the streamed variables logging to file.
 */
void MainWindow::onLogToFileCheckboxToggled()
{
    if(ui->logToFileCheckbox->isChecked())
    {
        if(!hriBoard.startLoggingToFile(logsDirectory))
        {
            // If the logging cannot start, uncheck the box.
            QSignalBlocker signalBlocker(ui->logToFileCheckbox);
            ui->logToFileCheckbox->setChecked(false);
        }
    }
    else
        hriBoard.stopLoggingToFile();
}

/**
 * @brief Sets the directory to save the logfiles.
 */
void MainWindow::setLogfilesDirectory()
{
    QString newDir = QFileDialog::getExistingDirectory(this,
        "Select the directory to save the logfiles", logsDirectory);

    if(!newDir.isEmpty())
    {
        logsDirectory = newDir;
        ui->logToFileCheckbox->setText(LOG_CHECKBOX_BASE_LABEL +
                                       " (in " + logsDirectory + ")");
    }
}

/**
 * @brief Refreshes the plot frame.
 */
void MainWindow::updateGraph()
{
    // Do not refresh the graph in "pause" mode.
    if(ui->pausePlotButton->isChecked())
        return;

    // Get the variables scales.
    const QList<SyncVarBase*> &streamedVars = hriBoard.getStreamedVars();
    QList<double> plotScales;

    for(SyncVarBase* sv : streamedVars)
    {
        int varIndex = syncVars->indexOf(sv);

        if(varIndex >= 0)
            plotScales.append(syncVarsWidgets[varIndex].scaleSpinbox->value());
        else
            return;
    }

    // Update the values in the variables list.
    if(!streamedVarsValuesBuffer.isEmpty())
    {
        QList<double> &values = streamedVarsValuesBuffer.last();

        for(int i=0; i<streamedVars.size(); i++)
        {
            int varIndex = syncVars->indexOf(streamedVars[i]);

            if(varIndex >= 0)
                syncVarsWidgets[varIndex].valueLineEdit->setText(QString::number(values[i+1]));
            else
                return;
        }
    }

    // Add the new points to the graph series.
    int decimationCounter = 0;

    while(!streamedVarsValuesBuffer.isEmpty())
    {
        if(decimationCounter <= 0)
        {
            decimationCounter = ui->plotDecimSpinbox->value();

            QList<double> &values = streamedVarsValuesBuffer.first();

            for(int i=0; i<linesSeries.size(); i++)
                linesSeries[i]->append(values[0], values[i+1] * plotScales[i]);
        }
        else
            decimationCounter--;

        streamedVarsValuesBuffer.removeFirst();
    }

    // Remove the oldest points, to limit the number of points displayed.
    for(QtCharts::QLineSeries *ls : linesSeries)
    {
        if(ls->count() > ui->nPointsSpinbox->value())
            ls->removePoints(0, ls->count() - ui->nPointsSpinbox->value());
    }

    // Compute the display range.
    if(linesSeries.size() > 0 && linesSeries.first()->count() > 0)
    {
        // Compute the horizontal (time) range.
        double firstTime = linesSeries.first()->points().first().x();
        double lastTime = linesSeries.first()->points().last().x();

        chart->axes(Qt::Horizontal).first()->setRange(firstTime, lastTime);

        // Compute the vertical range.
        double min = std::numeric_limits<double>::max();
        double max = std::numeric_limits<double>::lowest();

        for(QtCharts::QLineSeries *ls : linesSeries)
        {
            QVector<QPointF> points = ls->pointsVector();

            for(QPointF p : points)
            {
                if(p.y() < min)
                    min = p.y();

                if(p.y() > max)
                    max = p.y();
            }
        }

        if(abs(min - max) < std::numeric_limits<double>::epsilon())
        {
            min -= 1.0;
            max += 1.0;
        }
        else
        {
            min -= (max-min) * 0.05; // Add a 5% margin at the top and the
            max += (max-min) * 0.05; // bottom.
        }

        chart->axes(Qt::Vertical).first()->setRange(min, max);
    }
}

/**
 * @brief Gets the variable index from a widget of the SyncVars list.
 * @param widget the widget belonging to the variables list.
 * @return The index of the SyncVar corresponding to the given widget.
 */
int MainWindow::getVarIndexFromWidget(QWidget *widget)
{
    int index = variablesListLayout->indexOf(widget);
    int row, column, rowSpan, columnSpan;
    variablesListLayout->getItemPosition(index, &row, &column,
                                         &rowSpan, &columnSpan);

    return row;
}
