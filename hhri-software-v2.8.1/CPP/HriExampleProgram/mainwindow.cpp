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
#include <QInputDialog>
#include <QDebug>

#define UPDATE_PERIOD 50 ///< Update period of the data display [ms].

/**
 * @brief Constructor.
 * @param parent parent widget.
 */
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

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

    // Establish the link with the HRI board.
    try
    {
        hriBoard.openLink(comPortName);
    }
    catch(std::runtime_error)
    {
        QMessageBox::critical(this, qApp->applicationName(),
                              "Could not open the COM port.");
    }

    connect(&hriBoard, SIGNAL(syncVarsListReceived(const QList<SyncVarBase*>&)),
            this, SLOT(onVarsListReceived(const QList<SyncVarBase*>&)));

    // Start the user interface update timer.
    updateTimer.setSingleShot(false);
    updateTimer.start(UPDATE_PERIOD);
    connect(&updateTimer, SIGNAL(timeout()), this, SLOT(updateDisplay()));

    // Setup the action when the LED intensity is changed on the GUI.
    connect(ui->ledSlider, SIGNAL(valueChanged(int)),
            this, SLOT(setLedIntensity()));
}

/**
 * @brief Destructor.
 */
MainWindow::~MainWindow()
{
    delete ui;
}

/**
 * @brief Gets the SyncVars handles from the list, and setup the data streaming.
 * @param syncVars syncVars list.
 */
void MainWindow::onVarsListReceived(const QList<SyncVarBase *> &syncVars)
{
    // List all the variables.
    for(SyncVarBase *sv : syncVars)
        qDebug() << sv->getName();

    // Get a convenient handle to the desired SyncVars.
    encoderPosition = hriBoard.getVarHandle<float>("encoder_paddle_pos [deg]");
    hallVoltage = hriBoard.getVarHandle<float>("hall_voltage [V]");
    ledIntensity = hriBoard.getVarHandle<float>("led_0 [0.0-1.0]");

    // If at least one SyncVar could not be found on the board's list, abort.
    if(encoderPosition == nullptr || ledIntensity == nullptr)
    {
        QMessageBox::critical(this, qApp->applicationName(),
                              "A SyncVar could not be found on the board.");
        exit(0);
    }

    // Setup the streaming to get the continuously get the encoder position.
    QList<SyncVarBase*> varsToStream;
    varsToStream.append(encoderPosition);
    varsToStream.append(hallVoltage);
    hriBoard.setStreamedVars(varsToStream, &streamedVarsValuesBuffer);
}

/**
 * @brief Updates the user interface widgets, with the latest variables values.
 */
void MainWindow::updateDisplay()
{
    if(!streamedVarsValuesBuffer.isEmpty())
    {
        // Get only the latest set of values and discard the rest, since we are
        // not interested in the older ones (for this simple example).
        // An other solution would be to read directly the SynVars (e.g.
        // encoderPosition->getLocalValue().
        QList<double> valuesSet = streamedVarsValuesBuffer.last();
        streamedVarsValuesBuffer.clear();

        // The first value is the timestamp, and the following are the values of
        // the streamed variables.
        ui->encoderProgressbar->setValue((int)valuesSet[1]);
        ui->hallVoltageLabel->setText(QString::number(valuesSet[2]) + " V");
    }
}

/**
 * @brief Set the intensity of the first LED (LED 0), from the slider position.
 */
void MainWindow::setLedIntensity()
{
    float intensityValue = ((double)ui->ledSlider->value()) / 100.0;

    hriBoard.writeRemoteVar(ledIntensity, intensityValue);
}
