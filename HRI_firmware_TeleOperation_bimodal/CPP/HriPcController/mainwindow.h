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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QGridLayout>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QCheckBox>
#include <QLabel>
#include <QChart>
#include <QLineSeries>
#include <QTimer>
#include <QLinkedList>
#include <QSettings>

#include "../HriBoardLib/hriboard.h"
#include "../HriBoardLib/syncvar.h"

namespace Ui {
class MainWindow;
}

/**
 * @addtogroup HriPcController
 * @{
 */

struct SyncVarWidgetsLine
{
    QLabel *name;
    QPushButton *getButton, *setButton;
    QLineEdit *valueLineEdit;
    QCheckBox *streamCheckbox;
    QDoubleSpinBox *scaleSpinbox;
};

/**
 * @brief Qt MainWindow with widgets to interact with SyncVars.
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void onVarsListReceived(const QList<SyncVarBase *> &syncVars);
    void onVarUpdated(SyncVarBase *var);

    void onGetButtonPressed();
    void onSetButtonPressed();
    void onStreamCheckboxToggled();
    void onShownPointsCountChanged();
    void clearPlot();
    void onPauseToggled(bool paused);

    void onLogToFileCheckboxToggled();
    void setLogfilesDirectory();

    void updateGraph();

private:
    int getVarIndexFromWidget(QWidget* widget);

    Ui::MainWindow *ui; ///< Graphical user interface from the ui file.
    HriBoard hriBoard; ///< HRI board interface.
    const QList<SyncVarBase*> *syncVars;
    QList<SyncVarWidgetsLine> syncVarsWidgets;

    QSettings settings;
    QString logsDirectory;
    QGridLayout *variablesListLayout;
    QtCharts::QChart *chart;
    QList<QtCharts::QLineSeries*> linesSeries;
    QTimer graphUpdateTimer;
    QLinkedList<QList<double>> streamedVarsValuesBuffer;
};

/**
 * @}
 */

#endif
