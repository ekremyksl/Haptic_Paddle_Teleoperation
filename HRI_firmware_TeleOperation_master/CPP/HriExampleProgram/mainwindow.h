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

#include "../HriBoardLib/hriboard.h"

namespace Ui {
class MainWindow;
}

/**
 * @addtogroup HriExampleProgram
 * @{
 */

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void onVarsListReceived(const QList<SyncVarBase *> &syncVars);
    void updateDisplay();
    void setLedIntensity();

private:
    Ui::MainWindow *ui; ///< Graphical user interface handle.

    HriBoard hriBoard; ///< HRI board interface.
    QLinkedList<QList<double>> streamedVarsValuesBuffer; ///< Queue to receive the all the values of the streamed variables.
    SyncVar<float> *encoderPosition, *hallVoltage, *ledIntensity; ///< Handle for the SyncVars used in this example.
    QTimer updateTimer; ///< Timer to update periodically the data display.
};

/**
 * @}
 */

#endif
