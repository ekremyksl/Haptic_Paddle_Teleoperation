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
#include <QApplication>

/** @defgroup HriExampleProgram Example app to interact with the HRI board
  * @brief This interface demonstrates how to interact with the HRI. It shows
  * how to visually display the encoder position and the Hall voltage, and how
  * to control the LED intensity.
  *
  * @addtogroup HriExampleProgram
  * @{
  */

/**
 * @brief Main function.
 * @param argc number of arguments.
 * @param argv arguments array.
 * @return 0 if the program exited normally, the error code otherwise.
 */

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    a.setApplicationName("HRI_example_program");
    a.setOrganizationDomain("lsro.epfl.ch");
    a.setOrganizationName("LSRO");

    MainWindow w;
    w.show();

    return a.exec();
}

/**
 * @}
 */
