# HHRI-Software

This is the software developed for the lab sessions of the "Haptics Human-Robot Interfaces" course. The most important part is the firmware of the board that controls the paddle. Two interfaces (C++/Qt and MATLAB) are also available to communicate remotely with the board.

## Environment setup

### Project source code
First, clone the repository using the following command:  
`git clone ssh://git@c4science.ch/diffusion/2671/hhri-software.git`.

### Firmware developement environment

1. Download and install Java runtime environment from Oracle's website:  
http://www.oracle.com/technetwork/java/javase/downloads/index.html  
For Windows, the "JRE" is sufficient, but you need the "JDK" for MacOSX.
2. Download and install the System Workbench IDE:  
http://www.openstm32.org/Downloading+the+System+Workbench+for+STM32+installer
3. Connect the ST-LINK programmer to the computer. In case it is not recognized (check in the device manager), download  and install the driver. This step is not required with MacOSX.  
http://www.st.com/en/embedded-software/stsw-link009.html
4. Download, extract and install the USB drivers for the HRI board (more specifically, for its USB-to-UART chip). This step is not required with MacOSX.  
https://www.silabs.com/products/mcu/Pages/USBtoUARTBridgeVCPDrivers.aspx
5. Import the Firmware project (HHRI-Software/Firmware/.project) into System Workbench.

### MATLAB interface

MATLAB 2016b (or later) is required. The only necessary toolbox is the "Instrument Control Toolbox".

### C++ interface

Go to https://www.qt.io/download-open-source/, download and install the Qt development environment for Desktop PC.
When in the “Select Components” page, choose the following items:
 * Qt 5.8 (or a later one if available):
   * MinGW 5.3.0 32 bit (or a later one if available).
   * Qt Charts
 * Tools:
   * MinGW 5.3.0 (the one that matches your Qt version).

## Source code organization

 * `Firmware/`: source code of the firmware of the motorboard. Most of the work during the lab sessions will be performed in the file `Firmware/src/haptic_controller.c`. The Doxygen code documentation can be found in `Firmware/doc/firmware_documentation.html`.
 * `CPP/`: contains a ready-to-use graphical user interface similar to the MATLAB one, and an example project that shows how to make a custom user interface. The latter will be useful for some specialization projects, if high performance is required. The Doxygen code documentation can be found in `CPP/doc/cpp_interface_documentation.html`.
 * `MATLAB/`: mainly contains a library and a graphical user interface to interact with the board (`hri_gui.m`).

## Support

Please contact [Romain Baud](mailto:romain.baud@epfl.ch) for technical support, or to report bugs.

## Contributing

Please clone the repository, make a new branch, and send your patch files for review to [Romain Baud](mailto:romain.baud@epfl.ch).

## Licence

This project is distributed under the Apache License, Version 2.0. More information can be found in the `NOTICE` and `LICENSE` files.