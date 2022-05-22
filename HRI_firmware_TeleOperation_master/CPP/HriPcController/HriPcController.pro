#-------------------------------------------------
#
# Project created by QtCreator 2015-02-18T17:52:49
#
#-------------------------------------------------

QT       += core gui serialport charts

CONFIG += c++11

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = HriPcController
TEMPLATE = app


SOURCES += main.cpp\
           mainwindow.cpp \
    ../HriBoardLib/hriboard.cpp \
    ../HriBoardLib/syncvar.cpp

HEADERS  += mainwindow.h \
            ../../Firmware/src/definitions.h \
    ../HriBoardLib/hriboard.h \
    ../HriBoardLib/syncvar.h

FORMS    += mainwindow.ui
