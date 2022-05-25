#-------------------------------------------------
#
# Project created by QtCreator 2017-03-05T16:55:17
#
#-------------------------------------------------

QT       += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = HriExampleProgram
TEMPLATE = app

SOURCES += main.cpp\
        mainwindow.cpp \
    ../HriBoardLib/hriboard.cpp \
    ../HriBoardLib/syncvar.cpp

HEADERS  += mainwindow.h \
    ../HriBoardLib/hriboard.h \
    ../HriBoardLib/syncvar.h

FORMS    += mainwindow.ui
