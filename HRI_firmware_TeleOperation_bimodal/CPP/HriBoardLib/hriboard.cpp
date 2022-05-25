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

#include "hriboard.h"

#include <QDebug>
#include <QSerialPortInfo>
#include <QDateTime>
#include <QMessageBox>
#include <QApplication>
#include <QDir>

const int SYNCVAR_LIST_ITEM_SIZE = SYNCVAR_NAME_SIZE + 3;

/**
 * @brief Constructor.
 */
HriBoard::HriBoard()
{

}

/**
 * @brief Establish the link with the board.
 * Establish the link with the board, then stops the streaming and requests the
 * variables list.
 * @param comPortName serial port name, in the format "COM1" on Windows, or
 * "/dev/ttyO1" on UNIX.
 * @throws A runtime_error is thrown if the serial port could not be opened.
 */
void HriBoard::openLink(QString comPortName)
{
    streamID = 0;
    streamedVarsMaxSize = 1000;

    // Setup the serial port.
    serial.setPortName(comPortName);
    serial.setBaudRate(UART_BAUDRATE);
    serial.setDataBits(QSerialPort::Data8);
    serial.setFlowControl(QSerialPort::NoFlowControl);
    serial.setParity(QSerialPort::NoParity);

    connect(&serial, SIGNAL(readyRead()), this, SLOT(onReceivedData()));

    qDebug() << "Opening the serial COM port...";

    if(serial.open(QIODevice::ReadWrite))
        qDebug() << "COM port opened successfully.";
    else
        throw std::runtime_error("Can't open the COM port.");

    // Stop the streaming, in case it was enabled.
    QByteArray ba;
    ba.append((char)0);
    sendPacket(PC_MESSAGE_SET_STREAMED_VAR, ba);

    // Request the variables list.
    sendPacket(PC_MESSAGE_GET_VARS_LIST);
}

/**
 * @brief Sets the SyncVars to stream.
 * @param varsToStream list of the SyncVars to stream.
 * @param streamedVarsValues pointer to array where the streaming values will be
 * appended. This parameter can be nullptr to not use this feature.
 */
void HriBoard::setStreamedVars(QList<SyncVarBase *> varsToStream,
                               QLinkedList<QList<double> > *streamedVarsValues)
{
    this->streamedVarsValues = streamedVarsValues;
    streamedVarsValues->clear();

    // Copy the list of variables to stream, and compute the size of a streaming
    // packet.
    streamedVars.clear();
    streamPacketSize = sizeof(quint8) + sizeof(quint32); // Stream ID + timestamp.

    for(SyncVarBase* sv : varsToStream)
    {
        streamedVars.append(syncVars[sv->getIndex()]);
        streamPacketSize += sv->getSize();
    }

    //
    streamID++;

    QByteArray ba;
    ba.append((quint8)varsToStream.size());
    ba.append(streamID);

    for(SyncVarBase* sv : varsToStream)
    {
        int varIndex = sv->getIndex();
        ba.append((quint8)varIndex);
    }

    sendPacket(PC_MESSAGE_SET_STREAMED_VAR, ba);
}

/**
 * @brief Updates a SyncVar on the board with the value of the local one.
 * @param var the SyncVar to synchronize.
 */
void HriBoard::writeRemoteVar(SyncVarBase *var)
{
    QByteArray ba;
    ba.append(var->getIndex());
    ba.append(var->getData());

    sendPacket(PC_MESSAGE_SET_VAR, ba);
}

/**
 * @brief Updates a local SyncVar with the value of the SyncVar on the board.
 * @param var the SyncVar to synchronize.
 */
void HriBoard::readRemoteVar(SyncVarBase *var)
{
    QByteArray ba;
    ba.append(var->getIndex());

    sendPacket(PC_MESSAGE_GET_VAR, ba);

    var->setOutOfDate();
}

/**
 * @brief Makes a list of all the candidate serial ports.
 * @return a list of all the serial port that use the right USB-to-UART chip.
 */
QStringList HriBoard::getComPorts()
{
    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();

    // Remove from the list all the serial COM ports that are not the CP210x
    // USB-to-serial chip.
    for(int i=ports.size()-1; i >= 0; i--)
    {
        if(!ports[i].description().contains("CP210"))
            ports.removeAt(i);
    }

    // When there are cu/tty pairs, remove the tty port.
    for(int i=ports.size()-1; i >= 0; i--)
    {
        if(ports[i].portName().startsWith("tty."))
        {
            // Try to find the corresponding "cu" port.
            QString cuPortName = "cu." +
                                 ports[i].portName().remove(QRegExp("^tty."));

            for(int j=0; j<ports.size(); j++)
            {
                if(ports[j].portName() == cuPortName)
                {
                    // Corresponding "cu" found, remove the "tty" port.
                    ports.removeAt(i);
                    break;
                }
            }
        }
    }

    // Make a list of the COM ports names.
    QStringList portsNames;
    for(QSerialPortInfo comInfo : ports)
        portsNames << comInfo.portName();

    return portsNames;
}

/**
 * @brief Start logging the streamed variables to a CSV file.
 * @return true if the logfile could be created, false otherwise.
 */
bool HriBoard::startLoggingToFile(QString directory)
{
    stopLoggingToFile();

    QDateTime now = QDateTime::currentDateTime();
    logFile.setFileName(directory + "/log_" +
                        now.toString("yyyy-MM-dd_hh-mm-ss") + ".csv");

    if(QDir().mkpath(directory) &&
       !logFile.open(QFile::WriteOnly | QFile::Truncate))
    {
        QMessageBox::warning(nullptr, qApp->applicationName(),
                             "Could not create the logfile.");
        return false;
    }

    logStream.setDevice(&logFile);

    // Write the file header.
    logStream << "timestamp [s];";
    logStream.setRealNumberPrecision(10);

    for(int i=0; i<syncVars.size(); i++)
    {
        logStream << syncVars[i]->getName();

        if(i < syncVars.size()-1)
            logStream << ";";
    }

    logStream << endl;

    return true;
}

/**
 * @brief Stop logging the streamed variables to a file.
 * @remark This function does nothing if the logging was not enabled.
 */
void HriBoard::stopLoggingToFile()
{
    if(logFile.isOpen())
    {
        logFile.close();

        QFileInfo fileInfo(logFile.fileName());
        QMessageBox::information(nullptr, qApp->applicationName(),
                                 "Logfile saved as " +
                                 fileInfo.absoluteFilePath());
    }
}

/**
 * @brief Return the list of the streamed variables.
 * @return A const reference to the list of the streamed variables.
 */
const QList<SyncVarBase *> &HriBoard::getStreamedVars()
{
    return streamedVars;
}

/**
 * @brief Sets the maximum amount of streaming samples that can be queued.
 * @param maxSize the size limit of the streamed variables queue.
 * @remark If the streamed samples queue reaches the maxSize, then the oldest
 * samples will be discarded.
 */
void HriBoard::setStreamingBufferSize(int maxSize)
{
    streamedVarsMaxSize = maxSize;
}

/**
 * @brief Interprets the received bytes, and reacts accordingly.
 */
void HriBoard::onReceivedData()
{
    QByteArray rxData = serial.readAll();
    //qDebug() << "RX:" << rxData;

    for(int i=0; i<rxData.size(); i++)
    {
        quint8 rxByte = rxData[i];

        if(rxByte & (1<<7)) // The start byte has the most significant bit high.
        {
            rxCurrentMessageType = (rxByte & ~(1<<7)); // Remove the start bit.
            rxBytesCount = 0;
            rxDataBytesBuffer.clear();
        }
        else // The data bytes have the most significant byte low.
            rxBytesCount++;

        if(rxBytesCount % 2 == 1) // First half of the data byte has been received.
            firstHalfByte = rxByte; // Store it until the second half arrives.
        else // Second half of the data byte has been received.
        {
            int dataBytesReady = rxBytesCount/2;

            if(dataBytesReady > 0)
                rxDataBytesBuffer.append((firstHalfByte<<4) + (rxByte & 0xf));

            switch(rxCurrentMessageType)
            {
            case STM_MESSAGE_START_INFO:
                if(dataBytesReady == 0)
                {
                    // Request variables list.
                    sendPacket(PC_MESSAGE_GET_VARS_LIST);
                }
                break;

            case STM_MESSAGE_VAR:
                if(dataBytesReady >= 1)
                {
                    quint8 varIndex = rxDataBytesBuffer[0];

                    if(dataBytesReady == 1 + syncVars[varIndex]->getSize())
                    {
                        QByteArray ba((char*)&rxDataBytesBuffer[1],
                                       syncVars[varIndex]->getSize());
                        syncVars[varIndex]->setData(ba);

                        emit syncVarUpdated(syncVars[varIndex]);
                    }
                }
                break;

            case STM_MESSAGE_VARS_LIST:
                if(dataBytesReady >= 1)
                {
                    quint8 nVars = rxDataBytesBuffer[0];

                    if(dataBytesReady == 1 + nVars * SYNCVAR_LIST_ITEM_SIZE)
                    {
                        // Clear the SyncVar array.
                        for(SyncVarBase *sv : syncVars)
                            delete sv;
                        syncVars.clear();

                        //
                        quint8* p = &rxDataBytesBuffer[1];

                        for(int i=0; i<nVars; i++)
                        {
                            QString varName((char*)p);
                            p += SYNCVAR_NAME_SIZE;

                            VarType varType = (VarType)*p;
                            p++;

                            VarAccess varAccess = (VarAccess)*p;
                            p++;

                            //int varSize = (int)*p; // Size is ignored.
                            p++;

                            syncVars.append(makeSyncVar(varType, i, varName,
                                                        varAccess));
                        }

                        //
                        emit syncVarsListReceived(syncVars);

                        // Stop logging, if in progress.
                        stopLoggingToFile();
                    }
                }
                break;

            case STM_MESSAGE_STREAMING_PACKET:
                if(dataBytesReady == streamPacketSize)
                {
                    // If streaming was not requested, ignore the packet.
                    if(streamedVars.empty())
                        break;

                    // Decode the packet.
                    if(rxDataBytesBuffer[0] == (quint8)streamID)
                    {
                        QList<double> values;

                        // Decode the timestamp.
                        quint32 timestamp;
                        memcpy(&timestamp, &rxDataBytesBuffer[1],
                               sizeof(timestamp));
                        double time = ((double)timestamp) / 1000000.0;
                        values.append(time);

                        // Decode the variables values.
                        quint8 const* p = &rxDataBytesBuffer[5];

                        for(int i=0; i<streamedVars.size(); i++)
                        {
                            QByteArray value((char*)p,
                                             streamedVars[i]->getSize());
                            streamedVars[i]->setData(value);
                            p += streamedVars[i]->getSize();

                            values.append(streamedVars[i]->toDouble());
                        }

                        if(streamedVarsValues != nullptr)
                        {
                            streamedVarsValues->append(values);

                            // If too many samples have ben accumulated, discard
                            // the oldest oness.
                            while(streamedVarsValues->size()
                                  > streamedVarsMaxSize)
                            {
                                streamedVarsValues->pop_front();
                            }
                        }

                        emit streamedSyncVarsUpdated(time, streamedVars);

                        // Log to file, if enabled.
                        if(logFile.isOpen())
                        {
                            logStream << time << ";";

                            for(int i=0; i<syncVars.size(); i++)
                            {
                                if(syncVars[i]->isUpToDate())
                                    logStream << syncVars[i]->toDouble();
                                else
                                    logStream << 0;

                                if(i < syncVars.size()-1)
                                    logStream << ";";
                            }

                            logStream << endl;
                        }
                    }
                }
                break;

            case STM_MESSAGE_DEBUG_TEXT:
                if(dataBytesReady > 0 && rxDataBytesBuffer.last() == '\0')
                {
                    qDebug() << QString((const char*)rxDataBytesBuffer.data());
                    rxCurrentMessageType = 0;
                }
                break;

            default: // Ignore.
                break;
            }
        }
    }
}

/**
 * @brief Sends a communication packet to the board.
 * @param messageType the type of the message.
 * @param dataBytes the data that will be part of the packet, which is
 * type-specific. If omitted, there will be no data in the message, so it will
 * contain only the type.
 */
void HriBoard::sendPacket(comm_PcMessage messageType, QByteArray dataBytes)
{
    QByteArray txBuffer;
    txBuffer.append((1<<7) + (uint8_t)messageType);

    for(int i=0; i<dataBytes.size(); i++)
    {
        txBuffer.append(((quint8)dataBytes[i]) >> 4); // MSB.
        txBuffer.append(((quint8)dataBytes[i]) & 0xf); // LSB.
    }

    serial.write(txBuffer);
}
