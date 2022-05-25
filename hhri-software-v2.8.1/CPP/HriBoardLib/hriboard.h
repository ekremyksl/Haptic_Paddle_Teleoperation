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

#ifndef HRIBOARD_H
#define HRIBOARD_H

#include <QObject>
#include <QList>
#include <QSerialPort>
#include <QFile>
#include <QTextStream>
#include <QLinkedList>

#include <stdexcept>

#include "syncvar.h"

/** @defgroup HriBoardLib HRI board
  * @brief Set of classes to easily interface with the HRI board.
  *
  * @addtogroup HriBoardLib
  * @{
  */

/**
 * @brief Class to interface with a HRI board.
 *
 * After creating an instance of this class, connect your custom slot functions
 * to the signals
 *
 * Then, call openLink() to establish the communication link with the board.
 * If successfull, this object will automatically request the synchronized
 * variables list, and call the given slot function. You can then inspect this
 * link and find useful variables to stream or modify. To operate on these
 * SyncVars, you can either directly interact with the SyncVarBase (generic)
 * pointers of the list, or create typed pointer by doing dynamic_cast or
 * calling getVarHandle().
 *
 * To set the value of a SyncVar on the board, call writeRemoteVar().
 * To get the value of a SyncVar from the board, call readRemoteVar(), and
 * wait until SyncVar::isUpToDate() becomes true.
 * To continuously receive the value of several variables, setup the streaming
 * with setStreamedVars(). The given queue object will then be filled
 * continuously, as the values are received from the board.
 */
class HriBoard : public QObject
{
    Q_OBJECT

public:
    HriBoard();
    void openLink(QString comPortName);
    void setStreamedVars(QList<SyncVarBase *> varsToStream,
                         QLinkedList<QList<double>> *streamedVarsValues);

    void writeRemoteVar(SyncVarBase *var);

    /**
     * @brief Updates a SyncVar on the board with the value of the local one.
     * @param var the SyncVar to synchronize.
     * @param value the new value that both the local and remote SyncVar should
     * have.
     */
    template<typename T> void writeRemoteVar(SyncVarBase *var, T value)
    {
        SyncVar<T> *svt = dynamic_cast<SyncVar<T>*>(var);

        if(svt == nullptr)
            throw std::runtime_error("writeRemoteVar(): value and SyncVar type do not match.");
        else
        {
            svt->setLocalValue(value);
            writeRemoteVar(svt);
        }
    }

    void readRemoteVar(SyncVarBase* var);

    static QStringList getComPorts();

    /**
     * @brief Gets a typed handle to a SyncVar, from its name.
     * @param name name of the SyncVar.
     * return The typed pointer to the SyncVar, or nullptr if the given type
     * did not match the SyncVar's one, or if a variable with the given name
     * was not found.
     */
    template<typename T> SyncVar<T>* getVarHandle(QString name)
    {
        for(SyncVarBase *sv : syncVars)
        {
            if(sv->getName() == name)
                return dynamic_cast<SyncVar<T>*>(sv);
        }

        return nullptr; // Variable not found.
    }

    bool startLoggingToFile(QString directory);
    void stopLoggingToFile();

    const QList<SyncVarBase *> &getStreamedVars();
    void setStreamingBufferSize(int maxSize);

public slots:
    void onReceivedData();

signals:
    /**
     * @brief Signal emitted when the syncVars list has been received.
     * @param syncVars list of all the SyncVars of the board.
     */
    void syncVarsListReceived(const QList<SyncVarBase*> &syncVars);

    /**
     * @brief Signal emitted when a single variable value was received.
     * @param var pointer to the SyncVar that was just updated.
     */
    void syncVarUpdated(SyncVarBase* var);

    /**
     * @brief Signal emitted when a streaming packet was received.
     * @param time board timestamp of the streamed variables values snapshot.
     * @param streamedVars list of the streamed variables, that have just been
     * updated.
     */
    void streamedSyncVarsUpdated(double time,
                                 const QList<SyncVarBase*>& streamedVars);

protected:
    void sendPacket(comm_PcMessage messageType,
                    QByteArray dataBytes = QByteArray());

private:
    QSerialPort serial; ///< Serial port to communicate with the board.
    QList<SyncVarBase*> syncVars; ///< SyncVars list.
    QList<SyncVarBase*> streamedVars; ///< List of pointers to the SyncVars streamed by the board.

    QByteArray rxBuffer; ///< Byte buffer used for receiving bytes from the board.
    int rxCurrentMessageType; ///< Type of the board message being interpreted.
    int rxBytesCount; ///< Number of bytes of the board message being interpreted.
    quint8 firstHalfByte; ///< First byte of a data byte.
    QVector<quint8> rxDataBytesBuffer; ///< Temporary buffer to store the data bytes (made of two received bytes) of the message being interpreted.
    int streamID; ///< Identifier of the current streaming configuration, to check if the received streaming packet correspond to the request.
    int streamPacketSize; ///< Expected size of a streaming packet [byte].

    QFile logFile; ///< CSV file to store the states of the streamed variables.
    QTextStream logStream; ///< Text stream interface for the logFile.

    QLinkedList<QList<double>> *streamedVarsValues; ///< Pointer to a user array, to be appended with the values of the streamed variables.
    int streamedVarsMaxSize; /// Maximum size of the streamedVarsValues list.
};

/**
 * @}
 */

#endif
