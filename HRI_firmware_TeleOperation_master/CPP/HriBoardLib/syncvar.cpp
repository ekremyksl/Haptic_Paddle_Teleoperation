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

#include "syncvar.h"
#include "hriboard.h"

#include <QThread>

/**
 * @brief Constructor.
 * @param index index in the SyncVars list.
 * @param name name of the SyncVar.
 * @param access access right of the SyncVar.
 * @param data address of the bytes array to hold the value of the variable.
 * @param size size of the bytes array that holds the value of the variable.
 */
SyncVarBase::SyncVarBase(int index, QString name, VarAccess access,
                         uint8_t *data, int size) :
    index(index), name(name), access(access), data(data), size(size)
{
    upToDate = false;
}

/**
 * @brief Gets the index in the SyncVars list.
 * @return the index.
 */
int SyncVarBase::getIndex() const
{
    return index;
}

/**
 * @brief Gets the name of the SyncVar.
 * @return the name.
 */
QString SyncVarBase::getName() const
{
    return name;
}

/**
 * @brief Gets the size of the SyncVar value.
 * @return the value size [byte].
 */
int SyncVarBase::getSize() const
{
    return size;
}

/**
 * @brief Gets the SyncVar access rights.
 * @return the SyncVar access rights.
 */
VarAccess SyncVarBase::getAccess() const
{
    return access;
}

/**
 * @brief Gets the local value in raw bytes form.
 * @return a copy of the value data.
 * @remark this value may not match the actual value of the variable on the
 * board, because this function does not perform the synchronisation.
 */
QByteArray SyncVarBase::getData() const
{
    QByteArray ba((char*)data, size);
    return ba;
}

/**
 * @brief Sets the local value with raw bytes.
 * @param newData the new value data.
 * @remark this function only sets the local value of the SyncVar, not the value
 * of the one on the board (no synchronisation performed). For this you need to
 * call HriBoard::writeRemoteVar().
 */
void SyncVarBase::setData(QByteArray newData)
{
    if(newData.size() != size)
        throw std::runtime_error("SyncVar::setData(): size do not match.");

    memcpy(data, newData.data(), size);
    upToDate = true;
}

/**
 * @brief Gets if the variable is up-to-date.
 * @return true if the variable value has been set, false if it has not been set
 * since the beginning, or the call to setOutOfDate().
 */
bool SyncVarBase::isUpToDate() const
{
    return upToDate;
}

/**
 * @brief Sets the variable as out-of-date.
 * This function is useful when the user need to know when the variable value
 * has been updated.
 */
void SyncVarBase::setOutOfDate()
{
    upToDate = false;
}

/**
 * @brief Construct a SyncVar object with the given characteristics.
 * @param type type of the SyncVar.
 * @param index index of the SyncVar, as defined by the board.
 * @param name user-readable name of the SyncVar.
 * @param access access rights for this variable.
 * @return A pointer to the constructed SyncVar.
 */
SyncVarBase *makeSyncVar(VarType type, int index, QString name,
                         VarAccess access)
{
    switch(type)
    {
    case BOOL: return new SyncVar<bool>(index, name, access, 1);
    case UINT8: return new SyncVar<uint8_t>(index, name, access, 1);
    case INT8: return new SyncVar<int8_t>(index, name, access, 1);
    case UINT16: return new SyncVar<uint16_t>(index, name, access, 2);
    case INT16: return new SyncVar<int16_t>(index, name, access, 2);
    case UINT32: return new SyncVar<uint32_t>(index, name, access, 4);
    case INT32: return new SyncVar<int32_t>(index, name, access, 4);
    case UINT64: return new SyncVar<uint64_t>(index, name, access, 8);
    case INT64: return new SyncVar<int64_t>(index, name, access, 8);
    case FLOAT32: return new SyncVar<float>(index, name, access, 4);
    case FLOAT64: return new SyncVar<double>(index, name, access, 8);
    default: return nullptr;
    }
}
