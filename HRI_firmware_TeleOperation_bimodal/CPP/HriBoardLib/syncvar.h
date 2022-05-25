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

#ifndef SYNCVAR_H
#define SYNCVAR_H

#include <stdexcept>
#include <limits>
#include <QString>
#include <QList>

#include "../../Firmware/src/definitions.h"
typedef comm_VarType VarType;
typedef comm_VarAccess VarAccess;

/**
  * @addtogroup HriBoardLib
  * @{
  */

/**
 * @brief Variable that can be synchronized with its HRI board counterpart.
 */
class SyncVarBase
{
public:
    SyncVarBase(int index, QString name, VarAccess access,
                uint8_t* data, int size);
    virtual ~SyncVarBase() = default;

    int getIndex() const;
    QString getName() const;
    int getSize() const;
    VarAccess getAccess() const;
    QByteArray getData() const;
    void setData(QByteArray newData);

    bool isUpToDate() const;
    void setOutOfDate();

    /**
     * @brief Gets the variable value, as a floating-point number.
     * @return The variable value, casted to the double type.
     */
    virtual double toDouble() const = 0;

    /**
     * @brief Gets the variable value, as text.
     * @return A string with the variable value printed.
     */
    virtual QString toString() const = 0;

    /**
     * @brief Sets the variable local value from a floating-point number.
     * @param value the new value of the variable. It will be casted to the
     * actual variable type.
     * @remark This does not change the value of the variable on the board. For
     * this you need to call HriBoard::writeRemoteVar().
     */
    virtual void fromDouble(double value) = 0;

    /**
     * @brief Sets the variable local value from a string.
     * This method tries to convert the given string to a number, then checks
     * that it fits into the variable type. If the given number is invalid, or
     * out-of-range, it will be ignored and the variable value will not change.
     * @param text the new value of the variable. It will interpreted and
     * converted to the actual variable type.
     * @return true if the text represented a valid number, false otherwise.
     * @remark This does not change the value of the variable on the board. For
     * this you need to call HriBoard::writeRemoteVar().
     */
    virtual bool fromString(QString text) = 0;

private:  
    int index; ///< Index of the SyncVar in the list.
    QString name; ///< Name describing the SyncVar.
    VarAccess access; ///< Access rights of the variable.
    bool upToDate; ///< Indicates whether the local value is up-to-date or not.
    uint8_t *const data;
    const int size;
};

template<typename T>
class SyncVar : public SyncVarBase
{
public:
    SyncVar(int index, QString name, VarAccess access, int varSize) :
        SyncVarBase(index, name, access, (uint8_t*)&value, varSize)
    {

    }

    ~SyncVar() = default;

    void setLocalValue(T value)
    {
        this->value = value;
    }

    T getLocalValue()
    {
        return value;
    }

    double toDouble() const override
    {
        return (double)value;
    }

    QString toString() const override
    {
        return QString::number(value);
    }

    void fromDouble(double value) override
    {
        this->value = (T)value;
    }

    bool fromString(QString text) override
    {
        // If the number is supposed to be integer, check that the string does
        // not contain characters for floating-point numbers.
        if(std::numeric_limits<T>::is_integer &&
           text.contains(QRegExp("[.,eE]")))
        {
            return false;
        }

        // Convert from string to number, and check that the conversion was
        // successful, and the number fits the type range.
        if(std::numeric_limits<T>::is_integer)
        {
            if(std::numeric_limits<T>::is_signed)
            {
                bool ok;
                int64_t tempValue = text.toInt(&ok);

                if(!ok)
                    return false;

                if(tempValue >= (signed)std::numeric_limits<T>::lowest() &&
                   tempValue <= (signed)std::numeric_limits<T>::max())
                {
                    value = (T)tempValue;
                    return true;
                }
                else
                    return false;
            }
            else
            {
                bool ok;
                uint64_t tempValue = text.toInt(&ok);

                if(!ok)
                    return false;

                if(tempValue >= (unsigned)std::numeric_limits<T>::lowest() &&
                   tempValue <= (unsigned)std::numeric_limits<T>::max())
                {
                    value = (T)tempValue;
                    return true;
                }
                else
                    return false;
            }
        }
        else
        {
            bool ok;
            double tempValue = text.toDouble(&ok);

            if(!ok)
                return false;

            if(tempValue >= std::numeric_limits<T>::lowest() &&
               tempValue <= std::numeric_limits<T>::max())
            {
                value = (T)tempValue;
                return true;
            }
            else
                return false;
        }
    }

protected:
    T value;
};

SyncVarBase* makeSyncVar(VarType type, int index, QString name,
                         VarAccess access);

/**
 * @}
 */

#endif // SYNCVAR_H
