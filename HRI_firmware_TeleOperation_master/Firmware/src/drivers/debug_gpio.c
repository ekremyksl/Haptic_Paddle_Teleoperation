/*
 * Copyright (C) 2021 EPFL-REHAssist (Rehabilitation and Assistive Robotics Group).
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

#include "debug_gpio.h"

#define N_GPIOS 2
#define DIO_PORT GPIOD

const uint32_t dio_gpios[N_GPIOS] = { GPIO_Pin_0, GPIO_Pin_1};

void dio_Init(void)
{
    int i;
    GPIO_InitTypeDef GPIO_InitStruct;  

	GPIO_InitStruct.GPIO_Pin   = dio_gpios[0];
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(DIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin   = dio_gpios[1];
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_Init(DIO_PORT, &GPIO_InitStruct);

	GPIO_WriteBit(DIO_PORT, dio_gpios[i], Bit_RESET);
	GPIO_WriteBit(DIO_PORT, dio_gpios[i], Bit_RESET);
}

bool dio_Get(int pinIndex)
{
    if(pinIndex >= 0 && pinIndex < N_GPIOS)
        return GPIO_ReadInputDataBit(DIO_PORT, dio_gpios[pinIndex]);
    else
        return 0;
}

void dio_Set(int pinIndex, bool high)
{
    if(pinIndex >= 0 && pinIndex < N_GPIOS)
    {
        GPIO_WriteBit(DIO_PORT, dio_gpios[pinIndex],
                      high ? Bit_SET : Bit_RESET);
    }
}

void dio_Toggle(int pinIndex)
{
    if(pinIndex >= 0 && pinIndex < N_GPIOS)
    {
        bool newState = !GPIO_ReadInputDataBit(DIO_PORT, dio_gpios[pinIndex]);
        GPIO_WriteBit(DIO_PORT, dio_gpios[pinIndex],
                      newState ? Bit_SET : Bit_RESET);
    }
}
