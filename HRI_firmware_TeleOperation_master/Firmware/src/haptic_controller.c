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

#include "haptic_controller.h"
#include "communication.h"
#include "drivers/adc.h"
#include "drivers/incr_encoder.h"
#include "drivers/hall.h"
#include "drivers/callback_timers.h"
#include "lib/utils.h"
#include "torque_regulator.h"
#include "drivers/ext_uart.h"

#define DEFAULT_HAPTIC_CONTROLLER_PERIOD 350 // Default control loop period [us].
#define START_BYTE 0x4D   //starting byte for synchronization

volatile uint32_t  hapt_timestamp; // Time base of the controller, also used to timestamp the samples sent by streaming [us].
volatile float32_t hapt_hallVoltage; // Hall sensor output voltage [V].
volatile float32_t hapt_encoderPaddleAngle; // Paddle angle measured by the incremental encoder [deg].
volatile float32_t hapt_motorTorque; // Motor torque [N.m].

volatile float32_t temp = 0.0f;


volatile uint8_t slave_bits;	//to check bits received by the slave
volatile uint32_t bytes_read = 0;
volatile float32_t temp_float32 = 0.0f;
volatile float32_t gui_variable = 0.0f;
volatile bool enable_master = false;

void hapt_Update(void);

/**
  * @brief Initializes the haptic controller.
  */
void hapt_Init(void)
{
	exuart_Init(256000);
    hapt_timestamp = 0;
    hapt_motorTorque = 0.0f;

    // Make the timers call the update function periodically.
    cbt_SetHapticControllerTimer(hapt_Update, DEFAULT_HAPTIC_CONTROLLER_PERIOD);

    // Share some variables with the computer.
    comm_monitorUint32Func("timestep [us]", cbt_GetHapticControllerPeriod,
                           cbt_SetHapticControllerPeriod);
    comm_monitorFloat("motor_torque [N.m]", (float32_t*)&hapt_motorTorque, READWRITE);
    comm_monitorFloat("encoder_paddle_pos [deg]", (float32_t*)&hapt_encoderPaddleAngle, READONLY);
    comm_monitorFloat("hall_voltage [V]", (float32_t*)&hapt_hallVoltage, READONLY);


    comm_monitorFloat("slave torque [N.m]", (float32_t*)&gui_variable, READONLY);
    comm_monitorBool("enable master torque", (bool*)&enable_master, READWRITE);
}

/**
  * @brief Updates the haptic controller state.
  */
void hapt_Update()
{

	static uint32_t temp_int32 = 0;
	void *temp_point = NULL;
	void *temp_pointer = &temp;
	uint32_t temp_word = *(uint32_t *)temp_pointer;
	static uint32_t second_div = 0;

    float32_t motorShaftAngle; // [deg].

    // Compute the dt (uncomment if you need it).
    float32_t dt = ((float32_t)cbt_GetHapticControllerPeriod()) / 1000000.0f; // [s].

    // Increment the timestamp.
    hapt_timestamp += cbt_GetHapticControllerPeriod();
    
    // Get the Hall sensor voltage.
    hapt_hallVoltage = hall_GetVoltage();

    // Get the encoder position.
    motorShaftAngle = enc_GetPosition();
    hapt_encoderPaddleAngle = motorShaftAngle / REDUCTION_RATIO;


    // sending bits to slave
    second_div++;

    if(second_div % 1 == 0){
    	uint8_t * byte_pointer = &hapt_encoderPaddleAngle;
    	exuart_SendByteAsync(START_BYTE);
    	exuart_SendByteAsync(*(byte_pointer)++); //sending
    	exuart_SendByteAsync(*(byte_pointer)++); //sending
    	exuart_SendByteAsync(*(byte_pointer)++); //sending
    	exuart_SendByteAsync(*(byte_pointer)++); //sending
    }

    // reading torque values from slave
    if(exuart_ReceivedBytesCount() >= 5){
    	//should keep reading the bytes until you see header
	   while(exuart_ReceivedBytesCount() >= 5){ //if any bits received
			slave_bits = exuart_GetByte();
			if(slave_bits == START_BYTE){ //check the header byte
				break;
			}
	   }
		if(exuart_ReceivedBytesCount() >= 4){ //if number of bytes received is >= 4 bytes, then keep receiving

			slave_bits = exuart_GetByte();
			temp_int32 = slave_bits;

			slave_bits = exuart_GetByte();
			temp_int32 |= slave_bits << 8;

			slave_bits = exuart_GetByte();
			temp_int32 |= slave_bits << 16;

			slave_bits = exuart_GetByte();
			temp_int32 |= slave_bits << 24;

			temp_point = &temp_int32;
			temp_float32 = *(float32_t *) temp_point;
			bytes_read = temp_int32;

			gui_variable = temp_float32;
		}

		if(enable_master){
			hapt_motorTorque = -temp_float32;
			// saturation block
			if(hapt_motorTorque > MOTOR_NOMINAL_TORQUE){
				hapt_motorTorque = MOTOR_NOMINAL_TORQUE;
			}
			else if(hapt_motorTorque < -MOTOR_NOMINAL_TORQUE){
				hapt_motorTorque = -MOTOR_NOMINAL_TORQUE;
			}
		}
		else{
			hapt_motorTorque = 0;
		}

		torq_SetTorque(hapt_motorTorque);
	}
}

