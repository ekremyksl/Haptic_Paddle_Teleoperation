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

#include "main.h"
#include "communication.h"
#include "torque_regulator.h"
#include "haptic_controller.h"
#include "drivers/adc.h"
#include "drivers/callback_timers.h"
#include "drivers/dac.h"
#include "drivers/debug_gpio.h"
#include "drivers/h_bridge.h"
#include "drivers/hall.h"
#include "drivers/incr_encoder.h"
#include "drivers/led.h"
#include "lib/utils.h"

/**
  * @brief  Main function, sets up all the drivers and controllers.
  */
int main(void)
{
    // Set up the GPIOs and the interrupts.
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB |
                           RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD |
                           RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    //
    cbt_Init(); // Set up the timers that will call the loops functions.
    
	adc_Init(); // Set up the ADC.
	
	comm_Init(); // Set up the communication module.
    
    torq_Init(); // Set up the torque (current) regulator.

    hapt_Init(); // Set up the haptic controller.
	
    enc_Init(); // Set up the incremental encoders.
    
    hb_Init();   // Set up the H-bridge.
    hb_Enable(); //
    
    dac_Init(); // Set up the DAC.
    
    led_Init(); // Set up the LEDs.
    
    dio_Init();
    
    // Delay to let the power electronics stabilize before calibrating the
    // current sensor.
    utils_DelayMs(200);
    
    adc_CalibrateCurrentSens();
    
    // Start the current loop.
    torq_StartCurrentLoop();
    
    // Init the Hall position sensor.
    hall_Init(ADC_CHANNEL_9);
    
    // End of the initialization. Lock the SyncVar list, and notify the PC that
    // the board has (re)started.
    comm_LockSyncVarsList();
    comm_NotifyReady();

	// Endless loop. The low priority functions are called here.
	while(1)
	{
		// Update the communication.
        comm_Step();
	}
}
