/*
 * Copyright (c) 2017, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    tarea4_sm2.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_pit.h"

#define SWITCH2_MASK	(1<<6)
#define SWITCH3_MASK	(1<<4)
#define SECOND_TICK		(55000000U)

static uint8_t Interruptor = 0;

uint8_t sequence_led(uint8_t type_sequence)
{
	static uint8_t state = 0;

	if(1 == PIT_GetStatusFlags(PIT, kPIT_Chnl_0))
	{
		if(0 == type_sequence)
		{
			state++;
			state = (4 == state)?0:state;
		}
		if(1 == type_sequence)
		{
			state = (0 == state)?3:state;
			state--;
		}

		if(1 == state)
		{
			GPIO_WritePinOutput(GPIOE,26,1);
			GPIO_WritePinOutput(GPIOB,22,0);
			GPIO_WritePinOutput(GPIOB,21,1);
		}
		if(2 == state)
		{
			GPIO_WritePinOutput(GPIOE,26,0);
			GPIO_WritePinOutput(GPIOB,22,1);
			GPIO_WritePinOutput(GPIOB,21,1);
		}
		if(3 == state)
		{
			GPIO_WritePinOutput(GPIOE,26,1);
			GPIO_WritePinOutput(GPIOB,22,1);
			GPIO_WritePinOutput(GPIOB,21,0);
		}
    	PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
	}
	return (state);
}

void PORTA_IRQHandler()
{
	Interruptor = (0 == Interruptor)?1:0;
	PIT_StartTimer(PIT, kPIT_Chnl_0);
	PORT_ClearPinsInterruptFlags(PORTA, SWITCH3_MASK);
	PORT_ClearPinsInterruptFlags(PORTC, SWITCH2_MASK);
}
void PORTC_IRQHandler()
{
	PIT_StopTimer(PIT, kPIT_Chnl_0);
}


int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

    //Clock to enable the switch3 PTA4
	CLOCK_EnableClock(kCLOCK_PortA);
	//Clock to enable the leds RED(PTB22) and BLUE(PTB21)
	CLOCK_EnableClock(kCLOCK_PortB);
	//Clock to enable the switch2 PTC6
	CLOCK_EnableClock(kCLOCK_PortC);
	//Clock to enable the led GREEN PTE26
	CLOCK_EnableClock(kCLOCK_PortE);

	//Configuration of the led
	port_pin_config_t config_led =
	{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
			kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAsGpio,
			kPORT_UnlockRegister,
	};
	//Set the configuration of the BLUE led
	PORT_SetPinConfig(PORTB, 21, &config_led);

	//Set the configuration of the RED led
	PORT_SetPinConfig(PORTB, 22, &config_led);

	//Set the configuration of the GREEN led
	PORT_SetPinConfig(PORTE, 26, &config_led);


	//Configuration of the SWITCH
	port_pin_config_t config_switch =
	{ kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
			kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAsGpio,
			kPORT_UnlockRegister
	};
	//Set the interruption mode of SWITCH 3
	PORT_SetPinInterruptConfig(PORTA, 4, kPORT_InterruptFallingEdge);
	//Set the configuration of the SWITCH 3
	PORT_SetPinConfig(PORTA, 4, &config_switch);


	//Set the interruption mode of SWITCH 2
	PORT_SetPinInterruptConfig(PORTC, 6, kPORT_InterruptFallingEdge);
	//Set the configuration of the SWITCH 2
	PORT_SetPinConfig(PORTC, 6, &config_switch);


	gpio_pin_config_t led_config_gpio =
	{ kGPIO_DigitalOutput, 1 };

	GPIO_PinInit(GPIOB, 21, &led_config_gpio);
	GPIO_PinInit(GPIOB, 22, &led_config_gpio);
	GPIO_PinInit(GPIOE, 26, &led_config_gpio);


	gpio_pin_config_t switch_config_gpio =
	{ kGPIO_DigitalInput, 1 };

	GPIO_PinInit(GPIOA, 4, &switch_config_gpio);
	GPIO_PinInit(GPIOC, 6, &switch_config_gpio);


	pit_config_t config_pit0;

	PIT_GetDefaultConfig(&config_pit0);
	PIT_Init (PIT, &config_pit0);
	PIT_EnableInterrupts (PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
	PIT_GetEnabledInterrupts(PIT, kPIT_Chnl_0);
	PIT_SetTimerPeriod (PIT, kPIT_Chnl_0, SECOND_TICK);

	NVIC_SetPriority(PORTA_IRQn, 9);
	NVIC_SetPriority(PORTC_IRQn, 9);
	NVIC_SetPriority(PIT0_IRQn, 10);

	NVIC_EnableIRQ(PORTA_IRQn);
	NVIC_EnableIRQ(PORTC_IRQn);

	PIT_StartTimer(PIT, kPIT_Chnl_0);

    while(1)
    {
    	if(0 == PORT_GetPinsInterruptFlags(PORTC))
    	{
        	sequence_led(Interruptor);
    	}
    }
    return 0;
}
