
/********************************************
*			Pin Mapping														*
*			Developed for the STM32								*
*			Author: Matthew Pulis									*
*			Header File														*
********************************************/

#include "stm32f439xx.h"


#ifndef __PINMAPPING_H__
#define __PINMAPPING_H__

	// General Pinout Description
	// Debug (printf) UART
	#define USART_PORT		USART3

	// ****************** ******** ****************** //
	// ********* Power Rail Control Mapping ********* //
	// ****************** ******** ****************** //
	#define PowerRail_Enable_1V2A_pin 			GPIO_PIN_4
	#define PowerRail_Enable_1V2A_port			GPIOH

	#define PowerRail_PowerGood_1V2A_pin 		GPIO_PIN_7
	#define PowerRail_PowerGood_1V2A_port		GPIOH

	#define PowerRail_PowerGood_3V3A_pin 		GPIO_PIN_6
	#define PowerRail_PowerGood_3V3A_port		GPIOH


#endif
