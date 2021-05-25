/**********************************************************
*			STM32F439 Board Support Package	  									*
*			Developed for the STM32						  								*
*			Author: Dr. Glenn Matthews					  							*
*			Header File								      										*
***********************************************************/

// Guard include
#ifndef __BOARDSUPPORT_H__
#define __BOARDSUPPORT_H__

#include "stm32f439xx.h"
#include <stdint.h>
		
void boardSupport_init(void);							// Initialise the underlying core hardware for the STM32
void configureUART(void);
void configureLEDS(void);
void enableTIM6TIM7(void);
void delay_software_ms(uint32_t );					// Software delay (can be used before the scheduler starts)
void delay_software_us(uint32_t);					// Crude software delay (in microseconds)


#endif
