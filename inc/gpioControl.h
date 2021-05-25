/********************************************
*			STM32F439 GPIO Controller							*
*			Developed for the STM32								*
*			Author: Dr. Glenn Matthews						*
*			Header File														*
********************************************/

// Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_CONTROLLER_H__
#define __GPIO_CONTROLLER_H__

#include "stm32f439xx.h"


// Friendly name for the GPIO Prts (external use only)
typedef enum {GPIO_A = 0, GPIO_B, GPIO_C,	GPIO_D,	GPIO_E,	GPIO_F,	GPIO_G, GPIO_H, GPIO_I} GPIOPort;
typedef enum {Pin0 = 0, Pin1, Pin2, Pin3, Pin4, Pin5, Pin6, Pin7, Pin8, Pin9, Pin10, Pin11, Pin12, Pin13, Pin14, Pin15}GPIOPin;
typedef enum {GPIO_Input = 0, GPIO_Output, GPIO_AF, GPIO_Analog}GPIOMode;																													// Port Mode
typedef enum {GPIO_No_Pull = 0, GPIO_Pull_Up, GPIO_Pull_Down, GPIO_Reserved}GPIOPullUpDown;																				// Pull-up / Pull down register
typedef enum {GPIO_Output_PushPull = 0, GPIO_Output_OpenDrain} GPIOOutputType;																										// Value stored in the OTYPER register
typedef enum {GPIO_2MHz = 0, GPIO_25MHz, GPIO_50MHz, GPIO_100MHz }GPIOSpeed;																											// Value stored in the OSPEEDR

// Port configuration bits.
#define GPIO_OUT_PP			(unsigned char)	0
#define GPIO_OUT_OD			(unsigned char) 1
#define GPIO_AF_PP			(unsigned char) 2
#define GPIO_AF_OD			(unsigned char) 3

#define GPIO_IN_AN			(unsigned char) 0
#define GPIO_IN_FL			(unsigned char) 1
#define GPIO_IN_PUPD		(unsigned char) 2
		
// STM32F437 MODER Configuration Bits
#define GPIO_IN					(unsigned char) 0x00
#define GPIO_OUT				(unsigned char) 0x01
#define GPIO_AF					(unsigned char) 0x02
#define GPIO_ANALOG			(unsigned char) 0x03
	
// Defines for individual bit masks access - potentially faster?
#define setPin_0					0x00000001
#define setPin_1					0x00000002
#define setPin_2					0x00000004
#define setPin_3					0x00000008
#define setPin_4					0x00000010
#define setPin_5					0x00000020
#define setPin_6					0x00000040
#define setPin_7					0x00000080
#define setPin_8					0x00000100
#define setPin_9					0x00000200
#define setPin_10					0x00000400
#define setPin_11					0x00000800
#define setPin_12					0x00001000
#define setPin_13					0x00002000
#define setPin_14					0x00004000
#define setPin_15					0x00008000


#define resetPin_0					0x00010000
#define resetPin_1					0x00020000
#define resetPin_2					0x00040000
#define resetPin_3					0x00080000
#define resetPin_4					0x00100000
#define resetPin_5					0x00200000
#define resetPin_6					0x00400000
#define resetPin_7					0x00800000
#define resetPin_8					0x01000000
#define resetPin_9					0x02000000
#define resetPin_10					0x04000000
#define resetPin_11					0x08000000
#define resetPin_12					0x10000000
#define resetPin_13					0x20000000
#define resetPin_14					0x40000000
#define resetPin_15					0x80000000

typedef struct
{
		GPIO_TypeDef * port;							// GPIO Port on which the pin resides.
		GPIOPin  pin;										// The actual pin to modify (0 through to 15).
		GPIOMode mode;									// The mode of the GPIO Pin (Input, Output, Alternate Function or Analog Mode)
		GPIOPullUpDown pullUpDown;			// The pull-up/down configuration 
		GPIOOutputType outputType;			// If configured as an output, is it open drain or push pull
		GPIOSpeed speed;								// The maximum speed of the pin (2, 25, 50 or 100MHz)
	
} GPIO_Config;

			
		void gpio_configureGPIO(GPIO_Config *);										// Configure the GPIO Pin of interest
		uint8_t gpio_getPinValue(GPIO_TypeDef *, GPIOPin);					// Obtain the current value of an I/O pin (IDR)

		void gpio_setGPIO(GPIO_TypeDef *, GPIOPin);								// Set an individual I/O Pin
		void gpio_resetGPIO(GPIO_TypeDef *, GPIOPin);							// Clear an individual I/O Pin
			

#endif
