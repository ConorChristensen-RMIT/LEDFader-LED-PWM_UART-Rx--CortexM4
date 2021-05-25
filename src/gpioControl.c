/********************************************
*			STM32F439 GPIO Controller							*
*			Developed for the STM32								*
*			Author: Dr. Glenn Matthews						*
*			Source File														*
********************************************/

#include "stm32f439xx.h"
#include "gpioControl.h"


// The GPIO Controller is designed to take care of the peripheral mapping
// for the various devices on the STM32F439. 

//******************************************************************************//
// Function: configureGPIOPin()
// Input : The GPIO Port
// Return : None
// Description : Configure an individual GPIO Pin
// *****************************************************************************//
void gpio_configureGPIO(GPIO_Config * inputGPIO)
{
	// The struct GPIO contains all of the necessary information to set the GPIO Pin.
		
	// Clear out the configuration registers for the port.
	inputGPIO->port->MODER &= ~(0x03 << (inputGPIO->pin * 2));				// Mode Register
	inputGPIO->port->OTYPER &= ~(0x01 << inputGPIO->pin);						// Output Type Register
	inputGPIO->port->OSPEEDR &= ~(0x03 << (inputGPIO->pin * 2));			// Output Speed Register
	inputGPIO->port->PUPDR &= ~(0x03 << (inputGPIO->pin * 2));				// Pull-up / Pull-down register
	
	// Set the configuration of the pins (defined in enums, so no checking is necessary)
	inputGPIO->port->MODER |= inputGPIO->mode << (inputGPIO->pin * 2);			// Set the mode register		
	inputGPIO->port->OTYPER |= inputGPIO->outputType << inputGPIO->pin;						// Output Type Register
	inputGPIO->port->OSPEEDR |= inputGPIO->speed << (inputGPIO->pin * 2);					// Output Speed Register
	inputGPIO->port->PUPDR |= inputGPIO->pullUpDown << (inputGPIO->pin * 2);				// Pull-up / Pull-down register
}


//******************************************************************************//
// Function: setGPIO()
// Input : The GPIO port and pin to be set to logic 1
// Return : None
// Description : Bit set the GPIO Pin passed in..
// *****************************************************************************//
void gpio_setGPIO(GPIO_TypeDef * inputPort, GPIOPin inputPin)
{
	// Set the bit of the port by 'or-ing' the bit set register
	inputPort->BSRR |= 0x01 << inputPin;

}

//******************************************************************************//
// Function: resetGPIO()
// Input : The GPIO port and pin to be set to logic 0
// Return : None
// Description : Bit clear the GPIO Pin passed in..
// *****************************************************************************//
void gpio_resetGPIO(GPIO_TypeDef * inputPort, GPIOPin inputPin)
{	
	// To 'reset' a bit, the upper 16-bits can be utilised.
	inputPort->BSRR |= 0x01 << (inputPin + 16);
}


//******************************************************************************//
// Function: getPinValue()
// Input : The GPIO port and pin to read
// Return : None
// Description : Obtain the value of an I/O Pin
// *****************************************************************************//
uint8_t gpio_getPinValue(GPIO_TypeDef * inputPort, GPIOPin inputPin)
{
	uint8_t bitValue = 0;
	
	bitValue = (inputPort->IDR >> inputPin) & 0x00000001;
	
	return bitValue;

}



