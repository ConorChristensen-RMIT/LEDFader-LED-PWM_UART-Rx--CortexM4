/**********************************************************
*			STM32F439 Board Support Package	 									  *
*			Developed for the STM32						  								*
*			Author: Dr. Glenn Matthews					  							*
*			Source File								      										*
***********************************************************/

#include <stdint.h>
#include "stm32f439xx.h"
#include "ioMapping.h"
#include "boardSupport.h"
#include "gpioControl.h"


// Local function prototypes - These are all board specific.
static void boardSupport_initGPIO(void);


//******************************************************************************//
// Function: boardSupport_init()
// Input:
// Output:
// Description: Perform the basic board initialisation requests for the RMIT SOM
// *****************************************************************************//
void boardSupport_init()
{

	// STM32F439II Daughter Board Configuration
	// The clocks for the various GPIO should be brought up here.
	boardSupport_initGPIO();

}


//******************************************************************************//
// Function: boardSupport_initGPIO()
// Input:
// Output:
// Description: Perform the basic board initialisation requests for the RMIT SOM
// *****************************************************************************//
void boardSupport_initGPIO()
{
	// Create an instance of the GPIO Configuration Struct.
	GPIO_Config portConfig;

	// The GPIO clocks for the power management have alredy been configured by the 
	// RAM sub-system. For other GPIO and peripherals they will need to be enabled.

	// Configure PH4 as output - 1.2V rail.
	portConfig.port = GPIOH;
	portConfig.pin = Pin4;	
	portConfig.mode = GPIO_Output;
	portConfig.pullUpDown = GPIO_No_Pull;
	portConfig.outputType = GPIO_Output_PushPull;
	portConfig.speed = GPIO_2MHz;
	
	// Configure the I/O Port.
	gpio_configureGPIO(&portConfig);
	
	// Enable the 1.2VA rail.
	gpio_setGPIO(GPIOH, Pin4);
	
	// Configure Port PH6 as output - 3.3V rail.
	portConfig.pin = Pin7;
	gpio_configureGPIO(&portConfig);
	gpio_setGPIO(GPIOH, Pin7);
	
	/*portConfig.port = GPIOG;
	portConfig.pin = Pin12;	
	gpio_configureGPIO(&portConfig);
	gpio_setGPIO(GPIOG, Pin12);
	*/
	return;
}


//******************************************************************************//
// Function: delay_software_ms()
// Input: Delay time (in milliseconds)
// Output: None
// Description: Software driven delay loop.
// *****************************************************************************//
void delay_software_ms(uint32_t millisecondDelay)
{
	// If optimisation is off, then the delay should be set at 14000
	// In this function a basic software delay is utilised.
	uint32_t i = 0;
	for(i = 0; i < (millisecondDelay * 21000); i++);
}


//******************************************************************************//
// Function: delay_software_us()
// Input: Delay time (in microseconds)
// Output: None
// Description: Software driven delay loop.
// *****************************************************************************//
void delay_software_us( uint32_t usec )
{
    // To avoid delaying for less than usec, always round up.;
		uint32_t i = 0;
    for(i = 0; i < (usec * 21); i++);

}

//******************************************************************************//
// Function: configureUART()
// Input: None
// Output: None
// Description: Set the default configuration for the UART
// *****************************************************************************//
void configureUART()
{
	//USART is configured 115500, 8, N, 1
	// Enable the clock configuration for GPIOA, GPIOB, GPIOF, and USART3
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	// Set the reset bits
	RCC->AHB1RSTR |= RCC_AHB1RSTR_GPIOBRST;
	RCC->APB1RSTR |= RCC_APB1RSTR_USART3RST;
	//Wait two clock cycles
	__ASM("NOP"); __ASM("NOP");
	// Clear the reset bits
	RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_GPIOBRST);
	RCC->APB1RSTR &= ~(RCC_APB1RSTR_USART3RST);
	//Wait two clock cycles
	__ASM("NOP"); __ASM("NOP");
	//Configure the GPIOB bits for AF and output (pins 10, 11 for AF)
	GPIOB->MODER &= ~(GPIO_MODER_MODE11_Msk | GPIO_MODER_MODE10_Msk);	//Clear the desired bits
	GPIOB->MODER |= (0x02 << GPIO_MODER_MODE11_Pos) | (0x02 << GPIO_MODER_MODE10_Pos); //Set the Mode bits to 10 for AF, to 01 for output
	//Set alternate function to AF7
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL10_Msk | GPIO_AFRH_AFSEL11_Msk); //Clear the AFRH select bits 
	GPIOB->AFR[1] |= (0x07 << GPIO_AFRH_AFSEL10_Pos) | (0x07 << GPIO_AFRH_AFSEL11_Pos); //Set the AF to AF7
	// Turn on 16 time oversampling
	USART3->CR1 &= ~(USART_CR1_OVER8);
	// Clear the baud rate bits
	USART3->BRR &= 0xFFFF0000;
	//Set the baud rate (115,500). Mantissa = 0x16, mantissa = 0x0C
	USART3->BRR |= (0x016 << USART_BRR_DIV_Mantissa_Pos) | (0x0C << USART_BRR_DIV_Fraction_Pos);
	//Set the number of transfer bits (8)
	USART3->CR1 &= ~(USART_CR1_M);
	//Set the number of stop bits (1)
	USART3->CR2 &= ~(USART_CR2_STOP_Msk);
	USART2->CR2 |= (0X00 << USART_CR2_STOP_Pos);
	//Disable the parity bit
	USART3->CR1 &= ~(USART_CR1_PCE);
	// Select asynchronous mode
	USART3->CR2 &= ~(USART_CR2_CLKEN | USART_CR2_CPOL | USART_CR2_CPHA);
	//Disable hardware flow control
	USART3->CR2 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);
	//Finally, enable the USART trans and rec 
	USART3->CR1 |= (USART_CR1_TE | USART_CR1_UE | USART_CR1_RE);
	
}
	
//******************************************************************************//
// Function: configureLEDS()
// Input: None
// Output: None
// Description: Configure the relevant GPIO for the LED outputs
// *****************************************************************************//
void configureLEDS(){
	//Enable clock for GPIOA, GPIOB, GPIOF
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
	//Set the reset bits
	RCC->AHB1RSTR |= RCC_AHB1RSTR_GPIOARST;
	RCC->AHB1RSTR |= RCC_AHB1RSTR_GPIOBRST;
	RCC->AHB1RSTR |= RCC_AHB1RSTR_GPIOFRST;
	//Wait two clock cycles
	__ASM("NOP"); __ASM("NOP");
	//Clear the reset bits
	RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_GPIOARST);
	RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_GPIOBRST);
	RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_GPIOFRST);
	//Wait two clock cycles
	__ASM("NOP"); __ASM("NOP");
	//Configure the GPIOB (pins 0, 1, 8), GPIOA (pins 3, 8, 9, 10) and GPIOF (pins 8) bits for LED outupt
	//Clear the mode for the pins
	GPIOA->MODER &= ~(GPIO_MODER_MODE3_Msk | GPIO_MODER_MODE8_Msk | GPIO_MODER_MODE9_Msk | GPIO_MODER_MODE10_Msk);
	GPIOF->MODER &= ~(GPIO_MODER_MODE8_Msk);
	GPIOB->MODER &= ~(GPIO_MODER_MODE8_Msk | GPIO_MODER_MODE1_Msk | GPIO_MODER_MODE0_Msk);
	//Set the pin modes to output (01)
	GPIOA->MODER |= (0x01 << GPIO_MODER_MODE3_Pos) | (0x01 << GPIO_MODER_MODE8_Pos) | (0x01 << GPIO_MODER_MODE9_Pos) | (0x01 << GPIO_MODER_MODE10_Pos);
	GPIOF->MODER |= (0x01 << GPIO_MODER_MODE8_Pos);
	GPIOB->MODER |= (0x01 << GPIO_MODER_MODE8_Pos) | (0x01 << GPIO_MODER_MODE1_Pos) | (0x01 << GPIO_MODER_MODE0_Pos);
	//Clear OTYPER bits to enable push/pull output
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT3 | GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9 | GPIO_OTYPER_OT10);
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1 | GPIO_OTYPER_OT8);
	GPIOF->OTYPER &= ~(GPIO_OTYPER_OT8);
	//set the speed to medium (clear first, then set)
	GPIOA->OSPEEDR &= ~((0x03 << GPIO_OSPEEDR_OSPEED3_Pos) | (0x03 << GPIO_OSPEEDR_OSPEED8_Pos)  | (0x03 << GPIO_OSPEEDR_OSPEED9_Pos) | (0x03 << GPIO_OSPEEDR_OSPEED10_Pos));
	GPIOB->OSPEEDR &= ~((0x03 << GPIO_OSPEEDR_OSPEED0_Pos) | (0x03 << GPIO_OSPEEDR_OSPEED1_Pos)  | (0x03 << GPIO_OSPEEDR_OSPEED8_Pos));
	GPIOF->OSPEEDR &= ~(0x03 << GPIO_OSPEEDR_OSPEED8_Pos);
	GPIOA->OSPEEDR |= (0x01 << GPIO_OSPEEDR_OSPEED3_Pos) | (0x01 << GPIO_OSPEEDR_OSPEED8_Pos) | (0x01 << GPIO_OSPEEDR_OSPEED9_Pos) | (0x01 << GPIO_OSPEEDR_OSPEED10_Pos);
	GPIOB->OSPEEDR |= (0x01 << GPIO_OSPEEDR_OSPEED0_Pos) | (0x01 << GPIO_OSPEEDR_OSPEED1_Pos) | (0x01 << GPIO_OSPEEDR_OSPEED8_Pos);
	GPIOF->OSPEEDR |= (0x01 << GPIO_OSPEEDR_OSPEED8_Pos);
	//Clear the pull up pull down registers
	GPIOA->PUPDR &= ~((0x03 << GPIO_PUPDR_PUPD3_Pos) | (0x03 << GPIO_PUPDR_PUPD8_Pos)  | (0x03 << GPIO_PUPDR_PUPD9_Pos) | (0x03 << GPIO_PUPDR_PUPD10_Pos));
	GPIOB->PUPDR &= ~((0x03 << GPIO_PUPDR_PUPD0_Pos) | (0x03 << GPIO_PUPDR_PUPD1_Pos)  | (0x03 << GPIO_PUPDR_PUPD8_Pos));
	GPIOF->PUPDR &= ~(0x03 << GPIO_PUPDR_PUPD8_Pos);
	//Set the output data high (LED off)
	GPIOA->ODR |= (GPIO_ODR_OD3 | GPIO_ODR_OD8 | GPIO_ODR_OD9 | GPIO_ODR_OD10);
	GPIOB->ODR |= (GPIO_ODR_OD0 | GPIO_ODR_OD1 | GPIO_ODR_OD8);
	GPIOF->ODR |= GPIO_ODR_OD8;
}


//******************************************************************************//
// Function: enableTIM6TIM7()
// Input: None
// Output: None
// Description: Enable Timer 6 and Timer 7
// *****************************************************************************//
void enableTIM6TIM7(){
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
	RCC->APB1RSTR |= RCC_APB1RSTR_TIM6RST;
	RCC->APB1RSTR |= RCC_APB1RSTR_TIM7RST;
	__ASM("NOP");
	__ASM("NOP");
	RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM6RST);
	RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM7RST);	
	__ASM("NOP");
	__ASM("NOP");
	
}












