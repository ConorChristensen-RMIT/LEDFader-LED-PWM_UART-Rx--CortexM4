
/********************************************
*			STM32F439 Main (C Startup File)  			*
*			Developed for the STM32						*
*			Author: Dr. Glenn Matthews					*
*			Source File									*
********************************************/

#include <stdint.h>
#include "boardSupport.h"
#include "main.h"
#include "stm32f439xx.h"


void startSingleShotTIM6(uint16_t, uint16_t);
void startSingleShotTIM7(uint16_t, uint16_t);
void toggleLEDS(void);
int8_t getCharacter(void);
void transmissionWait(void);
void turnOFFLEDS(void);

//******************************************************************************//
// Function: main()
// Input : None
// Return : None
// Description : Entry point into the application.
// *****************************************************************************//
int main(void){
	int8_t old_number = 153;				//intialise previous characters value as "100"
	int8_t recieved_character1;					//initialise the three incoming characters
	int8_t recieved_character2;
	int8_t recieved_character3;
	int16_t recieved_number;
	uint16_t prescale;
	uint16_t prescale1s = 840;					//Prescale value for 1 second pulse		
	uint16_t tim6_count = 50000;				//The count value to load into TIM6
	uint16_t tim7_count = 1000;					//Initialise the count value for TIM7 at 1000 (This is the PWM timer)
	uint16_t tim7_total_count = 1000;					//What the total timer delay for the PWM cycle should be
	// Bring up the GPIO for the power regulators.
	boardSupport_init();
	//Configure the UART
	configureUART();
	//Configure the Leds
	configureLEDS();
	enableTIM6TIM7();
	
  while (1){
		recieved_character1 = getCharacter();
		transmissionWait();
		recieved_character2 = getCharacter();
		transmissionWait();
		recieved_character3 = getCharacter();
		transmissionWait();
		
		recieved_number = recieved_character1 + recieved_character2 + recieved_character3;
		if (recieved_number == 144) {
			turnOFFLEDS();																					//If the recieved number is 0 then do not complete the loop
			continue;}											
		else if (recieved_number == 145) {
			if (recieved_character2 == 49) prescale = 10*prescale1s; //If the recieved number is 10 set the prescale to 10s
			else prescale = prescale1s;															 //If the recieved number is 100 set the prescale to 1s
		}
		else if (recieved_number == 153) prescale = 2*prescale1s; //If the recieved number is 90 set the prescale to 2s
		else if (recieved_number == 152) prescale = 3*prescale1s; //If the recieved number is 80 set the prescale to 3s
		else if (recieved_number == 151) prescale = 4*prescale1s; //If the recieved number is 70 set the prescale to 4s
		else if (recieved_number == 150) prescale = 5*prescale1s; //If the recieved number is 60 set the prescale to 5s
		else if (recieved_number == 149) prescale = 6*prescale1s; //If the recieved number is 50 set the prescale to 6s
		else if (recieved_number == 148) prescale = 7*prescale1s; //If the recieved number is 40 set the prescale to 7s
		else if (recieved_number == 147) prescale = 8*prescale1s; //If the recieved number is 30 set the prescale to 8s
		else if (recieved_number == 146) prescale = 9*prescale1s; //If the recieved number is 20 set the prescale to 9s
		else {
			recieved_number = old_number;	//If no character recieved or invalid character, default to the last value (initially 1s fade)
			continue;
		}
		tim7_count = 1000;
		startSingleShotTIM6(prescale1s, tim6_count);
		while ((TIM6->SR & TIM_SR_UIF) == 0){					//While TIM6 hasn't expired 
			toggleLEDS();																				//toggleLEDS the leds 
			startSingleShotTIM7(prescale1s, tim7_count);
			while ((TIM7->SR & TIM_SR_UIF) == 0);
			toggleLEDS();																				//toggleLEDS the LEDs again
			startSingleShotTIM7(prescale1s, tim7_total_count - tim7_count);		//Load the count for the remaining time
			while ((TIM7->SR & TIM_SR_UIF) == 0);								//Wait for the total time - the first half of the duty cycle
			tim7_count -= 20;																		//Change the count to alter the duty by 1/50th ((1/50)*1000)
		}
		toggleLEDS(); //Toggle the LEDs one more time to change the direction of the fade
		old_number = recieved_number;
  }
} 

void transmissionWait(void)
{
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");		//Wait 10 clock cycles for input
}

 //******************************************************************************//
// Function: getCharacter()
// Input: None
// Output: int8_t
// Description: Check the recieved UART for a character and return the integer of that character accordingly
// *****************************************************************************//
int8_t getCharacter()
{
	int8_t recieved = -1;
	int8_t valid= -1;
	// Check if a character has been recieved
	if(USART3->SR & USART_SR_RXNE)
	{
		//Get the character from the data register
		recieved = USART3->DR;
		
		//check if the character recieved was valid
		//Valid number check
		if (recieved >= '0' && recieved <= '9') valid = 1;
		//Control characters
		else if ((recieved == 0x0D) || (recieved == 0x0A)) valid = 1;
	}
	if (valid == -1) recieved = -1;
	return recieved;
}

void turnOFFLEDS(void)
{
	//Set the output data high (LED off)
	GPIOA->ODR &= ~(GPIO_ODR_OD3 | GPIO_ODR_OD8 | GPIO_ODR_OD9 | GPIO_ODR_OD10);
	GPIOB->ODR &= ~(GPIO_ODR_OD0 | GPIO_ODR_OD1 | GPIO_ODR_OD8);
	GPIOF->ODR &= ~GPIO_ODR_OD8;
}
void toggleLEDS(void)
{
	//Set the output data high (LED off)
	GPIOA->ODR ^= (GPIO_ODR_OD3 | GPIO_ODR_OD8 | GPIO_ODR_OD9 | GPIO_ODR_OD10);
	GPIOB->ODR ^= (GPIO_ODR_OD0 | GPIO_ODR_OD1 | GPIO_ODR_OD8);
	GPIOF->ODR ^= GPIO_ODR_OD8;
}

void startSingleShotTIM6(uint16_t pre_scale, uint16_t count)
{
	TIM6->CR1 &= ~(TIM_CR1_CEN);				//Make sure timer is disabled
	TIM6->PSC &= ~(TIM_PSC_PSC_Msk);		//Clear the prescale
	TIM6->PSC |= pre_scale;							//Bitwise OR with the prescale argument to load the value
	TIM6->ARR &= ~(TIM_ARR_ARR_Msk);		//Clear the autoload reg
	TIM6->ARR |= count;									//Set the count
	TIM6->CR1 |= TIM_CR1_OPM;						//Set Single Shot mode ON
	TIM6->CR1 |= TIM_CR1_CEN;						//enableTIM6TIM7 the timer (Start it)
}

void startSingleShotTIM7(uint16_t pre_scale, uint16_t count)
{
	TIM7->CR1 &= ~(TIM_CR1_CEN);				//Make sure timer is disabled
	TIM7->PSC &= ~(TIM_PSC_PSC_Msk);		//Clear the prescale
	TIM7->PSC |= pre_scale;							//Bitwise OR with the prescale argument to load the value
	TIM7->ARR &= ~(TIM_ARR_ARR_Msk);		//Clear the autoload reg
	TIM7->ARR |= count;									//Set the count
	TIM7->CR1 |= TIM_CR1_OPM;						//Set Single Shot mode ON
	TIM7->CR1 |= TIM_CR1_CEN;						//enableTIM6TIM7 the timer (Start it)
}
