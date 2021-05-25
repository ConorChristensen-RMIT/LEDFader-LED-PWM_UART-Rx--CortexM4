/**
  ******************************************************************************
  * @file    system_stm32f4xx.c
  * @author  MCD Application Team
  * @brief   CMSIS Cortex-M4 Device Peripheral Access Layer System Source File.
  *
  *   This file provides two functions and one global variable to be called from 
  *   user application:
  *      - SystemInit(): This function is called at startup just after reset and 
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32f4xx.s" file.
  *
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick 
  *                                  timer or configure other parameters.
  *                                     
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.
  *
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32f4xx_system
  * @{
  */  
  
/** @addtogroup STM32F4xx_System_Private_Includes
  * @{
  */


#include "stm32f4xx.h"
#include "config.h"
#include "../platform/rtk_gnss/libraries/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

// Function prototypes
void SystemCoreClockUpdate(void);							// Update the variables relating to the system clock.
static void SetSysClock(void);								// Set the system clock.
void SystemInit_ExtMemCtl(void);

#if !defined  (HSE_VALUE) 
  #define HSE_VALUE    ((uint32_t)12000000) /*!< Default value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)16000000) /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Defines
  * @{
  */

/************************* Miscellaneous Configuration ************************/
/*!< Uncomment the following line if you need to use external SRAM or SDRAM as data memory  */
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx)\
 || defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx)
/* #define DATA_IN_ExtSRAM */
#endif /* STM32F40xxx || STM32F41xxx || STM32F42xxx || STM32F43xxx || STM32F469xx || STM32F479xx ||\
          STM32F412Zx || STM32F412Vx */
 
#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx)
/* #define DATA_IN_ExtSDRAM */
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F446xx || STM32F469xx ||\
          STM32F479xx */

/*!< Uncomment the following line if you need to relocate your vector Table in
     Internal SRAM. */
/* #define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET  0x00 /*!< Vector Table base offset field. 
                                   This value must be a multiple of 0x200. */
/******************************************************************************/

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Variables
  * @{
  */
  /* This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetHCLKFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency 
         Note: If you use this function to configure the system clock; then there
               is no need to call the 2 first functions listed above, since SystemCoreClock
               variable is updated automatically.
  */
uint32_t SystemCoreClock = 16800000;
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};
/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_FunctionPrototypes
  * @{
  */

#if defined (DATA_IN_ExtSRAM) || defined (DATA_IN_ExtSDRAM)
  static void SystemInit_ExtMemCtl(void); 
#endif /* DATA_IN_ExtSRAM || DATA_IN_ExtSDRAM */

/**
  * @}
  */

/** @addtogroup STM32F4xx_System_Private_Functions
  * @{
  */



/**
  * @brief  Setup the microcontroller system
  *         Initialize the FPU setting, vector table location and External memory 
  *         configuration.
  * @param  None
  * @retval None
  */
void SystemInit(void)
{
  /* FPU settings ------------------------------------------------------------*/
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
  #endif
  /* Reset the RCC clock configuration to the default reset state ------------*/
  /* Set HSION bit */
  RCC->CR |= (uint32_t)0x00000001;

  /* Reset CFGR register */
  RCC->CFGR = 0x00000000;

  /* Reset HSEON, CSSON and PLLON bits */
  RCC->CR &= (uint32_t)0xFEF6FFFF;

  /* Reset PLLCFGR register, HSI used by default */
  RCC->PLLCFGR = 0x24003010;

  /* Reset HSEBYP bit */
  RCC->CR &= (uint32_t)0xFFFBFFFF;

  /* Disable all interrupts */
  RCC->CIR = 0x00000000;

  // Set the custom configuration for the system clock.
  // The HAL variables are updated at the end of this function.
  SetSysClock();

  SystemInit_ExtMemCtl( );

#if defined (DATA_IN_ExtSRAM) || defined (DATA_IN_ExtSDRAM)
  SystemInit_ExtMemCtl(); 
#endif /* DATA_IN_ExtSRAM || DATA_IN_ExtSDRAM */

  /* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM
  SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#else
  SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif
}

/**
   * @brief  Update SystemCoreClock variable according to Clock Register Values.
  *         The SystemCoreClock variable contains the core clock (HCLK), it can
  *         be used by the user application to setup the SysTick timer or configure
  *         other parameters.
  *           
  * @note   Each time the core clock (HCLK) changes, this function must be called
  *         to update SystemCoreClock variable value. Otherwise, any configuration
  *         based on this variable will be incorrect.         
  *     
  * @note   - The system frequency computed by this function is not the real 
  *           frequency in the chip. It is calculated based on the predefined 
  *           constant and the selected clock source:
  *             
  *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(*)
  *                                              
  *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(**)
  *                          
  *           - If SYSCLK source is PLL, SystemCoreClock will contain the HSE_VALUE(**) 
  *             or HSI_VALUE(*) multiplied/divided by the PLL factors.
  *         
  *         (*) HSI_VALUE is a constant defined in stm32f4xx_hal_conf.h file (default value
  *             16 MHz) but the real value may vary depending on the variations
  *             in voltage and temperature.   
  *    
  *         (**) HSE_VALUE is a constant defined in stm32f4xx_hal_conf.h file (its value
  *              depends on the application requirements), user has to ensure that HSE_VALUE
  *              is same as the real frequency of the crystal used. Otherwise, this function
  *              may have wrong result.
  *                
  *         - The result of this function could be not correct when using fractional
  *           value for HSE crystal.
  *     
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate(void)
{
  uint32_t tmp = 0, pllvco = 0, pllp = 2, pllsource = 0, pllm = 2;
  
  /* Get SYSCLK source -------------------------------------------------------*/
  tmp = RCC->CFGR & RCC_CFGR_SWS;

  switch (tmp)
  {
    case 0x00:  /* HSI used as system clock source */
      SystemCoreClock = HSI_VALUE;
      break;
    case 0x04:  /* HSE used as system clock source */
      SystemCoreClock = HSE_VALUE;
      break;
    case 0x08:  /* PLL used as system clock source */

      /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N
         SYSCLK = PLL_VCO / PLL_P
         */    
      pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
      pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
      
      if (pllsource != 0)
      {
        /* HSE used as PLL clock source */
        pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      }
      else
      {
        /* HSI used as PLL clock source */
        pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
      }

      pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >>16) + 1 ) *2;
      SystemCoreClock = pllvco/pllp;
      break;
    default:
      SystemCoreClock = HSI_VALUE;
      break;
  }
  /* Compute HCLK frequency --------------------------------------------------*/
  /* Get HCLK prescaler */
  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
  /* HCLK frequency */
  SystemCoreClock >>= tmp;
}


//******************************************************************************//
// Function: SetSysClock()
// Input : None
// Return : None
// Description : Configure the system clocks
// *****************************************************************************//
static void SetSysClock(void)
{
	// By default the High Speed External Osciallator (HSE) is used as the
	// PLL clock source.
	// For the STM32F4 Discovery, the external crystal oscillator is 8MHz
	// For the Emcraft SOM, the external crystal oscillator is 12MHz


	//volatile unsigned int tmp = 0x00;
	#if HSE_USED
		// Enable the High Speed Oscillator
		RCC->CR |= RCC_CR_HSEON;

		// Wait until the High Speed Oscillator is ready, or a timeout reached.
		volatile unsigned int StartUpCounter = 0;
		while(((RCC->CR & RCC_CR_HSERDY) == 0) && StartUpCounter!= HSE_STARTUP_TIMEOUT)
		{
			StartUpCounter++;
		}
	//	/while(((RCC->CR & RCC_CR_HSERDY) == 0));

		// Check to see that the High Speed Oscillator started successfully.
		if((RCC->CR & RCC_CR_HSERDY) == RCC_CR_HSERDY)
		{
			// The oscillator started successfully

			// Enable the power control (PWR) clock
			RCC->APB1ENR |= RCC_APB1ENR_PWREN;

			/* Select regulator voltage output Scale 1 mode, System frequency up to 168 MHz */
			PWR->CR |= PWR_CR_VOS;

			// Set the HCLK to the same speed as the system clock (AHB Bus (STM32F407 = 168MHz))
			RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE2 | RCC_CFGR_PPRE1);			// This should clear the bits.

			// Set the PCLK2 to the HCLK divided by 2. (APB2 Bus (High Speed) (STM32F407 = 84MHz))
			RCC->CFGR |= (0x04 << RCC_CFGR_PPRE2_Pos);

			// Set the PCLK1 to the HCLK divided by 4. (APB1 Bus (Low Speed) (STM32F407 = 42MHz))
			RCC->CFGR |= (0x05 << RCC_CFGR_PPRE1_Pos);

			// Set the PLL configuration register. The PLL_VCO frequency is given by:
			// PLL_VCO = (HSE_VALUE / PLL_M) * PLL_N
			// SYS_CLK = PLL_VCO / PLL_P (168MHz)
			// USB OTG FS, SDIO and RNG Clock = PLL_VCO / PLLQ (48MHz)
			// For the STM32F407, this equates to a PLL_VCO of
			// For the Emcraft board, this equates to a PLL_VCO of 336MHz (Sysclock = 168MHz)

			// Configure the main PLL
			RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) | (1 << 22) | (PLL_Q << 24);

			// Now that the main configuration is complete, enable the PLL
			RCC->CR |= RCC_CR_PLLON;

			// Wait until the main PLL has started
			while((RCC->CR & RCC_CR_PLLRDY) == 0);

			// Configure Flash prefetch, Instruction cache, Data cache and wait state
			FLASH->ACR &= ~(FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | (FLASH_ACR_LATENCY));
			FLASH->ACR |= (FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | (0x05 << FLASH_ACR_LATENCY_Pos));

			// FLASH->ACR.b.prften = 1; 					// Flash prefetch enable
			// FLASH->ACR.b.icen = 1;						// Instruction cache enable
			// FLASH->ACR.b.dcen = 1;						// Data cache enable
			// FLASH->ACR.b.latency = 0x05;			// 5 wait states.

			// Select the main PLL as system clock source
			RCC->CFGR &= ~(RCC_CFGR_SW_0 | RCC_CFGR_SW_1);
			RCC->CFGR |= RCC_CFGR_SW_1;
			// RCC->CFGR.b.sw0 = 0x00;
			// RCC->CFGR.b.sw1 = 0x01;

			// Wait until the main PLL is used as the system clock source
			while((RCC->CR & RCC_CR_PLLON) != RCC_CR_PLLON);
		}
		else
		{
			// Oscillator failed to start.
		//	HSEStatus = 0x00;

			/* If HSE fails to start-up, the application will have wrong clock
			 configuration. User can add here some code to deal with this error */
		}

	#elif !HSE_USED
		// Adjust the frequency of the internal oscillator
		// First clear the adjustment values (HSICAL and HSITRIM), before setting them
		//RCC->CR &= ~( 0xF << RCC_CR_HSITRIM_Pos);
		//RCC->CR &= ~( 0xFF << RCC_CR_HSICAL_Pos);
		//RCC->CR |= ( (HSITRIM & 0xF) << RCC_CR_HSITRIM_Pos);
		//RCC->CR |= ( (HSICAL & 0xFF) << RCC_CR_HSICAL_Pos);

		if((RCC->CR & RCC_CR_HSIRDY) == RCC_CR_HSIRDY)
		{
			// The oscillator started successfully

			// Enable the power control (PWR) clock
			RCC->APB1ENR |= RCC_APB1ENR_PWREN;

			/* Select regulator voltage output Scale 1 mode, System frequency up to 168 MHz */
			PWR->CR |= PWR_CR_VOS;

			// Set the HCLK to the same speed as the system clock (AHB Bus (STM32F407 = 168MHz))
			RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE2 | RCC_CFGR_PPRE1);			// This should clear the bits.

			// Set the PCLK2 to the HCLK divided by 2. (APB2 Bus (High Speed) (STM32F407 = 84MHz))
			RCC->CFGR |= (0x04 << RCC_CFGR_PPRE2_Pos);

			// Set the PCLK1 to the HCLK divided by 4. (APB1 Bus (Low Speed) (STM32F407 = 42MHz))
			RCC->CFGR |= (0x05 << RCC_CFGR_PPRE1_Pos);

			// Set the PLL configuration register. The PLL_VCO frequency is given by:
			// PLL_VCO = (HSE_VALUE / PLL_M) * PLL_N
			// SYS_CLK = PLL_VCO / PLL_P (168MHz)
			// USB OTG FS, SDIO and RNG Clock = PLL_VCO / PLLQ (48MHz)
			// For the STM32F407, this equates to a PLL_VCO of
			// For the Emcraft board, this equates to a PLL_VCO of 336MHz (Sysclock = 168MHz)

			// Configure the main PLL
			RCC->PLLCFGR = HSIPLL_M | (HSIPLL_N << 6) | (((HSIPLL_P >> 1) -1) << 16) | (HSIPLL_Q << 24);

			// Now that the main configuration is complete, enable the PLL
			RCC->CR |= RCC_CR_PLLON;

			// Wait until the main PLL has started
			while((RCC->CR & RCC_CR_PLLRDY) == 0);

			// Configure Flash prefetch, Instruction cache, Data cache and wait state
			FLASH->ACR &= ~(FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | (FLASH_ACR_LATENCY));
			FLASH->ACR |= (FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | (0x05 << FLASH_ACR_LATENCY_Pos));

			// FLASH->ACR.b.prften = 1; 					// Flash prefetch enable
			// FLASH->ACR.b.icen = 1;						// Instruction cache enable
			// FLASH->ACR.b.dcen = 1;						// Data cache enable
			// FLASH->ACR.b.latency = 0x05;			// 5 wait states.

			// Select the main PLL as system clock source
			RCC->CFGR &= ~(RCC_CFGR_SW_0 | RCC_CFGR_SW_1);
			RCC->CFGR |= RCC_CFGR_SW_1;
			// RCC->CFGR.b.sw0 = 0x00;
			// RCC->CFGR.b.sw1 = 0x01;

			// Wait until the main PLL is used as the system clock source
			while((RCC->CR & RCC_CR_PLLON) != RCC_CR_PLLON);
		}
		{
			// HSI Oscillator failed to start!
		}
	#endif

	#if defined (RTC_ENABLE)
		volatile unsigned int BackupResetCounter = 0;

		// Enable access to the real time clock (use external low-speed oscillator).
		PWR->CR |= PWR_CR_DBP;

		// Reset the backup domain interface and delay.
		RCC->BDCR |= RCC_BDCR_BDRST;
		for(BackupResetCounter = 0; BackupResetCounter <= 5; BackupResetCounter++);
		RCC->BDCR &= ~(RCC_BDCR_BDRST);
		for(BackupResetCounter = 0; BackupResetCounter <= 5; BackupResetCounter++);

		// Clear out the individual bits for the backup domain clocks (including wait states)
		RCC->BDCR &= ~(RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL | RCC_BDCR_LSEON);

		// Now set the bits of interest.
		// Enable the real-time clock peripheral.
		// External LSE as the clock source.
		// Enable the external oscillator.
		RCC->BDCR |= RCC_BDCR_RTCEN | (0x01 << RCC_BDCR_RTCSEL_Pos) | RCC_BDCR_LSEON;

		// Wait for the LSE to become ready.
		while((RCC->BDCR & RCC_BDCR_LSERDY_Msk) != 0x02)

		// Disable access to the backup domain.
		PWR->CR &= ~(PWR_CR_DBP);
	#endif

	// Set the variables relating to the system clock speed.
	SystemCoreClockUpdate();
}

//******************************************************************************//
// Function: SystemInit_ExtMemCtl()
// Input : None
// Return : None
// Description : Setup the external memory controller.
// This function configures the external memories (SRAM/SDRAM)
// This SRAM/SDRAM will be used as program data memory (including heap and stack).
// *****************************************************************************//
void SystemInit_ExtMemCtl(void)
{
	register uint32_t tmpreg = 0, timeout = 0xFFFF;
	register uint32_t index;

	// Enable GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH and GPIOI interface clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN | RCC_AHB1ENR_GPIOGEN | RCC_AHB1ENR_GPIOHEN | RCC_AHB1ENR_GPIOIEN ;

	// FMC Configuration for Port D
	GPIOD->MODER = 0xAAAA0A0A;
	GPIOD->OTYPER = 0x00000000;
	GPIOD->OSPEEDR = 0xAAAA0A0A;
	GPIOD->PUPDR = 0x00000000;
	GPIOD->AFR[1] = 0xCCCCCCCC;
	GPIOD->AFR[0] = 0x00CC00CC;

	// FMC Configuration for Port E
	GPIOE->MODER = 0xAAAAAAAA;
	GPIOE->OTYPER = 0x00000000;
	GPIOE->OSPEEDR = 0xAAAAAAAA;
	GPIOE->PUPDR = 0x00000000;
	GPIOE->AFR[1] = 0xCCCCCCCC;
	GPIOE->AFR[0] = 0xCCCCCCCC;

	// FMC Configuration for Port F
	GPIOF->MODER = 0xAA800AAA;
	GPIOF->OTYPER = 0x00000000;
	GPIOF->OSPEEDR = 0xAA800AAA;
	GPIOF->PUPDR = 0x00000000;
	GPIOF->AFR[1] = 0xCCCCC000;
	GPIOF->AFR[0] = 0xCCCCCC;

	// FMC Configuration for Port G - Modified for Reset.
	GPIOG->MODER = 0x800A9AAA;
	GPIOG->OTYPER = 0x00000000;
	GPIOG->OSPEEDR = 0x800AAAAA;
	GPIOG->PUPDR = 0x00001000;
	GPIOG->AFR[1] = 0xC00000CC;
	GPIOG->AFR[0] = 0xC0CCCCCC;

	// FMC Configuration for Port H
	GPIOH->MODER = 0xAAAA08A0;
	GPIOH->OTYPER = 0x00000000;
	GPIOH->OSPEEDR = 0xAAAA08A0;
	GPIOH->PUPDR = 0x00000000;
	GPIOH->AFR[1] = 0xCCCCCCCC;
	GPIOH->AFR[0] = 0xC0CC00;

	// FMC Configuration for Port I
	GPIOI->MODER = 0x28AAAA;
	GPIOI->OTYPER = 0x00000000;
	GPIOI->OSPEEDR = 0x28AAAA;
	GPIOI->PUPDR = 0x00000000;
	GPIOI->AFR[1] = 0xCC0;
	GPIOI->AFR[0] = 0xCCCCCCCC;

	// Reset the Flash Memory (while all other configuration is taking place) - G6
	GPIOG->BSRR = (1 << GPIO_BSRR_BR6_Pos);

//		// FMC Configuration for Port D					// PD0 (D2), PD1 (D3), PD8 (D13), PD9(D14), PD10(D15), PD14(D0), PD15(D1)
//		// Set Alternate Function (AF) 12
//		GPIOD->AFR[0] = (GPIO_AF12_FMC << GPIO_AFRL_AFSEL1_Pos) | (GPIO_AF12_FMC << GPIO_AFRL_AFSEL0_Pos);
//		GPIOD->AFR[1] = (GPIO_AF12_FMC << GPIO_AFRH_AFSEL15_Pos) | (GPIO_AF12_FMC << GPIO_AFRH_AFSEL14_Pos) | (GPIO_AF12_FMC << GPIO_AFRH_AFSEL10_Pos) | (GPIO_AF12_FMC << GPIO_AFRH_AFSEL9_Pos) | (GPIO_AF12_FMC << GPIO_AFRH_AFSEL8_Pos);
//		GPIOD->MODER = 0xA02A000A;						// Enable alternate function mode.
//		GPIOD->OSPEEDR = 0xA02A000A; 					// Speed of I/O pins set to high speed (need to check this).
//		GPIOD->OTYPER = 0x00000000;						// Push-pull outputs.
//		GPIOD->PUPDR = 0x00000000;						// No pull-up / No pull down.
//
//		// FMC Configuration for Port E						// PE0 (FMC_DQM0), PE1 (FMC_DQM1), PE7	(FMC_D4), PE8	(FMC_D5), PE9	(FMC_D6), PE10 (FMC_D7),
//																							// PE11	(FMC_D8), PE12 (FMC_D9), PE13	(FMC_D10), PE14	(FMC_D11), PE15	(FMC_D12)
//		GPIOE->AFR[0] = 0xC00000CC;						// Set Alternate Function (AF) 12.
//		GPIOE->AFR[1] = 0xCCCCCCCC;						// Set Alternate Function (AF) 12.
//		GPIOE->MODER = 0xAAAA800A;						// Enable alternate function mode.
//		GPIOE->OSPEEDR = 0xAAAA800A;					// Speed of I/O pins set to high speed (need to check this).
//		GPIOE->OTYPER = 0x00000000;						// Push-pull outputs.
//		GPIOE->PUPDR = 0x00000000;						// No pull-up / No pull down.
//
//
//		// FMC Configuration for Port F					// PF0 (FMC_A0), PF1	(FMC_A1), PF2 (FMC_A2), PF3	(FMC_A3), PF4	(FMC_A4), PF5	(FMC_A5), PF11 (FMC_NRAS)
//																							// PF12 (FMC_A6), PF13	(FMC_A7), PF14	(FMC_A8), PF15 (FMC_A9)
//		GPIOF->AFR[0] = 0x00CCCCCC;						// Set Alternate Function (AF) 12.
//		GPIOF->AFR[1] = 0xCCCCC000;						// Set Alternate Function (AF) 12.
//		GPIOF->MODER = 0xAA800AAA;						// Enable alternate function mode.
//		GPIOF->OSPEEDR = 0xAA800AAA;					// Speed of I/O pins set to high speed (need to check this).
//		GPIOF->OTYPER = 0x00000000;						// Push-pull outputs.
//		GPIOF->PUPDR = 0x00000000;						// No pull-up / No pull down.
//
//		// FMC Configuration for Port G					// PG0	(FMC_A10), PG1	(FMC_A11), PG2	(FMC_A12), PG4	(FMC_BA0), PG5	(FMC_BA1), PG8	(FMC_CLK),PG15 (FMC_NCAS)
//		GPIOG->AFR[0] = 0x00CC0CCC;						// Set Alternate Function (AF) 12.
//		GPIOG->AFR[1] = 0xC000000C;						// Set Alternate Function (AF) 12.
//		GPIOG->MODER = 0x80020A2A;						// Enable alternate function mode.
//		GPIOG->OSPEEDR = 0x80020A2A;					// Speed of I/O pins set to high speed (need to check this).
//		GPIOG->OTYPER = 0x00000000;						// Push-pull outputs.
//		GPIOG->PUPDR = 0x00000000;						// No pull-up / No pull down.
//
//		// FMC Configuration for Port H					// H8 (FMC_D16), H9 (FMC_D17), H10 (FMC_D18), H11 (FMC_D19), H12	(FMC_D20), H13 (FMC_D21), H14 (FMC_D22), H15 (FMC_D23)
//		GPIOH->AFR[0] = 0x00C0CC00;						// PH5 (NWE), PH3 (NCS), PH2 (CLKE)
//		GPIOH->AFR[1] = 0xCCCCCCCC;						// Set Alternate Function (AF) 12.
//		GPIOH->MODER = 0xAAAA08A0;						// Enable alternate function mode.
//		GPIOH->OSPEEDR = 0xAAAA0CF0;					// Speed of I/O pins set to high speed (need to check this).
//		GPIOH->OTYPER = 0x00000000;						// Push-pull outputs.
//		GPIOH->PUPDR = 0x00000000;						// No pull-up / No pull down.
//
//
//		// FMC Configuration for Pin I 					// PI0	(FMC_D24), PI1 (FMC_D25), PI2 (FMC_D26), PI3 (FMC_D27), PI4 (FMC_DQM2), PI5 (FMC_DQM3),
//																						// PI6 (FMC_D28), PI7 (FMC_D29), PI9 (FMC_D30), PI10 (FMC_D31),
//		GPIOI->AFR[0] = 0xCCCCCCCC;						// Set Alternate Function (AF) 12.
//		GPIOI->AFR[1] = 0x00000CC0;						// Set Alternate Function (AF) 12.
//		GPIOI->MODER = 0x0028AAAA;						// Enable alternate function mode.
//		GPIOI->OSPEEDR = 0x0028AAAA;					// Speed of I/O pins set to high speed (need to check this).
//		GPIOI->OTYPER = 0x00000000;						// Push-pull outputs.
//		GPIOI->PUPDR = 0x00000000;						// No pull-up / No pull down.

	// Enable the FMC interface clock
	RCC->AHB3ENR |= 0x00000001;

//	// Configure and enable SDRAM bank1 (512MBit -> 64MB)
//		FMC_BANK5_6->SDCR_1.b.rpipe = 0x00;				// No HCLK Delay Cycle
//		FMC_BANK5_6->SDCR_1.b.rburst = 0x01;			// Single read requests managed as a burst.
//		FMC_BANK5_6->SDCR_1.b.sdclk = 0x02;				// SDCLK Period = 2 x HCLK.
//		FMC_BANK5_6->SDCR_1.b.wp = 0x00;					// Write access allowed.
//		FMC_BANK5_6->SDCR_1.b.cas = 0x02;					// 2 Cycles CAS - change to 3.
//		FMC_BANK5_6->SDCR_1.b.nb = 0x01;					// 4 Internal Banks
//		FMC_BANK5_6->SDCR_1.b.mwid = 0x02;				// 32-bit wide memory bus.
//		FMC_BANK5_6->SDCR_1.b.nr = 0x02;					// 13-bit Row Address
//		FMC_BANK5_6->SDCR_1.b.nc = 0x01;					// 9-bit Column Address
//		FMC_Bank5_6->SDCR[0] = 0x000019E5;				// Confirmed based on previous bitwise implementation.

	//FMC_Bank5_6->SDCR[0] = 0x00001965;
	// MP: actually 13 row address bits, not 12
	FMC_Bank5_6->SDCR[0] = 0x00001969;
//
//		// Set the SDRAM interface timing registers.
//		FMC_BANK5_6->SDTR_1.b.trcd = 0x01;			// Row to column delay - 2 cycles.
//		FMC_BANK5_6->SDTR_1.b.trp = 0x01;				// Row precharge delay - 2 cycles.
//		FMC_BANK5_6->SDTR_1.b.twr = 0x02; 			// Recovery delay - 3 cycles.
//		FMC_BANK5_6->SDTR_1.b.trc = 0x07;				// Row cycle delay - 8 cycles.
//		FMC_BANK5_6->SDTR_1.b.tras = 0x04;			// Self-refresh time - 5 cycles.
//		FMC_BANK5_6->SDTR_1.b.txsr = 0x05;			// Exit self-refresh delay - 6 cycles.
//		FMC_BANK5_6->SDTR_1.b.tmrd = 0x01;			//Load mode Register to active - 2 cycles.
	FMC_Bank5_6->SDTR[0] = 0x01127451;			// Confirmed based on previous bitwise implementation.

	// SDRAM initialization sequence
	// Set mode bits to 001 to deliver clock on bank 1.
	FMC_Bank5_6->SDCMR  = SDRAM_BANK_1 | 0x00000001; 			// Clock enable command

	// Get the busy flag
	tmpreg = (FMC_Bank5_6->SDSR & FMC_SDSR_BUSY_Pos); //

	// Check to see whether it is cleared
	while(((FMC_Bank5_6->SDSR & FMC_SDSR_BUSY_Pos) != 0) && (timeout-- > 0));

	// Software delay after the busy flag has been cleared.
	for (index = 0; index<1000; index++);

	// Issue the Precharge All Command
	FMC_Bank5_6->SDCMR = SDRAM_BANK_1 | 0x00000002;

	// Check to see whether the busy flag has been cleared.
	timeout = 0xFFFF;
	while(((FMC_Bank5_6->SDSR & FMC_SDSR_BUSY_Pos) != 0) && (timeout-- > 0));

	// Auto refresh command (Bank 1, 2 Auto-Refresh Cycles)
	FMC_Bank5_6->SDCMR = SDRAM_BANK_1 | 0x00000020 | 0x00000003;

	// Check to see whether the busy flag has been cleared.
	timeout = 0xFFFF;
	while(((FMC_Bank5_6->SDSR & FMC_SDSR_BUSY_Pos) != 0) && (timeout-- > 0));

	// MRD register program
	// Set to zero burst length, Sequential access, /CAS of 2, and single access location.
	FMC_Bank5_6->SDCMR = SDRAM_BANK_1 | (0x00000220 << 13) | 0x00000004;

	//FMC_BANK5_6->SDCMR.b.mrd = 0x220;
	//FMC_BANK5_6->SDCMR.d32|= SDRAM_BANK_1 | 0x00000004;

	// Check to see whether the busy flag has been cleared.
	timeout = 0xFFFF;
	while(((FMC_Bank5_6->SDSR & FMC_SDSR_BUSY_Pos) != 0) && (timeout-- > 0));

	// Set refresh count
	tmpreg = FMC_Bank5_6->SDRTR;
	FMC_Bank5_6->SDRTR = (tmpreg | (0x0000027D<<1));

	// Disable write protection
	tmpreg = FMC_Bank5_6->SDCR[0];
	FMC_Bank5_6->SDCR[1] = (tmpreg & 0xFFFFFDFF);

	// Release the Flash from Reset - GPIOG6.
	GPIOG->BSRR = (1 << GPIO_BSRR_BS6_Pos);

	// Configure the Flash Memory Access - Specific configuration for S29GL256S
	FMC_Bank1->BTCR[2] = 0x000010D9;			// Set the flash configuration options
	FMC_Bank1->BTCR[3] = 0x1201131;				// SEt the timing constrains for read access.

	//	/*##-1- Configure the NOR device ################################*/
	//	hNOR.Instance = FMC_NORSRAM_DEVICE;
	//	hNOR.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
	//
	//	/* NOR device configuration */
	//	// NOR_Timing.AddressSetupTime = 1;
	//	NOR_Timing.AddressSetupTime = 17;				// 100ns access time.
	//	NOR_Timing.AddressHoldTime = 2;					// Address
	//	NOR_Timing.DataSetupTime = 17;					// 14 x 6 HCLK = Data Setup Time. 84ns?
	//	NOR_Timing.BusTurnAroundDuration = 0;
	//	NOR_Timing.CLKDivision = 2;
	//	NOR_Timing.DataLatency = 4;
	//	NOR_Timing.AccessMode = FMC_ACCESS_MODE_B; // GM - FMC_ACCESS_MODE_B;
	//
	//	hNOR.Init.NSBank = FMC_NORSRAM_BANK2;
	//	hNOR.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
	//	hNOR.Init.MemoryType = FMC_MEMORY_TYPE_NOR;
	//	hNOR.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16;
	//	hNOR.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
	//
	//	hNOR.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
	//	hNOR.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
	//	hNOR.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
	//	hNOR.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
	// 	hNOR.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;					// Changed by GM - Was set to disabled.
	// 	hNOR.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
	// 	hNOR.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
	// 	hNOR.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
	// 	hNOR.Init.WriteFifo = 0; // GM - FMC_WRITE_FIFO_DISABLE;
	// 	hNOR.Init.PageSize = FMC_PAGE_SIZE_NONE;
}


#if defined (DATA_IN_ExtSRAM) && defined (DATA_IN_ExtSDRAM)
#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F469xx) || defined(STM32F479xx)
/**
  * @brief  Setup the external memory controller.
  *         Called in startup_stm32f4xx.s before jump to main.
  *         This function configures the external memories (SRAM/SDRAM)
  *         This SRAM/SDRAM will be used as program data memory (including heap and stack).
  * @param  None
  * @retval None
  */
void SystemInit_ExtMemCtl(void)
{
  __IO uint32_t tmp = 0x00;

  register uint32_t tmpreg = 0, timeout = 0xFFFF;
  register __IO uint32_t index;

  /* Enable GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH and GPIOI interface clock */
  RCC->AHB1ENR |= 0x000001F8;

  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);
  
  /* Connect PDx pins to FMC Alternate function */
  GPIOD->AFR[0]  = 0x00CCC0CC;
  GPIOD->AFR[1]  = 0xCCCCCCCC;
  /* Configure PDx pins in Alternate function mode */  
  GPIOD->MODER   = 0xAAAA0A8A;
  /* Configure PDx pins speed to 100 MHz */  
  GPIOD->OSPEEDR = 0xFFFF0FCF;
  /* Configure PDx pins Output type to push-pull */  
  GPIOD->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PDx pins */ 
  GPIOD->PUPDR   = 0x00000000;

  /* Connect PEx pins to FMC Alternate function */
  GPIOE->AFR[0]  = 0xC00CC0CC;
  GPIOE->AFR[1]  = 0xCCCCCCCC;
  /* Configure PEx pins in Alternate function mode */ 
  GPIOE->MODER   = 0xAAAA828A;
  /* Configure PEx pins speed to 100 MHz */ 
  GPIOE->OSPEEDR = 0xFFFFC3CF;
  /* Configure PEx pins Output type to push-pull */  
  GPIOE->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PEx pins */ 
  GPIOE->PUPDR   = 0x00000000;
  
  /* Connect PFx pins to FMC Alternate function */
  GPIOF->AFR[0]  = 0xCCCCCCCC;
  GPIOF->AFR[1]  = 0xCCCCCCCC;
  /* Configure PFx pins in Alternate function mode */   
  GPIOF->MODER   = 0xAA800AAA;
  /* Configure PFx pins speed to 50 MHz */ 
  GPIOF->OSPEEDR = 0xAA800AAA;
  /* Configure PFx pins Output type to push-pull */  
  GPIOF->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PFx pins */ 
  GPIOF->PUPDR   = 0x00000000;

  /* Connect PGx pins to FMC Alternate function */
  GPIOG->AFR[0]  = 0xCCCCCCCC;
  GPIOG->AFR[1]  = 0xCCCCCCCC;
  /* Configure PGx pins in Alternate function mode */ 
  GPIOG->MODER   = 0xAAAAAAAA;
  /* Configure PGx pins speed to 50 MHz */ 
  GPIOG->OSPEEDR = 0xAAAAAAAA;
  /* Configure PGx pins Output type to push-pull */  
  GPIOG->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PGx pins */ 
  GPIOG->PUPDR   = 0x00000000;
  
  /* Connect PHx pins to FMC Alternate function */
  GPIOH->AFR[0]  = 0x00C0CC00;
  GPIOH->AFR[1]  = 0xCCCCCCCC;
  /* Configure PHx pins in Alternate function mode */ 
  GPIOH->MODER   = 0xAAAA08A0;
  /* Configure PHx pins speed to 50 MHz */ 
  GPIOH->OSPEEDR = 0xAAAA08A0;
  /* Configure PHx pins Output type to push-pull */  
  GPIOH->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PHx pins */ 
  GPIOH->PUPDR   = 0x00000000;
  
  /* Connect PIx pins to FMC Alternate function */
  GPIOI->AFR[0]  = 0xCCCCCCCC;
  GPIOI->AFR[1]  = 0x00000CC0;
  /* Configure PIx pins in Alternate function mode */ 
  GPIOI->MODER   = 0x0028AAAA;
  /* Configure PIx pins speed to 50 MHz */ 
  GPIOI->OSPEEDR = 0x0028AAAA;
  /* Configure PIx pins Output type to push-pull */  
  GPIOI->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PIx pins */ 
  GPIOI->PUPDR   = 0x00000000;
  
/*-- FMC Configuration -------------------------------------------------------*/
  /* Enable the FMC interface clock */
  RCC->AHB3ENR |= 0x00000001;
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);

  FMC_Bank5_6->SDCR[0] = 0x000019E4;
  FMC_Bank5_6->SDTR[0] = 0x01115351;      
  
  /* SDRAM initialization sequence */
  /* Clock enable command */
  FMC_Bank5_6->SDCMR = 0x00000011; 
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }

  /* Delay */
  for (index = 0; index<1000; index++);
  
  /* PALL command */
  FMC_Bank5_6->SDCMR = 0x00000012;           
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }
  
  /* Auto refresh command */
  FMC_Bank5_6->SDCMR = 0x00000073;
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }
 
  /* MRD register program */
  FMC_Bank5_6->SDCMR = 0x00046014;
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  } 
  
  /* Set refresh count */
  tmpreg = FMC_Bank5_6->SDRTR;
  FMC_Bank5_6->SDRTR = (tmpreg | (0x0000027C<<1));
  
  /* Disable write protection */
  tmpreg = FMC_Bank5_6->SDCR[0]; 
  FMC_Bank5_6->SDCR[0] = (tmpreg & 0xFFFFFDFF);

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)
  /* Configure and enable Bank1_SRAM2 */
  FMC_Bank1->BTCR[2]  = 0x00001011;
  FMC_Bank1->BTCR[3]  = 0x00000201;
  FMC_Bank1E->BWTR[2] = 0x0fffffff;
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx */ 
#if defined(STM32F469xx) || defined(STM32F479xx)
  /* Configure and enable Bank1_SRAM2 */
  FMC_Bank1->BTCR[2]  = 0x00001091;
  FMC_Bank1->BTCR[3]  = 0x00110212;
  FMC_Bank1E->BWTR[2] = 0x0fffffff;
#endif /* STM32F469xx || STM32F479xx */

  (void)(tmp); 
}
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F469xx || STM32F479xx */
#elif defined (DATA_IN_ExtSRAM) || defined (DATA_IN_ExtSDRAM)
/**
  * @brief  Setup the external memory controller.
  *         Called in startup_stm32f4xx.s before jump to main.
  *         This function configures the external memories (SRAM/SDRAM)
  *         This SRAM/SDRAM will be used as program data memory (including heap and stack).
  * @param  None
  * @retval None
  */
void SystemInit_ExtMemCtl(void)
{
  __IO uint32_t tmp = 0x00;
#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx)
#if defined (DATA_IN_ExtSDRAM)
  register uint32_t tmpreg = 0, timeout = 0xFFFF;
  register __IO uint32_t index;

#if defined(STM32F446xx)
  /* Enable GPIOA, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG interface
      clock */
  RCC->AHB1ENR |= 0x0000007D;
#else
  /* Enable GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH and GPIOI interface 
      clock */
  RCC->AHB1ENR |= 0x000001F8;
#endif /* STM32F446xx */  
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);
  
#if defined(STM32F446xx)
  /* Connect PAx pins to FMC Alternate function */
  GPIOA->AFR[0]  |= 0xC0000000;
  GPIOA->AFR[1]  |= 0x00000000;
  /* Configure PDx pins in Alternate function mode */
  GPIOA->MODER   |= 0x00008000;
  /* Configure PDx pins speed to 50 MHz */
  GPIOA->OSPEEDR |= 0x00008000;
  /* Configure PDx pins Output type to push-pull */
  GPIOA->OTYPER  |= 0x00000000;
  /* No pull-up, pull-down for PDx pins */
  GPIOA->PUPDR   |= 0x00000000;

  /* Connect PCx pins to FMC Alternate function */
  GPIOC->AFR[0]  |= 0x00CC0000;
  GPIOC->AFR[1]  |= 0x00000000;
  /* Configure PDx pins in Alternate function mode */
  GPIOC->MODER   |= 0x00000A00;
  /* Configure PDx pins speed to 50 MHz */
  GPIOC->OSPEEDR |= 0x00000A00;
  /* Configure PDx pins Output type to push-pull */
  GPIOC->OTYPER  |= 0x00000000;
  /* No pull-up, pull-down for PDx pins */
  GPIOC->PUPDR   |= 0x00000000;
#endif /* STM32F446xx */

  /* Connect PDx pins to FMC Alternate function */
  GPIOD->AFR[0]  = 0x000000CC;
  GPIOD->AFR[1]  = 0xCC000CCC;
  /* Configure PDx pins in Alternate function mode */  
  GPIOD->MODER   = 0xA02A000A;
  /* Configure PDx pins speed to 50 MHz */  
  GPIOD->OSPEEDR = 0xA02A000A;
  /* Configure PDx pins Output type to push-pull */  
  GPIOD->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PDx pins */ 
  GPIOD->PUPDR   = 0x00000000;

  /* Connect PEx pins to FMC Alternate function */
  GPIOE->AFR[0]  = 0xC00000CC;
  GPIOE->AFR[1]  = 0xCCCCCCCC;
  /* Configure PEx pins in Alternate function mode */ 
  GPIOE->MODER   = 0xAAAA800A;
  /* Configure PEx pins speed to 50 MHz */ 
  GPIOE->OSPEEDR = 0xAAAA800A;
  /* Configure PEx pins Output type to push-pull */  
  GPIOE->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PEx pins */ 
  GPIOE->PUPDR   = 0x00000000;

  /* Connect PFx pins to FMC Alternate function */
  GPIOF->AFR[0]  = 0xCCCCCCCC;
  GPIOF->AFR[1]  = 0xCCCCCCCC;
  /* Configure PFx pins in Alternate function mode */   
  GPIOF->MODER   = 0xAA800AAA;
  /* Configure PFx pins speed to 50 MHz */ 
  GPIOF->OSPEEDR = 0xAA800AAA;
  /* Configure PFx pins Output type to push-pull */  
  GPIOF->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PFx pins */ 
  GPIOF->PUPDR   = 0x00000000;

  /* Connect PGx pins to FMC Alternate function */
  GPIOG->AFR[0]  = 0xCCCCCCCC;
  GPIOG->AFR[1]  = 0xCCCCCCCC;
  /* Configure PGx pins in Alternate function mode */ 
  GPIOG->MODER   = 0xAAAAAAAA;
  /* Configure PGx pins speed to 50 MHz */ 
  GPIOG->OSPEEDR = 0xAAAAAAAA;
  /* Configure PGx pins Output type to push-pull */  
  GPIOG->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PGx pins */ 
  GPIOG->PUPDR   = 0x00000000;

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F469xx) || defined(STM32F479xx)  
  /* Connect PHx pins to FMC Alternate function */
  GPIOH->AFR[0]  = 0x00C0CC00;
  GPIOH->AFR[1]  = 0xCCCCCCCC;
  /* Configure PHx pins in Alternate function mode */ 
  GPIOH->MODER   = 0xAAAA08A0;
  /* Configure PHx pins speed to 50 MHz */ 
  GPIOH->OSPEEDR = 0xAAAA08A0;
  /* Configure PHx pins Output type to push-pull */  
  GPIOH->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PHx pins */ 
  GPIOH->PUPDR   = 0x00000000;
  
  /* Connect PIx pins to FMC Alternate function */
  GPIOI->AFR[0]  = 0xCCCCCCCC;
  GPIOI->AFR[1]  = 0x00000CC0;
  /* Configure PIx pins in Alternate function mode */ 
  GPIOI->MODER   = 0x0028AAAA;
  /* Configure PIx pins speed to 50 MHz */ 
  GPIOI->OSPEEDR = 0x0028AAAA;
  /* Configure PIx pins Output type to push-pull */  
  GPIOI->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PIx pins */ 
  GPIOI->PUPDR   = 0x00000000;
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F469xx || STM32F479xx */
  
/*-- FMC Configuration -------------------------------------------------------*/
  /* Enable the FMC interface clock */
  RCC->AHB3ENR |= 0x00000001;
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);

  /* Configure and enable SDRAM bank1 */
#if defined(STM32F446xx)
  FMC_Bank5_6->SDCR[0] = 0x00001954;
#else  
  FMC_Bank5_6->SDCR[0] = 0x000019E4;
#endif /* STM32F446xx */
  FMC_Bank5_6->SDTR[0] = 0x01115351;      
  
  /* SDRAM initialization sequence */
  /* Clock enable command */
  FMC_Bank5_6->SDCMR = 0x00000011; 
  tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }

  /* Delay */
  for (index = 0; index<1000; index++);
  
  /* PALL command */
  FMC_Bank5_6->SDCMR = 0x00000012;           
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }
  
  /* Auto refresh command */
#if defined(STM32F446xx)
  FMC_Bank5_6->SDCMR = 0x000000F3;
#else  
  FMC_Bank5_6->SDCMR = 0x00000073;
#endif /* STM32F446xx */
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  }
 
  /* MRD register program */
#if defined(STM32F446xx)
  FMC_Bank5_6->SDCMR = 0x00044014;
#else  
  FMC_Bank5_6->SDCMR = 0x00046014;
#endif /* STM32F446xx */
  timeout = 0xFFFF;
  while((tmpreg != 0) && (timeout-- > 0))
  {
    tmpreg = FMC_Bank5_6->SDSR & 0x00000020; 
  } 
  
  /* Set refresh count */
  tmpreg = FMC_Bank5_6->SDRTR;
#if defined(STM32F446xx)
  FMC_Bank5_6->SDRTR = (tmpreg | (0x0000050C<<1));
#else    
  FMC_Bank5_6->SDRTR = (tmpreg | (0x0000027C<<1));
#endif /* STM32F446xx */
  
  /* Disable write protection */
  tmpreg = FMC_Bank5_6->SDCR[0]; 
  FMC_Bank5_6->SDCR[0] = (tmpreg & 0xFFFFFDFF);
#endif /* DATA_IN_ExtSDRAM */
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx || STM32F446xx || STM32F469xx || STM32F479xx */

#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx)\
 || defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)\
 || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx)

#if defined(DATA_IN_ExtSRAM)
/*-- GPIOs Configuration -----------------------------------------------------*/
   /* Enable GPIOD, GPIOE, GPIOF and GPIOG interface clock */
  RCC->AHB1ENR   |= 0x00000078;
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);
  
  /* Connect PDx pins to FMC Alternate function */
  GPIOD->AFR[0]  = 0x00CCC0CC;
  GPIOD->AFR[1]  = 0xCCCCCCCC;
  /* Configure PDx pins in Alternate function mode */  
  GPIOD->MODER   = 0xAAAA0A8A;
  /* Configure PDx pins speed to 100 MHz */  
  GPIOD->OSPEEDR = 0xFFFF0FCF;
  /* Configure PDx pins Output type to push-pull */  
  GPIOD->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PDx pins */ 
  GPIOD->PUPDR   = 0x00000000;

  /* Connect PEx pins to FMC Alternate function */
  GPIOE->AFR[0]  = 0xC00CC0CC;
  GPIOE->AFR[1]  = 0xCCCCCCCC;
  /* Configure PEx pins in Alternate function mode */ 
  GPIOE->MODER   = 0xAAAA828A;
  /* Configure PEx pins speed to 100 MHz */ 
  GPIOE->OSPEEDR = 0xFFFFC3CF;
  /* Configure PEx pins Output type to push-pull */  
  GPIOE->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PEx pins */ 
  GPIOE->PUPDR   = 0x00000000;

  /* Connect PFx pins to FMC Alternate function */
  GPIOF->AFR[0]  = 0x00CCCCCC;
  GPIOF->AFR[1]  = 0xCCCC0000;
  /* Configure PFx pins in Alternate function mode */   
  GPIOF->MODER   = 0xAA000AAA;
  /* Configure PFx pins speed to 100 MHz */ 
  GPIOF->OSPEEDR = 0xFF000FFF;
  /* Configure PFx pins Output type to push-pull */  
  GPIOF->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PFx pins */ 
  GPIOF->PUPDR   = 0x00000000;

  /* Connect PGx pins to FMC Alternate function */
  GPIOG->AFR[0]  = 0x00CCCCCC;
  GPIOG->AFR[1]  = 0x000000C0;
  /* Configure PGx pins in Alternate function mode */ 
  GPIOG->MODER   = 0x00085AAA;
  /* Configure PGx pins speed to 100 MHz */ 
  GPIOG->OSPEEDR = 0x000CAFFF;
  /* Configure PGx pins Output type to push-pull */  
  GPIOG->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PGx pins */ 
  GPIOG->PUPDR   = 0x00000000;
  
/*-- FMC/FSMC Configuration --------------------------------------------------*/
  /* Enable the FMC/FSMC interface clock */
  RCC->AHB3ENR         |= 0x00000001;

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);
  /* Configure and enable Bank1_SRAM2 */
  FMC_Bank1->BTCR[2]  = 0x00001011;
  FMC_Bank1->BTCR[3]  = 0x00000201;
  FMC_Bank1E->BWTR[2] = 0x0fffffff;
#endif /* STM32F427xx || STM32F437xx || STM32F429xx || STM32F439xx */ 
#if defined(STM32F469xx) || defined(STM32F479xx)
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FMCEN);
  /* Configure and enable Bank1_SRAM2 */
  FMC_Bank1->BTCR[2]  = 0x00001091;
  FMC_Bank1->BTCR[3]  = 0x00110212;
  FMC_Bank1E->BWTR[2] = 0x0fffffff;
#endif /* STM32F469xx || STM32F479xx */
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx)|| defined(STM32F417xx)\
   || defined(STM32F412Zx) || defined(STM32F412Vx)
  /* Delay after an RCC peripheral clock enabling */
  tmp = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FSMCEN);
  /* Configure and enable Bank1_SRAM2 */
  FSMC_Bank1->BTCR[2]  = 0x00001011;
  FSMC_Bank1->BTCR[3]  = 0x00000201;
  FSMC_Bank1E->BWTR[2] = 0x0FFFFFFF;
#endif /* STM32F405xx || STM32F415xx || STM32F407xx || STM32F417xx || STM32F412Zx || STM32F412Vx */

#endif /* DATA_IN_ExtSRAM */
#endif /* STM32F405xx || STM32F415xx || STM32F407xx || STM32F417xx || STM32F427xx || STM32F437xx ||\
          STM32F429xx || STM32F439xx || STM32F469xx || STM32F479xx || STM32F412Zx || STM32F412Vx  */ 
  (void)(tmp); 
}
#endif /* DATA_IN_ExtSRAM && DATA_IN_ExtSDRAM */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
