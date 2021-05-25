#line 1 "src\\gpioControl.c"





 

#line 1 ".\\inc\\stm32f439xx.h"







































 



 



 
    










	

 

	

 






	

 

	

 

	


 
	typedef enum
	{
	 
	  NonMaskableInt_IRQn         = -14,     
	  MemoryManagement_IRQn       = -12,     
	  BusFault_IRQn               = -11,     
	  UsageFault_IRQn             = -10,     
	  SVCall_IRQn                 = -5,      
	  DebugMonitor_IRQn           = -4,      
	  PendSV_IRQn                 = -2,      
	  SysTick_IRQn                = -1,      
	 
	  WWDG_IRQn                   = 0,       
	  PVD_IRQn                    = 1,       
	  TAMP_STAMP_IRQn             = 2,       
	  RTC_WKUP_IRQn               = 3,       
	  FLASH_IRQn                  = 4,       
	  RCC_IRQn                    = 5,       
	  EXTI0_IRQn                  = 6,       
	  EXTI1_IRQn                  = 7,       
	  EXTI2_IRQn                  = 8,       
	  EXTI3_IRQn                  = 9,       
	  EXTI4_IRQn                  = 10,      
	  DMA1_Stream0_IRQn           = 11,      
	  DMA1_Stream1_IRQn           = 12,      
	  DMA1_Stream2_IRQn           = 13,      
	  DMA1_Stream3_IRQn           = 14,      
	  DMA1_Stream4_IRQn           = 15,      
	  DMA1_Stream5_IRQn           = 16,      
	  DMA1_Stream6_IRQn           = 17,      
	  ADC_IRQn                    = 18,      
	  CAN1_TX_IRQn                = 19,      
	  CAN1_RX0_IRQn               = 20,      
	  CAN1_RX1_IRQn               = 21,      
	  CAN1_SCE_IRQn               = 22,      
	  EXTI9_5_IRQn                = 23,      
	  TIM1_BRK_TIM9_IRQn          = 24,      
	  TIM1_UP_TIM10_IRQn          = 25,      
	  TIM1_TRG_COM_TIM11_IRQn     = 26,      
	  TIM1_CC_IRQn                = 27,      
	  TIM2_IRQn                   = 28,      
	  TIM3_IRQn                   = 29,      
	  TIM4_IRQn                   = 30,      
	  I2C1_EV_IRQn                = 31,      
	  I2C1_ER_IRQn                = 32,      
	  I2C2_EV_IRQn                = 33,      
	  I2C2_ER_IRQn                = 34,      
	  SPI1_IRQn                   = 35,      
	  SPI2_IRQn                   = 36,      
	  USART1_IRQn                 = 37,      
	  USART2_IRQn                 = 38,      
	  USART3_IRQn                 = 39,      
	  EXTI15_10_IRQn              = 40,      
	  RTC_Alarm_IRQn              = 41,      
	  OTG_FS_WKUP_IRQn            = 42,      
	  TIM8_BRK_TIM12_IRQn         = 43,      
	  TIM8_UP_TIM13_IRQn          = 44,      
	  TIM8_TRG_COM_TIM14_IRQn     = 45,      
	  TIM8_CC_IRQn                = 46,      
	  DMA1_Stream7_IRQn           = 47,      
	  FMC_IRQn                    = 48,      
	  SDIO_IRQn                   = 49,      
	  TIM5_IRQn                   = 50,      
	  SPI3_IRQn                   = 51,      
	  UART4_IRQn                  = 52,      
	  UART5_IRQn                  = 53,      
	  TIM6_DAC_IRQn               = 54,      
	  TIM7_IRQn                   = 55,      
	  DMA2_Stream0_IRQn           = 56,      
	  DMA2_Stream1_IRQn           = 57,      
	  DMA2_Stream2_IRQn           = 58,      
	  DMA2_Stream3_IRQn           = 59,      
	  DMA2_Stream4_IRQn           = 60,      
	  ETH_IRQn                    = 61,      
	  ETH_WKUP_IRQn               = 62,      
	  CAN2_TX_IRQn                = 63,      
	  CAN2_RX0_IRQn               = 64,      
	  CAN2_RX1_IRQn               = 65,      
	  CAN2_SCE_IRQn               = 66,      
	  OTG_FS_IRQn                 = 67,      
	  DMA2_Stream5_IRQn           = 68,      
	  DMA2_Stream6_IRQn           = 69,      
	  DMA2_Stream7_IRQn           = 70,      
	  USART6_IRQn                 = 71,      
	  I2C3_EV_IRQn                = 72,      
	  I2C3_ER_IRQn                = 73,      
	  OTG_HS_EP1_OUT_IRQn         = 74,      
	  OTG_HS_EP1_IN_IRQn          = 75,      
	  OTG_HS_WKUP_IRQn            = 76,      
	  OTG_HS_IRQn                 = 77,      
	  DCMI_IRQn                   = 78,      
	  CRYP_IRQn                   = 79,      
	  HASH_RNG_IRQn               = 80,      
	  FPU_IRQn                    = 81,      
	  UART7_IRQn                  = 82,      
	  UART8_IRQn                  = 83,      
	  SPI4_IRQn                   = 84,      
	  SPI5_IRQn                   = 85,      
	  SPI6_IRQn                   = 86,      
	  SAI1_IRQn                   = 87,      
	  LTDC_IRQn                   = 88,      
	  LTDC_ER_IRQn                = 89,      
	  DMA2D_IRQn                  = 90       
	} IRQn_Type;

	

 

#line 1 ".\\inc\\CMSIS\\core_cm4.h"
 




 

























 
#line 1 ".\\inc\\config.h"






 

 













#line 29 ".\\inc\\config.h"















	








#line 34 ".\\inc\\CMSIS\\core_cm4.h"










#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 45 ".\\inc\\CMSIS\\core_cm4.h"


















 




 



 

 













#line 121 ".\\inc\\CMSIS\\core_cm4.h"



 
#line 136 ".\\inc\\CMSIS\\core_cm4.h"

#line 210 ".\\inc\\CMSIS\\core_cm4.h"

#line 1 ".\\inc\\CMSIS\\core_cmInstr.h"
 




 

























 












 



 

 
#line 1 ".\\inc\\CMSIS\\cmsis_armcc.h"
 




 

























 










 



 

 
 





 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}






 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}






 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}






 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}






 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}






 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}






 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}






 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}






 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}






 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}






 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}








 







 







 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}






 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xFFU);
}







 
static __inline void __set_BASEPRI_MAX(uint32_t basePri)
{
  register uint32_t __regBasePriMax      __asm("basepri_max");
  __regBasePriMax = (basePri & 0xFFU);
}






 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}






 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & (uint32_t)1);
}










 
static __inline uint32_t __get_FPSCR(void)
{

  register uint32_t __regfpscr         __asm("fpscr");
  return(__regfpscr);



}






 
static __inline void __set_FPSCR(uint32_t fpscr)
{

  register uint32_t __regfpscr         __asm("fpscr");
  __regfpscr = (fpscr);

}





 


 



 




 






 







 






 








 










 










 











 








 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}







 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}









 









 








 
#line 455 ".\\inc\\CMSIS\\cmsis_armcc.h"







 










 












 












 














 














 














 










 









 









 









 

__attribute__((section(".rrx_text"))) static __inline __asm uint32_t __RRX(uint32_t value)
{
  rrx r0, r0
  bx lr
}








 








 








 








 








 








 




   


 



 



#line 720 ".\\inc\\CMSIS\\cmsis_armcc.h"











 


#line 54 ".\\inc\\CMSIS\\core_cmInstr.h"

 
#line 84 ".\\inc\\CMSIS\\core_cmInstr.h"

   

#line 212 ".\\inc\\CMSIS\\core_cm4.h"
#line 1 ".\\inc\\CMSIS\\core_cmFunc.h"
 




 

























 












 



 

 
#line 54 ".\\inc\\CMSIS\\core_cmFunc.h"

 
#line 84 ".\\inc\\CMSIS\\core_cmFunc.h"

 

#line 213 ".\\inc\\CMSIS\\core_cm4.h"
#line 1 ".\\inc\\CMSIS\\core_cmSimd.h"
 




 

























 
















 



 

 
#line 58 ".\\inc\\CMSIS\\core_cmSimd.h"

 
#line 88 ".\\inc\\CMSIS\\core_cmSimd.h"

 






#line 214 ".\\inc\\CMSIS\\core_cm4.h"
















 
#line 257 ".\\inc\\CMSIS\\core_cm4.h"

 






 
#line 273 ".\\inc\\CMSIS\\core_cm4.h"

 




 













 



 






 



 
typedef union
{
  struct
  {
    uint32_t _reserved0:16;               
    uint32_t GE:4;                        
    uint32_t _reserved1:7;                
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;

 





















 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;

 






 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:7;                
    uint32_t GE:4;                        
    uint32_t _reserved1:4;                
    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;

 






























 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 









 







 



 
typedef struct
{
  volatile uint32_t ISER[8U];                
        uint32_t RESERVED0[24U];
  volatile uint32_t ICER[8U];                
        uint32_t RSERVED1[24U];
  volatile uint32_t ISPR[8U];                
        uint32_t RESERVED2[24U];
  volatile uint32_t ICPR[8U];                
        uint32_t RESERVED3[24U];
  volatile uint32_t IABR[8U];                
        uint32_t RESERVED4[56U];
  volatile uint8_t  IP[240U];                
        uint32_t RESERVED5[644U];
  volatile  uint32_t STIR;                    
}  NVIC_Type;

 



 







 



 
typedef struct
{
  volatile const  uint32_t CPUID;                   
  volatile uint32_t ICSR;                    
  volatile uint32_t VTOR;                    
  volatile uint32_t AIRCR;                   
  volatile uint32_t SCR;                     
  volatile uint32_t CCR;                     
  volatile uint8_t  SHP[12U];                
  volatile uint32_t SHCSR;                   
  volatile uint32_t CFSR;                    
  volatile uint32_t HFSR;                    
  volatile uint32_t DFSR;                    
  volatile uint32_t MMFAR;                   
  volatile uint32_t BFAR;                    
  volatile uint32_t AFSR;                    
  volatile const  uint32_t PFR[2U];                 
  volatile const  uint32_t DFR;                     
  volatile const  uint32_t ADR;                     
  volatile const  uint32_t MMFR[4U];                
  volatile const  uint32_t ISAR[5U];                
        uint32_t RESERVED0[5U];
  volatile uint32_t CPACR;                   
} SCB_Type;

 















 






























 



 





















 









 


















 










































 









 









 















 







 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile const  uint32_t ICTR;                    
  volatile uint32_t ACTLR;                   
} SCnSCB_Type;

 



 















 







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t LOAD;                    
  volatile uint32_t VAL;                     
  volatile const  uint32_t CALIB;                   
} SysTick_Type;

 












 



 



 









 







 



 
typedef struct
{
  volatile  union
  {
    volatile  uint8_t    u8;                  
    volatile  uint16_t   u16;                 
    volatile  uint32_t   u32;                 
  }  PORT [32U];                          
        uint32_t RESERVED0[864U];
  volatile uint32_t TER;                     
        uint32_t RESERVED1[15U];
  volatile uint32_t TPR;                     
        uint32_t RESERVED2[15U];
  volatile uint32_t TCR;                     
        uint32_t RESERVED3[29U];
  volatile  uint32_t IWR;                     
  volatile const  uint32_t IRR;                     
  volatile uint32_t IMCR;                    
        uint32_t RESERVED4[43U];
  volatile  uint32_t LAR;                     
  volatile const  uint32_t LSR;                     
        uint32_t RESERVED5[6U];
  volatile const  uint32_t PID4;                    
  volatile const  uint32_t PID5;                    
  volatile const  uint32_t PID6;                    
  volatile const  uint32_t PID7;                    
  volatile const  uint32_t PID0;                    
  volatile const  uint32_t PID1;                    
  volatile const  uint32_t PID2;                    
  volatile const  uint32_t PID3;                    
  volatile const  uint32_t CID0;                    
  volatile const  uint32_t CID1;                    
  volatile const  uint32_t CID2;                    
  volatile const  uint32_t CID3;                    
} ITM_Type;

 



 



























 



 



 



 









   







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t CYCCNT;                  
  volatile uint32_t CPICNT;                  
  volatile uint32_t EXCCNT;                  
  volatile uint32_t SLEEPCNT;                
  volatile uint32_t LSUCNT;                  
  volatile uint32_t FOLDCNT;                 
  volatile const  uint32_t PCSR;                    
  volatile uint32_t COMP0;                   
  volatile uint32_t MASK0;                   
  volatile uint32_t FUNCTION0;               
        uint32_t RESERVED0[1U];
  volatile uint32_t COMP1;                   
  volatile uint32_t MASK1;                   
  volatile uint32_t FUNCTION1;               
        uint32_t RESERVED1[1U];
  volatile uint32_t COMP2;                   
  volatile uint32_t MASK2;                   
  volatile uint32_t FUNCTION2;               
        uint32_t RESERVED2[1U];
  volatile uint32_t COMP3;                   
  volatile uint32_t MASK3;                   
  volatile uint32_t FUNCTION3;               
} DWT_Type;

 






















































 



 



 



 



 



 



 



























   







 



 
typedef struct
{
  volatile uint32_t SSPSR;                   
  volatile uint32_t CSPSR;                   
        uint32_t RESERVED0[2U];
  volatile uint32_t ACPR;                    
        uint32_t RESERVED1[55U];
  volatile uint32_t SPPR;                    
        uint32_t RESERVED2[131U];
  volatile const  uint32_t FFSR;                    
  volatile uint32_t FFCR;                    
  volatile const  uint32_t FSCR;                    
        uint32_t RESERVED3[759U];
  volatile const  uint32_t TRIGGER;                 
  volatile const  uint32_t FIFO0;                   
  volatile const  uint32_t ITATBCTR2;               
        uint32_t RESERVED4[1U];
  volatile const  uint32_t ITATBCTR0;               
  volatile const  uint32_t FIFO1;                   
  volatile uint32_t ITCTRL;                  
        uint32_t RESERVED5[39U];
  volatile uint32_t CLAIMSET;                
  volatile uint32_t CLAIMCLR;                
        uint32_t RESERVED7[8U];
  volatile const  uint32_t DEVID;                   
  volatile const  uint32_t DEVTYPE;                 
} TPI_Type;

 



 



 












 






 



 





















 



 





















 



 



 


















 






   








 



 
typedef struct
{
  volatile const  uint32_t TYPE;                    
  volatile uint32_t CTRL;                    
  volatile uint32_t RNR;                     
  volatile uint32_t RBAR;                    
  volatile uint32_t RASR;                    
  volatile uint32_t RBAR_A1;                 
  volatile uint32_t RASR_A1;                 
  volatile uint32_t RBAR_A2;                 
  volatile uint32_t RASR_A2;                 
  volatile uint32_t RBAR_A3;                 
  volatile uint32_t RASR_A3;                 
} MPU_Type;

 









 









 



 









 






























 









 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile uint32_t FPCCR;                   
  volatile uint32_t FPCAR;                   
  volatile uint32_t FPDSCR;                  
  volatile const  uint32_t MVFR0;                   
  volatile const  uint32_t MVFR1;                   
} FPU_Type;

 



























 



 












 
























 












 








 



 
typedef struct
{
  volatile uint32_t DHCSR;                   
  volatile  uint32_t DCRSR;                   
  volatile uint32_t DCRDR;                   
  volatile uint32_t DEMCR;                   
} CoreDebug_Type;

 




































 






 







































 







 






 







 


 







 

 
#line 1542 ".\\inc\\CMSIS\\core_cm4.h"

#line 1551 ".\\inc\\CMSIS\\core_cm4.h"











 










 


 



 





 









 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);              

  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;                                                    
  reg_value &= ~((uint32_t)((0xFFFFUL << 16U) | (7UL << 8U)));  
  reg_value  =  (reg_value                                   |
                ((uint32_t)0x5FAUL << 16U) |
                (PriorityGroupTmp << 8U)                      );               
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR =  reg_value;
}






 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) >> 8U));
}






 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}






 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}








 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) < 0)
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - 4U)) & (uint32_t)0xFFUL);
  }
  else
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)(int32_t)IRQn)]               = (uint8_t)((priority << (8U - 4U)) & (uint32_t)0xFFUL);
  }
}










 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) < 0)
  {
    return(((uint32_t)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] >> (8U - 4U)));
  }
  else
  {
    return(((uint32_t)((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)(int32_t)IRQn)]               >> (8U - 4U)));
  }
}












 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4U)) ? (uint32_t)(4U) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4U)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4U));

  return (
           ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
           ((SubPriority     & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL)))
         );
}












 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* const pPreemptPriority, uint32_t* const pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4U)) ? (uint32_t)(4U) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4U)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4U));

  *pPreemptPriority = (Priority >> SubPriorityBits) & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL);
  *pSubPriority     = (Priority                   ) & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL);
}





 
static __inline void NVIC_SystemReset(void)
{
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                          
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = (uint32_t)((0x5FAUL << 16U)    |
                           (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) |
                            (1UL << 2U)    );          
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                           

  for(;;)                                                            
  {
    __nop();
  }
}

 



 





 













 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);                                                    
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (uint32_t)(ticks - 1UL);                          
  NVIC_SetPriority (SysTick_IRQn, (1UL << 4U) - 1UL);  
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0UL;                                              
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2U) |
                   (1UL << 1U)   |
                   (1UL );                          
  return (0UL);                                                      
}



 



 





 

extern volatile int32_t ITM_RxBuffer;                     










 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if (((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL )) != 0UL) &&       
      ((((ITM_Type *) (0xE0000000UL) )->TER & 1UL               ) != 0UL)   )      
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0U].u32 == 0UL)
    {
      __nop();
    }
    ((ITM_Type *) (0xE0000000UL) )->PORT[0U].u8 = (uint8_t)ch;
  }
  return (ch);
}







 
static __inline int32_t ITM_ReceiveChar (void)
{
  int32_t ch = -1;                            

  if (ITM_RxBuffer != 0x5AA55AA5U)
  {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5U;        
  }

  return (ch);
}







 
static __inline int32_t ITM_CheckChar (void)
{

  if (ITM_RxBuffer == 0x5AA55AA5U)
  {
    return (0);                               
  }
  else
  {
    return (1);                               
  }
}

 










#line 195 ".\\inc\\stm32f439xx.h"
#line 1 ".\\RTE\\Device\\STM32F439IITx\\system_stm32f4xx.h"

































  



 



   
  


 









 



 




 
  






 
extern uint32_t SystemCoreClock;           

extern const uint8_t  AHBPrescTable[16];     
extern const uint8_t  APBPrescTable[8];      



 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
 
#line 196 ".\\inc\\stm32f439xx.h"
#line 197 ".\\inc\\stm32f439xx.h"

	

 

	

 

	typedef struct
	{
	  volatile uint32_t SR;      
	  volatile uint32_t CR1;     
	  volatile uint32_t CR2;     
	  volatile uint32_t SMPR1;   
	  volatile uint32_t SMPR2;   
	  volatile uint32_t JOFR1;   
	  volatile uint32_t JOFR2;   
	  volatile uint32_t JOFR3;   
	  volatile uint32_t JOFR4;   
	  volatile uint32_t HTR;     
	  volatile uint32_t LTR;     
	  volatile uint32_t SQR1;    
	  volatile uint32_t SQR2;    
	  volatile uint32_t SQR3;    
	  volatile uint32_t JSQR;    
	  volatile uint32_t JDR1;    
	  volatile uint32_t JDR2;    
	  volatile uint32_t JDR3;    
	  volatile uint32_t JDR4;    
	  volatile uint32_t DR;      
	} ADC_TypeDef;

	typedef struct
	{
	  volatile uint32_t CSR;     
	  volatile uint32_t CCR;     
	  volatile uint32_t CDR;    
 
	} ADC_Common_TypeDef;


	

 

	typedef struct
	{
	  volatile uint32_t TIR;   
	  volatile uint32_t TDTR;  
	  volatile uint32_t TDLR;  
	  volatile uint32_t TDHR;  
	} CAN_TxMailBox_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t RIR;   
	  volatile uint32_t RDTR;  
	  volatile uint32_t RDLR;  
	  volatile uint32_t RDHR;  
	} CAN_FIFOMailBox_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t FR1;  
	  volatile uint32_t FR2;  
	} CAN_FilterRegister_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t              MCR;                  
	  volatile uint32_t              MSR;                  
	  volatile uint32_t              TSR;                  
	  volatile uint32_t              RF0R;                 
	  volatile uint32_t              RF1R;                 
	  volatile uint32_t              IER;                  
	  volatile uint32_t              ESR;                  
	  volatile uint32_t              BTR;                  
	  uint32_t                   RESERVED0[88];        
	  CAN_TxMailBox_TypeDef      sTxMailBox[3];        
	  CAN_FIFOMailBox_TypeDef    sFIFOMailBox[2];      
	  uint32_t                   RESERVED1[12];        
	  volatile uint32_t              FMR;                  
	  volatile uint32_t              FM1R;                 
	  uint32_t                   RESERVED2;            
	  volatile uint32_t              FS1R;                 
	  uint32_t                   RESERVED3;            
	  volatile uint32_t              FFA1R;                
	  uint32_t                   RESERVED4;            
	  volatile uint32_t              FA1R;                 
	  uint32_t                   RESERVED5[8];         
	  CAN_FilterRegister_TypeDef sFilterRegister[28];  
	} CAN_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t DR;          
	  volatile uint8_t  IDR;         
	  uint8_t       RESERVED0;   
	  uint16_t      RESERVED1;   
	  volatile uint32_t CR;          
	} CRC_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t CR;        
	  volatile uint32_t SWTRIGR;   
	  volatile uint32_t DHR12R1;   
	  volatile uint32_t DHR12L1;   
	  volatile uint32_t DHR8R1;    
	  volatile uint32_t DHR12R2;   
	  volatile uint32_t DHR12L2;   
	  volatile uint32_t DHR8R2;    
	  volatile uint32_t DHR12RD;   
	  volatile uint32_t DHR12LD;   
	  volatile uint32_t DHR8RD;    
	  volatile uint32_t DOR1;      
	  volatile uint32_t DOR2;      
	  volatile uint32_t SR;        
	} DAC_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t IDCODE;   
	  volatile uint32_t CR;       
	  volatile uint32_t APB1FZ;   
	  volatile uint32_t APB2FZ;   
	}DBGMCU_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t CR;        
	  volatile uint32_t SR;        
	  volatile uint32_t RISR;      
	  volatile uint32_t IER;       
	  volatile uint32_t MISR;      
	  volatile uint32_t ICR;       
	  volatile uint32_t ESCR;      
	  volatile uint32_t ESUR;      
	  volatile uint32_t CWSTRTR;   
	  volatile uint32_t CWSIZER;   
	  volatile uint32_t DR;        
	} DCMI_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t CR;      
	  volatile uint32_t NDTR;    
	  volatile uint32_t PAR;     
	  volatile uint32_t M0AR;    
	  volatile uint32_t M1AR;    
	  volatile uint32_t FCR;     
	} DMA_Stream_TypeDef;

	typedef struct
	{
	  volatile uint32_t LISR;    
	  volatile uint32_t HISR;    
	  volatile uint32_t LIFCR;   
	  volatile uint32_t HIFCR;   
	} DMA_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t CR;             
	  volatile uint32_t ISR;            
	  volatile uint32_t IFCR;           
	  volatile uint32_t FGMAR;          
	  volatile uint32_t FGOR;           
	  volatile uint32_t BGMAR;          
	  volatile uint32_t BGOR;           
	  volatile uint32_t FGPFCCR;        
	  volatile uint32_t FGCOLR;         
	  volatile uint32_t BGPFCCR;        
	  volatile uint32_t BGCOLR;         
	  volatile uint32_t FGCMAR;         
	  volatile uint32_t BGCMAR;         
	  volatile uint32_t OPFCCR;         
	  volatile uint32_t OCOLR;          
	  volatile uint32_t OMAR;           
	  volatile uint32_t OOR;            
	  volatile uint32_t NLR;            
	  volatile uint32_t LWR;            
	  volatile uint32_t AMTCR;          
	  uint32_t      RESERVED[236];  
	  volatile uint32_t FGCLUT[256];    
	  volatile uint32_t BGCLUT[256];    
	} DMA2D_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t MACCR;
	  volatile uint32_t MACFFR;
	  volatile uint32_t MACHTHR;
	  volatile uint32_t MACHTLR;
	  volatile uint32_t MACMIIAR;
	  volatile uint32_t MACMIIDR;
	  volatile uint32_t MACFCR;
	  volatile uint32_t MACVLANTR;              
	  uint32_t      RESERVED0[2];
	  volatile uint32_t MACRWUFFR;              
	  volatile uint32_t MACPMTCSR;
	  uint32_t      RESERVED1;
	  volatile uint32_t MACDBGR;
	  volatile uint32_t MACSR;                  
	  volatile uint32_t MACIMR;
	  volatile uint32_t MACA0HR;
	  volatile uint32_t MACA0LR;
	  volatile uint32_t MACA1HR;
	  volatile uint32_t MACA1LR;
	  volatile uint32_t MACA2HR;
	  volatile uint32_t MACA2LR;
	  volatile uint32_t MACA3HR;
	  volatile uint32_t MACA3LR;                
	  uint32_t      RESERVED2[40];
	  volatile uint32_t MMCCR;                  
	  volatile uint32_t MMCRIR;
	  volatile uint32_t MMCTIR;
	  volatile uint32_t MMCRIMR;
	  volatile uint32_t MMCTIMR;                
	  uint32_t      RESERVED3[14];
	  volatile uint32_t MMCTGFSCCR;             
	  volatile uint32_t MMCTGFMSCCR;
	  uint32_t      RESERVED4[5];
	  volatile uint32_t MMCTGFCR;
	  uint32_t      RESERVED5[10];
	  volatile uint32_t MMCRFCECR;
	  volatile uint32_t MMCRFAECR;
	  uint32_t      RESERVED6[10];
	  volatile uint32_t MMCRGUFCR;
	  uint32_t      RESERVED7[334];
	  volatile uint32_t PTPTSCR;
	  volatile uint32_t PTPSSIR;
	  volatile uint32_t PTPTSHR;
	  volatile uint32_t PTPTSLR;
	  volatile uint32_t PTPTSHUR;
	  volatile uint32_t PTPTSLUR;
	  volatile uint32_t PTPTSAR;
	  volatile uint32_t PTPTTHR;
	  volatile uint32_t PTPTTLR;
	  volatile uint32_t RESERVED8;
	  volatile uint32_t PTPTSSR;
	  uint32_t      RESERVED9[565];
	  volatile uint32_t DMABMR;
	  volatile uint32_t DMATPDR;
	  volatile uint32_t DMARPDR;
	  volatile uint32_t DMARDLAR;
	  volatile uint32_t DMATDLAR;
	  volatile uint32_t DMASR;
	  volatile uint32_t DMAOMR;
	  volatile uint32_t DMAIER;
	  volatile uint32_t DMAMFBOCR;
	  volatile uint32_t DMARSWTR;
	  uint32_t      RESERVED10[8];
	  volatile uint32_t DMACHTDR;
	  volatile uint32_t DMACHRDR;
	  volatile uint32_t DMACHTBAR;
	  volatile uint32_t DMACHRBAR;
	} ETH_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t IMR;     
	  volatile uint32_t EMR;     
	  volatile uint32_t RTSR;    
	  volatile uint32_t FTSR;    
	  volatile uint32_t SWIER;   
	  volatile uint32_t PR;      
	} EXTI_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t ACR;       
	  volatile uint32_t KEYR;      
	  volatile uint32_t OPTKEYR;   
	  volatile uint32_t SR;        
	  volatile uint32_t CR;        
	  volatile uint32_t OPTCR;     
	  volatile uint32_t OPTCR1;    
	} FLASH_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t BTCR[8];     
	} FMC_Bank1_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t BWTR[7];     
	} FMC_Bank1E_TypeDef;
	

 

	typedef struct
	{
	  volatile uint32_t PCR2;        
	  volatile uint32_t SR2;         
	  volatile uint32_t PMEM2;       
	  volatile uint32_t PATT2;       
	  uint32_t      RESERVED0;   
	  volatile uint32_t ECCR2;       
	  uint32_t      RESERVED1;   
	  uint32_t      RESERVED2;   
	  volatile uint32_t PCR3;        
	  volatile uint32_t SR3;         
	  volatile uint32_t PMEM3;       
	  volatile uint32_t PATT3;       
	  uint32_t      RESERVED3;   
	  volatile uint32_t ECCR3;       
	} FMC_Bank2_3_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t PCR4;        
	  volatile uint32_t SR4;         
	  volatile uint32_t PMEM4;       
	  volatile uint32_t PATT4;       
	  volatile uint32_t PIO4;        
	} FMC_Bank4_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t SDCR[2];         
	  volatile uint32_t SDTR[2];         
	  volatile uint32_t SDCMR;           
	  volatile uint32_t SDRTR;           
	  volatile uint32_t SDSR;            
	} FMC_Bank5_6_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t MODER;     
	  volatile uint32_t OTYPER;    
	  volatile uint32_t OSPEEDR;   
	  volatile uint32_t PUPDR;     
	  volatile uint32_t IDR;       
	  volatile uint32_t ODR;       
	  volatile uint32_t BSRR;      
	  volatile uint32_t LCKR;      
	  volatile uint32_t AFR[2];    
	} GPIO_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t MEMRMP;        
	  volatile uint32_t PMC;           
	  volatile uint32_t EXTICR[4];     
	  uint32_t      RESERVED[2];   
	  volatile uint32_t CMPCR;         
	} SYSCFG_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t CR1;         
	  volatile uint32_t CR2;         
	  volatile uint32_t OAR1;        
	  volatile uint32_t OAR2;        
	  volatile uint32_t DR;          
	  volatile uint32_t SR1;         
	  volatile uint32_t SR2;         
	  volatile uint32_t CCR;         
	  volatile uint32_t TRISE;       
	  volatile uint32_t FLTR;        
	} I2C_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t KR;    
	  volatile uint32_t PR;    
	  volatile uint32_t RLR;   
	  volatile uint32_t SR;    
	} IWDG_TypeDef;

	

 

	typedef struct
	{
	  uint32_t      RESERVED0[2];   
	  volatile uint32_t SSCR;           
	  volatile uint32_t BPCR;           
	  volatile uint32_t AWCR;           
	  volatile uint32_t TWCR;           
	  volatile uint32_t GCR;            
	  uint32_t      RESERVED1[2];   
	  volatile uint32_t SRCR;           
	  uint32_t      RESERVED2[1];   
	  volatile uint32_t BCCR;           
	  uint32_t      RESERVED3[1];   
	  volatile uint32_t IER;            
	  volatile uint32_t ISR;            
	  volatile uint32_t ICR;            
	  volatile uint32_t LIPCR;          
	  volatile uint32_t CPSR;           
	  volatile uint32_t CDSR;           
	} LTDC_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t CR;             
	  volatile uint32_t WHPCR;          
	  volatile uint32_t WVPCR;          
	  volatile uint32_t CKCR;           
	  volatile uint32_t PFCR;           
	  volatile uint32_t CACR;           
	  volatile uint32_t DCCR;           
	  volatile uint32_t BFCR;           
	  uint32_t      RESERVED0[2];   
	  volatile uint32_t CFBAR;          
	  volatile uint32_t CFBLR;          
	  volatile uint32_t CFBLNR;         
	  uint32_t      RESERVED1[3];   
	  volatile uint32_t CLUTWR;         
	} LTDC_Layer_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t CR;    
	  volatile uint32_t CSR;   
	} PWR_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t CR;             
	  volatile uint32_t PLLCFGR;        
	  volatile uint32_t CFGR;           
	  volatile uint32_t CIR;            
	  volatile uint32_t AHB1RSTR;       
	  volatile uint32_t AHB2RSTR;       
	  volatile uint32_t AHB3RSTR;       
	  uint32_t      RESERVED0;      
	  volatile uint32_t APB1RSTR;       
	  volatile uint32_t APB2RSTR;       
	  uint32_t      RESERVED1[2];   
	  volatile uint32_t AHB1ENR;        
	  volatile uint32_t AHB2ENR;        
	  volatile uint32_t AHB3ENR;        
	  uint32_t      RESERVED2;      
	  volatile uint32_t APB1ENR;        
	  volatile uint32_t APB2ENR;        
	  uint32_t      RESERVED3[2];   
	  volatile uint32_t AHB1LPENR;      
	  volatile uint32_t AHB2LPENR;      
	  volatile uint32_t AHB3LPENR;      
	  uint32_t      RESERVED4;      
	  volatile uint32_t APB1LPENR;      
	  volatile uint32_t APB2LPENR;      
	  uint32_t      RESERVED5[2];   
	  volatile uint32_t BDCR;           
	  volatile uint32_t CSR;            
	  uint32_t      RESERVED6[2];   
	  volatile uint32_t SSCGR;          
	  volatile uint32_t PLLI2SCFGR;     
	  volatile uint32_t PLLSAICFGR;     
	  volatile uint32_t DCKCFGR;        
	} RCC_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t TR;       
	  volatile uint32_t DR;       
	  volatile uint32_t CR;       
	  volatile uint32_t ISR;      
	  volatile uint32_t PRER;     
	  volatile uint32_t WUTR;     
	  volatile uint32_t CALIBR;   
	  volatile uint32_t ALRMAR;   
	  volatile uint32_t ALRMBR;   
	  volatile uint32_t WPR;      
	  volatile uint32_t SSR;      
	  volatile uint32_t SHIFTR;   
	  volatile uint32_t TSTR;     
	  volatile uint32_t TSDR;     
	  volatile uint32_t TSSSR;    
	  volatile uint32_t CALR;     
	  volatile uint32_t TAFCR;    
	  volatile uint32_t ALRMASSR; 
	  volatile uint32_t ALRMBSSR; 
	  uint32_t RESERVED7;     
	  volatile uint32_t BKP0R;    
	  volatile uint32_t BKP1R;    
	  volatile uint32_t BKP2R;    
	  volatile uint32_t BKP3R;    
	  volatile uint32_t BKP4R;    
	  volatile uint32_t BKP5R;    
	  volatile uint32_t BKP6R;    
	  volatile uint32_t BKP7R;    
	  volatile uint32_t BKP8R;    
	  volatile uint32_t BKP9R;    
	  volatile uint32_t BKP10R;   
	  volatile uint32_t BKP11R;   
	  volatile uint32_t BKP12R;   
	  volatile uint32_t BKP13R;   
	  volatile uint32_t BKP14R;   
	  volatile uint32_t BKP15R;   
	  volatile uint32_t BKP16R;   
	  volatile uint32_t BKP17R;   
	  volatile uint32_t BKP18R;   
	  volatile uint32_t BKP19R;   
	} RTC_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t GCR;       
	} SAI_TypeDef;

	typedef struct
	{
	  volatile uint32_t CR1;       
	  volatile uint32_t CR2;       
	  volatile uint32_t FRCR;      
	  volatile uint32_t SLOTR;     
	  volatile uint32_t IMR;       
	  volatile uint32_t SR;        
	  volatile uint32_t CLRFR;     
	  volatile uint32_t DR;        
	} SAI_Block_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t POWER;                  
	  volatile uint32_t CLKCR;                  
	  volatile uint32_t ARG;                    
	  volatile uint32_t CMD;                    
	  volatile const uint32_t  RESPCMD;         
	  volatile const uint32_t  RESP1;           
	  volatile const uint32_t  RESP2;           
	  volatile const uint32_t  RESP3;           
	  volatile const uint32_t  RESP4;           
	  volatile uint32_t DTIMER;                 
	  volatile uint32_t DLEN;                   
	  volatile uint32_t DCTRL;                  
	  volatile const uint32_t  DCOUNT;          
	  volatile const uint32_t  STA;             
	  volatile uint32_t ICR;                    
	  volatile uint32_t MASK;                   
	  uint32_t      RESERVED0[2];           
	  volatile const uint32_t  FIFOCNT;         
	  uint32_t      RESERVED1[13];          
	  volatile uint32_t FIFO;                   
	} SDIO_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t CR1;         
	  volatile uint32_t CR2;         
	  volatile uint32_t SR;          
	  volatile uint32_t DR;          
	  volatile uint32_t CRCPR;       
	  volatile uint32_t RXCRCR;      
	  volatile uint32_t TXCRCR;      
	  volatile uint32_t I2SCFGR;     
	  volatile uint32_t I2SPR;       
	} SPI_TypeDef;


	

 

	typedef struct
	{
	  volatile uint32_t CR1;          
	  volatile uint32_t CR2;          
	  volatile uint32_t SMCR;         
	  volatile uint32_t DIER;         
	  volatile uint32_t SR;           
	  volatile uint32_t EGR;          
	  volatile uint32_t CCMR1;        
	  volatile uint32_t CCMR2;        
	  volatile uint32_t CCER;         
	  volatile uint32_t CNT;          
	  volatile uint32_t PSC;          
	  volatile uint32_t ARR;          
	  volatile uint32_t RCR;          
	  volatile uint32_t CCR1;         
	  volatile uint32_t CCR2;         
	  volatile uint32_t CCR3;         
	  volatile uint32_t CCR4;         
	  volatile uint32_t BDTR;         
	  volatile uint32_t DCR;          
	  volatile uint32_t DMAR;         
	  volatile uint32_t OR;           
	} TIM_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t SR;          
	  volatile uint32_t DR;          
	  volatile uint32_t BRR;         
	  volatile uint32_t CR1;         
	  volatile uint32_t CR2;         
	  volatile uint32_t CR3;         
	  volatile uint32_t GTPR;        
	} USART_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t CR;    
	  volatile uint32_t CFR;   
	  volatile uint32_t SR;    
	} WWDG_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t CR;          
	  volatile uint32_t SR;          
	  volatile uint32_t DR;          
	  volatile uint32_t DOUT;        
	  volatile uint32_t DMACR;       
	  volatile uint32_t IMSCR;       
	  volatile uint32_t RISR;        
	  volatile uint32_t MISR;        
	  volatile uint32_t K0LR;        
	  volatile uint32_t K0RR;        
	  volatile uint32_t K1LR;        
	  volatile uint32_t K1RR;        
	  volatile uint32_t K2LR;        
	  volatile uint32_t K2RR;        
	  volatile uint32_t K3LR;        
	  volatile uint32_t K3RR;        
	  volatile uint32_t IV0LR;       
	  volatile uint32_t IV0RR;       
	  volatile uint32_t IV1LR;       
	  volatile uint32_t IV1RR;       
	  volatile uint32_t CSGCMCCM0R;  
	  volatile uint32_t CSGCMCCM1R;  
	  volatile uint32_t CSGCMCCM2R;  
	  volatile uint32_t CSGCMCCM3R;  
	  volatile uint32_t CSGCMCCM4R;  
	  volatile uint32_t CSGCMCCM5R;  
	  volatile uint32_t CSGCMCCM6R;  
	  volatile uint32_t CSGCMCCM7R;  
	  volatile uint32_t CSGCM0R;     
	  volatile uint32_t CSGCM1R;     
	  volatile uint32_t CSGCM2R;     
	  volatile uint32_t CSGCM3R;     
	  volatile uint32_t CSGCM4R;     
	  volatile uint32_t CSGCM5R;     
	  volatile uint32_t CSGCM6R;     
	  volatile uint32_t CSGCM7R;     
	} CRYP_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t CR;                
	  volatile uint32_t DIN;               
	  volatile uint32_t STR;               
	  volatile uint32_t HR[5];             
	  volatile uint32_t IMR;               
	  volatile uint32_t SR;                
		   uint32_t RESERVED[52];      
	  volatile uint32_t CSR[54];           
	} HASH_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t HR[8];      
	} HASH_DIGEST_TypeDef;

	

 

	typedef struct
	{
	  volatile uint32_t CR;   
	  volatile uint32_t SR;   
	  volatile uint32_t DR;   
	} RNG_TypeDef;

	

 
	typedef struct
	{
	  volatile uint32_t GOTGCTL;               
	  volatile uint32_t GOTGINT;               
	  volatile uint32_t GAHBCFG;               
	  volatile uint32_t GUSBCFG;               
	  volatile uint32_t GRSTCTL;               
	  volatile uint32_t GINTSTS;               
	  volatile uint32_t GINTMSK;               
	  volatile uint32_t GRXSTSR;               
	  volatile uint32_t GRXSTSP;               
	  volatile uint32_t GRXFSIZ;               
	  volatile uint32_t DIEPTXF0_HNPTXFSIZ;    
	  volatile uint32_t HNPTXSTS;              
	  uint32_t Reserved30[2];              
	  volatile uint32_t GCCFG;                 
	  volatile uint32_t CID;                   
	  uint32_t  Reserved40[48];            
	  volatile uint32_t HPTXFSIZ;              
	  volatile uint32_t DIEPTXF[0x0F];         
	} USB_OTG_GlobalTypeDef;

	

 
	typedef struct
	{
	  volatile uint32_t DCFG;             
	  volatile uint32_t DCTL;             
	  volatile uint32_t DSTS;             
	  uint32_t Reserved0C;            
	  volatile uint32_t DIEPMSK;          
	  volatile uint32_t DOEPMSK;          
	  volatile uint32_t DAINT;            
	  volatile uint32_t DAINTMSK;         
	  uint32_t  Reserved20;           
	  uint32_t Reserved9;             
	  volatile uint32_t DVBUSDIS;         
	  volatile uint32_t DVBUSPULSE;       
	  volatile uint32_t DTHRCTL;          
	  volatile uint32_t DIEPEMPMSK;       
	  volatile uint32_t DEACHINT;         
	  volatile uint32_t DEACHMSK;         
	  uint32_t Reserved40;            
	  volatile uint32_t DINEP1MSK;        
	  uint32_t  Reserved44[15];       
	  volatile uint32_t DOUTEP1MSK;       
	} USB_OTG_DeviceTypeDef;

	

 
	typedef struct
	{
	  volatile uint32_t DIEPCTL;            
	  uint32_t Reserved04;              
	  volatile uint32_t DIEPINT;            
	  uint32_t Reserved0C;              
	  volatile uint32_t DIEPTSIZ;           
	  volatile uint32_t DIEPDMA;            
	  volatile uint32_t DTXFSTS;            
	  uint32_t Reserved18;              
	} USB_OTG_INEndpointTypeDef;

	

 
	typedef struct
	{
	  volatile uint32_t DOEPCTL;        
	  uint32_t Reserved04;          
	  volatile uint32_t DOEPINT;        
	  uint32_t Reserved0C;          
	  volatile uint32_t DOEPTSIZ;       
	  volatile uint32_t DOEPDMA;        
	  uint32_t Reserved18[2];       
	} USB_OTG_OUTEndpointTypeDef;

	

 
	typedef struct
	{
	  volatile uint32_t HCFG;              
	  volatile uint32_t HFIR;              
	  volatile uint32_t HFNUM;             
	  uint32_t Reserved40C;            
	  volatile uint32_t HPTXSTS;           
	  volatile uint32_t HAINT;             
	  volatile uint32_t HAINTMSK;          
	} USB_OTG_HostTypeDef;

	

 
	typedef struct
	{
	  volatile uint32_t HCCHAR;            
	  volatile uint32_t HCSPLT;            
	  volatile uint32_t HCINT;             
	  volatile uint32_t HCINTMSK;          
	  volatile uint32_t HCTSIZ;            
	  volatile uint32_t HCDMA;             
	  uint32_t Reserved[2];            
	} USB_OTG_HostChannelTypeDef;

	

 

	

 
#line 1131 ".\\inc\\stm32f439xx.h"

	 



	 





	 
#line 1172 ".\\inc\\stm32f439xx.h"

	 
#line 1182 ".\\inc\\stm32f439xx.h"
	 
#line 1200 ".\\inc\\stm32f439xx.h"

	 
#line 1240 ".\\inc\\stm32f439xx.h"

	 






	 







	 

	 



#line 1274 ".\\inc\\stm32f439xx.h"




	

 

	

 
#line 1323 ".\\inc\\stm32f439xx.h"
	 
#line 1388 ".\\inc\\stm32f439xx.h"

	

 

	

 

	  

 

	 
	 
	 

	 
	 
	 
	 
	 
	

 


	 
#line 1434 ".\\inc\\stm32f439xx.h"

	 
#line 1488 ".\\inc\\stm32f439xx.h"

	 
#line 1538 ".\\inc\\stm32f439xx.h"

	 
#line 1594 ".\\inc\\stm32f439xx.h"

	 
#line 1656 ".\\inc\\stm32f439xx.h"

	 




	 




	 




	 




	 




	 




	 
#line 1727 ".\\inc\\stm32f439xx.h"

	 
#line 1777 ".\\inc\\stm32f439xx.h"

	 
#line 1827 ".\\inc\\stm32f439xx.h"

	 
#line 1866 ".\\inc\\stm32f439xx.h"

	 




	 




	 




	 




	 
#line 1894 ".\\inc\\stm32f439xx.h"

	 
#line 1950 ".\\inc\\stm32f439xx.h"

	 




	 
#line 1991 ".\\inc\\stm32f439xx.h"

	 
#line 1999 ".\\inc\\stm32f439xx.h"

	 



	 
	 
	 
	 
	 
	 
	 
#line 2041 ".\\inc\\stm32f439xx.h"
	 
#line 2069 ".\\inc\\stm32f439xx.h"

	 
#line 2119 ".\\inc\\stm32f439xx.h"

#line 2132 ".\\inc\\stm32f439xx.h"

#line 2145 ".\\inc\\stm32f439xx.h"

	 
#line 2159 ".\\inc\\stm32f439xx.h"

	 
#line 2173 ".\\inc\\stm32f439xx.h"

	 
#line 2218 ".\\inc\\stm32f439xx.h"

	 
#line 2229 ".\\inc\\stm32f439xx.h"

#line 2236 ".\\inc\\stm32f439xx.h"

#line 2243 ".\\inc\\stm32f439xx.h"

	 
#line 2272 ".\\inc\\stm32f439xx.h"


	 
	 
#line 2291 ".\\inc\\stm32f439xx.h"

	 
#line 2302 ".\\inc\\stm32f439xx.h"

	 
#line 2316 ".\\inc\\stm32f439xx.h"

	 
#line 2330 ".\\inc\\stm32f439xx.h"

	 
#line 2347 ".\\inc\\stm32f439xx.h"

	 
#line 2358 ".\\inc\\stm32f439xx.h"

	 
#line 2372 ".\\inc\\stm32f439xx.h"

	 
#line 2386 ".\\inc\\stm32f439xx.h"

	 
#line 2403 ".\\inc\\stm32f439xx.h"

	 
#line 2414 ".\\inc\\stm32f439xx.h"

	 
#line 2428 ".\\inc\\stm32f439xx.h"

	 
#line 2442 ".\\inc\\stm32f439xx.h"

	 
#line 2456 ".\\inc\\stm32f439xx.h"

	 
#line 2467 ".\\inc\\stm32f439xx.h"

	 
#line 2481 ".\\inc\\stm32f439xx.h"

	 
#line 2495 ".\\inc\\stm32f439xx.h"

	 
#line 2509 ".\\inc\\stm32f439xx.h"

	 
#line 2520 ".\\inc\\stm32f439xx.h"

	 
#line 2534 ".\\inc\\stm32f439xx.h"

	 
#line 2548 ".\\inc\\stm32f439xx.h"

	 
	 
#line 2557 ".\\inc\\stm32f439xx.h"

	 
#line 2646 ".\\inc\\stm32f439xx.h"

	 
#line 2735 ".\\inc\\stm32f439xx.h"

	 
#line 2824 ".\\inc\\stm32f439xx.h"

	 
#line 2913 ".\\inc\\stm32f439xx.h"


	 
#line 3012 ".\\inc\\stm32f439xx.h"

	 
#line 3110 ".\\inc\\stm32f439xx.h"

	 
#line 3208 ".\\inc\\stm32f439xx.h"

	 
#line 3306 ".\\inc\\stm32f439xx.h"

	 
#line 3404 ".\\inc\\stm32f439xx.h"

	 
#line 3502 ".\\inc\\stm32f439xx.h"

	 
#line 3600 ".\\inc\\stm32f439xx.h"

	 
#line 3698 ".\\inc\\stm32f439xx.h"

	 
#line 3796 ".\\inc\\stm32f439xx.h"

	 
#line 3894 ".\\inc\\stm32f439xx.h"

	 
#line 3992 ".\\inc\\stm32f439xx.h"

	 
#line 4090 ".\\inc\\stm32f439xx.h"

	 
#line 4188 ".\\inc\\stm32f439xx.h"

	 
#line 4286 ".\\inc\\stm32f439xx.h"

	 
#line 4384 ".\\inc\\stm32f439xx.h"

	 
#line 4482 ".\\inc\\stm32f439xx.h"

	 
#line 4580 ".\\inc\\stm32f439xx.h"

	 
#line 4678 ".\\inc\\stm32f439xx.h"

	 
#line 4776 ".\\inc\\stm32f439xx.h"

	 
#line 4874 ".\\inc\\stm32f439xx.h"

	 
#line 4972 ".\\inc\\stm32f439xx.h"

	 
#line 5070 ".\\inc\\stm32f439xx.h"

	 
#line 5168 ".\\inc\\stm32f439xx.h"

	 
#line 5266 ".\\inc\\stm32f439xx.h"

	 
#line 5364 ".\\inc\\stm32f439xx.h"

	 
#line 5462 ".\\inc\\stm32f439xx.h"

	 
#line 5560 ".\\inc\\stm32f439xx.h"

	 
#line 5658 ".\\inc\\stm32f439xx.h"

	 
	 
	 
	 
	 
	 





	 





	 




	 
	 
	 
	 
	 
	 




#line 5719 ".\\inc\\stm32f439xx.h"

#line 5736 ".\\inc\\stm32f439xx.h"

#line 5743 ".\\inc\\stm32f439xx.h"

	 
#line 5760 ".\\inc\\stm32f439xx.h"
	 
#line 5767 ".\\inc\\stm32f439xx.h"
	 
#line 5774 ".\\inc\\stm32f439xx.h"
	 
#line 5781 ".\\inc\\stm32f439xx.h"
	 
#line 5788 ".\\inc\\stm32f439xx.h"

	 
	 
	 
	 
	 
	

 

	 
#line 5808 ".\\inc\\stm32f439xx.h"

#line 5815 ".\\inc\\stm32f439xx.h"







#line 5829 ".\\inc\\stm32f439xx.h"

#line 5845 ".\\inc\\stm32f439xx.h"

#line 5852 ".\\inc\\stm32f439xx.h"







#line 5866 ".\\inc\\stm32f439xx.h"

#line 5873 ".\\inc\\stm32f439xx.h"

	 
#line 5881 ".\\inc\\stm32f439xx.h"

	 




	 




	 




	 




	 




	 




	 
#line 5919 ".\\inc\\stm32f439xx.h"

	 
#line 5927 ".\\inc\\stm32f439xx.h"

	 
#line 5935 ".\\inc\\stm32f439xx.h"

	 




	 




	 
#line 5953 ".\\inc\\stm32f439xx.h"

	 
	 
	 
	 
	 
	 
#line 5991 ".\\inc\\stm32f439xx.h"

	 
#line 6002 ".\\inc\\stm32f439xx.h"

	 
#line 6019 ".\\inc\\stm32f439xx.h"
	 
#line 6026 ".\\inc\\stm32f439xx.h"

	 
#line 6043 ".\\inc\\stm32f439xx.h"
	 


	 
#line 6062 ".\\inc\\stm32f439xx.h"

	 






	 
#line 6086 ".\\inc\\stm32f439xx.h"

	 


	 
#line 6103 ".\\inc\\stm32f439xx.h"

	 
#line 6117 ".\\inc\\stm32f439xx.h"

	 
#line 6125 ".\\inc\\stm32f439xx.h"

	 
#line 6133 ".\\inc\\stm32f439xx.h"

	 
#line 6147 ".\\inc\\stm32f439xx.h"

	 
	 
	 
	 
	 
	 
#line 6226 ".\\inc\\stm32f439xx.h"

	 




	 
#line 6252 ".\\inc\\stm32f439xx.h"

	 
#line 6271 ".\\inc\\stm32f439xx.h"

	 
#line 6333 ".\\inc\\stm32f439xx.h"

	 
#line 6395 ".\\inc\\stm32f439xx.h"

	 
#line 6457 ".\\inc\\stm32f439xx.h"

	 
#line 6519 ".\\inc\\stm32f439xx.h"

	 




	 




	 





	 
	 
	 
	 
	 

	 

#line 6576 ".\\inc\\stm32f439xx.h"

	 

#line 6597 ".\\inc\\stm32f439xx.h"

	 

#line 6618 ".\\inc\\stm32f439xx.h"

	 
#line 6626 ".\\inc\\stm32f439xx.h"

	 





	 





	 





	 





	 

#line 6677 ".\\inc\\stm32f439xx.h"

	 

#line 6689 ".\\inc\\stm32f439xx.h"

	 

#line 6716 ".\\inc\\stm32f439xx.h"

	 

#line 6728 ".\\inc\\stm32f439xx.h"

	 





	 





	 

#line 6749 ".\\inc\\stm32f439xx.h"

	 

	 






	 




	 





	 





	 





	 





	 

#line 6796 ".\\inc\\stm32f439xx.h"

	 





	 

#line 6811 ".\\inc\\stm32f439xx.h"

	 

	 


	 
	 
	 
	 
	 
	 
#line 6892 ".\\inc\\stm32f439xx.h"

	 
#line 6920 ".\\inc\\stm32f439xx.h"

	 
#line 6991 ".\\inc\\stm32f439xx.h"

	 
#line 7016 ".\\inc\\stm32f439xx.h"

	 
#line 7087 ".\\inc\\stm32f439xx.h"

	 
#line 7158 ".\\inc\\stm32f439xx.h"

	 
#line 7229 ".\\inc\\stm32f439xx.h"

	 
#line 7300 ".\\inc\\stm32f439xx.h"

	 
	 
	 
	 
	 
	 
#line 7318 ".\\inc\\stm32f439xx.h"

#line 7348 ".\\inc\\stm32f439xx.h"

	 
#line 7374 ".\\inc\\stm32f439xx.h"

	 
#line 7411 ".\\inc\\stm32f439xx.h"

	 
#line 7419 ".\\inc\\stm32f439xx.h"

#line 7469 ".\\inc\\stm32f439xx.h"

	 
#line 7486 ".\\inc\\stm32f439xx.h"

	 
	 
	 
	 
	 
	 
#line 7499 ".\\inc\\stm32f439xx.h"













#line 7551 ".\\inc\\stm32f439xx.h"

	 
#line 7559 ".\\inc\\stm32f439xx.h"













#line 7608 ".\\inc\\stm32f439xx.h"

	 
#line 7616 ".\\inc\\stm32f439xx.h"













#line 7665 ".\\inc\\stm32f439xx.h"

	 
#line 7673 ".\\inc\\stm32f439xx.h"













#line 7722 ".\\inc\\stm32f439xx.h"

	 
#line 7731 ".\\inc\\stm32f439xx.h"

#line 7739 ".\\inc\\stm32f439xx.h"

#line 7751 ".\\inc\\stm32f439xx.h"

#line 7759 ".\\inc\\stm32f439xx.h"

#line 7767 ".\\inc\\stm32f439xx.h"

#line 7775 ".\\inc\\stm32f439xx.h"







	 
#line 7790 ".\\inc\\stm32f439xx.h"

#line 7798 ".\\inc\\stm32f439xx.h"

#line 7810 ".\\inc\\stm32f439xx.h"

#line 7818 ".\\inc\\stm32f439xx.h"

#line 7826 ".\\inc\\stm32f439xx.h"

#line 7834 ".\\inc\\stm32f439xx.h"







	 
#line 7849 ".\\inc\\stm32f439xx.h"

#line 7857 ".\\inc\\stm32f439xx.h"

#line 7869 ".\\inc\\stm32f439xx.h"

#line 7877 ".\\inc\\stm32f439xx.h"

#line 7885 ".\\inc\\stm32f439xx.h"

#line 7893 ".\\inc\\stm32f439xx.h"







	 
#line 7908 ".\\inc\\stm32f439xx.h"

#line 7916 ".\\inc\\stm32f439xx.h"

#line 7928 ".\\inc\\stm32f439xx.h"

#line 7936 ".\\inc\\stm32f439xx.h"

#line 7944 ".\\inc\\stm32f439xx.h"

#line 7952 ".\\inc\\stm32f439xx.h"







	 
#line 7967 ".\\inc\\stm32f439xx.h"

#line 7975 ".\\inc\\stm32f439xx.h"

#line 7987 ".\\inc\\stm32f439xx.h"

#line 7995 ".\\inc\\stm32f439xx.h"







	 
#line 8010 ".\\inc\\stm32f439xx.h"

#line 8018 ".\\inc\\stm32f439xx.h"

#line 8030 ".\\inc\\stm32f439xx.h"

#line 8038 ".\\inc\\stm32f439xx.h"







	 
#line 8053 ".\\inc\\stm32f439xx.h"

#line 8061 ".\\inc\\stm32f439xx.h"

#line 8073 ".\\inc\\stm32f439xx.h"

#line 8081 ".\\inc\\stm32f439xx.h"







	 
#line 8096 ".\\inc\\stm32f439xx.h"

#line 8104 ".\\inc\\stm32f439xx.h"

#line 8116 ".\\inc\\stm32f439xx.h"

#line 8124 ".\\inc\\stm32f439xx.h"







	 

#line 8142 ".\\inc\\stm32f439xx.h"











#line 8160 ".\\inc\\stm32f439xx.h"

#line 8168 ".\\inc\\stm32f439xx.h"

#line 8175 ".\\inc\\stm32f439xx.h"

	 
#line 8186 ".\\inc\\stm32f439xx.h"











#line 8204 ".\\inc\\stm32f439xx.h"

#line 8212 ".\\inc\\stm32f439xx.h"

#line 8219 ".\\inc\\stm32f439xx.h"

	 
#line 8230 ".\\inc\\stm32f439xx.h"











#line 8248 ".\\inc\\stm32f439xx.h"

#line 8256 ".\\inc\\stm32f439xx.h"

#line 8263 ".\\inc\\stm32f439xx.h"

	 
#line 8286 ".\\inc\\stm32f439xx.h"

	 
#line 8309 ".\\inc\\stm32f439xx.h"

	 
#line 8332 ".\\inc\\stm32f439xx.h"

	 
#line 8345 ".\\inc\\stm32f439xx.h"

#line 8357 ".\\inc\\stm32f439xx.h"

#line 8369 ".\\inc\\stm32f439xx.h"

#line 8381 ".\\inc\\stm32f439xx.h"

	 
#line 8394 ".\\inc\\stm32f439xx.h"

#line 8406 ".\\inc\\stm32f439xx.h"

#line 8418 ".\\inc\\stm32f439xx.h"

#line 8430 ".\\inc\\stm32f439xx.h"

	 
#line 8443 ".\\inc\\stm32f439xx.h"

#line 8455 ".\\inc\\stm32f439xx.h"

#line 8467 ".\\inc\\stm32f439xx.h"

#line 8479 ".\\inc\\stm32f439xx.h"

	 
#line 8492 ".\\inc\\stm32f439xx.h"

#line 8504 ".\\inc\\stm32f439xx.h"

#line 8516 ".\\inc\\stm32f439xx.h"

#line 8528 ".\\inc\\stm32f439xx.h"

	 
#line 8541 ".\\inc\\stm32f439xx.h"

#line 8553 ".\\inc\\stm32f439xx.h"

#line 8565 ".\\inc\\stm32f439xx.h"

#line 8577 ".\\inc\\stm32f439xx.h"

	 
#line 8590 ".\\inc\\stm32f439xx.h"

#line 8602 ".\\inc\\stm32f439xx.h"

#line 8614 ".\\inc\\stm32f439xx.h"

#line 8626 ".\\inc\\stm32f439xx.h"

	 
#line 8639 ".\\inc\\stm32f439xx.h"

#line 8651 ".\\inc\\stm32f439xx.h"

#line 8663 ".\\inc\\stm32f439xx.h"

#line 8675 ".\\inc\\stm32f439xx.h"


	 




	 




	 
















































	 
















































	 
#line 8793 ".\\inc\\stm32f439xx.h"

#line 8801 ".\\inc\\stm32f439xx.h"

#line 8809 ".\\inc\\stm32f439xx.h"

#line 8816 ".\\inc\\stm32f439xx.h"

#line 8823 ".\\inc\\stm32f439xx.h"

#line 8830 ".\\inc\\stm32f439xx.h"

#line 8837 ".\\inc\\stm32f439xx.h"

	 
#line 8846 ".\\inc\\stm32f439xx.h"

#line 8854 ".\\inc\\stm32f439xx.h"

#line 8862 ".\\inc\\stm32f439xx.h"

#line 8869 ".\\inc\\stm32f439xx.h"

#line 8876 ".\\inc\\stm32f439xx.h"

#line 8883 ".\\inc\\stm32f439xx.h"

#line 8890 ".\\inc\\stm32f439xx.h"

	 
#line 8898 ".\\inc\\stm32f439xx.h"









#line 8914 ".\\inc\\stm32f439xx.h"





	 












	 










#line 8951 ".\\inc\\stm32f439xx.h"

	 
	 
	 
	 
	 
	 
#line 9038 ".\\inc\\stm32f439xx.h"

	 
#line 9120 ".\\inc\\stm32f439xx.h"

	 
#line 9170 ".\\inc\\stm32f439xx.h"

	 
#line 9188 ".\\inc\\stm32f439xx.h"

	 
#line 9270 ".\\inc\\stm32f439xx.h"

	 
#line 9320 ".\\inc\\stm32f439xx.h"

	 
#line 9402 ".\\inc\\stm32f439xx.h"

	 
#line 9452 ".\\inc\\stm32f439xx.h"

	 
#line 9502 ".\\inc\\stm32f439xx.h"

	 
#line 9520 ".\\inc\\stm32f439xx.h"

	 
#line 9570 ".\\inc\\stm32f439xx.h"
	 
#line 9587 ".\\inc\\stm32f439xx.h"

	 
#line 9685 ".\\inc\\stm32f439xx.h"

	 
#line 9719 ".\\inc\\stm32f439xx.h"
	 
#line 9771 ".\\inc\\stm32f439xx.h"
	 
#line 9828 ".\\inc\\stm32f439xx.h"

	 
#line 9870 ".\\inc\\stm32f439xx.h"

	 
#line 9928 ".\\inc\\stm32f439xx.h"

	 
#line 9970 ".\\inc\\stm32f439xx.h"

	 
#line 10020 ".\\inc\\stm32f439xx.h"


	 
	 
	 
	 
	 
	 
#line 10063 ".\\inc\\stm32f439xx.h"

	 
#line 10076 ".\\inc\\stm32f439xx.h"
	 
#line 10083 ".\\inc\\stm32f439xx.h"

	 
#line 10091 ".\\inc\\stm32f439xx.h"
	 



	 
#line 10108 ".\\inc\\stm32f439xx.h"

	 
	 
	 
	 
	 
	 
#line 10157 ".\\inc\\stm32f439xx.h"

	 
#line 10168 ".\\inc\\stm32f439xx.h"

#line 10184 ".\\inc\\stm32f439xx.h"

	 



#line 10219 ".\\inc\\stm32f439xx.h"





	 
#line 10231 ".\\inc\\stm32f439xx.h"

	 




	 
#line 10280 ".\\inc\\stm32f439xx.h"

	 
#line 10306 ".\\inc\\stm32f439xx.h"

	 
#line 10317 ".\\inc\\stm32f439xx.h"

	 




	 
#line 10330 ".\\inc\\stm32f439xx.h"

	 
	 
	 
	 
	 
	 




	 
#line 10348 ".\\inc\\stm32f439xx.h"

	 




	 
#line 10361 ".\\inc\\stm32f439xx.h"


	 
	 
	 
	 
	 

	 

#line 10377 ".\\inc\\stm32f439xx.h"

	 

#line 10386 ".\\inc\\stm32f439xx.h"

	 

#line 10395 ".\\inc\\stm32f439xx.h"

	 

#line 10404 ".\\inc\\stm32f439xx.h"

	 

#line 10434 ".\\inc\\stm32f439xx.h"

	 


	 

#line 10446 ".\\inc\\stm32f439xx.h"

	 

#line 10458 ".\\inc\\stm32f439xx.h"

	 

#line 10473 ".\\inc\\stm32f439xx.h"

	 

#line 10488 ".\\inc\\stm32f439xx.h"

	 

#line 10503 ".\\inc\\stm32f439xx.h"

	 





	 

#line 10518 ".\\inc\\stm32f439xx.h"

	 

#line 10533 ".\\inc\\stm32f439xx.h"

	 

#line 10545 ".\\inc\\stm32f439xx.h"

	 

#line 10554 ".\\inc\\stm32f439xx.h"

	 

#line 10563 ".\\inc\\stm32f439xx.h"

	 

#line 10575 ".\\inc\\stm32f439xx.h"

	 





	 





	 

#line 10602 ".\\inc\\stm32f439xx.h"

	 

#line 10611 ".\\inc\\stm32f439xx.h"

	 





	 

#line 10626 ".\\inc\\stm32f439xx.h"

	 





	 

#line 10647 ".\\inc\\stm32f439xx.h"


	 
	 
	 
	 
	 
	 
#line 10670 ".\\inc\\stm32f439xx.h"

#line 10677 ".\\inc\\stm32f439xx.h"

	 
#line 10718 ".\\inc\\stm32f439xx.h"

	 




	 
#line 10755 ".\\inc\\stm32f439xx.h"
	 


	 


	 
	 
	 
	 
	 
	 
#line 10773 ".\\inc\\stm32f439xx.h"

#line 10782 ".\\inc\\stm32f439xx.h"

#line 10794 ".\\inc\\stm32f439xx.h"

#line 10813 ".\\inc\\stm32f439xx.h"
	

 


#line 10824 ".\\inc\\stm32f439xx.h"
	

 


#line 10835 ".\\inc\\stm32f439xx.h"

	 
#line 10846 ".\\inc\\stm32f439xx.h"

#line 10859 ".\\inc\\stm32f439xx.h"







#line 10873 ".\\inc\\stm32f439xx.h"

#line 10881 ".\\inc\\stm32f439xx.h"


	 
	 










	 










	 
#line 10914 ".\\inc\\stm32f439xx.h"

#line 10924 ".\\inc\\stm32f439xx.h"

	 
#line 10932 ".\\inc\\stm32f439xx.h"







	 
#line 10946 ".\\inc\\stm32f439xx.h"







	 
#line 10962 ".\\inc\\stm32f439xx.h"

	 










#line 10980 ".\\inc\\stm32f439xx.h"

#line 10987 ".\\inc\\stm32f439xx.h"







	 
#line 11013 ".\\inc\\stm32f439xx.h"

#line 11038 ".\\inc\\stm32f439xx.h"

#line 11063 ".\\inc\\stm32f439xx.h"





	 
#line 11120 ".\\inc\\stm32f439xx.h"

	 
#line 11131 ".\\inc\\stm32f439xx.h"
	  
#line 11139 ".\\inc\\stm32f439xx.h"
	 





	 
#line 11221 ".\\inc\\stm32f439xx.h"

	 
#line 11271 ".\\inc\\stm32f439xx.h"

	 


	 
#line 11345 ".\\inc\\stm32f439xx.h"
	 
	

 


#line 11366 ".\\inc\\stm32f439xx.h"

	 
	

 






	 
#line 11453 ".\\inc\\stm32f439xx.h"

	 
#line 11509 ".\\inc\\stm32f439xx.h"

	 
#line 11571 ".\\inc\\stm32f439xx.h"

#line 11590 ".\\inc\\stm32f439xx.h"

	 
#line 11607 ".\\inc\\stm32f439xx.h"

	 




	 
#line 11689 ".\\inc\\stm32f439xx.h"

	 
#line 11745 ".\\inc\\stm32f439xx.h"

	 
#line 11756 ".\\inc\\stm32f439xx.h"







#line 11769 ".\\inc\\stm32f439xx.h"

	 
#line 11801 ".\\inc\\stm32f439xx.h"
	 



	 
#line 11818 ".\\inc\\stm32f439xx.h"

	 
#line 11832 ".\\inc\\stm32f439xx.h"

#line 11846 ".\\inc\\stm32f439xx.h"

	 
#line 11860 ".\\inc\\stm32f439xx.h"


#line 11869 ".\\inc\\stm32f439xx.h"

#line 11876 ".\\inc\\stm32f439xx.h"

	 
#line 11886 ".\\inc\\stm32f439xx.h"

#line 11900 ".\\inc\\stm32f439xx.h"

#line 11914 ".\\inc\\stm32f439xx.h"


	 
	 
	 
	 
	 
	 
#line 11928 ".\\inc\\stm32f439xx.h"

	 
#line 11945 ".\\inc\\stm32f439xx.h"

	 
	 
	 
	 
	 
	

 


	 
#line 11998 ".\\inc\\stm32f439xx.h"

	 
#line 12042 ".\\inc\\stm32f439xx.h"

	 
#line 12112 ".\\inc\\stm32f439xx.h"

	 


	 
#line 12165 ".\\inc\\stm32f439xx.h"

	 
#line 12173 ".\\inc\\stm32f439xx.h"

	 




	 
#line 12186 ".\\inc\\stm32f439xx.h"

	 
#line 12256 ".\\inc\\stm32f439xx.h"

	 
#line 12326 ".\\inc\\stm32f439xx.h"

	 




	 




	 
#line 12344 ".\\inc\\stm32f439xx.h"

	 
#line 12387 ".\\inc\\stm32f439xx.h"

	 
#line 12417 ".\\inc\\stm32f439xx.h"

	 




	 
#line 12445 ".\\inc\\stm32f439xx.h"

	 
#line 12493 ".\\inc\\stm32f439xx.h"

	 


	 
#line 12508 ".\\inc\\stm32f439xx.h"

	 
#line 12520 ".\\inc\\stm32f439xx.h"

	 




	 




	 




	 




	 




	 




	 




	 




	 




	 




	 




	 




	 




	 




	 




	 




	 




	 




	 




	 




	 


	 
	 
	 
	 
	 
	 












	 












#line 12661 ".\\inc\\stm32f439xx.h"

#line 12668 ".\\inc\\stm32f439xx.h"







#line 12690 ".\\inc\\stm32f439xx.h"

#line 12698 ".\\inc\\stm32f439xx.h"

	 
#line 12706 ".\\inc\\stm32f439xx.h"

#line 12719 ".\\inc\\stm32f439xx.h"

#line 12729 ".\\inc\\stm32f439xx.h"











	 
#line 12752 ".\\inc\\stm32f439xx.h"

#line 12763 ".\\inc\\stm32f439xx.h"

#line 12773 ".\\inc\\stm32f439xx.h"
	 


	 
#line 12785 ".\\inc\\stm32f439xx.h"







#line 12799 ".\\inc\\stm32f439xx.h"





	 
#line 12826 ".\\inc\\stm32f439xx.h"

	 
#line 12849 ".\\inc\\stm32f439xx.h"

#line 12856 ".\\inc\\stm32f439xx.h"

	 
#line 12879 ".\\inc\\stm32f439xx.h"

	 





	 
	 
	 
	 
	 
	 






	 
#line 12911 ".\\inc\\stm32f439xx.h"







#line 12924 ".\\inc\\stm32f439xx.h"

	 




	 










#line 12962 ".\\inc\\stm32f439xx.h"

	 




	 




	 




	 




	 




	 




	 




	 




	 
#line 13016 ".\\inc\\stm32f439xx.h"

#line 13024 ".\\inc\\stm32f439xx.h"

#line 13037 ".\\inc\\stm32f439xx.h"

	 




	 
#line 13116 ".\\inc\\stm32f439xx.h"

	 
#line 13157 ".\\inc\\stm32f439xx.h"

	 
#line 13231 ".\\inc\\stm32f439xx.h"

	 




	 




	 
	 
	 
	 
	 


	 
#line 13259 ".\\inc\\stm32f439xx.h"

#line 13266 ".\\inc\\stm32f439xx.h"

#line 13297 ".\\inc\\stm32f439xx.h"

	 
#line 13320 ".\\inc\\stm32f439xx.h"

	 
#line 13349 ".\\inc\\stm32f439xx.h"

	 




	 




	 




	 




	 






























#line 13407 ".\\inc\\stm32f439xx.h"

	 
#line 13418 ".\\inc\\stm32f439xx.h"

	 
	 
	 
	 
	 
	 
#line 13438 ".\\inc\\stm32f439xx.h"
	 

	 
#line 13456 ".\\inc\\stm32f439xx.h"
	 


	 
#line 13472 ".\\inc\\stm32f439xx.h"
	

 
#line 13486 ".\\inc\\stm32f439xx.h"

	

 
#line 13501 ".\\inc\\stm32f439xx.h"

	

 
#line 13516 ".\\inc\\stm32f439xx.h"

	

 
#line 13531 ".\\inc\\stm32f439xx.h"

	 
#line 13545 ".\\inc\\stm32f439xx.h"

	

 
#line 13560 ".\\inc\\stm32f439xx.h"

	

 
#line 13575 ".\\inc\\stm32f439xx.h"

	

 
#line 13590 ".\\inc\\stm32f439xx.h"

	

 
#line 13605 ".\\inc\\stm32f439xx.h"

	 
#line 13619 ".\\inc\\stm32f439xx.h"

	

 
#line 13633 ".\\inc\\stm32f439xx.h"

	

 
#line 13647 ".\\inc\\stm32f439xx.h"

	

 
#line 13661 ".\\inc\\stm32f439xx.h"

	

 
#line 13675 ".\\inc\\stm32f439xx.h"


	 
#line 13690 ".\\inc\\stm32f439xx.h"

	

 
#line 13704 ".\\inc\\stm32f439xx.h"

	

 
#line 13718 ".\\inc\\stm32f439xx.h"

	

 
#line 13732 ".\\inc\\stm32f439xx.h"

	

 
#line 13746 ".\\inc\\stm32f439xx.h"

	 
#line 13754 ".\\inc\\stm32f439xx.h"

	 
	 
	 
	 
	 
	 
#line 13776 ".\\inc\\stm32f439xx.h"

















	 
#line 13803 ".\\inc\\stm32f439xx.h"

#line 13810 ".\\inc\\stm32f439xx.h"

#line 13835 ".\\inc\\stm32f439xx.h"

	 
#line 13843 ".\\inc\\stm32f439xx.h"

#line 13850 ".\\inc\\stm32f439xx.h"





#line 13862 ".\\inc\\stm32f439xx.h"







#line 13875 ".\\inc\\stm32f439xx.h"

	 
#line 13922 ".\\inc\\stm32f439xx.h"

	 
#line 13960 ".\\inc\\stm32f439xx.h"

	 
#line 13986 ".\\inc\\stm32f439xx.h"

	 






#line 14000 ".\\inc\\stm32f439xx.h"

#line 14007 ".\\inc\\stm32f439xx.h"











#line 14024 ".\\inc\\stm32f439xx.h"

#line 14031 ".\\inc\\stm32f439xx.h"





	 







#line 14051 ".\\inc\\stm32f439xx.h"







#line 14065 ".\\inc\\stm32f439xx.h"

	 






#line 14079 ".\\inc\\stm32f439xx.h"

#line 14086 ".\\inc\\stm32f439xx.h"











#line 14103 ".\\inc\\stm32f439xx.h"

#line 14110 ".\\inc\\stm32f439xx.h"





	 







#line 14130 ".\\inc\\stm32f439xx.h"







#line 14144 ".\\inc\\stm32f439xx.h"

	 
#line 14191 ".\\inc\\stm32f439xx.h"

	 




	 




	 




	 




	 




	 




	 




	 




	 
#line 14244 ".\\inc\\stm32f439xx.h"







#line 14269 ".\\inc\\stm32f439xx.h"

	 
#line 14279 ".\\inc\\stm32f439xx.h"

#line 14288 ".\\inc\\stm32f439xx.h"

	 




	 






#line 14311 ".\\inc\\stm32f439xx.h"


	 
	 
	 
	 
	 
	 
#line 14349 ".\\inc\\stm32f439xx.h"

	 




	 
#line 14362 ".\\inc\\stm32f439xx.h"

	 
#line 14409 ".\\inc\\stm32f439xx.h"

	 
#line 14432 ".\\inc\\stm32f439xx.h"











	 
#line 14480 ".\\inc\\stm32f439xx.h"

	 
#line 14493 ".\\inc\\stm32f439xx.h"





	 
	 
	 
	 
	 
	 
#line 14514 ".\\inc\\stm32f439xx.h"
	 
#line 14522 ".\\inc\\stm32f439xx.h"





	 
#line 14538 ".\\inc\\stm32f439xx.h"
	 
#line 14546 ".\\inc\\stm32f439xx.h"






	 







	 





	 
	 
	 
	 
	 
	 
#line 14578 ".\\inc\\stm32f439xx.h"

	 
#line 14592 ".\\inc\\stm32f439xx.h"







	 
#line 14651 ".\\inc\\stm32f439xx.h"
	 


	 
#line 14670 ".\\inc\\stm32f439xx.h"

	 
	 
	 
	 
	 
	 
#line 14735 ".\\inc\\stm32f439xx.h"

	 
#line 14779 ".\\inc\\stm32f439xx.h"

	 




	 




	 
#line 14819 ".\\inc\\stm32f439xx.h"

	 




	 
#line 14857 ".\\inc\\stm32f439xx.h"

	 
#line 14865 ".\\inc\\stm32f439xx.h"

	 



	
 
	







 

	 
#line 14904 ".\\inc\\stm32f439xx.h"

	 
#line 14984 ".\\inc\\stm32f439xx.h"

	 
#line 15001 ".\\inc\\stm32f439xx.h"

	 
#line 15009 ".\\inc\\stm32f439xx.h"

	 




	 




	 
#line 15039 ".\\inc\\stm32f439xx.h"

	 




	 
#line 15064 ".\\inc\\stm32f439xx.h"

	 




	 
#line 15089 ".\\inc\\stm32f439xx.h"

	 




	 
	 
	 

	 
#line 15118 ".\\inc\\stm32f439xx.h"

	 
#line 15129 ".\\inc\\stm32f439xx.h"

	 
#line 15140 ".\\inc\\stm32f439xx.h"

	 
#line 15151 ".\\inc\\stm32f439xx.h"

	 
#line 15162 ".\\inc\\stm32f439xx.h"

	 




	 




	 




	 




	 




	 




	 
	 
	 

	 
#line 15225 ".\\inc\\stm32f439xx.h"

#line 15244 ".\\inc\\stm32f439xx.h"

	 




	 




	 
#line 15262 ".\\inc\\stm32f439xx.h"

	 




	 
#line 15275 ".\\inc\\stm32f439xx.h"

	 




	 




	 




	 
#line 15298 ".\\inc\\stm32f439xx.h"

	 
	 
	 

	 
#line 15365 ".\\inc\\stm32f439xx.h"

	 




	 




	 




	 




	 
#line 15399 ".\\inc\\stm32f439xx.h"
	   
#line 15492 ".\\inc\\stm32f439xx.h"

	 
#line 15542 ".\\inc\\stm32f439xx.h"

	 
#line 15589 ".\\inc\\stm32f439xx.h"

	 
#line 15603 ".\\inc\\stm32f439xx.h"

	 




	 




	 




	 




	 
	 
	 
	 
	 
	 
#line 15660 ".\\inc\\stm32f439xx.h"

	 

#line 15671 ".\\inc\\stm32f439xx.h"

	 

#line 15682 ".\\inc\\stm32f439xx.h"

#line 15693 ".\\inc\\stm32f439xx.h"













	 
#line 15716 ".\\inc\\stm32f439xx.h"

	 
#line 15736 ".\\inc\\stm32f439xx.h"

	 
#line 15750 ".\\inc\\stm32f439xx.h"

#line 15772 ".\\inc\\stm32f439xx.h"

	 




	 
#line 15785 ".\\inc\\stm32f439xx.h"

	 




#line 15802 ".\\inc\\stm32f439xx.h"

	 
#line 15824 ".\\inc\\stm32f439xx.h"

	 

#line 15888 ".\\inc\\stm32f439xx.h"

	 
#line 15905 ".\\inc\\stm32f439xx.h"


#line 15921 ".\\inc\\stm32f439xx.h"

	 
#line 15947 ".\\inc\\stm32f439xx.h"

	 
#line 15963 ".\\inc\\stm32f439xx.h"

#line 15975 ".\\inc\\stm32f439xx.h"

	 




	 
#line 16003 ".\\inc\\stm32f439xx.h"

	 
#line 16083 ".\\inc\\stm32f439xx.h"

	 
#line 16163 ".\\inc\\stm32f439xx.h"

	 
#line 16171 ".\\inc\\stm32f439xx.h"

	 




	 
#line 16190 ".\\inc\\stm32f439xx.h"

	 
#line 16198 ".\\inc\\stm32f439xx.h"

	 




	 




	 
#line 16222 ".\\inc\\stm32f439xx.h"

	 




	 




#line 16244 ".\\inc\\stm32f439xx.h"

#line 16255 ".\\inc\\stm32f439xx.h"

	 
#line 16263 ".\\inc\\stm32f439xx.h"

#line 16279 ".\\inc\\stm32f439xx.h"

#line 16295 ".\\inc\\stm32f439xx.h"

	 




	 
#line 16308 ".\\inc\\stm32f439xx.h"

	 
#line 16328 ".\\inc\\stm32f439xx.h"

	 
#line 16336 ".\\inc\\stm32f439xx.h"

	 




	 
#line 16370 ".\\inc\\stm32f439xx.h"

	 
#line 16399 ".\\inc\\stm32f439xx.h"

#line 16408 ".\\inc\\stm32f439xx.h"

#line 16416 ".\\inc\\stm32f439xx.h"







	 
#line 16457 ".\\inc\\stm32f439xx.h"

	 
#line 16465 ".\\inc\\stm32f439xx.h"

	 
#line 16479 ".\\inc\\stm32f439xx.h"

#line 16488 ".\\inc\\stm32f439xx.h"

#line 16514 ".\\inc\\stm32f439xx.h"

	 




#line 16533 ".\\inc\\stm32f439xx.h"













#line 16565 ".\\inc\\stm32f439xx.h"

	 

#line 16578 ".\\inc\\stm32f439xx.h"

#line 16589 ".\\inc\\stm32f439xx.h"

#line 16601 ".\\inc\\stm32f439xx.h"

	 
#line 16636 ".\\inc\\stm32f439xx.h"

	 
#line 16671 ".\\inc\\stm32f439xx.h"

	 
#line 16706 ".\\inc\\stm32f439xx.h"

	 

#line 16718 ".\\inc\\stm32f439xx.h"
	 
#line 16733 ".\\inc\\stm32f439xx.h"

	 




	 




	 




	 
#line 16756 ".\\inc\\stm32f439xx.h"

	 

#line 16797 ".\\inc\\stm32f439xx.h"

	 
#line 16817 ".\\inc\\stm32f439xx.h"

	 

#line 16826 ".\\inc\\stm32f439xx.h"







	 
#line 16843 ".\\inc\\stm32f439xx.h"

	 
	 
#line 16856 ".\\inc\\stm32f439xx.h"







#line 16870 ".\\inc\\stm32f439xx.h"

#line 16878 ".\\inc\\stm32f439xx.h"

#line 16886 ".\\inc\\stm32f439xx.h"
	

 

	

 

	

 

	 








	 


	 


	 


	 


	 


	 
#line 16939 ".\\inc\\stm32f439xx.h"

	 
#line 16952 ".\\inc\\stm32f439xx.h"

	 




	 


	 




	 


	 


	 

	 


	 


	 


	 



	 
#line 16994 ".\\inc\\stm32f439xx.h"


	 
#line 17011 ".\\inc\\stm32f439xx.h"

	 
#line 17025 ".\\inc\\stm32f439xx.h"

	 
#line 17035 ".\\inc\\stm32f439xx.h"

	 
#line 17043 ".\\inc\\stm32f439xx.h"

	 
#line 17051 ".\\inc\\stm32f439xx.h"

	 



	 
#line 17063 ".\\inc\\stm32f439xx.h"

	 
#line 17073 ".\\inc\\stm32f439xx.h"

	 
#line 17081 ".\\inc\\stm32f439xx.h"

	 
#line 17089 ".\\inc\\stm32f439xx.h"

	 
#line 17097 ".\\inc\\stm32f439xx.h"

	 
#line 17107 ".\\inc\\stm32f439xx.h"

	 
#line 17117 ".\\inc\\stm32f439xx.h"

	 



	 
#line 17129 ".\\inc\\stm32f439xx.h"

	 




	 
#line 17192 ".\\inc\\stm32f439xx.h"

	 
#line 17204 ".\\inc\\stm32f439xx.h"

	 
#line 17212 ".\\inc\\stm32f439xx.h"

	 
#line 17226 ".\\inc\\stm32f439xx.h"

	 




	 
#line 17239 ".\\inc\\stm32f439xx.h"

	 
#line 17249 ".\\inc\\stm32f439xx.h"

	 
#line 17257 ".\\inc\\stm32f439xx.h"

	 



	 
#line 17271 ".\\inc\\stm32f439xx.h"
	 
#line 17278 ".\\inc\\stm32f439xx.h"
	 



	 





	 
#line 17297 ".\\inc\\stm32f439xx.h"

	 


	 




	 


	 





	 
#line 17324 ".\\inc\\stm32f439xx.h"

	 



	 



	 


	 


	 


	 





	

 




#line 17363 ".\\inc\\stm32f439xx.h"
























	 
	 
	 
	 
	 
	 
	 
	 


	 


	

 

	

 

	

 









 
#line 9 "src\\gpioControl.c"
#line 1 ".\\inc\\gpioControl.h"





 





#line 13 ".\\inc\\gpioControl.h"



typedef enum {GPIO_A = 0, GPIO_B, GPIO_C,	GPIO_D,	GPIO_E,	GPIO_F,	GPIO_G, GPIO_H, GPIO_I} GPIOPort;
typedef enum {Pin0 = 0, Pin1, Pin2, Pin3, Pin4, Pin5, Pin6, Pin7, Pin8, Pin9, Pin10, Pin11, Pin12, Pin13, Pin14, Pin15}GPIOPin;
typedef enum {GPIO_Input = 0, GPIO_Output, GPIO_AF, GPIO_Analog}GPIOMode;																													
typedef enum {GPIO_No_Pull = 0, GPIO_Pull_Up, GPIO_Pull_Down, GPIO_Reserved}GPIOPullUpDown;																				
typedef enum {GPIO_Output_PushPull = 0, GPIO_Output_OpenDrain} GPIOOutputType;																										
typedef enum {GPIO_2MHz = 0, GPIO_25MHz, GPIO_50MHz, GPIO_100MHz }GPIOSpeed;																											










		





	

#line 56 ".\\inc\\gpioControl.h"


#line 74 ".\\inc\\gpioControl.h"

typedef struct
{
		GPIO_TypeDef * port;							
		GPIOPin  pin;										
		GPIOMode mode;									
		GPIOPullUpDown pullUpDown;			
		GPIOOutputType outputType;			
		GPIOSpeed speed;								
	
} GPIO_Config;

			
		void gpio_configureGPIO(GPIO_Config *);										
		uint8_t gpio_getPinValue(GPIO_TypeDef *, GPIOPin);					

		void gpio_setGPIO(GPIO_TypeDef *, GPIOPin);								
		void gpio_resetGPIO(GPIO_TypeDef *, GPIOPin);							
			

#line 10 "src\\gpioControl.c"











void gpio_configureGPIO(GPIO_Config * inputGPIO)
{
	
		
	
	inputGPIO->port->MODER &= ~(0x03 << (inputGPIO->pin * 2));				
	inputGPIO->port->OTYPER &= ~(0x01 << inputGPIO->pin);						
	inputGPIO->port->OSPEEDR &= ~(0x03 << (inputGPIO->pin * 2));			
	inputGPIO->port->PUPDR &= ~(0x03 << (inputGPIO->pin * 2));				
	
	
	inputGPIO->port->MODER |= inputGPIO->mode << (inputGPIO->pin * 2);			
	inputGPIO->port->OTYPER |= inputGPIO->outputType << inputGPIO->pin;						
	inputGPIO->port->OSPEEDR |= inputGPIO->speed << (inputGPIO->pin * 2);					
	inputGPIO->port->PUPDR |= inputGPIO->pullUpDown << (inputGPIO->pin * 2);				
}








void gpio_setGPIO(GPIO_TypeDef * inputPort, GPIOPin inputPin)
{
	
	inputPort->BSRR |= 0x01 << inputPin;

}







void gpio_resetGPIO(GPIO_TypeDef * inputPort, GPIOPin inputPin)
{	
	
	inputPort->BSRR |= 0x01 << (inputPin + 16);
}








uint8_t gpio_getPinValue(GPIO_TypeDef * inputPort, GPIOPin inputPin)
{
	uint8_t bitValue = 0;
	
	bitValue = (inputPort->IDR >> inputPin) & 0x00000001;
	
	return bitValue;

}



