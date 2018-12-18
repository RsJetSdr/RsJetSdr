/*
 * Time.c
 *
 *  Created on: 17 àïð. 2016 ã.
 *      Author: IvanHome
 */

#include "Time.h"




//#define TEST_TIC


//volatile uint32_t TicTimer=0;
uint32_t __attribute__((optimize("O0"))) GetSysTime(void){
/*
	uint32_t Time = (GPIO->lpp_gpio_pin[GPIO_TIMER%8].status &
	            ~CY_U3P_LPP_GPIO_INTR);
	  GPIO->lpp_gpio_pin[GPIO_TIMER%8].status = (Time |
	            (CY_U3P_GPIO_MODE_SAMPLE_NOW << CY_U3P_LPP_GPIO_MODE_POS));
	    while (GPIO->lpp_gpio_pin[GPIO_TIMER%8].status & CY_U3P_LPP_GPIO_MODE_MASK);
	    Time = GPIO->lpp_gpio_pin[GPIO_TIMER%8].threshold;

*/
	uint32_t Time;
	CyU3PGpioComplexSampleNow(GPIO_TIMER, &Time);
	return Time;
}
//==========================================
void __attribute__((optimize("O0"))) Delay_Us(uint32_t Time_uS){
	Time_uS<<=1;
	if(!Time_uS)
				return;
	volatile uint32_t Start=GetSysTime();
	do{
	}while( (GetSysTime() - Start) < Time_uS);
}
//==========================================

/*
void GPIO_InterruptCallback(uint8_t gpioId)
{
#ifdef TEST_TIC
	 //Test Tic
	uvint32_t *regPtr = &GPIO->lpp_gpio_simple[GPIO_SHB];
	uint32_t regVal=(*regPtr & ~CY_U3P_LPP_GPIO_INTR);
	if(TicTimer & 1)
			regVal |= CY_U3P_LPP_GPIO_OUT_VALUE;
	else
		regVal &= ~CY_U3P_LPP_GPIO_OUT_VALUE;
	*regPtr = regVal;
	regVal = *regPtr;
#endif

	TicTimer++;
}
*/
//==============================================
CyU3PReturnStatus_t InitGpioClocks(void)
{
	//CyU3PReturnStatus_t Status;
    CyU3PGpioClock_t GpioClock;
    //416/2 = 208 Mhz (fastClkDiv * clkSrc)
  /*
    GpioClock.fastClkDiv = 2;                       // 2...16
    GpioClock.slowClkDiv = 0;
    GpioClock.simpleDiv = CY_U3P_GPIO_SIMPLE_DIV_BY_2;	// 2,4,16,64
    GpioClock.clkSrc = CY_U3P_SYS_CLK; 					// 1,2,4,16
    GpioClock.halfDiv = 0;
   */
    //416/(13*16) = 2 Mhz (fastClkDiv * clkSrc)

    GpioClock.fastClkDiv = 13;                           // 2...16
    GpioClock.slowClkDiv = 0;
    GpioClock.simpleDiv  = CY_U3P_GPIO_SIMPLE_DIV_BY_2;	// 0
    GpioClock.clkSrc    = CY_U3P_SYS_CLK_BY_16; 			//1,2,4,16
    GpioClock.halfDiv = 0;




    //Add Interrupt;
    return
    		//CyU3PGpioInit(&GpioClock,GPIO_InterruptCallback);
    		CyU3PGpioInit(&GpioClock,NULL);
    /*
     *
     * SYS_CLK_PLL is 403.2 MHz, a 0.1-µs unit timestamp needs the timer running at around 10 MHz frequency
     *
     * 1. Configure Fast GPIO Clock at approximately 10 MHz.
     *   Fast GPIO Clock is derived from SYS_CLK_PLL/4 with divider set as 10,
     *   so the frequency of Fast GPIO Clock is 10.08 MHz
     *
     *       CyU3PGpioClock_t gpioClock;
         //  Initialize the GPIO module
             gpioClock.fastClkDiv = 10;
             gpioClock.slowClkDiv = 0;
             gpioClock.simpleDiv = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
             gpioClock.clkSrc = CY_U3P_SYS_CLK_BY_4;
             gpioClock.halfDiv = 0;
             return CyU3PGpioInit(&gpioClock, NULL);
     */

}
CyU3PReturnStatus_t
Timer_uS_Init (uint8_t gpioId)
{
	CyU3PGpioComplexConfig_t gpioConfig;

//	tic_gpio_uS = glSysClkFreq/2000000UL;

 //apiRetStatus =
		 CyU3PGpioSetIoMode (50, CY_U3P_GPIO_IO_MODE_WPU);
// Configure GPIO 50
   gpioConfig.outValue    = CyFalse;
   gpioConfig.inputEn     = CyFalse;
   gpioConfig.driveLowEn  = CyFalse;
   gpioConfig.driveHighEn = CyFalse;
   gpioConfig.pinMode 	  = CY_U3P_GPIO_MODE_STATIC;  //CY_U3P_GPIO_MODE_TOGGLE; //CY_U3P_GPIO_MODE_PWM;
   gpioConfig.intrMode 	  = CY_U3P_GPIO_NO_INTR;
   	   	   	   	   	   	    //CY_U3P_GPIO_INTR_TIMER_THRES;
                            //CY_U3P_GPIO_INTR_TIMER_ZERO;
   gpioConfig.timerMode   =
		   	   	   	   	   CY_U3P_GPIO_TIMER_HIGH_FREQ;
   gpioConfig.timer       = 0x0;

/*
#define FREQ_IO_MHZ 2
   gpioConfig.period      = (50 * FREQ_IO_MHZ)-1; // 50 uS
   gpioConfig.threshold   = (50 * FREQ_IO_MHZ)-1;
*/
   gpioConfig.period      = -1; // 5 uS
   gpioConfig.threshold   = -1;



   return
		   CyU3PGpioSetComplexConfig(gpioId, &gpioConfig);
   /*
   gpioComplexConfig.outValue = CyFalse;
          gpioComplexConfig.inputEn = CyFalse;
          gpioComplexConfig.driveLowEn = CyTrue;
          gpioComplexConfig.driveHighEn = CyTrue;
          gpioComplexConfig.pinMode = CY_U3P_GPIO_MODE_STATIC;
          gpioComplexConfig.intrMode = CY_U3P_GPIO_INTR_TIMER_ZERO;
          gpioComplexConfig.timerMode = CY_U3P_GPIO_TIMER_HIGH_FREQ;
          gpioComplexConfig.timer = 0;
          gpioComplexConfig.period = 0xffffffff;
          gpioComplexConfig.threshold = 0xffffffff;
          status = CyU3PGpioSetComplexConfig(DUMMY_COMPLEX_GPIO, &gpioComplexConfig);

          uint32_t TimeStamp;
          CyU3PGpioComplexSampleNow(DUMMY_COMPLEX_GPIO, &TimeStamp);
*/


}

/*
 *
 */
