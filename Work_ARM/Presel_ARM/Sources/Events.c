/* ###################################################################
**     Filename    : Events.c
**     Project     : Uart_Dma
**     Processor   : MK20DX128VLH5
**     Component   : Events
**     Version     : Driver 01.00
**     Compiler    : GNU C Compiler
**     Date/Time   : 2016-09-28, 13:35, # CodeGen: 0
**     Abstract    :
**         This is user's event module.
**         Put your event handler code here.
**     Settings    :
**     Contents    :
**         Cpu_OnNMIINT - void Cpu_OnNMIINT(void);
**
** ###################################################################*/
/*!
** @file Events.c
** @version 01.00
** @brief
**         This is user's event module.
**         Put your event handler code here.
*/         
/*!
**  @addtogroup Events_module Events module documentation
**  @{
*/         
/* MODULE Events */

#include "Cpu.h"
#include "Events.h"

#ifdef __cplusplus
extern "C" {
#endif 
uint8_t InBuffer[0x100] __attribute__ ((aligned(0x100)));
volatile uint8_t pntTail;
volatile uint8_t pIn;
volatile uint32_t AdjustValue;

/**
 * Delay for number of milliseconds using SysTick timer
 * @param[in] value The number of millisecond to delay
 * max 335 mS
 */
void Delay1mS(uint32_t value)
{
    if (value == 0) return;

    // Use the SysTick to generate the timeout in msecs
    SYST_RVR = value * (CPU_CORE_CLK_HZ / 1000);
    SYST_CVR  = 0;
    SYST_CSR = SysTick_CSR_CLKSOURCE_MASK | SysTick_CSR_ENABLE_MASK;

    while (!(SYST_CSR & SysTick_CSR_COUNTFLAG_MASK)) {
        ;
    }
}

/**
 * Delay for number of microseconds using SysTick timer
 * @param[in] value The number of microseconds to delay
 */
void Delay1uS(uint32_t value)
{
    //const uint32_t AdjustValue = 50;
    // Use the SysTick to generate the timeout in us
    SYST_RVR = value * (CPU_CORE_CLK_HZ / 1000000) - AdjustValue;
    SYST_CVR  = 0;
    SYST_CSR = SysTick_CSR_CLKSOURCE_MASK | SysTick_CSR_ENABLE_MASK;

    while (!(SYST_CSR & SysTick_CSR_COUNTFLAG_MASK)) {
        ;
    }
}

PE_ISR(isrUartCmd){
	/* Read status register */
		register uint32_t StatReg = UART0_S1;
		/* Is any error flag set? */
		if (StatReg & (
						 UART_S1_NF_MASK | //Noise
						 UART_S1_OR_MASK | //Overrun
						 UART_S1_FE_MASK | //Framing
						 UART_S1_PF_MASK | //Parity
						 0
						))
		{
			(void) UART0_D;  /* Dummy read of the UART0_S1 register to clear flags */

   			pIn=0; //Wait 0x05
   			Init_Serial(0);
			/* Todo
			 * Set Default Baud Rate
			 */

		}

}
/* User includes (#include below this line is not maintained by Processor Expert) */

/*
** ===================================================================
**     Event       :  AS1_OnError (module Events)
**
**     Component   :  AS1 [AsynchroSerial]
**     Description :
**         This event is called when a channel error (not the error
**         returned by a given method) occurs. The errors can be read
**         using <GetError> method.
**         The event is available only when the <Interrupt
**         service/event> property is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void AS1_OnError(void)
{
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  AS1_OnRxChar (module Events)
**
**     Component   :  AS1 [AsynchroSerial]
**     Description :
**         This event is called after a correct character is received.
**         The event is available only when the <Interrupt
**         service/event> property is enabled and either the <Receiver>
**         property is enabled or the <SCI output mode> property (if
**         supported) is set to Single-wire mode.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void AS1_OnRxChar(void)
{
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  AS1_OnTxChar (module Events)
**
**     Component   :  AS1 [AsynchroSerial]
**     Description :
**         This event is called after a character is transmitted.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void AS1_OnTxChar(void)
{
  /* Write your code here ... */
}

/*
** ===================================================================
**     Event       :  AS1_OnFreeTxBuf (module Events)
**
**     Component   :  AS1 [AsynchroSerial]
**     Description :
**         This event is called after the last character in output
**         buffer is transmitted.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
void AS1_OnFreeTxBuf(void)
{
  /* Write your code here ... */
}

/* END Events */

#ifdef __cplusplus
}  /* extern "C" */
#endif 

/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.3 [05.09]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
