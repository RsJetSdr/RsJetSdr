/* ###################################################################
**     Filename    : Events.h
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
** @file Events.h
** @version 01.00
** @brief
**         This is user's event module.
**         Put your event handler code here.
*/         
/*!
**  @addtogroup Events_module Events module documentation
**  @{
*/         

#ifndef __Events_H
#define __Events_H
/* MODULE Events */

#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "UART_Control.h"
#include "DMAT1.h"
#include "DMA1.h"
#include "IFsh1.h"
#include "IntFlashLdd1.h"
#include "SW_IF.h"
#include "SW_PRESEL.h"
#include "OUT_RFC.h"
#include "IN_RFC.h"
#include "CS_ATTN1.h"
#include "CS_ATTN2.h"
#include "CS_LO1.h"
#include "CS_LO2.h"
#include "LD1.h"
#include "LD2.h"
#include "PWM1.h"
#include "PwmLdd1.h"
#include "TU1.h"
#include "SPI_CLK.h"
#include "MOSI.h"
#include "AS1.h"
#include "ASerialLdd1.h"

#ifdef __cplusplus
extern "C" {
#endif 
extern uint8_t InBuffer[];
extern volatile uint8_t pntTail;
extern volatile uint8_t pIn;
extern volatile uint32_t AdjustValue;
void Delay1mS(uint32_t value);
void Delay1uS(uint32_t value);
void Init_Serial(word speed);
//#define MAX_INDX 82
#define MAX_INDX 169


typedef struct
{
	word freq;
	byte kluch;
	byte att1;
	byte att2;
	byte att3;
} FREQSTRUCT;
//=====================================
typedef struct
{
	word  freq_pw;
	word  size;
	FREQSTRUCT freq[MAX_INDX];
} FLASHRAMDATA;

//=====================================
typedef struct
{
	word freq;
	byte att1;
	byte att2;
	byte att3;
}RecData,*pRecData;

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
void AS1_OnError(void);

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
void AS1_OnRxChar(void);

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
void AS1_OnTxChar(void);

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
void AS1_OnFreeTxBuf(void);

/* END Events */

#ifdef __cplusplus
}  /* extern "C" */
#endif 

#endif 
/* ifndef __Events_H*/
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
