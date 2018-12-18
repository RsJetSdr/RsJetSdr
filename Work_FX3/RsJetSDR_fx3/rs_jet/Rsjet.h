/*
 * Rsjet.h
 *
 *  Created on: 27 сент. 2018 г.
 *      Author: Piton
 */

#ifndef RSJET_H_
#define RSJET_H_

#include "stdint.h"
#include "stddef.h"
#include "cyu3system.h"
#include "cyu3tx.h"
#include "cyu3uart.h"
#include "cyu3usb.h"
#include "cyu3error.h"
#include "constant.h"
#include "cyu3utils.h"
#include "main.h"
#include "Time.h"


#define MB_RAW_CTRL         1
#define MODE_STOP           (0<<13)
#define MODE_RAW            (1<<13)
#define MODE_BURST          (2<<13)

#define MAX_BUFF  256


typedef enum mUartBaudrate_t
{
 UART_DEFAULT =  115200UL,
 UART_ARM_REC = 1000000UL,
 UART_DCS_REC = 1083333UL
}mUartBaudrate_t;

typedef struct
{
	mUartBaudrate_t divisor;
	uint8_t dataBaud[5];
}Baud_struct;

typedef struct
{
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t Len;
	uint8_t inData[MAX_BUFF];
}Command_Task;
//============================================
typedef struct
{
  uint8_t  mode;
  uint8_t  atn1;
  uint8_t  atn2;
  uint8_t  atn3;
}Rec_Atn,*pRec_Atn;
//============================================
typedef struct
{
	uint16_t Size;
	uint16_t StartFreq;
	uint16_t StopFreq;
	Rec_Atn Atn;
}Task_SpectrHigh,*pTask_SpectrHigh;

typedef struct
{
  uint16_t Freq;
  uint8_t  mode;
  uint8_t attn[3];
}Freq_Attn,*pFreq_Attn;

#define  UARTSIZE 32

typedef struct {
	uint32_t f_Rx;
	uint32_t f_Tx;
	uint32_t size;
	uint8_t dataIn[UARTSIZE];
}Uart_Status,*pUart_Status;

extern volatile Uart_Status mUart_Status;

#define MAX_TASK  4
void 	Command_Thread_Task (uint32_t input);
extern  CyU3PEvent Command_AppEvent;
extern  CyU3PThread Command_Task_Thread;
extern Command_Task command_Host[MAX_TASK];
extern volatile uint32_t pnt_HeadCmd,pnt_TailCmd;

CyBool_t  Out_CmdFPGA(Command_Task *pcmd);

void Spectr_High(Command_Task *pcmd);

CyU3PReturnStatus_t OutToRecev(uint8_t *data,uint32_t rxSize);

CyU3PReturnStatus_t CyFxUartLpApplnInit (void);

uint32_t InputFromRecev(uint32_t Time_Out);

CyU3PReturnStatus_t Set_Freq_Attn(Freq_Attn *freq_attn);

//void CheckStatus(char* StringPtr, CyU3PReturnStatus_t Status);
void CheckStatus(CyU3PReturnStatus_t Status);

uint32_t TestSpeedUart(uint32_t baudRate);

#endif /* RSJET_H_ */

