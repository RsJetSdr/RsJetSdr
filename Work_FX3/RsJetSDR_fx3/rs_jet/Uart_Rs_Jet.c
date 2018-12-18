/*
 * Uart_Rs_Jet.c
 *
 *  Created on: 11 апр. 2016 г.
 *      Author: IvanHome
 */

#include <cyu3error.h>
#include <cyu3gpio.h>
#include <cyu3usb.h>
#include <cyu3uart.h>
#include "Rsjet.h"
#include <uart_regs.h>
#include <cyu3lpp.h>
#include <gpio_regs.h>
//#include <gctl_regs.h>
#include "Time.h"





#define IndexUart 1

// --   Set Baud
 const Baud_struct mBaud_struct[3] =
{
 {UART_DEFAULT,{0x05,0x05,0x01,0x00,0xF5}},
 {UART_ARM_REC,{0x05,0x05,0x01,0x01,0xF4}},
 {UART_DCS_REC,{0x05,0x05,0x01,0x02,0xF3}}
};



/*
// -- Set freq,atn1...atn3
static uint8_t CmdFreqAtn[9]	={0x05,0x09,0x00,//cmd
		0x00,0x00,0x00,0x00,0x00,
		0x00};
// --   Set Baud
static uint8_t  CmdBaud[5] 		={0x05,0x05,0x01,//cmd
		0x00,
0x00};
// -- Set Internal/External
static uint8_t CmdSwitch[5]		={0x05,0x05,0x02,//cmd
			  0x01,
0x00};
// --  Set freq
static uint8_t CmdFreq[6] 		={0x05,0x06,0x06,//cmd
	 0x00,0x00,
0x00 };
// --  Get  Lock Detect
static uint8_t CmdLockDetect[4]	={0x05,0x04,0x08,//cmd
0x00};
// -- Set freq,atn1...atn3
static uint8_t CmdFreqAtn1[7] 	={0x05,0x07,0x0A,//cmd
	0x00,0x00,0x00,	//data
0x00};
// --  Set freq
static uint8_t CmdFreqNew[6]  	={0x05,0x06,14,  //cmd
	  0x00,0x00,
0x00};
// -- Set freq,atn1...atn3
static uint8_t CmdFreqAtn1New[7] = {0x05,0x07,15,//cmd
	  0x00,0x00,0x00,	//data
0x00};
// -- Set freq,atn1...atn3
static uint8_t CmdFreqAtnNew[9] ={0x05,0x09,16,//cmd
	 0x00,0x00,0x00,0x00,0x00,
0x00};
*/


volatile  CyU3PUartEvt_t tEvent;
volatile Uart_Status mUart_Status={0};
CyU3PDmaChannel glUartTxHandle;   /* SCI Tx channel handle */
CyU3PDmaChannel glUartRxHandle;   /* SCI Rx channel handle */
CyU3PDmaBuffer_t TxBuf;

void UartCallback(CyU3PUartEvt_t Event, CyU3PUartError_t Error)
// Handle characters typed in by the developer
{
	CyU3PDmaBuffer_t InDmaBuffer;
	//char InputChar;
	switch(Event){
	case CY_U3P_UART_EVENT_RX_DONE:
		//Wraps up the current active buffer for the channel from the producer side.
		CyU3PDmaChannelSetWrapUp(&glUartRxHandle);
		CyU3PDmaChannelGetBuffer(&glUartRxHandle, &InDmaBuffer, CYU3P_NO_WAIT);
		CyU3PMemCopy((uint8_t *)mUart_Status.dataIn,InDmaBuffer.buffer,InDmaBuffer.count);
		CyU3PDmaChannelDiscardBuffer(&glUartRxHandle);
		mUart_Status.size = InDmaBuffer.count;
		mUart_Status.f_Rx = CyTrue;

	break;
	case CY_U3P_UART_EVENT_TX_DONE:
		mUart_Status.f_Tx = CyTrue;
	break;
	default:
		tEvent=Event;
	}

}
//=================================================================================
CyU3PReturnStatus_t OutToRecev(uint8_t *data,uint32_t wait_echo_rx_size)
{
	uint8_t		CS=0;

	uint8_t		*pOut = TxBuf.buffer; //data
	TxBuf.count  =  data[1];         //size

	uint32_t size=(uint32_t)(data[1]-1);


	for(uint32_t i=0; i<size; i++){
		CS+= *data;
		*pOut++ = *data++;
	}

	*pOut++ =(uint8_t)(~CS+1);

	wait_echo_rx_size ? CyU3PUartRxSetBlockXfer(wait_echo_rx_size) : CyU3PUartRxSetBlockXfer(0xFFFFFFFFU);

	//Clear Uart_Status
	mUart_Status.f_Rx = CyFalse;
	mUart_Status.f_Tx = CyFalse;
	mUart_Status.size = 0;

	CyU3PUartTxSetBlockXfer(TxBuf.count);

	//Send an external buffer to the consumer.
	 return  CyU3PDmaChannelSetupSendBuffer (&glUartTxHandle,&TxBuf);

/*
    return CyU3PDmaChannelWaitForCompletion(&glUartTxHandle,3000);          //Time Out 3 mS
*/

}
//=====================================================
uint32_t __attribute__((optimize("O0"))) InputFromRecev(/*uint8_t **pbuf*/ uint32_t Time_Out)
{
  uvint32_t t_1;
  uvint32_t t_stop = Time_Out * 2000; //TIME_MS; //convert mS to uS

  if(!t_stop){
	  if(mUart_Status.f_Rx){
		   mUart_Status.f_Rx=0;
		   return mUart_Status.size;
	  }
	  else return 0;
  }

  t_1 = GetSysTime();
   do{
	  	if(mUart_Status.f_Rx){
	  	 //  buffer = (uint8_t *)&mUart_input.dataIn;
	  		mUart_Status.f_Rx=0;
	  	  return mUart_Status.size;
	  	}

   }while((GetSysTime() - t_1) < t_stop);

 return 0;
}


//------------------------------------------------------------
CyU3PReturnStatus_t Set_Freq_Attn(Freq_Attn *freq_attn)
{
	uint8_t cmdbuf[16];
    cmdbuf[0] = 0x05;
	cmdbuf[1] =   10;   // Size 10 * 10 uS = 100 uS
	cmdbuf[2] = 0x20;   // command
	CyU3PMemCopy(cmdbuf+3,(uint8_t *)freq_attn,sizeof(Freq_Attn));
	return OutToRecev(cmdbuf,4);
}

typedef struct {
	uint32_t DIV:16;
	uint32_t HALFDIV:1;
	uint32_t SRC:2;
	uint32_t EMPTY:12;
	uint32_t EN:1;
} uart_core_clk;

//========================================================================================
/* This function initializes the UART module */
CyU3PReturnStatus_t CyFxUartLpApplnInit (void)
{

	//volatile   uart_core_clk muart_core_clk;

	/* Initialize the UART module */
    CyU3PReturnStatus_t apiRetStatus = CyU3PUartInit ();
    CheckStatus(apiRetStatus);
// base clock
// 26 MHz FX3
// 52 MHz DCS
	/* Configure the UART:
       Baud-rate = 115200, Two stop bit, No parity, Flow control disable.
     */
	CyU3PUartConfig_t uartConfig;
    CyU3PMemSet ((uint8_t *)&uartConfig, 0, sizeof(uartConfig));
    uartConfig.baudRate = UART_DEFAULT;
    /*
     *  Todo STOP BIT !!!!!
     */
    uartConfig.stopBit =  CY_U3P_UART_TWO_STOP_BIT;//CY_U3P_UART_TWO_STOP_BIT;

    uartConfig.parity =   CY_U3P_UART_NO_PARITY;
    uartConfig.flowCtrl = CyFalse;
    uartConfig.txEnable = CyTrue;
    uartConfig.rxEnable = CyFalse; //Rx Disable
    uartConfig.isDma   =  CyFalse; /* Register mode */

    /* Set the UART configuration */
    apiRetStatus = CyU3PUartSetConfig (&uartConfig,NULL);
    CheckStatus(apiRetStatus);
    //uart_core_clk;                      /* 0xe0052024 */
    //    muart_core_clk = (*(uart_core_clk *)0xe0052024u);
//80 07 00E0
//div 225.5
    /* We set a timeout of 1 for receive data and 10000 for transmit data.
    * Then wait until the callback notifies us of incoming data.
    * Once all of the data has been fetched, we loop the data back on the transmit side.
    */

    CyU3PUartSetTimeout (1, 10000);

    // UART_TX_BREAK
     uint8_t tmp = 0;
     CyU3PUartTransmitBytes (&tmp, 1, &apiRetStatus); //BREAK

     CyU3PThreadSleep(10);
    /* Set new Baud Rate
     *
     *
     * Todo Check IndexUart !!!!!!!!!!!!!!
     *
     */
     CyU3PUartTransmitBytes ((uint8_t *)&mBaud_struct[IndexUart].dataBaud,5,&apiRetStatus);

    uartConfig.baudRate = mBaud_struct[IndexUart].divisor;
    uartConfig.rxEnable = CyTrue;
    uartConfig.isDma   =  CyTrue; /* DMA mode */

    //Set UART Callbac
    apiRetStatus = CyU3PUartSetConfig (&uartConfig,UartCallback);

    //uart_core_clk;                      /* 0xe0052024 */
    //  muart_core_clk = (*(uart_core_clk *)0xe0052024u);

    CheckStatus(apiRetStatus);
    //div 24
//80 06 00 17

    apiRetStatus = CyU3PUartTxSetBlockXfer(0xFFFFFFFF);					// Send as much data as I need to

    CheckStatus(apiRetStatus);

    /* Create the DMA channels for UART Tx. */
    CyU3PDmaChannelConfig_t dmaConfig;
    CyU3PMemSet((uint8_t *)&dmaConfig, 0, sizeof(dmaConfig));


    TxBuf.buffer 	= CyU3PDmaBufferAlloc(UARTSIZE);
    TxBuf.size   	= UARTSIZE;


    /* No buffers need to be allocated as this channel
     * will be used only in override mode. */
    dmaConfig.count          = 0;
    dmaConfig.prodAvailCount = 0;
    dmaConfig.size  		 = UARTSIZE;

    dmaConfig.dmaMode        = CY_U3P_DMA_MODE_BYTE;
    dmaConfig.prodHeader     = 0;
    dmaConfig.prodFooter     = 0;
    dmaConfig.consHeader     = 0;

    dmaConfig.notification   = CY_U3P_DMA_CB_PROD_EVENT;
    dmaConfig.prodSckId 	 = CY_U3P_CPU_SOCKET_PROD;        //CPU
    dmaConfig.consSckId 	 = CY_U3P_LPP_SOCKET_UART_CONS;  //to UART

    apiRetStatus = CyU3PDmaChannelCreate (
    				&glUartTxHandle,
              	    CY_U3P_DMA_TYPE_MANUAL_OUT,
              	    &dmaConfig);


    CheckStatus(apiRetStatus);


    /* Set UART Tx transfer Size to infinite */
    apiRetStatus = CyU3PUartTxSetBlockXfer(0xFFFFFFFFU);
        if(apiRetStatus != CY_U3P_SUCCESS) {
      	   	   	   return apiRetStatus;
           }
 //=======================================================================
  // Now setup a DMA channel to receive characters from the Uart Rx

        CheckStatus(CyU3PUartRxSetBlockXfer(1));

        CyU3PMemSet((uint8_t *)&dmaConfig, 0, sizeof(dmaConfig));
        	dmaConfig.size  		= UARTSIZE;			// Minimum size allowed, I only need 1 byte
        	dmaConfig.count 		= 1;		// I can't type faster than the Uart Callback routine!
        	dmaConfig.dmaMode 		= CY_U3P_DMA_MODE_BYTE;

        	dmaConfig.notification	= CY_U3P_DMA_CB_PROD_EVENT;
        	dmaConfig.prodSckId		= CY_U3P_LPP_SOCKET_UART_PROD; //UART
        	dmaConfig.consSckId 	= CY_U3P_CPU_SOCKET_CONS;      //to CPU

        	apiRetStatus = CyU3PDmaChannelCreate(&glUartRxHandle,
        	              CY_U3P_DMA_TYPE_MANUAL_IN, &dmaConfig);



            if (apiRetStatus != CY_U3P_SUCCESS){
            	        CyU3PDmaChannelDestroy(&glUartRxHandle);
            	        return apiRetStatus;
            }
            else
            {
            	apiRetStatus = CyU3PDmaChannelSetXfer(&glUartRxHandle, 0);
        		CheckStatus(apiRetStatus);
            }
//test

        Freq_Attn mFreq_Attn;
        mFreq_Attn.Freq=1000;
        mFreq_Attn.mode=0;
        mFreq_Attn.attn[0]=1;
        mFreq_Attn.attn[1]=2;
        mFreq_Attn.attn[2]=3;
        Set_Freq_Attn(&mFreq_Attn);
        InputFromRecev(10);

        return   apiRetStatus;
}

//====================================================
uint32_t TestSpeedUart(uint32_t baudRate)
{

	uint32_t clkFreq = glSysClkFreq;
	if (baudRate < CY_U3P_UART_BAUDRATE_600){
	            clkFreq = (glSysClkFreq >> 4);
	}

	/* UART core clock must be 16X the baud rate. */
	uint32_t    clkdiv = (clkFreq << 2) / (baudRate * 16);

	if ((clkdiv & 0x03) == 0){
	              clkdiv >>= 2;  /*  (x - floor(x)) < 0.25. */
	          }
	else
	if (((clkdiv & 0x03) == 1) || ((clkdiv & 0x03) == 2)){
	     /* (((x - floor(x)) >= 0.25) && (x - floor(x)) < 0.5) */
	              clkdiv >>= 2;
	            //  regVal |= CY_U3P_GCTL_UARTCLK_HALFDIV;
	}else /* ((clkdiv & 0x03) == 3) */
	          {
	              /* (x - floor(x)) >= 0.75. */
	              clkdiv = ((clkdiv >> 2)  + 1);
	}
	         // clkdiv--;
      return  clkdiv;


}
