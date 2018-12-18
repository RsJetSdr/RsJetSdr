#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyu3usb.h"
#include "cyu3gpio.h"
#include "cyu3spi.h"
#include "cyu3uart.h"
#include "main.h"
#include "cyu3gpif.h"
#include "cyu3pib.h"
#include "pib_regs.h"
#include "cyu3utils.h"
#include "fpga.h"
#include "RS_JET\rsjet.h"
#include <spi_regs.h>
#include <uart_regs.h>
#include "Time.h"
extern CyU3PReturnStatus_t InitGpioClocks(void);
void SlFifoAppThread_Entry (uint32_t input);

#define SERIAL 1
/* This file should be included only once as it contains
 * structure definitions. Including it in multiple places
 * can result in linker error. */
//#include "cyfxgpif_syncsf.h"
#include "RsJetGPIF.h"
/* Firmware ID variable that may be used to verify SPI firmware. */
//const uint8_t glFirmwareID[32] __attribute__ ((aligned (32))) = { 'F', 'X', '3', ' ', 'S', 'P', 'I', '\0' };
//D_PR_USB30_V1.rbf
//const uint8_t glFirmwareID[32] __attribute__ ((aligned (32))) = {
//		'D','_','P','R','_','U','S','B','3','0','_','V','1','.','R','B','F','\0' };

const uint8_t glFirmwareID[32] __attribute__ ((aligned (32))) = {
		"rs_fx3_5.rbf\0"
	};

CyU3PThread slFifoAppThread;	        /* Slave FIFO application thread structure */
//CyU3PDmaChannel glChHandleSlFifoUtoP;   /* DMA Channel handle for U2P transfer. */
CyU3PDmaChannel glChHandleSlFifoPtoU;    /* DMA Channel handle for P2U transfer. */

uint32_t glDMARxCount = 0;               /* Counter to track the number of buffers received from USB. */
uint32_t glDMATxCount = 0;               /* Counter to track the number of buffers sent to USB. */
volatile CyBool_t glIsApplnActive = CyFalse;      /* Whether the loopback application is active or not. */
uint8_t glEp0Buffer[512];                 /* Buffer used for sending EP0 data.    */

uint8_t seqnum_p;
uvint32_t *regPtrDebug;

/* Application Error Handler */
void
CyFxAppErrorHandler (
        CyU3PReturnStatus_t apiRetStatus    /* API return status */
        )
{
	// Need a system timer to debounce the pushbutton
	//    Status = tx_timer_create(&DebounceTimer, "DebounceTimer", DebounceTimerExpired, 0, DEBOUNCE_TIME, DEBOUNCE_TIME, TX_AUTO_ACTIVATE);
	CyU3PBusyWait(1); 		//micro
	CyFx3BusyWait(1);

	/* Loop Indefinitely */
    for (;;)
    {
        /* Thread sleep : 100 ms */
        CyU3PThreadSleep (100);
    }
}
//================================================================
CyU3PReturnStatus_t CyFxSpiTransfer (
        uint16_t  wIndex,   //adr read
        uint16_t  wLength,
        uint8_t  *buffer,
        CyBool_t  isRead)
{

	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
	uint8_t adr=(uint8_t)wIndex;
    if (wLength == 0)
    {
        return CY_U3P_SUCCESS;
    }
    if(isRead){
    	 CyU3PGpioSetValue(GPIO_nCS, CyFalse);
    	 	  CyU3PSpiTransmitWords(&adr,1);
    	 	 status = CyU3PSpiReceiveWords(buffer,wLength);
    	 CyU3PGpioSetValue(GPIO_nCS, CyTrue);
    }
    else{
    	   CyU3PGpioSetValue(GPIO_nCS, CyFalse);
            	status = CyU3PSpiTransmitWords (buffer,wLength);
           	CyU3PGpioSetValue(GPIO_nCS, CyTrue);
    }
    return status;
}

/* This function starts the slave FIFO loop application. This is called
 * when a SET_CONF event is received from the USB host. The endpoints
 * are configured and the DMA pipe is setup in this function. */
void
CyFxSlFifoApplnStart (void)
{
    uint16_t size = 0;
    CyU3PEpConfig_t epCfg;
    CyU3PDmaChannelConfig_t dmaCfg;
  //  CyU3PReturnStatus_t apiRetStatus ;
    CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed();

    /* First identify the usb speed. Once that is identified,
     * create a DMA channel and start the transfer on this. */

    /* Based on the Bus Speed configure the endpoint packet size */
    switch (usbSpeed)
    {
        case      CY_U3P_FULL_SPEED:   size = 64;  break;
        case 	  CY_U3P_HIGH_SPEED:   size = 512;  break;
        case     CY_U3P_SUPER_SPEED:  size = 1024; break;
        default: CyFxAppErrorHandler (CY_U3P_ERROR_FAILURE);   break;
    }

    CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));

    epCfg.enable = CyTrue;
    epCfg.epType = CY_U3P_USB_EP_BULK;
    epCfg.burstLen = (usbSpeed == CY_U3P_SUPER_SPEED ? CY_FX_EP_BURST_LENGTH : 1);
    epCfg.streams = 0;
    epCfg.pcktSize = size;

    /* Producer endpoint configuration */
    CyU3PSetEpConfig(CY_FX_EP_PRODUCER, &epCfg);
    CyU3PUsbSetEpSeqNum (CY_FX_EP_PRODUCER, seqnum_p);
    CyU3PSetEpConfig(CY_FX_EP_CONSUMER, &epCfg);

       /* Create a DMA AUTO channel for P2U transfer. */
       dmaCfg.size      =  16384; //2048(sample) * 4 Byte (2 bloks)
    		       // size*8;  //increase buffer size for higher performance    !!!!!!!
       dmaCfg.count     =  CY_FX_FIFO_DMA_BUF_COUNT;       // increase buffer count for higher performance
       dmaCfg.prodSckId =  CY_FX_PRODUCER_PPORT_SOCKET;
       dmaCfg.consSckId =  CY_FX_CONSUMER_USB_SOCKET;
       dmaCfg.dmaMode =    CY_U3P_DMA_MODE_BYTE;
       /* Enabling the callback for produce event. */
       dmaCfg.notification = 0;
       dmaCfg.cb = NULL;
       dmaCfg.prodHeader = 0;
       dmaCfg.prodFooter = 0;
       dmaCfg.consHeader = 0;
       dmaCfg.prodAvailCount = 0;

   //  apiRetStatus =
     CyU3PDmaChannelCreate (&glChHandleSlFifoPtoU,
               	   	   	   	   	   	   	   CY_U3P_DMA_TYPE_AUTO,
               	   	   	   	   	   	   	   &dmaCfg);
     /* Flush the Endpoint memory */
    CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);
 	CyU3PDmaChannelSetXfer (&glChHandleSlFifoPtoU, CY_FX_SLFIFO_DMA_RX_SIZE);
    /* Update the status flag. */
    glIsApplnActive = CyTrue;
}
//===================================================================================
/* This function stops the slave FIFO loop application. This shall be called
 * whenever a RESET or DISCONNECT event is received from the USB host. The
 * endpoints are disabled and the DMA pipe is destroyed by this function. */
void CyFxSlFifoApplnStop (void)
{
    CyU3PEpConfig_t epCfg;


    /* Flush the endpoint memory */
    CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);
    /* Destroy the channel */
    CyU3PDmaChannelDestroy (&glChHandleSlFifoPtoU);
    /* Disable endpoints. */
    CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
    epCfg.enable = CyFalse;
    /* Producer endpoint configuration. */
 	CyU3PSetEpConfig(CY_FX_EP_PRODUCER, &epCfg);
    /* Consumer endpoint configuration. */
 	CyU3PSetEpConfig(CY_FX_EP_CONSUMER, &epCfg);

  	glIsApplnActive = CyFalse; /* Update the flag. */

}
//=========================================================================
/* Callback to handle the USB setup requests. */
CyBool_t CyFxSlFifoApplnUSBSetupCB (uint32_t setupdat0, uint32_t setupdat1 )
{
    /* Fast enumeration is used. Only requests addressed to the interface, class,
     * vendor and unknown control requests are received by this function.
     * This application does not support any class or vendor requests. */
	uint8_t  bRequest, bReqType;
    uint8_t  bType, bTarget;
  //  uint8_t  *p_out=NULL; //pointer UART data Rx to Host
    uint16_t wValue, wIndex, wLength;
    CyBool_t isHandled = CyTrue;

   // CyU3PReturnStatus_t Status;

    union {     uint32_t SetupData[2];
    			uint8_t RawBytes[8];
    			struct { uint8_t Target:5;
    					 uint8_t Type:2;
    					 uint8_t Direction:1;
    				 	 uint8_t Request;
    				 	 uint16_t Value;
    				 	 uint16_t Index;
    				 	 uint16_t Length;
    			};
     } Setup;

     // Copy the incoming Setup Packet into my Setup union which will "unpack" the variables
     	Setup.SetupData[0] = setupdat0;
     	Setup.SetupData[1] = setupdat1;

    /* Decode the fields from the setup request. */

    bReqType = (setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
    bType    = (bReqType & CY_U3P_USB_TYPE_MASK);
    bTarget  = (bReqType & CY_U3P_USB_TARGET_MASK);
    bRequest = ((setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
    wValue   = ((setupdat0 & CY_U3P_USB_VALUE_MASK)   >> CY_U3P_USB_VALUE_POS);
    wIndex   = ((setupdat1 & CY_U3P_USB_INDEX_MASK)   >> CY_U3P_USB_INDEX_POS);
    wLength  = ((setupdat1 & CY_U3P_USB_LENGTH_MASK)  >> CY_U3P_USB_LENGTH_POS);

    if (bType == CY_U3P_USB_STANDARD_RQT)
    {
        /* Handle SET_FEATURE(FUNCTION_SUSPEND) and CLEAR_FEATURE(FUNCTION_SUSPEND)
         * requests here. It should be allowed to pass if the device is in configured
         * state and failed otherwise. */
        if ((bTarget == CY_U3P_USB_TARGET_INTF) && ((bRequest == CY_U3P_USB_SC_SET_FEATURE)
                    || (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)) && (wValue == 0))
        {
            if (glIsApplnActive)   CyU3PUsbAckSetup ();
            else      CyU3PUsbStall (0, CyTrue, CyFalse);

        }

        /* CLEAR_FEATURE request for endpoint is always passed to the setup callback
         * regardless of the enumeration model used. When a clear feature is received,
         * the previous transfer has to be flushed and cleaned up. This is done at the
         * protocol level. Since this is just a loopback operation, there is no higher
         * level protocol. So flush the EP memory and reset the DMA channel associated
         * with it. If there are more than one EP associated with the channel reset both
         * the EPs. The endpoint stall and toggle / sequence number is also expected to be
         * reset. Return CyFalse to make the library clear the stall and reset the endpoint
         * toggle. Or invoke the CyU3PUsbStall (ep, CyFalse, CyTrue) and return CyTrue.
         * Here we are clearing the stall. */

        if ((bTarget == CY_U3P_USB_TARGET_ENDPT) && (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)
                && (wValue == CY_U3P_USBX_FS_EP_HALT))
        {
            if (glIsApplnActive)
            {
                if (wIndex == CY_FX_EP_PRODUCER)
                {
              //    CyU3PDmaChannelReset (&glChHandleSlFifoUtoP);
                    CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
                    CyU3PUsbResetEp (CY_FX_EP_PRODUCER);
               //  CyU3PDmaChannelSetXfer (&glChHandleSlFifoUtoP, CY_FX_SLFIFO_DMA_TX_SIZE);
                }

                if (wIndex == CY_FX_EP_CONSUMER)//to
                {
                    CyU3PDmaChannelReset (&glChHandleSlFifoPtoU);
                    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);
                    CyU3PUsbResetEp (CY_FX_EP_CONSUMER);
                    CyU3PDmaChannelSetXfer (&glChHandleSlFifoPtoU, CY_FX_SLFIFO_DMA_RX_SIZE);
                }

                CyU3PUsbStall (wIndex, CyFalse, CyTrue);
            }
        }
    }
    else  // ==========     VENDOR ===============
   		if (bType == CY_U3P_USB_VENDOR_RQT){
   	   	    isHandled = CyTrue;
   			switch(bRequest){

   			case VND_CMD_ID_CHECK:
                CyU3PUsbSendEP0Data (sizeof(glFirmwareID), (uint8_t *)glFirmwareID);
             break;

			case VND_CMD_CFGLOAD:
       	    	CyU3PUsbGetEP0Data (wLength, glEp0Buffer, NULL);
				if(!glConfigDone){
       	    	filelen = (uint32_t)(glEp0Buffer[3]<<24)|(glEp0Buffer[2]<<16)|(glEp0Buffer[1]<<8)|glEp0Buffer[0];
       	    	glConfigDone = CyFalse;
       	      	/* Set CONFIGFPGAAPP_START_EVENT to start configuring FPGA */
				CyU3PEventSet(&glFxConfigFpgaAppEvent,	START_EVENT,CYU3P_EVENT_OR);
				}
			break;

			case VND_CMD_CFGSTAT:

				glEp0Buffer[0]= glConfigDone;

				CyU3PGpioSimpleGetValue(GPIO_CONFDONE,(CyBool_t *)&glEp0Buffer[1]);
				{
				uint32_t	TicTimer =GetSysTime();
				glEp0Buffer[2]=TicTimer>>24;
				glEp0Buffer[3]=TicTimer>>16;
				glEp0Buffer[4]=TicTimer>>8;
				glEp0Buffer[5]=TicTimer;
				}
		   		CyU3PUsbSendEP0Data (wLength, glEp0Buffer);

				/* Switch to slaveFIFO interface when FPGA is configured successfully*/
		   		if (glConfigDone)
		   		CyU3PEventSet(&glFxConfigFpgaAppEvent,SW_TO_SLFIFO_EVENT,CYU3P_EVENT_OR);

			break;

		    case VND_CMD_SPI_WRITE:
		    	CyU3PUsbGetEP0Data (wLength, glEp0Buffer, NULL);
            	 	 /*
            	 	 	 CyU3PEventSet(&glFxConfigFpgaAppEvent,
			             SPI_WRITE_EVENT,
			             CYU3P_EVENT_OR);
					*/
               	CyFxSpiTransfer(wIndex,wLength,glEp0Buffer, CyFalse);
            break;

		    case VND_CMD_SPI_READ:
	                 CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
                      CyFxSpiTransfer (wIndex, wLength,
		                           glEp0Buffer, CyTrue);
		             CyU3PUsbSendEP0Data (wLength, glEp0Buffer);
		    break;
		    case VND_CMD_COMMAND:
		    	command_Host[pnt_HeadCmd].wValue	=	wValue;
		    	command_Host[pnt_HeadCmd].wIndex	=	wIndex;
		    	command_Host[pnt_HeadCmd].Len		=	wLength;
		    	if(wLength)
		    		CyU3PUsbGetEP0Data (wLength, (uint8_t *)&command_Host[pnt_HeadCmd++].inData, NULL);

		    	pnt_HeadCmd &=	(MAX_TASK-1);
		    	CyU3PEventSet(&glFxConfigFpgaAppEvent,COMMAND_EVENT,CYU3P_EVENT_OR);
		    break;
	#if(SERIAL == 1)

		case 0xAE:	       				// Out Serial
			//0 Host to Device
			//1 Device to Host
			isHandled = CyTrue;
			if(Setup.Direction)
			{

				wLength = InputFromRecev(wIndex); 				//wIndex = TimeOut
				CyU3PUsbSendEP0Data(wLength,(uint8_t *)&mUart_Status.dataIn);
			}else
			if(wLength){
				CyU3PUsbGetEP0Data (wLength, glEp0Buffer, NULL);
				if(OutToRecev(glEp0Buffer,wIndex))
							isHandled = CyFalse; //Error to Out
			   }
		  break;
	#endif
			default:
						isHandled = CyFalse;
			break;
            }//switch(bRequest)
   		}
   		return isHandled;
 }

//===========================================================================
/* This is the callback function to handle the USB events. */
void CyFxSlFifoApplnUSBEventCB ( CyU3PUsbEventType_t evtype, uint16_t  evdata )
{
    switch (evtype)
    {
        case CY_U3P_USB_EVENT_SETCONF:
            /* Stop the application before re-starting. */
            if (glIsApplnActive)
            {
                CyFxSlFifoApplnStop ();

            }
            /* Start the loop back function. */
            CyU3PUsbLPMDisable();
            CyFxConfigFpgaApplnStart();
            break;

        case CY_U3P_USB_EVENT_RESET:
        case CY_U3P_USB_EVENT_DISCONNECT:
            /* Stop the loop back function. */
            if (glIsApplnActive)
            {
                CyFxSlFifoApplnStop ();
            }
            break;

        default:
            break;
    }
}

/* Callback function to handle LPM requests from the USB 3.0 host. This function is invoked by the API
   whenever a state change from U0 -> U1 or U0 -> U2 happens. If we return CyTrue from this function, the
   FX3 device is retained in the low power state. If we return CyFalse, the FX3 device immediately tries
   to trigger an exit back to U0.

   This application does not have any state in which we should not allow U1/U2 transitions; and therefore
   the function always return CyTrue.
 */
CyBool_t
CyFxApplnLPMRqtCB (
        CyU3PUsbLinkPowerMode link_mode)
{
    return CyTrue;
}

//=========================================================================================
/* This function initializes the GPIF interface and initializes
 * the USB interface. */
void
CyFxSlFifoApplnInit (void)
{
    CyU3PPibClock_t pibClock;
    CyU3PSpiConfig_t spiConfig;

   // CyU3PReturnStatus_t apiRetStatus ;

    /* Start the SPI module and configure the master. */
   // 	apiRetStatus =
    			CyU3PSpiInit();
      /* Start the SPI master block. Run the SPI clock at 25MHz
       * and configure the word length to 8 bits. Also configure
       * the slave select using FW. */
      CyU3PMemSet ((uint8_t *)&spiConfig, 0, sizeof(spiConfig));
      spiConfig.isLsbFirst = CyFalse;
      spiConfig.cpol       = CyFalse;
      spiConfig.ssnPol     = CyFalse;
      spiConfig.cpha       = CyFalse;
      spiConfig.leadTime   = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
      spiConfig.lagTime    = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
      spiConfig.ssnCtrl    = CY_U3P_SPI_SSN_CTRL_FW;
      spiConfig.clock      = 20000000;
      spiConfig.wordLen    = 8;
     // apiRetStatus =
    		  CyU3PSpiSetConfig (&spiConfig, NULL);

     /* Initialize the p-port block. */
    pibClock.clkDiv = 2;
    pibClock.clkSrc = CY_U3P_SYS_CLK;
    pibClock.isHalfDiv = CyFalse;
    /* Disable DLL for sync GPIF */
    pibClock.isDllEnable = CyFalse;
    //apiRetStatus =
    		CyU3PPibInit(CyTrue, &pibClock);

    /* Load the GPIF configuration for Slave FIFO sync mode. */
    //apiRetStatus =
    		CyU3PGpifLoad (&CyFxGpifConfig);
    CyU3PGpifSocketConfigure (0,						//threadIndex
    						  CY_U3P_PIB_SOCKET_0,  	//socketNum
    						  WATERMARK,						//watermark
    						  CyFalse,					//flagOnData
    						  1							//burst
    						  );
    /* Start the state machine. */
    //apiRetStatus =
    CyU3PGpifSMStart (RESET,ALPHA_RESET);
}
//=================================================================================================
void CyFxSwitchtoslFifo (void)
{
	CyU3PIoMatrixConfig_t io_cfg;
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

	    io_cfg.useUart   = CyTrue;
	    io_cfg.useI2C    = CyFalse;
	    io_cfg.useI2S    = CyFalse;
	    io_cfg.useSpi    = CyTrue;
	    io_cfg.isDQ32Bit = CyFalse;
	    io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_DEFAULT;
	    /* GPIOs 23 are enabled. */
	    io_cfg.gpioSimpleEn[0]  = 0x00800000;  //GPI 23
	    io_cfg.gpioSimpleEn[1]  = 0x00000000;
	    io_cfg.gpioComplexEn[0] = 0;
	    io_cfg.gpioComplexEn[1] = 0x00040000;//pin 50
	    status = CyU3PDeviceConfigureIOMatrix (&io_cfg);
	    if (status != CY_U3P_SUCCESS)
	    {
	    	while (1);		// Cannot recover from this error.
	    }
}
/* Application define function which creates the threads. */
void
CyFxApplicationDefine (
        void)
{
    void *ptr = NULL;
    uint32_t retThrdCreate = CY_U3P_SUCCESS;
    /* Allocate the memory for the thread */
    ptr = CyU3PMemAlloc (CY_FX_SLFIFO_THREAD_STACK);
    /* Create the thread for the application */
    retThrdCreate = CyU3PThreadCreate (&slFifoAppThread,           /* Slave FIFO app thread structure */
                          "21:Slave_FIFO_sync",                    /* Thread ID and thread name */
                          SlFifoAppThread_Entry,                   /* Slave FIFO app thread entry function */
                          0,                                       /* No input parameter to thread */
                          ptr,                                     /* Pointer to the allocated thread stack */
                          CY_FX_SLFIFO_THREAD_STACK,               /* App Thread stack size */
                          CY_FX_SLFIFO_THREAD_PRIORITY,            /* App Thread priority */
                          CY_FX_SLFIFO_THREAD_PRIORITY,            /* App Thread pre-emption threshold */
                          CYU3P_NO_TIME_SLICE,                     /* No time slice for the application thread */
                          CYU3P_AUTO_START                         /* Start the thread immediately */
                          );

    /* Check the return code */
    if (retThrdCreate != 0)
    {
        /* Loop indefinitely */
        while(1);
    }
}

/* Entry function for the slFifoAppThread. */
void
SlFifoAppThread_Entry (
        uint32_t input)
{


	/******************/
	    /* Configure GPIO */
	    /******************/
	CyU3PReturnStatus_t apiRetStatus;

	CheckStatus(InitGpioClocks());
	CheckStatus( Timer_uS_Init(GPIO_TIMER));

//	apiRetStatus =  CyFxUartLpApplnInit();
//	CheckStatus("CyFxUartLpApplnInit", apiRetStatus);


	/* Initialize the FPGA configuration application */
    CyFxConfigFpgaApplnInit();
    CyBool_t first= CyTrue;
    uint32_t eventFlag;
    Command_Task *pCommand_Task;
/*
	//Debug
    regPtrDebug = &GPIO->lpp_gpio_simple[GPIO_SHB];
    uint32_t Timer=0;
    uvint32_t regVal=(*regPtrDebug & ~CY_U3P_LPP_GPIO_INTR);

    for(;;Timer++){
    	regVal ^= CY_U3P_LPP_GPIO_OUT_VALUE;
	    *regPtrDebug = regVal;
		  for(eventFlag=0;(eventFlag<_100us) ;eventFlag++)
		  {
				   if(pnt_TailCmd != pnt_HeadCmd)
					break;
		  }


	}

*/


/*
    //Test Tic
	uint32_t Timer=0;
	uvint32_t *regPtr = &GPIO->lpp_gpio_simple[GPIO_SHB];
	uvint32_t regVal;
	for(;;Timer++){
		Delay_Us(50);
	regVal=(*regPtr & ~CY_U3P_LPP_GPIO_INTR);
	if(Timer & 1)
			regVal |= CY_U3P_LPP_GPIO_OUT_VALUE;
	else
		regVal &= ~CY_U3P_LPP_GPIO_OUT_VALUE;
	*regPtr = regVal;
	regVal = *regPtr;
	}
*/




    for(;;){
   if(glIsApplnActive){
       	/* Wait for events to configure FPGA */
    	  if (!CyU3PEventGet(&glFxConfigFpgaAppEvent,
    		  (COMMAND_EVENT |
    		   START_EVENT |
    		   SW_TO_SLFIFO_EVENT),
    		   CYU3P_EVENT_OR_CLEAR,&eventFlag,
    		   //CYU3P_NO_WAIT)
    		   CYU3P_WAIT_FOREVER)
    			  )
      {
        if(eventFlag & START_EVENT){/* Start configuring FPGA */
        	            	 CyFxConfigFpga(filelen);

        	            	 apiRetStatus =  CyFxUartLpApplnInit();
        	            	 CheckStatus(apiRetStatus);


        } //============================================
        else if ((eventFlag & SW_TO_SLFIFO_EVENT)){ /* Switch to SlaveFIFO interface */
        	                if(first){
        	                	CyFxConfigFpgaApplnStop();
        	                	//CyFxSwitchtoslFifo();
        	                	CyFxSlFifoApplnInit(); //Init SPI and GPIF
        	                	CyFxSlFifoApplnStart();

        	                	first=CyFalse;
        	                }
        }//============================================
        else if(COMMAND_EVENT){
        	while(pnt_TailCmd != pnt_HeadCmd){
    			pCommand_Task = &command_Host[pnt_TailCmd++];
    			pnt_TailCmd &=(MAX_TASK-1);
        		switch(pCommand_Task->wValue){
        		case 0:
        			Out_CmdFPGA(pCommand_Task);
        		break;
        		case 6:
        			Spectr_High(pCommand_Task);
        		break;
        		}

        	}
        } //===========================================
        //else if(){} //===========================================
        //else if(){} //===========================================
        //else if(){} //===========================================
        //else if(){} //===========================================
      }//if Event
    }//if (glIsApplnActive)

}//for (;;)
}
//==========================================================================

/* [ ] */

