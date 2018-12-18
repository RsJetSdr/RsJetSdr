/*
 * cyfxconfigfpga.c
 *
 *  Created on: Nov 26, 2012
 *      Author: rskv
 */

#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyu3usb.h"
#include "cyu3gpio.h"
#include "cyu3spi.h"
#include "cyu3uart.h"
#include "cyu3utils.h"
#include "main.h"
#include "cyu3gpif.h"
#include "cyu3pib.h"
#include "pib_regs.h"
#include "fpga.h"
#include "constant.h"


/* This file illustrates the configure FPGA in Slave Serial mode example */


CyU3PDmaChannel glChHandleUtoCPU;     /* DMA Channel handle for U2CPU transfer. */
CyBool_t glConfigDone = CyTrue;		/* Flag to indicate that FPGA configuration is done */
volatile uint32_t filelen = 0;				/* length of Configuration file (.bin) */
CyU3PEvent glFxConfigFpgaAppEvent;   /* Configure FPGA event group. */

volatile uint16_t uiPacketSize = 0;
volatile uint16_t GlbError;
//USB serial number from FX3 die id
static const char hex_digit[16] = "0123456789ABCDEF";
static uint32_t *EFUSE_DIE_ID = ((uint32_t *)0xE0055010);
uint32_t die_id[2];

void
gpif_error_cb(CyU3PPibIntrType cbType, uint16_t cbArg)
{

if(cbType==CYU3P_PIB_INTR_ERROR)
{
    switch (CYU3P_GET_PIB_ERROR_TYPE(cbArg))
    {
        case CYU3P_PIB_ERR_THR0_WR_OVERRUN:
        	GlbError=CYU3P_PIB_ERR_THR0_WR_OVERRUN;
        break;
        case CYU3P_PIB_ERR_THR1_WR_OVERRUN:
        	GlbError=CYU3P_PIB_ERR_THR1_WR_OVERRUN;
        break;
        case CYU3P_PIB_ERR_THR2_WR_OVERRUN:
        	GlbError=CYU3P_PIB_ERR_THR2_WR_OVERRUN;
        break;
        case CYU3P_PIB_ERR_THR3_WR_OVERRUN:
        	GlbError=CYU3P_PIB_ERR_THR3_WR_OVERRUN;
        break;

        case CYU3P_PIB_ERR_THR0_RD_UNDERRUN:
        	GlbError=CYU3P_PIB_ERR_THR0_RD_UNDERRUN;
        break;
        case CYU3P_PIB_ERR_THR1_RD_UNDERRUN:
        	GlbError=CYU3P_PIB_ERR_THR1_RD_UNDERRUN;
        break;
        case CYU3P_PIB_ERR_THR2_RD_UNDERRUN:
        	GlbError=CYU3P_PIB_ERR_THR2_RD_UNDERRUN;
        break;
        case CYU3P_PIB_ERR_THR3_RD_UNDERRUN:
        	GlbError=CYU3P_PIB_ERR_THR3_RD_UNDERRUN;
        break;

        default:
        	GlbError=0;
       // CyU3PDebugPrint (4, "No Error :%d\n ",CYU3P_GET_PIB_ERROR_TYPE(cbArg));
            break;
    }
}

}
/* This function writes configuration data to the xilinx FPGA */
CyU3PReturnStatus_t CyFxConfigFpga(uint32_t uiLen)
{
      uint32_t uiIdx;
      CyU3PReturnStatus_t apiRetStatus;
      CyU3PDmaBuffer_t inBuf_p;
      CyBool_t xFpga_Done, xFpga_nSTATUS;

      glConfigDone = CyFalse;

      /* Pull nCONFIG line to reset FPGA */
      apiRetStatus = CyU3PGpioSetValue(GPIO_nCONFIG, CyFalse);
      CyU3PThreadSleep(1);
      CyU3PGpioSimpleGetValue (GPIO_nSTATUS, &xFpga_nSTATUS);
      CyU3PGpioSimpleGetValue (GPIO_nSTATUS, &xFpga_nSTATUS);

  	 if (xFpga_nSTATUS)
  		  return apiRetStatus;
  	  CyU3PThreadSleep(1);
  	  /* Release nCONFIG line */
      apiRetStatus |= CyU3PGpioSetValue(GPIO_nCONFIG, CyTrue);
      CyU3PThreadSleep(10);   // Allow FPGA to startup
     /* Check if FPGA is now ready by testing the FPGA nSTATUS signal */
    apiRetStatus |= CyU3PGpioSimpleGetValue (GPIO_nSTATUS, &xFpga_nSTATUS);


    if( (xFpga_nSTATUS != CyTrue) || (apiRetStatus != CY_U3P_SUCCESS) ){

          return apiRetStatus;
    }


    /* Start shifting out configuration data */
    for(uiIdx = 0; (uiIdx < uiLen) && glIsApplnActive; uiIdx += uiPacketSize )
    {

      if(CyU3PDmaChannelGetBuffer (&glChHandleUtoCPU, &inBuf_p, 2000) != CY_U3P_SUCCESS){ // Wait 2000 ms(?)
    	  	  apiRetStatus = CY_U3P_ERROR_TIMEOUT;
    	  	  break;
      }

      apiRetStatus = CyU3PSpiTransmitWords(inBuf_p.buffer ,inBuf_p.count);
      if (apiRetStatus != CY_U3P_SUCCESS){
    	  break;
      }

         if(CyU3PDmaChannelDiscardBuffer (&glChHandleUtoCPU) != CY_U3P_SUCCESS){ // Wait 2000 ms(?)
            	apiRetStatus = CY_U3P_ERROR_TIMEOUT;
           break;
            }
    }
    CyU3PSpiTransmitWords(0 ,1);
    CyU3PThreadSleep(1);

    apiRetStatus |= CyU3PGpioSimpleGetValue (GPIO_CONFDONE, &xFpga_Done);
    if( (xFpga_Done != CyTrue) ){
      apiRetStatus = CY_U3P_ERROR_FAILURE;
    }
    else glConfigDone = CyTrue;

    return apiRetStatus;

}
//==========================================================================
void CyFxConfigFpgaApplnStart (void)
{
    uint16_t size = 0;
    CyU3PEpConfig_t epCfg;
    CyU3PDmaChannelConfig_t dmaCfg;
    //CyU3PReturnStatus_t apiRetStatus ;
    CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed();
    /* First identify the usb speed. Once that is identified,
     * create a DMA channel and start the transfer on this. */
    /* Based on the Bus Speed configure the endpoint packet size */
    switch (usbSpeed)
    {
        case CY_U3P_FULL_SPEED:   size = 64;   break;
        case CY_U3P_HIGH_SPEED:   size = 512;  break;
        case  CY_U3P_SUPER_SPEED:  size = 1024;  break;
        default: size = 64;   break;
    }


    CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
    epCfg.enable = CyTrue;
    epCfg.epType = CY_U3P_USB_EP_BULK;
    epCfg.burstLen = 1;
    epCfg.streams = 0;
    epCfg.pcktSize = size;

    uiPacketSize = size;

    /* Producer endpoint configuration */
    //apiRetStatus =
    CheckStatus(CyU3PSetEpConfig(CY_FX_EP_PRODUCER, &epCfg));
    /* Create a DMA MANUAL channel for U2CPU transfer.
     * DMA size is set based on the USB speed. */
    CyU3PMemSet ((uint8_t *)&dmaCfg, 0, sizeof (dmaCfg));
    dmaCfg.size  = size;
    dmaCfg.count = CY_FX_IN_DMA_BUF_COUNT;
    dmaCfg.prodSckId = CY_FX_PRODUCER_USB_SOCKET;
    dmaCfg.consSckId = CY_U3P_CPU_SOCKET_CONS;
    dmaCfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
    /* Enabling the callback for produce event. */
    dmaCfg.notification = 0;
    dmaCfg.cb = NULL;
    dmaCfg.prodHeader = 0;
    dmaCfg.prodFooter = 0;
    dmaCfg.consHeader = 0;
    dmaCfg.prodAvailCount = 0;

   // apiRetStatus =
    CheckStatus(CyU3PDmaChannelCreate (&glChHandleUtoCPU,
    CY_U3P_DMA_TYPE_MANUAL_IN, &dmaCfg));
    /* Flush the Endpoint memory */
    CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
    /* Set DMA channel transfer size. */
   // apiRetStatus =
    CyU3PDmaChannelSetXfer (&glChHandleUtoCPU, CY_FX_SLFIFO_DMA_TX_SIZE);
    /* Update the status flag. */
    glIsApplnActive = CyTrue;
}

//===============================================================================
void CyFxConfigFpgaApplnStop (void)
{
	CyU3PSpiDeInit();
    CyU3PUsbGetEpSeqNum(CY_FX_EP_PRODUCER, &seqnum_p);
    CyU3PUsbFlushEp(CY_FX_EP_PRODUCER); 		/* Flush the endpoint memory */
    CyU3PDmaChannelDestroy (&glChHandleUtoCPU); /* Destroy the channel */
	glIsApplnActive = CyFalse; /* Update the flag. */
}
//===================================================
typedef struct{
	uint32_t fastClkDiv:  4;
	uint32_t halfDiv: 1;
	uint32_t clkSrc: 2;
	uint32_t simpleDiv: 2;
	uint32_t temp:22;
	uint32_t en:1;
}gpio_fast_clk;

//=============================================================================

void CyFxConfigFpgaApplnInit (void){
//	CyU3PReturnStatus_t apiRetStatus;
    CyU3PGpioSimpleConfig_t gpioConfig;
    CyU3PSpiConfig_t spiConfig;


    //Create Event
     CyU3PEventCreate(&glFxConfigFpgaAppEvent);
        /* Start the SPI module and configure the master. */
     CheckStatus(CyU3PSpiInit());
        /* Start the SPI master block. Run the SPI clock at 25MHz
         * and configure the word length to 8 bits. Also configure
         * the slave select using FW. */
        CyU3PMemSet ((uint8_t *)&spiConfig, 0, sizeof(spiConfig));
        spiConfig.isLsbFirst = CyTrue;	// LSB FIRST
        spiConfig.cpol       = CyFalse;
        spiConfig.ssnPol     = CyFalse;
        spiConfig.cpha       = CyFalse;
        spiConfig.leadTime   = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
        spiConfig.lagTime    = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
        spiConfig.ssnCtrl    = CY_U3P_SPI_SSN_CTRL_FW;
        spiConfig.clock      = 10000000;
        spiConfig.wordLen    = 8;
        CheckStatus(CyU3PSpiSetConfig (&spiConfig, NULL));


    //========================  Configure GPIO 25 as input(GPIO_Empty)  ====================

          gpioConfig.outValue    = CyFalse;
          gpioConfig.inputEn     = CyTrue;
          gpioConfig.driveLowEn  = CyFalse;
          gpioConfig.driveHighEn = CyFalse;
          gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
          CheckStatus(CyU3PGpioSetSimpleConfig(GPIO_Empty, &gpioConfig));

        //====================  Configure GPIO 38 as output(nCONFIG). ================
           gpioConfig.outValue    = CyTrue;
           gpioConfig.inputEn     = CyFalse;
           gpioConfig.driveLowEn  = CyTrue;
           gpioConfig.driveHighEn = CyTrue;
           gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
           CheckStatus(CyU3PGpioSetSimpleConfig(GPIO_nCONFIG, &gpioConfig));

        //========================  Configure GPIO 33 as input(nSTATUS)  ====================
           gpioConfig.outValue    = CyFalse;
           gpioConfig.inputEn     = CyTrue;
           gpioConfig.driveLowEn  = CyFalse;
           gpioConfig.driveHighEn = CyFalse;
           gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
           CheckStatus(CyU3PGpioSetSimpleConfig(GPIO_nSTATUS, &gpioConfig));

       //========================  Configure GPIO 35 as input(GPIO_CONFDONE) =================
           gpioConfig.outValue    = CyFalse; //
           gpioConfig.inputEn     = CyTrue;
           gpioConfig.driveLowEn  = CyFalse;
           gpioConfig.driveHighEn = CyFalse;
           gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
           CheckStatus(CyU3PGpioSetSimpleConfig(GPIO_CONFDONE, &gpioConfig));

          //====================  Configure GPIO 23 as output(GPIO_nCS). ================
            gpioConfig.outValue    = CyTrue;
            gpioConfig.inputEn     = CyFalse;
            gpioConfig.driveLowEn  = CyTrue;
            gpioConfig.driveHighEn = CyTrue;
            gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
            CheckStatus(CheckStatus(CyU3PGpioSetSimpleConfig(GPIO_nCS, &gpioConfig)));

            //====================  Configure GPIO 52 as output(GPIO_SHB). ================
              gpioConfig.outValue    = CyTrue;
              gpioConfig.inputEn     = CyFalse;
              gpioConfig.driveLowEn  = CyTrue;
              gpioConfig.driveHighEn = CyTrue;
              gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
              CheckStatus(CyU3PGpioSetSimpleConfig(GPIO_SHB, &gpioConfig));


        //=========================================================================================
          /* Configure GPIO end*/
              /* callback to see if there is any overflow of data on the GPIF II side*/
      CyU3PPibRegisterCallback(gpif_error_cb,0xffff);
      CyU3PUsbSSCDisable();
    /* Start the USB functionality. */
      CheckStatus(CyU3PUsbStart());
    /* The fast enumeration is the easiest way to setup a USB connection,
     * where all enumeration phase is handled by the library. Only the
     * class / vendor requests need to be handled by the application. */
    CyU3PUsbRegisterSetupCallback(CyFxSlFifoApplnUSBSetupCB, CyTrue);
    /* Setup the callback to handle the USB events. */
    CyU3PUsbRegisterEventCallback(CyFxSlFifoApplnUSBEventCB);
    /* Register a callback to handle LPM requests from the USB 3.0 host. */
    CyU3PUsbRegisterLPMRequestCallback(CyFxApplnLPMRqtCB);
    /* Set the USB Enumeration descriptors */
    /* Super speed device descriptor. */
    CheckStatus(CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSB30DeviceDscr));
    /* High speed device descriptor. */
    CheckStatus(CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSB20DeviceDscr));
    /* BOS descriptor */
    CheckStatus(CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_BOS_DESCR, 0, (uint8_t *)CyFxUSBBOSDscr));
    /* Device qualifier descriptor */
    CheckStatus(CyU3PUsbSetDesc(CY_U3P_USB_SET_DEVQUAL_DESCR, 0, (uint8_t *)CyFxUSBDeviceQualDscr));
    /* Super speed configuration descriptor */
    CheckStatus(CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBSSConfigDscr));
    /* High speed configuration descriptor */
    CheckStatus(CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBHSConfigDscr));
    /* Full speed configuration descriptor */
    CheckStatus(CyU3PUsbSetDesc(CY_U3P_USB_SET_FS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBFSConfigDscr));
    /* String descriptor 0 */
    //apiRetStatus =
    CheckStatus(CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t *)CyFxUSBStringLangIDDscr));
    /* String descriptor 1 */
    //apiRetStatus =
    CheckStatus(CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t *)CyFxUSBManufactureDscr));
    /* String descriptor 2 */
    //apiRetStatus =
    CheckStatus(CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t *)CyFxUSBProductDscr));
    		//read FX3 die ID and set USB serial number
    		CyU3PReadDeviceRegisters(EFUSE_DIE_ID, 2, die_id);
    		int i;
    		for (i = 0; i < 2; i++)
    		{
    			CyFxUSBSerialNumDesc[i*16+ 2] = hex_digit[(die_id[1-i] >> 28) & 0xF];
    			CyFxUSBSerialNumDesc[i*16+ 4] = hex_digit[(die_id[1-i] >> 24) & 0xF];
    			CyFxUSBSerialNumDesc[i*16+ 6] = hex_digit[(die_id[1-i] >> 20) & 0xF];
    			CyFxUSBSerialNumDesc[i*16+ 8] = hex_digit[(die_id[1-i] >> 16) & 0xF];
    			CyFxUSBSerialNumDesc[i*16+10] = hex_digit[(die_id[1-i] >> 12) & 0xF];
    			CyFxUSBSerialNumDesc[i*16+12] = hex_digit[(die_id[1-i] >>  8) & 0xF];
    			CyFxUSBSerialNumDesc[i*16+14] = hex_digit[(die_id[1-i] >>  4) & 0xF];
    			CyFxUSBSerialNumDesc[i*16+16] = hex_digit[(die_id[1-i] >>  0) & 0xF];
    		}

    		/* String descriptor 3 - USB serial number */
    	//	apiRetStatus =
    		CheckStatus(CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 3, (uint8_t *)CyFxUSBSerialNumDesc));
    /* Connect the USB Pins with super speed operation enabled. */
    //apiRetStatus =
    		CheckStatus(CyU3PConnectState(CyTrue, CyTrue));
//get flag glConfigDone
    CyU3PGpioSimpleGetValue (GPIO_CONFDONE, &glConfigDone);

}

/*
 * Main function CyU3PSetEpConfig failed
 */
int main (void)
{
    CyU3PIoMatrixConfig_t io_cfg;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    CyU3PSysClockConfig_t clkCfg;

        /* setSysClk400 clock configurations */
        clkCfg.setSysClk400 = CyTrue;   /* FX3 device's master clock is set to a frequency > 400 MHz */
        clkCfg.cpuClkDiv = 2;           /* CPU clock divider */
        clkCfg.dmaClkDiv = 2;           /* DMA clock divider */
        clkCfg.mmioClkDiv = 2;          /* MMIO clock divider */
        clkCfg.useStandbyClk = CyFalse; /* device has no 32KHz clock supplied */
        clkCfg.clkSrc = CY_U3P_SYS_CLK; /* Clock source for a peripheral block  */

    /* Initialize the device */
    status = CyU3PDeviceInit (&clkCfg);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* Initialize the caches. Enable instruction cache and keep data cache disabled.
     * The data cache is useful only when there is a large amount of CPU based memory
     * accesses. When used in simple cases, it can decrease performance due to large
     * number of cache flushes and cleans and also it adds to the complexity of the
     * code. */
    status = CyU3PDeviceCacheControl (CyTrue, CyFalse, CyFalse);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* Configure the IO matrix for the device. On the FX3 DVK board, the COM port
     * is connected to the IO(53:56). This means that either DQ32 mode should be
     * selected or lppMode should be set to UART_ONLY. Here we are choosing
     * UART_ONLY configuration for 16 bit slave FIFO configuration and setting
     * isDQ32Bit for 32-bit slave FIFO configuration. */
    CyU3PMemSet ((uint8_t *)&io_cfg, 0, sizeof (io_cfg));
    io_cfg.useUart   = CyTrue;
    io_cfg.useI2C    = CyFalse;
    io_cfg.useI2S    = CyFalse;
    io_cfg.useSpi    = CyTrue;
    io_cfg.isDQ32Bit = CyFalse;
    io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_DEFAULT;
    /* GPIOs 23,33,35,38,52 . */
    io_cfg.gpioSimpleEn[0]  = 0x02800000;  // GPI 23 nCS  25
    io_cfg.gpioSimpleEn[1]  = 0x0010004A;  // GPIO 33,35,38,52
    io_cfg.gpioComplexEn[0] = 0;
    io_cfg.gpioComplexEn[1] = 0x00040000;  // GPIO 50
//    io_cfg.gpioComplexEn[1] = 0x00000000;  // GPIO 50
    status = CyU3PDeviceConfigureIOMatrix (&io_cfg);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }
    /* This is a non returnable call for initializing the RTOS kernel */
    CyU3PKernelEntry ();
    /* Dummy return to make the compiler happy */
    return 0;
handle_fatal_error:
    /* Cannot recover from this error. */
    while (1);
}

