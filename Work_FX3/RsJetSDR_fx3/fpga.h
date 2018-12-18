/*
 * cyfxconfigfpga.h
 *
 *  Created on: Nov 26, 2012
 *      Author: rskv
 */

#ifndef CYFXCONFIGFPGA_H_
#define CYFXCONFIGFPGA_H_

/*This file contains the constants and definitions used by the Configure FPGA application example */


//Vendor command code used in FPGA slave serial application
#define VND_CMD_COMMAND         (0xAB)
#define VND_CMD_ID_CHECK        (0xB0)
#define VND_CMD_CFGSTAT 		(0xB1)
#define VND_CMD_CFGLOAD 		(0xB2)
#define VND_CMD_SPI_WRITE       (0xC2)
#define VND_CMD_SPI_READ        (0xC3)




extern CyU3PDmaChannel glChHandleUtoCPU;   /* DMA Channel handle for U2CPU transfer. */

extern CyBool_t glConfigDone;			/* Flag to indicate the status of FPGA configuration  */

extern volatile uint32_t filelen;					/* length of Configuration file (.bin) */

extern volatile uint16_t uiPacketSize;

extern volatile CyBool_t glIsApplnActive;

extern CyU3PEvent glFxConfigFpgaAppEvent;    /* Configure FPGA event group. */

extern uint8_t seqnum_p;




extern void
CyFxAppErrorHandler (
        CyU3PReturnStatus_t apiRetStatus    /* API return status */
        );

extern CyBool_t
CyFxSlFifoApplnUSBSetupCB (
        uint32_t setupdat0,
        uint32_t setupdat1
    );

extern void
CyFxSlFifoApplnUSBEventCB (
    CyU3PUsbEventType_t evtype,
    uint16_t            evdata
    );

extern CyBool_t
CyFxApplnLPMRqtCB (
        CyU3PUsbLinkPowerMode link_mode);

extern void
CyFxConfigFpgaApplnStart (
        void);

extern void
CyFxConfigFpgaApplnInit (void);

extern CyU3PReturnStatus_t CyFxConfigFpga(uint32_t uiLen);


extern void
CyFxConfigFpgaApplnStop (
        void);


#endif /* CYFXCONFIGFPGA_H_ */
