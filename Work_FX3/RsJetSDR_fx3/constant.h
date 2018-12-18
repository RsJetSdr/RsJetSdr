/*
 * Constant.h
 *
 *  Created on: 10 апр. 2016 г.
 *      Author: IvanHome
 */

#ifndef CONSTANT_H_
#define CONSTANT_H_
/* Burst length in 1 KB packets. Only applicable to USB 3.0. */
#define CY_FX_EP_BURST_LENGTH     (16)
#define WATERMARK                 (4)
//Количество буферов DMA
#define CY_FX_IN_DMA_BUF_COUNT     (2)  /* channel U to CPU buffer count */
#define CY_FX_FIFO_DMA_BUF_COUNT   (4)  /* Slave FIFO P_2_U channel buffer count */

//define pins
#define GPIO_nCS	  	23
#define GPIO_Empty  	25

#define GPIO_nSTATUS 	33
#define GPIO_CONFDONE 	35
#define GPIO_nCONFIG 	38

#define GPIO_TIMER	  	50
#define GPIO_SHB	  	52


//Events
#define START_EVENT    		   (1 << 0)
#define SW_TO_SLFIFO_EVENT     (1 << 1)
#define SPI_WRITE_EVENT        (1 << 2)
#define SPI_READ_EVENT         (1 << 3)
#define COMMAND_EVENT	       (1 << 4)

#define  BM_115200BAUD 0    /* Baud rate constant for switch to the baud rate mode 0 */
#define  BM_250000BAUD 1    /* Baud rate constant for switch to the baud rate mode 1 */
#define BM_1250000BAUD 2    /* Baud rate constant for switch to the baud rate mode 3 */
#define BM_1875000BAUD 3    /* Baud rate constant for switch to the baud rate mode 2 */
#define BM_3750000BAUD 4    /* Baud rate constant for switch to the baud rate mode 4 */

#endif /* CONSTANT_H_ */
//-adhlns="$@.lst"
