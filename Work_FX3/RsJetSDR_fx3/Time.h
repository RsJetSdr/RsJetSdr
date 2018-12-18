/*
 * Time.h
 *
 *  Created on: 17 апр. 2016 г.
 *      Author: IvanHome
 */

#ifndef TIME_H_
#define TIME_H_
#include "cyu3types.h"
#include "cyu3gpio.h"
#include "cyu3uart.h"
#include <gpio_regs.h>
#include "constant.h"
// https://community.cypress.com/docs/DOC-10433
#define _100us 217
#define TIME_TIC 50     // uS
#define TIME_MS (1000/TIME_TIC)

//extern volatile uint32_t TicTimer;

void GPIO_InterruptCallback(uint8_t gpioId);
CyU3PReturnStatus_t Timer_uS_Init (uint8_t gpioId);
CyU3PReturnStatus_t InitGpioClocks(void);
uint32_t  GetSysTime(void);
void Delay_Us(uint32_t Time_uS);
#endif /* TIME_H_ */
