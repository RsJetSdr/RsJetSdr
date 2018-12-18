/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : CS_LO1.h
**     Project     : Presel_ARM
**     Processor   : MK20DX128VLH5
**     Component   : BitIO_LDD
**     Version     : Component 01.033, Driver 01.03, CPU db: 3.00.000
**     Repository  : Kinetis
**     Compiler    : GNU C Compiler
**     Date/Time   : 2018-12-18, 11:29, # CodeGen: 77
**     Abstract    :
**         The HAL BitIO component provides a low level API for unified
**         access to general purpose digital input/output pins across
**         various device designs.
**
**         RTOS drivers using HAL BitIO API are simpler and more
**         portable to various microprocessors.
**     Settings    :
**          Component name                                 : CS_LO1
**          Pin for I/O                                    : ADC0_SE13/TSI0_CH8/PTB3/I2C0_SDA/UART0_CTS_b/UART0_COL_b/FTM0_FLT0
**          Pin signal                                     : 
**          Direction                                      : Output
**          Initialization                                 : 
**            Init. direction                              : Output
**            Init. value                                  : 1
**            Auto initialization                          : yes
**          Safe mode                                      : no
**     Contents    :
**         Init   - LDD_TDeviceData* CS_LO1_Init(LDD_TUserData *UserDataPtr);
**         ClrVal - void CS_LO1_ClrVal(LDD_TDeviceData *DeviceDataPtr);
**         SetVal - void CS_LO1_SetVal(LDD_TDeviceData *DeviceDataPtr);
**
**     Copyright : 1997 - 2015 Freescale Semiconductor, Inc. 
**     All Rights Reserved.
**     
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**     
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**     
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**     
**     o Neither the name of Freescale Semiconductor, Inc. nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**     
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**     
**     http: www.freescale.com
**     mail: support@freescale.com
** ###################################################################*/
/*!
** @file CS_LO1.h
** @version 01.03
** @brief
**         The HAL BitIO component provides a low level API for unified
**         access to general purpose digital input/output pins across
**         various device designs.
**
**         RTOS drivers using HAL BitIO API are simpler and more
**         portable to various microprocessors.
*/         
/*!
**  @addtogroup CS_LO1_module CS_LO1 module documentation
**  @{
*/         

#ifndef __CS_LO1_H
#define __CS_LO1_H

/* MODULE CS_LO1. */

/* Include shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
/* Include inherited beans */
#include "IO_Map.h"
#include "GPIO_PDD.h"

#include "Cpu.h"

#ifdef __cplusplus
extern "C" {
#endif 



/*! Peripheral base address of a device allocated by the component. This constant can be used directly in PDD macros. */
#define CS_LO1_PRPH_BASE_ADDRESS  0x400FF040U
  
/*! Device data structure pointer used when auto initialization property is enabled. This constant can be passed as a first parameter to all component's methods. */
#define CS_LO1_DeviceData  ((LDD_TDeviceData *)PE_LDD_GetDeviceStructure(PE_LDD_COMPONENT_CS_LO1_ID))

/* Methods configuration constants - generated for all enabled component's methods */
#define CS_LO1_Init_METHOD_ENABLED     /*!< Init method of the component CS_LO1 is enabled (generated) */
#define CS_LO1_ClrVal_METHOD_ENABLED   /*!< ClrVal method of the component CS_LO1 is enabled (generated) */
#define CS_LO1_SetVal_METHOD_ENABLED   /*!< SetVal method of the component CS_LO1 is enabled (generated) */

/* Definition of implementation constants */
#define CS_LO1_MODULE_BASE_ADDRESS PTB_BASE_PTR /*!< Name of macro used as the base address */
#define CS_LO1_PORTCONTROL_BASE_ADDRESS PORTB_BASE_PTR /*!< Name of macro used as the base address */
#define CS_LO1_PORT_MASK 0x08U         /*!< Mask of the allocated pin from the port */



/*
** ===================================================================
**     Method      :  CS_LO1_Init (component BitIO_LDD)
*/
/*!
**     @brief
**         Initializes the device. Allocates memory for the device data
**         structure, allocates interrupt vectors and sets interrupt
**         priority, sets pin routing, sets timing, etc. If the "Enable
**         in init. code" is set to "yes" value then the device is also
**         enabled(see the description of the Enable() method). In this
**         case the Enable() method is not necessary and needn't to be
**         generated. 
**     @param
**         UserDataPtr     - Pointer to the user or
**                           RTOS specific data. This pointer will be
**                           passed as an event or callback parameter.
**     @return
**                         - Pointer to the dynamically allocated private
**                           structure or NULL if there was an error.
*/
/* ===================================================================*/
LDD_TDeviceData* CS_LO1_Init(LDD_TUserData *UserDataPtr);

/*
** ===================================================================
**     Method      :  CS_LO1_ClrVal (component BitIO_LDD)
*/
/*!
**     @brief
**         Clears (set to zero) the output value. It is equivalent to
**         the [PutVal(FALSE)]. This method is available only if the
**         direction = _[output]_ or _[input/output]_.
**     @param
**         DeviceDataPtr   - Pointer to device data
**                           structure returned by <Init> method.
*/
/* ===================================================================*/
void CS_LO1_ClrVal(LDD_TDeviceData *DeviceDataPtr);

/*
** ===================================================================
**     Method      :  CS_LO1_SetVal (component BitIO_LDD)
*/
/*!
**     @brief
**         Sets (to one) the output value. It is equivalent to the
**         [PutVal(TRUE)]. This method is available only if the
**         direction = _[output]_ or _[input/output]_.
**     @param
**         DeviceDataPtr   - Pointer to device data
**                           structure returned by <Init> method.
*/
/* ===================================================================*/
void CS_LO1_SetVal(LDD_TDeviceData *DeviceDataPtr);

/* END CS_LO1. */

#ifdef __cplusplus
}  /* extern "C" */
#endif 

#endif
/* ifndef __CS_LO1_H */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.5 [05.21]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
