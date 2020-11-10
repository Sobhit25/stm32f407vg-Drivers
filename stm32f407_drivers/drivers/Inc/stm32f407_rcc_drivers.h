/*
 * stm32f407_rcc_drivers.h
 *
 *  Created on: 20-Sep-2020
 *      Author: SOBHIT PANDA
 */

#ifndef INC_STM32F407_RCC_DRIVERS_H_
#define INC_STM32F407_RCC_DRIVERS_H_

#include "stm32f407vg.h"

//This returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//This returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);



#endif /* INC_STM32F407_RCC_DRIVERS_H_ */
