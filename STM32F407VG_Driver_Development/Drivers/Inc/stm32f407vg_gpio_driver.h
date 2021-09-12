/*
 * stm32f407vg_ gpio_driver.h
 *
 *  Created on: 18-Aug-2021
 *      Author: sobhit25
 */

#ifndef INC_STM32F407VG_GPIO_DRIVER_H_
#define INC_STM32F407VG_GPIO_DRIVER_H_

#include<stm32f407vg.h>

/*
 * Configuration Structure
 */
typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdCtrl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunc;
}GPIO_PinConfig_t;


/*
 * Handle structure for GPIO Pin
 */
typedef struct{
	GPIO_RegDef_t *pGPIOx; 					/*!< This holds the base address of the peripheral to which the pin belongs >*/
	GPIO_PinConfig_t GPIO_PinConfig;		/*!< This holds the Pin Configuration Settings >*/
}GPIO_Handle_t;


/*
 * GPIO PIN POSSIBLE MODES
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT 	5
#define GPIO_MODE_IT_FTRT	6


/*
 * GPIO POSSIBLE OUTPUT TYPE
 */
#define GPIO_OPTYPE_PP		0
#define GPIO_OPTYPE_OD		1

/*
 * GPIO POSSIBLE SPEED
 */
#define GPIO_OSPEED_LOW  	0
#define GPIO_OSPEED_MED		1
#define GPIO_OSPEED_HIGH	2
#define GPIO_OSPEED_VHIGH	3

/*
 * GPIO POSSIBLE PULL UP PULL DOWN CONFIG
 */
#define GPIO_PUPD_NOPUPD	0
#define GPIO_PUPD_PU		1
#define GPIO_PUPD_PD  		2

/*
 * GPIO POSSIBLE ALT FUNC
 */
#define GPIO_AF0	0
#define GPIO_AF1	1
#define GPIO_AF2	2
#define GPIO_AF3	3
#define GPIO_AF4	4
#define GPIO_AF5	5
#define GPIO_AF6	6
#define GPIO_AF7	7
#define GPIO_AF8	8
#define GPIO_AF9	9
#define GPIO_AF10	10
#define GPIO_AF11	11
#define GPIO_AF12	12
#define GPIO_AF13	13
#define GPIO_AF14	14
#define GPIO_AF15	15

/***********************************************************************************
 * 								APIs Supported by GPIO driver
 ***********************************************************************************/



/*
 * GPIO Peripheral clock control function
 */
void GPIO_ClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);





/*
 * GPIO Initialisation and Deinitialisation
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);



/*
 *  Gpio read and write functions
 */
uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx);

void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);

void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t word);

/*
 * GPIO alternate functionality
 */

void GPIO_AltFunc(void);

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * GPIO Interrupt configuration
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint32_t IRQPriority, uint8_t EnorDi);

void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F407VG_GPIO_DRIVER_H_ */
