/*
 * stm32f407vg_ gpio_driver.c
 *
 *  Created on: 18-Aug-2021
 *      Author: sobhit25
 */


#include <stm32f407vg_gpio_driver.h>

/***********************************************************************************
 * 								APIs Supported by GPIO driver
 ***********************************************************************************/




/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 ********************************************************************/
void GPIO_ClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA)
			GPIOA_PClk_EN();
		else if(pGPIOx == GPIOB)
			GPIOB_PClk_EN();
		else if(pGPIOx == GPIOC)
			GPIOC_PClk_EN();
		else if(pGPIOx == GPIOD)
			GPIOD_PClk_EN();
		else if(pGPIOx == GPIOE)
			GPIOE_PClk_EN();
	}
	else{
		if(pGPIOx == GPIOA)
			GPIOA_PClk_DI();
		else if(pGPIOx == GPIOB)
			GPIOB_PClk_DI();
		else if(pGPIOx == GPIOC)
			GPIOC_PClk_DI();
		else if(pGPIOx == GPIOD)
			GPIOD_PClk_DI();
		else if(pGPIOx == GPIOE)
			GPIOE_PClk_DI();
	}
}



/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function initializes the given GPIO Pin
 *
 * @param[in]         - GPIO Handle Structure
 *
 * @return            -  none
 *
 * @Note              -  none
 *********************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	//RCC Clock Enable for respective GPIO Port
	if(pGPIOHandle->pGPIOx == GPIOA)
		GPIOA_PClk_EN();
	else if(pGPIOHandle->pGPIOx == GPIOB)
			GPIOB_PClk_EN();
	else if(pGPIOHandle->pGPIOx == GPIOC)
			GPIOC_PClk_EN();
	else if(pGPIOHandle->pGPIOx == GPIOD)
			GPIOD_PClk_EN();
	else if(pGPIOHandle->pGPIOx == GPIOE)
			GPIOE_PClk_EN();

	uint32_t temp = 0;


	// Configure Pin GPIO_PinNumber
	uint8_t Pin = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

	// Configure Pin Mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= 0x03){
		temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) << (2*Pin));
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << (2*Pin));
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else{
		// Interrupt Mode Settings
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			//Configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Disable the RTSR
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//Configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Disable the FTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FTRT){
			//Configure the FTSR & RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}


		//2. Configure the GPIO Port Selection in SYSCFG_EXTIR
		uint8_t extiLine = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t extiPos  = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;

		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCGF_PClk_EN();

		SYSCFG->EXTICR[extiLine] |= (portCode << extiPos*4);


		//3. Enable the interrupt delivery from peripheral side using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}


	// Configure Pin OPType
	temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType) << Pin);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x03 << Pin);
	pGPIOHandle->pGPIOx->OTYPER |= temp;


	// Configure Pin PinSpeed
	temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed) << (2*Pin));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03 << (2*Pin));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;


	// Configure Pin PinPuPdCtrl
	temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdCtrl) << (2*Pin));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x03 << (2*Pin));
	pGPIOHandle->pGPIOx->PUPDR |= temp;



	// Configure Pin AltFunc
	if(Pin%8 == 0){
		temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunc) << (4*Pin));
		pGPIOHandle->pGPIOx->AFRL &= ~(0x0F << (4*Pin));
		pGPIOHandle->pGPIOx->AFRL |= temp;
	}
	else{
		temp = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunc) << (4*(Pin%8)));
		pGPIOHandle->pGPIOx->AFRH &= ~(0x0F << (4*(Pin%8)));
		pGPIOHandle->pGPIOx->AFRH |= temp;
	}


}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA)
		GPIOA_RESET();
	else if(pGPIOx == GPIOB)
		GPIOA_RESET();
	else if(pGPIOx == GPIOC)
		GPIOA_RESET();
	else if(pGPIOx == GPIOD)
		GPIOA_RESET();
	else if(pGPIOx == GPIOE)
		GPIOA_RESET();
}



/*
 *  GPIO read and write functions
 */
uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx){
	uint32_t value = pGPIOx->IDR;
	return value;
}

void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value){
	if(value == SET){
		pGPIOx->ODR |= 1 << PinNumber;
	}else{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t word){
	pGPIOx->ODR = word;
}

/*
 * GPIO alternate functionality
 */

void GPIO_AltFunc(void);

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= ( 1<<PinNumber ) ;
}

/*
 * GPIO Interrupt configuration
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint32_t IRQPriority, uint8_t EnorDi){

	//Set-reset the interrupt
	if(EnorDi == ENABLE){
		if(IRQNumber >=0  &&  IRQNumber<32)
			*NVIC_ISER0 |= (1 << IRQNumber);
		else if(IRQNumber >=32  &&  IRQNumber<64)
			*NVIC_ISER1 |= (1 << IRQNumber%32);
		else if(IRQNumber >=64  &&  IRQNumber<96)
			*NVIC_ISER2 |= (1 << IRQNumber%32);
	}
	else{
		if(IRQNumber >=0  &&  IRQNumber<32)
			*NVIC_ICER0 &= ~(1 << IRQNumber);
		else if(IRQNumber >=32  &&  IRQNumber<64)
			*NVIC_ICER1 &= ~(1 << IRQNumber%32);
		else if(IRQNumber >=64  &&  IRQNumber<96)
			*NVIC_ICER2 &= ~(1 << IRQNumber%32);
	}

	//Configure the Priority
	uint8_t irp_reg = IRQNumber/4;
	uint8_t irp_pos = IRQNumber%4;
	uint8_t shift   = (irp_pos * 8) + (8 - NO_IPR_BITS);

	*(NVIC_IPR_BASE + irp_reg) |= (IRQPriority << shift);
}

void GPIO_IRQHandling(uint8_t PinNumber){
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & ( 1 << PinNumber))
	{
		//clear
		EXTI->PR |= ( 1 << PinNumber);
	}
}
