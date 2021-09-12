/*
 * btnLed.c
 *
 *  Created on: 25-Aug-2021
 *      Author: sobhit25
 */


#include <stdint.h>
#include <stm32f407vg.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

//interrupt code

void delay(){
	for(uint32_t i=0;i<70000;i++){

	}
}

int main(void)
{
	GPIO_Handle_t LED;

	LED.pGPIOx = GPIOD;

	LED.GPIO_PinConfig.GPIO_PinNumber = 12;
	LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	LED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	LED.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PUPD_NOPUPD;
	LED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_MED;

	GPIO_Init(&LED);


	GPIO_Handle_t BTN;

	BTN.pGPIOx = GPIOA;

	BTN.GPIO_PinConfig.GPIO_PinNumber = 0;
	BTN.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	BTN.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PUPD_NOPUPD;
	BTN.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_MED;


	GPIO_Init(&BTN);

	GPIO_IRQConfig(IRQ_NO_EXTI0, 15, ENABLE);


	/* Loop forever */
	while(1);
}

void EXTI0_IRQHandler(void){
	delay();
	GPIO_IRQHandling(0);
	GPIO_ToggleOutputPin(GPIOD, 12);
}
