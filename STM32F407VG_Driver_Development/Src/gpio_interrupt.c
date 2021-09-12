/*
 * gpio_interrupt.c
 *
 *  Created on: 23-Aug-2021
 *      Author: sobhit25
 */


#include<stdint.h>
#include<stm32f407vg.h>

void delay(void){
	for(uint32_t i=0;i<50000;i++){

	}
}

int main(){
	GPIO_Handle_t Button;
	GPIO_Handle_t Led;

	//Configuration Settings for the Button
	Button.pGPIOx = GPIOD;
	Button.GPIO_PinConfig.GPIO_PinNumber = 0;
	Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_HIGH;
	Button.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PUPD_NOPUPD;

	//Configure the Led
	Led.pGPIOx = GPIOD;
	Led.GPIO_PinConfig.GPIO_PinNumber = 13;
	Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_HIGH;
	Led.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PUPD_NOPUPD;

	GPIO_Init(&Led);
	GPIO_Init(&Button);

	GPIO_IRQConfig(IRQ_NO_EXTI0, 15, ENABLE);
	while(1){

	}
}

void EXTI0_IRQHandler(void){
	delay();
	GPIO_IRQHandling(0);
	GPIO_ToggleOutputPin(GPIOD, 13);
}
