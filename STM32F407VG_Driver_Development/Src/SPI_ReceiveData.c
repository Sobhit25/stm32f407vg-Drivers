/*
 * SPI_ReceiveData.c
 *
 *  Created on: 10-Sep-2021
 *      Author: sobhit25
 */


#include <stm32f407vg.h>
#include <string.h>

void delay(void){
	for(int i=0;i<60000;i++){
	}
}

uint8_t *dummy_send = 0xFF;

int main(){

	char data[] = "Hello World";

	/*
	 * 		@Mapping:
	 * 		PB12 --> NSS
	 * 		PB13 --> SCLK
	 * 		PB14 --> MISO
	 * 		PB15 --> MOSI
	 *
	 */

	/*
	 * GPIO INIT FOR SPI PERIPH
	 */
	GPIO_Handle_t SPI_Pins;

	SPI_Pins.pGPIOx = GPIOB;

	SPI_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OPTYPE_PP;
	SPI_Pins.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PUPD_NOPUPD;
	SPI_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_HIGH;
	SPI_Pins.GPIO_PinConfig.GPIO_PinAltFunc = GPIO_AF5;

	//SCLK Pin
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = 13;
	GPIO_Init(&SPI_Pins);

	//MISO Pin
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = 14;
	GPIO_Init(&SPI_Pins);

	//MOSI Pin
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = 15;
	GPIO_Init(&SPI_Pins);

	//NSS
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = 12;
	GPIO_Init(&SPI_Pins);


	SPI_Handle_t 	SPI2_Handler;

	SPI2_Handler.pSPIx = SPI2;

	SPI2_Handler.SPI_Config.Device_Mode = SPI_DEVICE_MODE_MASTER;
	SPI2_Handler.SPI_Config.Bus_Config = SPI_BusConfig_FD;
	SPI2_Handler.SPI_Config.DFF = SPI_DFF_8BITS;
	SPI2_Handler.SPI_Config.SPEED = SPI_SCLK_SPEED_DIV2;
	SPI2_Handler.SPI_Config.CPHA = SPI_CPHA_LOW;
	SPI2_Handler.SPI_Config.CPOL = SPI_CPOL_LOW;
	SPI2_Handler.SPI_Config.SSM = SPI_SSM_DIS;

	SPI_Init(&SPI2_Handler);

	//Single Master Mode
	SPI2_Handler.pSPIx->CR2 |= (1 << SPI_CR2_SSOE);


	while(1){
		//Enabling the SPI Peripheral
		SPI2_Handler.pSPIx->CR1 |= (1 << SPI_CR1_SPE);

		SPI_SendData(SPI2,	dummy_send, 1);

		SPI_RecieveData(SPI2, (uint8_t*)data, strlen(data));

		if( !(SPI2_Handler.pSPIx->SR & (1 << SPI_SR_BSY)) )
			SPI2_Handler.pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
		delay();
	}

}
