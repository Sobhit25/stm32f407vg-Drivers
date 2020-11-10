/*
 * MPU6050.c
 *
 *  Created on: 02-Oct-2020
 *      Author: SOBHIT PANDA
 */



#include<stdio.h>
#include<string.h>
#include "stm32f407vg.h"


extern void initialise_monitor_handles(void);

#define MY_ADDR 0x61;

#define SLAVE_ADDR  0x68

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handle_t I2C1Handle;

//rcv buffer
uint8_t rcv_buffer[32];


/*
 * PB6-> SCL
 * PB7 -> SDA
 */

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins. GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);


	//sda
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);


}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_FM4K;

	I2C_Init(&I2C1Handle);

}



int main(void)
{

	uint8_t CommandCode, len;


	initialise_monitor_handles();

	printf("Application is running\n");



	//i2c pin inits
	I2C1_GPIOInits();

	//i2c peripheral configuration
	I2C1_Inits();

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	while(1)
	{

		CommandCode = 0x51;
		I2C_MasterSendData(&I2C1Handle, &CommandCode, 1 , SLAVE_ADDR , 0);

		I2C_MasterReceiveData(&I2C1Handle, &len , 1 , SLAVE_ADDR , 0);

		CommandCode = 0x52;
		I2C_MasterSendData(&I2C1Handle, &CommandCode, 1 , SLAVE_ADDR , 0);

		I2C_MasterReceiveData(&I2C1Handle, rcv_buffer , len , SLAVE_ADDR , 1);

		printf("Data : %s", rcv_buffer);

	}

}
