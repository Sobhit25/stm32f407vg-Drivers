/*
 * stm32f407vg_spi_driver.h
 *
 *  Created on: 27-Aug-2021
 *      Author: sobhit25
 */

#ifndef INC_STM32F407VG_SPI_DRIVER_H_
#define INC_STM32F407VG_SPI_DRIVER_H_

#include<stm32f407vg.h>

typedef struct{
	uint8_t Device_Mode;
	uint8_t Bus_Config;
	uint8_t DFF;
	uint8_t CPHA;
	uint8_t CPOL;
	uint8_t SSM;
	uint8_t SPEED;
}SPI_Config_t;

typedef struct{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPI_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;
}SPI_Handle_t;



/*
 * @SPI_DEVICE_MODE
 */
#define SPI_DEVICE_MODE_MASTER 1
#define SPI_DEVICE_MODE_SLAVE  0

/*
 * @SPI_BusConfig
 */
#define SPI_BusConfig_FD				1
#define SPI_BusConfig_HD				2
#define SPI_BusConfig_SIMPLEX_RXONLY	3

/*
 * @SPI_CLK_SPEED
 */
#define SPI_SCLK_SPEED_DIV2 	0
#define SPI_SCLK_SPEED_DIV4 	1
#define SPI_SCLK_SPEED_DIV8 	2
#define SPI_SCLK_SPEED_DIV16 	3
#define SPI_SCLK_SPEED_DIV32	4
#define SPI_SCLK_SPEED_DIV64 	5
#define SPI_SCLK_SPEED_DIV128 	6
#define SPI_SCLK_SPEED_DIV256 	7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS	0
#define SPI_DFF_16BITS	1


/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW	0
#define SPI_CPOL_HIGH	1

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_LOW	0
#define SPI_CPHA_HIGH	1

/*
 * @SPI_SSM
 */
#define SPI_SSM_DIS			0
#define SPI_SSM_EN		  	1


/*
 * SPI States
 */
#define SPI_READY 			0
#define SPI_BUSY_IN_RX 		1
#define SPI_BUSY_IN_TX 		2

/*
 * SPI Possible EVENTS
 */
#define SPI_EVENT_TX_CMPLT		0
#define SPI_EVENT_RX_CMPLT 		1
#define SPI_EVENT_OVR_ERR 		2



/***********************************************************************************
 * 								APIs Supported by SPI Driver
 ***********************************************************************************/


/*
 * SPI Peripheral clock control function
 */
void SPI_PeriClockControl(SPI_RegDef_t* pSPIx, uint8_t EnorDi);


/*
 * SPI Init De-Init functions
 */
void SPI_Init(SPI_Handle_t* pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/*
 * SPI Send & Recieve Data functions
 */
void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t len);
void SPI_RecieveData(SPI_RegDef_t* pSPIx,uint8_t* pRxBuffer, uint32_t len);

uint8_t SPI_SendDataIT(SPI_Handle_t* pSPIHandle, uint8_t* pTxBuffer, uint32_t len);
uint8_t SPI_RecieveDataIT(SPI_Handle_t* pSPIHandle,uint8_t* pRxBuffer, uint32_t len);


/*
 * SPI IRQ Configuration and Handling functions
 */

void SPI_IRQConfig(uint8_t IRQNumber, uint32_t IRQPriority, uint8_t EnorDi);

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);




#endif /* INC_STM32F407VG_SPI_DRIVER_H_ */
