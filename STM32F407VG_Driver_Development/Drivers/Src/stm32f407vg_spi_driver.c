/*
 * stm32f407vg_spi_driver.c
 *
 *  Created on: 27-Aug-2021
 *      Author: sobhit25
 */

#include <stm32f407vg_spi_driver.h>

static void spi_txe_interrupt_handler(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handler(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handler(SPI_Handle_t *pSPIHandle);

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t Event);


/***********************************************************************************
 * 								APIs Supported by GPIO driver
 ***********************************************************************************/


/*
 * SPI Peripheral clock control function
 */
void SPI_PeriClockControl(SPI_RegDef_t* pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pSPIx == SPI1)
			SPI1_PClk_EN();
		else if(pSPIx == SPI2)
			SPI2_PClk_EN();
		else if(pSPIx == SPI3)
			SPI3_PClk_EN();
	}
	else{
		if(pSPIx == SPI1)
			SPI1_PClk_DI();
		else if(pSPIx == SPI2)
			SPI2_PClk_DI();
		else if(pSPIx == SPI3)
			SPI3_PClk_DI();
	}
}


/*
 * SPI Init De-Init functions
 */
void SPI_Init(SPI_Handle_t* pSPIHandle){

	//Enable Peripheral Clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//1. Set Device Mode
	if(pSPIHandle->SPI_Config.Device_Mode == SPI_DEVICE_MODE_MASTER)
		pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_MSTR);
	else
		pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_MSTR);

	//2. Set Bus Configuration
	if(pSPIHandle->SPI_Config.Bus_Config == SPI_BusConfig_FD){
		pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.Bus_Config == SPI_BusConfig_HD){
		pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.Bus_Config == SPI_BusConfig_SIMPLEX_RXONLY){
		pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDIMODE);
		pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_RXONLY);
	}

	//3. Set the DFF bit for either 16 or 8 bit data
	if(pSPIHandle->SPI_Config.DFF == SPI_DFF_16BITS){
		pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_DFF);
	}
	else{
		pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_DFF);
	}

	//4. CPHA & CPOL
	if(pSPIHandle->SPI_Config.CPHA == SPI_CPHA_LOW)
		pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_CPHA);
	else if(pSPIHandle->SPI_Config.CPHA == SPI_CPHA_HIGH)
		pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_CPHA);

	if(pSPIHandle->SPI_Config.CPOL == SPI_CPOL_LOW)
		pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_CPOL);
	else if(pSPIHandle->SPI_Config.CPOL == SPI_CPOL_HIGH)
		pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_CPOL);

	//5. Configure SPI Baud Rate or SPEED
	uint8_t temp = pSPIHandle->SPI_Config.SPEED;
	pSPIHandle->pSPIx->CR1 &= ~(15 << SPI_CR1_BR);
	pSPIHandle->pSPIx->CR1 |= (temp << SPI_CR1_CPOL);

}

void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if(pSPIx == SPI1)
		SPI1_RESET();
	else if(pSPIx == SPI2)
		SPI2_RESET();
	else if(pSPIx == SPI3)
		SPI3_RESET();
}


/*
 * SPI Send & Receive Data functions
 */
void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t len){


	while(len>0){
		//Wait for TXE Flag
		while(!(pSPIx->SR & (1 << SPI_SR_TXE)));

		//Check for Frame format and then write data to DR reg
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			(uint16_t*)pTxBuffer++;
			len=len-2;
		}else{
			pSPIx->DR = *pTxBuffer;
			pTxBuffer++;
			len--;
		}
	}

}

void SPI_RecieveData(SPI_RegDef_t* pSPIx,uint8_t* pRxBuffer, uint32_t len){

	while(len>0){
		//Wait for RXNE Flag
		while(!(pSPIx->SR & (1 << SPI_SR_RXNE)));

		//Check for Frame format and then write data to DR reg
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			(uint16_t*)pRxBuffer++;
			len=len-2;
		}else{
			*pRxBuffer = pSPIx->DR;
			pRxBuffer++;
			len--;
		}
	}

}


/*
 * Send And Recieve Data Function with Interrupt Enabled
 */
uint8_t SPI_SendDataIT(SPI_Handle_t* pSPIHandle, uint8_t* pTxBuffer, uint32_t len){

	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX){
		//1. Save the TxBuffer address and Len information in Some Global Variables
		pSPIHandle->TxLen = len;
		pSPIHandle->pTxBuffer = pTxBuffer;
		//2. Make the SPI state as busy in information so that no other code can take over some SPI Peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );
	}

	//4. Data Transmission will be handled by the ISR Code

	return state;
}


uint8_t SPI_ReceiveDataIT(SPI_Handle_t* pSPIHandle, uint8_t* pRxBuffer, uint32_t len){

	int state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX){
		//1. Save the TxBuffer address and Len information in Some Global Variables
		pSPIHandle->RxLen = len;
		pSPIHandle->pRxBuffer = pRxBuffer;
		//2. Make the SPI state as busy in information so that no other code can take over some SPI Peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );
	}

	//4. Data Transmission will be handled by the ISR Code

	return state;
}

/*
 * SPI IRQ Configuration and Handling functions
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint32_t IRQPriority, uint8_t EnorDi){

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

void SPI_IRQHandling(SPI_Handle_t* pSPIHandle){

	uint8_t temp1, temp2;

	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2){
		//TXE Handle
		spi_txe_interrupt_handler(pSPIHandle);
	}

	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2){
		//TXE Handle
		spi_rxne_interrupt_handler(pSPIHandle);
	}

	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2){
		//TXE Handle
		spi_ovr_err_interrupt_handler(pSPIHandle);
	}
}




static void spi_txe_interrupt_handler(SPI_Handle_t *pSPIHandle){

	//Check for Frame format and then write data to DR reg
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)){
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		(uint16_t*)pSPIHandle->pTxBuffer++;
		pSPIHandle->TxLen = pSPIHandle->TxLen -2;
	}else{
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->pTxBuffer++;
		pSPIHandle->TxLen--;
	}

	if(!pSPIHandle->TxLen){
		pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
		pSPIHandle->pTxBuffer = NULL;
		pSPIHandle->TxLen = 0;
		pSPIHandle->TxState = SPI_READY;

		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}


static void spi_rxne_interrupt_handler(SPI_Handle_t *pSPIHandle){

	//Check for Frame format and then write data to DR reg
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)){
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		(uint16_t*)pSPIHandle->pRxBuffer++;
		pSPIHandle->RxLen = pSPIHandle->RxLen -2;
	}else{
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->RxLen--;
	}

	if(!pSPIHandle->RxLen){
		pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
		pSPIHandle->pRxBuffer = NULL;
		pSPIHandle->RxLen = 0;
		pSPIHandle->RxState = SPI_READY;

		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}


static void spi_ovr_err_interrupt_handler(SPI_Handle_t *pSPIHandle){

	uint8_t temp1,temp2;
	//Clear the OVR Flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		temp1 = pSPIHandle->pSPIx->DR;
		temp2 = pSPIHandle->pSPIx->SR;
	}
	(void)temp1;
	(void)temp2;
	//Inform the Application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}



__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t Event){

}
