/*
 * stm32f411ceu6_spi_driver.c
 *
 *  Created on: May 23, 2025
 *      Author: madhav
 */


#include "stm32f411ceu6_spi_driver.h"

static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx , uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PERI_CLOCK_EN();
		}
		else if(pSPIx == SPI2){
			SPI2_PERI_CLOCK_EN();
		}
		else if(pSPIx == SPI3){
			SPI3_PERI_CLOCK_EN();
		}
		else if(pSPIx == SPI4){
			SPI4_PERI_CLOCK_EN();
		}
		else if(pSPIx == SPI5){
			SPI5_PERI_CLOCK_EN();
		}
	}
	else {
		if(pSPIx == SPI1){
			SPI1_PERI_CLOCK_DI();
		}
		else if(pSPIx == SPI2){
			SPI2_PERI_CLOCK_DI();
		}
		else if(pSPIx == SPI3){
			SPI3_PERI_CLOCK_DI();
		}
		else if(pSPIx == SPI4){
			SPI4_PERI_CLOCK_DI();
		}
		else if(pSPIx == SPI5){
			SPI5_PERI_CLOCK_DI();
		}
	}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= (1<<6);
	}else{
		pSPIx->CR1 &= ~(1<<6);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= (1<<8);
	}else{
		pSPIx->CR1 &= ~(1<<8);
	}
}

void SPI_SSOE_Config(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pSPIx->CR2 |= (1<< 2);
	}
	else{
		pSPIx->CR2 &= ~(1<< 2);
	}
}


// Init and De-Init :
void SPI_Init(SPI_Handle_t *pSPIHandle){
	uint32_t tempreg = 0;
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
	//1. Configuring the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	//2. Configuring the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		//Bidirectional mode should be cleared
		tempreg &= ~(1<<15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		//Bidirection mode should be set
		tempreg |= (1<<15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_S_RXONLY){
		//make sure BiDi mode is cleared
		tempreg &= ~(1<< 15);
		//RxOnly bit must be set
		tempreg |= (1<<10);
	}

	//3. Configuring the Clock Speed
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << 3;

	//4. Configuring the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL<<1;

	//5. Configuring the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA <<0 ;

	//6. Configuring the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << 11;

	//7. Configuring the SSM
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << 9;

	pSPIHandle->pSPIx->CR1 = tempreg;
}



void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if(pSPIx == SPI1){
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2){
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3){
		SPI3_REG_RESET();
	}
	else if(pSPIx == SPI4){
		SPI4_REG_RESET();
	}
	else if(pSPIx == SPI5){
		SPI5_REG_RESET();
	}
}



uint8_t SPI_GetFlagSTatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){

	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}



//Data Read and Write :
void SPI_SendData(SPI_RegDef_t *pSPIx , uint8_t *pTxBuffer , uint32_t Len){
	while(Len > 0){
		//1. Wait until TXE is set
		while(SPI_GetFlagSTatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2. Check the DFF Bit
		if(pSPIx->CR1 & (1 << 11)){
			//16 bit DFF
			//1. Load the data into the DR
			pSPIx->DR = *(uint16_t*)pTxBuffer;
			Len--;
			Len--;
			(uint16_t*) pTxBuffer++ ;
		}
		else{
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1 . Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2.  Mark the SPI state as busy in transmission so that
		//    no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << 7 );

	}


	return state;
}

void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer , uint32_t Len){
	while(Len > 0){
		//1. Wait until RXNE is set
		while(SPI_GetFlagSTatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//2. Check the DFF Bit
		if(pSPIx->CR1 & (1 << 11)){
			//16 bit DFF
			//1. Load the data from DR to RxBuffer Address
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*) pRxBuffer++ ;
		}
		else{
			//8 bit DFF
			*(pRxBuffer) = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		//1 . Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		//2.  Mark the SPI state as busy in reception so that
		//    no other code can take over same SPI peripheral until reception is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << 6 );

	}


	return state;

}

//IRQ Config and ISR Handling :
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi== ENABLE){
		if(IRQNumber <=31){
			//program ISER0 register
			*NVIC_ISER0 |= (1<< IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber <64){
			//program ISER1 register
			*NVIC_ISER1 |= (1<< IRQNumber % 32);

		}
		else if(IRQNumber >=64 && IRQNumber <96){
			//program ISER2 register
			*NVIC_ISER2 |= (1<< IRQNumber % 64);

		}
	}else{
		if(IRQNumber <=31){
			//program ISER0 register
			*NVIC_ICER0 |= (1<< IRQNumber);


		}else if(IRQNumber > 31 && IRQNumber <64){
			//program ISER1 register
			*NVIC_ICER1 |= (1<< IRQNumber & 32);

		}
		else if(IRQNumber >=64 && IRQNumber <96){
			//program ISER2 register
			*NVIC_ICER2 |= (1<< IRQNumber & 64);

		}
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDRESS + iprx) |= (IRQPriority << shift_amount);
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){
	uint8_t temp1 , temp2;
	//first lets check for TXE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if( temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	// check for RXNE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	if( temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	// check for ovr flag
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if( temp1 && temp2)
	{
		//handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}
}
}
