/*
 * stm32f411ceu6_spi_driver.h
 *
 *  Created on: May 23, 2025
 *      Author: madhav
 */

#ifndef INC_STM32F411CEU6_SPI_DRIVER_H_
#define INC_STM32F411CEU6_SPI_DRIVER_H_

#include "stm32f411ceu6.h"

typedef struct{

	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;

typedef struct{

	SPI_RegDef_t		*pSPIx;
	SPI_Config_t		SPIConfig;
	uint8_t 			*pTxBuffer; /* !< To store the app. Tx buffer address > */
	uint8_t 			*pRxBuffer;	/* !< To store the app. Rx buffer address > */
	uint32_t 			TxLen;		/* !< To store Tx len > */
	uint32_t 			RxLen;		/* !< To store Tx len > */
	uint8_t 			TxState;	/* !< To store Tx state > */
	uint8_t 			RxState;	/* !< To store Rx state > */
}SPI_Handle_t;;

/*
 * SPI application states
 */
#define SPI_READY 					0
#define SPI_BUSY_IN_RX 				1
#define SPI_BUSY_IN_TX 				2

/*
 * SPI_DeviceMode
 */
#define SPI_MASTER		1
#define SPI_SLAVE		0

/*
 * SPI_BusConfig
 */

#define SPI_BUS_CONFIG_FD			0
#define SPI_BUS_CONFIG_HD			1
#define SPI_BUS_CONFIG_S_RXONLY		2

/*
 * SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7

/*
 * SPI_DFF
 */
#define SPI_DFF_8BITS				0
#define SPI_DFF_16BITS				1

/*
 * CPOL
 */
#define SPI_CPOL_HIGH				1
#define SPI_CPOL_LOW 				0

/*
 * CPHA
 */
#define SPI_CPHA_HIGH				1
#define SPI_CPHA_LOW 				0

/*
 * SSM
 */
#define SPI_SSM_EN					1
#define SPI_SSM_DI					0

#define SPI_TXE_FLAG				(1 << 1)
#define SPI_RXNE_FLAG				(1 << 0)
#define SPI_BUSY_FLAG				(1 << 7)

//possible SPI application events
#define SPI_EVENT_TX_CMPLT  1
#define SPI_EVENT_RX_CMPLT  2
#define SPI_EVENT_OVR_CMPLT 3


/****************************************************************************
 * 				APIs Supported by this driver
 * 		For more info about the APIs check the function definitions
 ****************************************************************************/

// Peripheral Clock Setup :
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx , uint8_t EnorDi);

// Init and De-Init :
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

//Data Read and Write :
void SPI_SendData(SPI_RegDef_t *pSPIx , uint8_t *pTxBuffer , uint32_t Len);
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer , uint32_t len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle , uint8_t *pTxBuffer , uint32_t Len);
uint8_t SPI_RecieveDataIT(SPI_RegDef_t *pSPIHandle, uint8_t *pRxBuffer , uint32_t len);

//IRQ Config and ISR Handling :
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

// Other peripheral control APIs

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOE_Config(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagSTatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

void SPI_Application_event_callback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent);



#endif /* INC_STM32F411CEU6_SPI_DRIVER_H_ */
