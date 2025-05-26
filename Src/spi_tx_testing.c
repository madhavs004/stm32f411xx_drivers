/*
 * spi_tx_testing.c
 *
 *  Created on: May 25, 2025
 *      Author: madhav
 */
#include "stm32f411ceu6.h"
#include "string.h"
/*
 * PB 15:  SPI2_MOSI
 * PB 14:  SPI2_MISO
 * PB 13:  SPI2_SCK
 * PB 12:  SPI2_NSS
 * Alternate Functionality Mode : 5
 */

	void SPI2_GPIOInits(void){
		GPIO_Handle_t SPIPins;
		SPIPins.pGPIOx = GPIOB;
		SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
		SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
		SPIPins.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
		SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
		SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

		//SCLK
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
		GPIO_Init(&SPIPins);

		//MOSI
		SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
		GPIO_Init(&SPIPins);
		//MISO
		//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
		//GPIO_Init(&SPIPins);
	}

	void SPI2_Inits(void){

		SPI_Handle_t spi2handle;

		spi2handle.pSPIx = SPI2;
		spi2handle.SPIConfig.SPI_DeviceMode = SPI_MASTER;
		spi2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
		spi2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;	//generates SCLK of 8 MHz
		spi2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
		spi2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
		spi2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
		spi2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;		//software slave management enabled for NSS pin

		SPI_Init(&spi2handle);
	}

int main(void){

	char user_data[] = "Hello World";

	// This function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//Initialize the handle structure of SPI
	SPI2_Inits();

	//this make NSS signal internally high and avoids MODF error
	SPI_SSIConfig( SPI2, ENABLE);

	//Enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	SPI_PeriClockControl(SPI2, DISABLE);

	while(1);
	return 0;
}
