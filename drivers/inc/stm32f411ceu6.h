/*
 * stm32f411ceu6.h
 *
 *  Created on: Jan 7, 2025
 *      Author: madhav
 */

#ifndef INC_STM32F411CEU6_H_
#define INC_STM32F411CEU6_H_

#include<stdint.h>
#include<string.h>
#include<stddef.h>
#define __vo						volatile

/****************************PROCESSOR SPECIFIC DETAILS*************************
 *
 * ARM Cortex MX Processors NVIC ISERx Register Addresses
 */
#define NVIC_ISER0					((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1					((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2					((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3					((__vo uint32_t*)0xE000E10c)

/*
 * ARM Cortex MX Processor NVIC ICERx register addresses
 */
#define NVIC_ICER0					((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1					((__vo uint32_t*)0xE000E180)
#define NVIC_ICER2					((__vo uint32_t*)0xE000E180)
#define NVIC_ICER3					((__vo uint32_t*)0xE000E180)

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDRESS		((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED		4

//Flash and SRAM memories base addresses

#define FLASH_BASE_ADDRESS           0x08000000U
#define SRAM_BASE_ADDRESS            0x20000000U
#define ROM_BASE_ADDRESS             0x1FFF0000U
#define SRAM						 SRAM_BASE_ADDRESS
#define ROM							 ROM_BASE_ADDRESS

//AHBx and APBx bus peripheral base addresses

#define PERIPHERAL_BASE_ADDRESS      0x40000000U
#define APB1_PERIPH_BASE_ADDRESS     PERIPHERAL_BASE_ADDRESS
#define APB2_PERIPH_BASE_ADDRESS     0x40010000U
#define AHB1_PERIPH_BASE_ADDRESS     0x40020000U
#define AHB2_PERIPH_BASE_ADDRESS     0x50000000U

//AHB1 bus peripheral base addresses

#define GPIOA_BASE_ADDRESS			(AHB1_PERIPH_BASE_ADDRESS + 0x0000)
#define GPIOB_BASE_ADDRESS			(AHB1_PERIPH_BASE_ADDRESS + 0x0400)
#define GPIOC_BASE_ADDRESS			(AHB1_PERIPH_BASE_ADDRESS + 0x0800)
#define GPIOD_BASE_ADDRESS			(AHB1_PERIPH_BASE_ADDRESS + 0x0C00)
#define GPIOE_BASE_ADDRESS			(AHB1_PERIPH_BASE_ADDRESS + 0x1000)
#define GPIOH_BASE_ADDRESS			(AHB1_PERIPH_BASE_ADDRESS + 0x1C00)

#define RCC_BASE_ADDRESS 			(AHB1_PERIPH_BASE_ADDRESS + 0X3800)
//APB1 bus peripheral base addresses

#define I2C1_BASE_ADDRESS			(APB1_PERIPH_BASE_ADDRESS + 0x5400)
#define I2C2_BASE_ADDRESS			(APB1_PERIPH_BASE_ADDRESS + 0x5800)
#define I2C3_BASE_ADDRESS			(APB1_PERIPH_BASE_ADDRESS + 0x5C00)

#define SPI2_BASE_ADDRESS			(APB1_PERIPH_BASE_ADDRESS + 0x3800)
#define SPI3_BASE_ADDRESS			(APB1_PERIPH_BASE_ADDRESS + 0x3C00)

#define USART2_BASE_ADDRESS 		(APB1_PERIPH_BASE_ADDRESS + 0x4400)

//APB2 bus peripheral base addresses

#define USART1_BASE_ADDRESS			(APB2_PERIPH_BASE_ADDRESS + 0X1000)
#define USART6_BASE_ADDRESS			(APB2_PERIPH_BASE_ADDRESS + 0X1400)

#define SPI1_BASE_ADDRESS			(APB2_PERIPH_BASE_ADDRESS + 0X3000)
#define SPI4_BASE_ADDRESS			(APB2_PERIPH_BASE_ADDRESS + 0X3400)
#define SPI5_BASE_ADDRESS			(APB2_PERIPH_BASE_ADDRESS + 0X5000)


#define SYSCFG_BASE_ADDRESS			(APB2_PERIPH_BASE_ADDRESS + 0X3800)

#define EXTI_BASE_ADDRESS			(APB2_PERIPH_BASE_ADDRESS + 0X3C00)

typedef struct{
	__vo uint32_t MODER;			//GPIO port mode register
	__vo uint32_t OTYPER;			//GPIO port output type register
	__vo uint32_t OSPEEDR;			//GPIO port output speed register
	__vo uint32_t PUPDR;			//GPIO port pull-up/pull-down register
	__vo uint32_t IDR;				//GPIO port input data register
	__vo uint32_t ODR;				//GPIO port output data register
	__vo uint32_t BSRR;				//GPIO port bit set/reset register
	__vo uint32_t LCKR;				//GPIO port configuration lock register
	__vo uint32_t AFR[2];			//GPIO alternate function: AFR[0]: Alternate function low register and AFR[1]: Alternate function high register

}GPIO_RegDef_t;

typedef struct{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t RESERVED0[2];
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t RESERVED2[2];
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t RESERVED4[2];
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t DCKCFGR;

}RCC_RegDef_t;

//peripheral register Definition structure for EXTI
typedef struct{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;

typedef struct{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
    uint32_t RESERVED1[2];
	__vo uint32_t CMPCR;
}SYSCFG_RegDef_t;


/*
 * peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t CRCPR;      /*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t RXCRCR;     /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t TXCRCR;     /*!< TODO,     										Address offset: 0x18 */
	__vo uint32_t I2SCFGR;    /*!< TODO,     										Address offset: 0x1C */
	__vo uint32_t I2SPR;      /*!< TODO,     										Address offset: 0x20 */
} SPI_RegDef_t;


// GPIO peripheral definitions( Peripheral base addresses typecasted to GPIO_RegDef_t structure type

#define GPIOA						((GPIO_RegDef_t*) GPIOA_BASE_ADDRESS)
#define GPIOB						((GPIO_RegDef_t*) GPIOB_BASE_ADDRESS)
#define GPIOC						((GPIO_RegDef_t*) GPIOC_BASE_ADDRESS)
#define GPIOD						((GPIO_RegDef_t*) GPIOD_BASE_ADDRESS)
#define GPIOE						((GPIO_RegDef_t*) GPIOE_BASE_ADDRESS)
#define GPIOH						((GPIO_RegDef_t*) GPIOH_BASE_ADDRESS)
#define RCC							((RCC_RegDef_t*) RCC_BASE_ADDRESS)
#define EXTI						((EXTI_RegDef_t*) EXTI_BASE_ADDRESS)
#define SYSCFG						((SYSCFG_RegDef_t*) SYSCFG_BASE_ADDRESS)

//SPI Peripheral definitions:
#define SPI1						((SPI_RegDef_t*) SPI1_BASE_ADDRESS)
#define SPI2						((SPI_RegDef_t*) SPI2_BASE_ADDRESS)
#define SPI3						((SPI_RegDef_t*) SPI3_BASE_ADDRESS)
#define SPI4						((SPI_RegDef_t*) SPI4_BASE_ADDRESS)
#define SPI5						((SPI_RegDef_t*) SPI5_BASE_ADDRESS)

//Clock enable macros for GPIOx peripherals
#define GPIOA_PERI_CLOCK_EN()	(RCC->AHB1ENR |=(1<<0))
#define GPIOB_PERI_CLOCK_EN()	(RCC->AHB1ENR |=(1<<1))
#define GPIOC_PERI_CLOCK_EN()	(RCC->AHB1ENR |=(1<<2))
#define GPIOD_PERI_CLOCK_EN()	(RCC->AHB1ENR |=(1<<3))
#define GPIOE_PERI_CLOCK_EN()	(RCC->AHB1ENR |=(1<<4))
#define GPIOH_PERI_CLOCK_EN()	(RCC->AHB1ENR |=(1<<7))

//Clock enable macros for I2Cx peripherals
#define I2C1_PERI_CLOCK_EN()		(RCC->APB1ENR |=(1<<21))
#define I2C2_PERI_CLOCK_EN()		(RCC->APB1ENR |=(1<<22))
#define I2C3_PERI_CLOCK_EN()		(RCC->APB1ENR |=(1<<23))



//Clock enable macros for SPIx peripherals
#define SPI2_PERI_CLOCK_EN()		(RCC->APB1ENR |=(1<<14))
#define SPI3_PERI_CLOCK_EN()		(RCC->APB1ENR |=(1<<15))
#define SPI1_PERI_CLOCK_EN()		(RCC->APB2ENR |=(1<<12))
#define SPI4_PERI_CLOCK_EN()		(RCC->APB2ENR |=(1<<13))
#define SPI5_PERI_CLOCK_EN()		(RCC->APB2ENR |=(1<<20))


//Clock enable macros for USARTx peripherals
#define USART2_PERI_CLOCK_EN()		(RCC->APB1ENR |=(1<<17))
#define USART1_PERI_CLOCK_EN()		(RCC->APB2ENR |=(1<<4))
#define USART6_PERI_CLOCK_EN()		(RCC->APB2ENR |=(1<<5))

//Clock enable macros for SYSCFG peripheral
#define SYSCFG_CLOCK_EN()			(RCC->APB2ENR |=(1<<14))

//Clock disable macros for GPIOx peripherals
#define GPIOA_PERI_CLOCK_DI()	(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PERI_CLOCK_DI()	(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PERI_CLOCK_DI()	(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PERI_CLOCK_DI()	(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PERI_CLOCK_DI()	(RCC->AHB1ENR &= ~(1<<4))
#define GPIOH_PERI_CLOCK_DI()	(RCC->AHB1ENR &= ~(1<<7))

//Clock disable macros for I2Cx peripherals
#define I2C1_PERI_CLOCK_DI()		(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PERI_CLOCK_DI()		(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PERI_CLOCK_DI()		(RCC->APB1ENR &= ~(1<<23))

//Clock disable macros for SPIx peripherals
#define SPI2_PERI_CLOCK_DI()		(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PERI_CLOCK_DI()		(RCC->APB1ENR &= ~(1<<15))
#define SPI1_PERI_CLOCK_DI()		(RCC->APB2ENR &= ~(1<<12))
#define SPI4_PERI_CLOCK_DI()		(RCC->APB2ENR &= ~(1<<13))
#define SPI5_PERI_CLOCK_DI()		(RCC->APB2ENR &= ~(1<<20))

//Clock disable macros for USARTx peripherals
#define USART2_PERI_CLOCK_DI()		(RCC->APB1ENR &= ~(1<<17))
#define USART1_PERI_CLOCK_DI()		(RCC->APB2ENR &= ~(1<<4))
#define USART6_PERI_CLOCK_DI()		(RCC->APB2ENR &= ~(1<<5))

//Clock disable macros for SYSCFG peripheral
#define SYSCFG_CLOCK_DI()			(RCC->APB2ENR &= ~(1<<14))

//Macros to reset GPIOx peripherals
#define GPIOA_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0));} while(0)
#define GPIOB_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1));} while(0)
#define GPIOC_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2));} while(0)
#define GPIOD_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3));} while(0)
#define GPIOE_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4));} while(0)
#define GPIOH_REG_RESET()			do{(RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7));} while(0)

#define GPIO_BASEADDRESS_TO_CODE(x)	((x==GPIOA) ? 0 :\
									(x==GPIOB) ? 1 :\
									(x==GPIOC) ? 2 :\
									(x==GPIOD) ? 3 :\
									(x==GPIOE) ? 4 :\
									(x==GPIOH) ? 7 :0)

//Macros to reset SPIx peripherals
#define SPI1_REG_RESET()    		do { RCC->APB2RSTR |= (1 << 12); RCC->APB2RSTR &= ~(1 << 12); } while(0)
#define SPI2_REG_RESET()   			do { RCC->APB1RSTR |= (1 << 14); RCC->APB1RSTR &= ~(1 << 14); } while(0)
#define SPI3_REG_RESET()    		do { RCC->APB1RSTR |= (1 << 15); RCC->APB1RSTR &= ~(1 << 15); } while(0)
#define SPI4_REG_RESET()    		do { RCC->APB2RSTR |= (1 << 13); RCC->APB2RSTR &= ~(1 << 13); } while(0)
#define SPI5_REG_RESET()    		do { RCC->APB2RSTR |= (1 << 20); RCC->APB2RSTR &= ~(1 << 20); } while(0)

/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

/* IRQ( Interrupt Request) numbers of STM32F411CEU6f MCU
 */
#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI9_5				23
#define IRQ_NO_EXTI15_10			40
#define IRQ_NO_SPI1					35
#define IRQ_NO_SPI2       		    36
#define IRQ_NO_SPI3       		    51
#define IRQ_NO_SPI4       		    84
#define IRQ_NO_SPI5       		    85

// IRQ Priority Macros :

#define NVIC_IRQ_PRI0				0
#define NVIC_IRQ_PRI1				1
#define NVIC_IRQ_PRI2				2
#define NVIC_IRQ_PRI3				3
#define NVIC_IRQ_PRI4				4
#define NVIC_IRQ_PRI5				5
#define NVIC_IRQ_PRI6				6
#define NVIC_IRQ_PRI7				7
#define NVIC_IRQ_PRI8				8
#define NVIC_IRQ_PRI9				9
#define NVIC_IRQ_PRI10				10
#define NVIC_IRQ_PRI11				11
#define NVIC_IRQ_PRI12				12
#define NVIC_IRQ_PRI13				13
#define NVIC_IRQ_PRI14				14
#define NVIC_IRQ_PRI15				15


//Some Generic Macros :
#define ENABLE						1
#define DISABLE						0
#define SET							ENABLE
#define RESET						DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET
#define FLAG_RESET					RESET
#define FLAG_SET					SET


#include "stm32f411ceu6_gpio_driver.h"
#include "stm32f411ceu6_spi_driver.h"

#endif /* INC_STM32F411CEU6_H_ */
