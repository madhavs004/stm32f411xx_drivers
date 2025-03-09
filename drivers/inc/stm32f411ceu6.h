/*
 * stm32f411ceu6.h
 *
 *  Created on: Jan 7, 2025
 *      Author: madhav
 */

#ifndef INC_STM32F411CEU6_H_
#define INC_STM32F411CEU6_H_

#include<stdint.h>
#define __vo						volatile
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


//peripheral definitions( Peripheral base addresses typecasted to GPIO_RegDef_t structure type

#define GPIOA						((GPIO_RegDef_t*) GPIOA_BASE_ADDRESS)
#define GPIOB						((GPIO_RegDef_t*) GPIOB_BASE_ADDRESS)
#define GPIOC						((GPIO_RegDef_t*) GPIOC_BASE_ADDRESS)
#define GPIOD						((GPIO_RegDef_t*) GPIOD_BASE_ADDRESS)
#define GPIOE						((GPIO_RegDef_t*) GPIOE_BASE_ADDRESS)
#define GPIOH						((GPIO_RegDef_t*) GPIOH_BASE_ADDRESS)
#define RCC							((RCC_RegDef_t*) RCC_BASE_ADDRESS)

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
#define GPIOA_REG_RESET()
#define GPIOB_REG_RESET()
#define GPIOC_REG_RESET()
#define GPIOD_REG_RESET()
#define GPIOE_REG_RESET()
#define GPIOH_REG_RESET()


//Some Generic Macros :
#define ENABLE						1
#define DISABLE						0
#define SET							ENABLE
#define RESET						DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET


#endif /* INC_STM32F411CEU6_H_ */
