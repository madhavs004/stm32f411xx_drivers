/*
 * stm32f411ceu6_gpio_driver.c
 *
 *  Created on: Feb 9, 2025
 *      Author: madhav
 */

#include "stm32f411ceu6_gpio_driver.h"

// Peripheral Clock Setup :
/*************************************************************************************
 * Function : 			GPIO_PeriClockControl
 * Brief:				Enables or disable peripheral clock for the given GPIO Port
 *
 * Param[in]:			Base address of the GPIO peripheral
 * Param[in]:			Enable or Disable macros
 *
 * Return: 				None
 * Note:				None
 *************************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx , uint8_t EnorDi){
	if(EnorDi==ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PERI_CLOCK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PERI_CLOCK_EN();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PERI_CLOCK_EN();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PERI_CLOCK_EN();
		}
		else if (pGPIOx == GPIOE){
			GPIOE_PERI_CLOCK_EN();
		}
		else if (pGPIOx == GPIOH){
			GPIOH_PERI_CLOCK_EN();
		}
	}
	else{
		if(pGPIOx == GPIOA){
			GPIOA_PERI_CLOCK_DI();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PERI_CLOCK_DI();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PERI_CLOCK_DI();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PERI_CLOCK_DI();
		}
		else if (pGPIOx == GPIOE){
			GPIOE_PERI_CLOCK_DI();
		}
		else if (pGPIOx == GPIOH){
			GPIOH_PERI_CLOCK_DI();
		}
	}
}

// Init and De-Init :

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp=0;
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	//1.Configure the mode of the GPIO pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		temp = pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);  // clearing
		pGPIOHandle->pGPIOx->MODER |= temp;		//setting
		temp=0;
	}
	else{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			//1.configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//1.configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			//1.configure both FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		//2.configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDRESS_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		//3.eNnable the exti interrupt delivery using IMR
		EXTI->IMR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp=0;

	//2.Configure the Speed of the GPIO pin
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);  // clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp =0;

	//3.Configure the PUPD Settings
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);  // clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp =0;

	//4.Configure the O/P Type
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);  // clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp =0;

	//5.Configure the Alternate Functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8 ;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8 ;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0XF << ( 4 * temp2)) ;
		pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2) ;
	}

}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();	}
	else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();	}
	else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();	}
	else if (pGPIOx == GPIOE){
		GPIOE_REG_RESET();	}
	else if (pGPIOx == GPIOH){
		GPIOH_REG_RESET();	}
}

//Data Read and Write :
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}



uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}



void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){
	if(Value == GPIO_PIN_SET){
		//write 1 to the output data register at the bit field corresponding to the Pin Number
		pGPIOx->ODR |= (1<< PinNumber);
	}
	else{
		//write 0 to the output data register at the bit field corresponding to the Pin Number
		pGPIOx->ODR &= ~(1<< PinNumber);
	}
}


void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR = Value;
}



void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber ){
	pGPIOx->ODR ^= (1<< PinNumber);
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDRESS + iprx) |= (IRQPriority << shift_amount);
}
//IRQ Config and ISR Handling :
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
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
}
void GPIO_IRQHandling(uint8_t PinNumber){
	//clear the EXTI PR register corresponding to the pin number
	if(EXTI->PR & (1<< PinNumber)){
		//clear
		EXTI->PR |= (1<<PinNumber);
	}
}
