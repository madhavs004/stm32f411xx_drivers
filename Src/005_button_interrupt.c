/*
 * 005_button_interrupt.c
 *
 *  Created on: Apr 14, 2025
 *      Author: madhav
 */

#include<string.h>
#include "stm32f411ceu6.h"
#include "stm32f411ceu6_gpio_driver.h"

void delay(void){
	for(uint32_t i =0; i<5000000 ; i++);
}

int main(void){

	GPIO_Handle_t GPIO_Led, GPIO_Button;
	memset(&GPIO_Led, 0 , sizeof(GPIO_Led));
	memset(&GPIO_Button, 0 , sizeof(GPIO_Button));


	GPIO_Led.pGPIOx = GPIOD;
	GPIO_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GPIO_Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GPIO_Led);

	GPIO_Button.pGPIOx = GPIOD;
	GPIO_Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIO_Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GPIO_Button);

	//IRQ Configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15 );
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	while(1);
	return 0;
}

void EXTI9_5_IRQHandler(void){
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_5); //clear the pending event from the EXTI line
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}
