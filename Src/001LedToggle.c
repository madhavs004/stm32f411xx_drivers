/*
 * 001LedToggle.c
 *
 *  Created on: Mar 19, 2025
 *      Author: madhav
 */


#include "stm32f411ceu6.h"
#include "stm32f411ceu6_gpio_driver.h"

void delay(void){
	for(uint32_t i =0; i<5000000 ; i++);
}

int main(void){

	GPIO_Handle_t GPIO_Led;
	GPIO_Led.pGPIOx = GPIOC;

	GPIO_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GPIO_Led);

	while(1){
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_13);
		delay();
	}

	return 0;
}
