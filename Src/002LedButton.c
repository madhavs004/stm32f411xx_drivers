/*
 * 002LedButton.c
 *
 *  Created on: Mar 23, 2025
 *      Author: madhav
 */


#include "stm32f411ceu6.h"
#define LOW 0
#define BTN_PRESSED LOW

void delay(void){
	for(uint32_t i =0; i<5000000/2 ; i++);
}

int main(void){

	GPIO_Handle_t GPIO_Led, GPIO_Button;
	GPIO_Led.pGPIOx = GPIOC;

	GPIO_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GPIO_Led);

	GPIO_Button.pGPIOx = GPIOA;
	GPIO_Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIO_Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIO_Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GPIO_Button);

	while(1){
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)== BTN_PRESSED){
			delay();
			GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_13);
		}
	}

	return 0;
}
