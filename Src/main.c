/*
 * main.c
 *
 *  Created on: Jun 2, 2025
 *      Author: madhav
 */


/*
 * 001LED_Toggle.c
 *
 *  Created on: Dec 24, 2024
 *      Author: bhoomika
 */
#include "stm32f411ceu6.h"
#include "stm32f411ceu6_gpio_driver.h"

void delay(void){
	for(uint32_t i = 0; i< 5000000/2; i++);
}

int main(void){
	GPIO_Handle_t GPIO_LED;
	GPIO_Handle_t GPIO_PC14;
	GPIO_PC14.pGPIOx = GPIOC;
	GPIO_PC14.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_PC14.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIO_PC14.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;
	GPIO_Init(&GPIO_PC14);

	GPIO_LED.pGPIOx                              = GPIOC;
	GPIO_LED.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN_NO_13;
	GPIO_LED.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_OUT;
	GPIO_LED.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_FAST;
	GPIO_LED.GPIO_PinConfig.GPIO_PinOPType       = GPIO_OP_TYPE_PP;
	GPIO_LED.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_PIN_PU;


	GPIO_Init(&GPIO_LED);
	while(1){
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_13);

	}
	return 0;

}
