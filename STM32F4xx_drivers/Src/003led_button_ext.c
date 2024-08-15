/*
 * 002led_button.c
 *
 *  Created on: Jun 25, 2024
 *      Author: bento
 */
#include "stm32f407xx.h"

#define HIGH			1
#define LOW				0
#define BTN_PRESSED		LOW

void delay(void)
{
	for(uint32_t i = 0; i<500000/2; i++);
}

int main(void){


	GPIO_Handle_t GpioButton;
	GPIO_Handle_t GpioLed;

	// Button Config
	GpioButton.pGPIOx = GPIOB;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClkControl(GPIOB, ENABLE);

	GPIO_Init(&GpioButton);

	// LED Config (to test code using internal LED use GPIOA Pin 12
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// need to enable the peripheral clock using our clock control API
	GPIO_PeriClkControl(GPIOA, ENABLE);

	// initialise our GPIO
	GPIO_Init(&GpioLed);

	while(1){

		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == BTN_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
		}

	}

	return 0;
}
