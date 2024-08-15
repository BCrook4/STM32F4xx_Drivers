/*
 * 002led_button.c
 *
 *  Created on: Jun 25, 2024
 *      Author: bento
 */
#include "stm32f407xx.h"

#define HIGH			1
#define BTN_PRESSED		HIGH

void delay(void)
{
	for(uint32_t i = 0; i<500000; i++);
}

int main(void){


	GPIO_Handle_t	GpioButton;
	GPIO_Handle_t GpioLed;

	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // the schematic shows an external pull down resister
																  // is already included in the circuit so we don't need
																  // to configure one

	GPIO_PeriClkControl(GPIOA, ENABLE);

	GPIO_Init(&GpioButton);

	// assign GPIO: from schematic, green led is connected to PD12 aka GPIOD Pin 12
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// need to enable the peripheral clock using our clock control API
	GPIO_PeriClkControl(GPIOD, ENABLE);

	// initialise our GPIO
	GPIO_Init(&GpioLed);

	while(1){

		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
		{
			delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}

	}

	return 0;
}
