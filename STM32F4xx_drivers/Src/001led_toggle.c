/*
 * 001led_toggle.c
 *
 *  Created on: Jun 25, 2024
 *      Author: bento
 */

/*
 * IF ICON IS GREYED OUT WITH A STRIKETHROUGH IT IS BECAUSE THE FILE HAS BEEN EXCLUDED FROM THE BUILD
 * can just go to the settings and include it if need be
 * right click file -> Resource Configurations -> Exclude from build... -> select which configurations to exclude from
 */

#include "stm32f407xx.h"

void delay(void)
{
	for(uint32_t i = 0; i<500000; i++);
}

int main(void)
{
	// create a GPIO Handler (using our defined struct)
	GPIO_Handle_t GpioLed;

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
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();
	}






	return 0;
}
