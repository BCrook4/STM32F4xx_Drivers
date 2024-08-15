/*
 * 005Button_interrupt.c
 *
 *  Created on: Jul 26, 2024
 *      Author: bento
 */

#include <string.h>
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

	// initialise our structures to 0 to ensure they're not populated by garbage values
	// memset is part of the standard library string.h
	memset(&GpioLed, 0, sizeof(GpioLed));
	memset(&GpioButton, 0, sizeof(GpioButton));



	// LED config
	// assign GPIO: from schematic, green led is connected to PD12 aka GPIOD Pin 12
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;


	// need to enable the peripheral clock using our clock control API
	GPIO_PeriClkControl(GPIOD, ENABLE);

	// initialise our GPIO
	GPIO_Init(&GpioLed);


	// Button Config (to test code using internal button use GPIOA pin 0)
	GpioButton.pGPIOx = GPIOD;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_PeriClkControl(GPIOD, ENABLE);

	GPIO_Init(&GpioButton);

	// IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	while(1);

	//return 0;
}

void EXTI9_5_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);

}
