/*
 * main.c
 *
 *  Created on: Jul 3, 2024
 *      Author: bento
 */


#include "stm32f407xx.h"

int main(void)
{
	return 0;
}


void EXTI0_IRQHandler(void)
{
	// handle the interrupt
	GPIO_IRQHandling(0);
}
