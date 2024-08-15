/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Jun 21, 2024
 *      Author: bento
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_


#include "stm32f407xx.h"


/*
 *  Configuration structure for a GPIO pin
 */

typedef struct{

	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;			// possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;			// possible values from @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;	// possible values from @GPIO_PUPD_CFG
	uint8_t GPIO_PinOPType;			// possible values from @GPIO_OP_TYPE
	uint8_t GPIO_PinAltFunMode;


}GPIO_PinConfig_t;


/*
 *  Handle  structure for a GPIO pin
 */

typedef struct{

	// pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx;					// this holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;		// this holds the GPIO pin configuration settings

}GPIO_Handle_t;			//_t indicates its a structure (maybe?) (convention)


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/*
 * @GPIO_PIN_MODES
 * GPIO possible pin modes
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4		// Interrupt Falling Edge Triggered
#define GPIO_MODE_IT_RT		5		// Interrupt Rising Edge Triggered
#define GPIO_MODE_IT_RFT	6		// Interrupt Rising or Falling Edge Triggered

/*
 * @GPIO_OP_TYPE
 * GPIO possible Output types
 */
// GPIO possible output types
#define GPIO_OP_TYPE_PP		0		// Output Push-Pull type
#define GPIO_OP_TYPE_OD		1		// Output Open-Drain type


/*
 * @GPIO_PIN_SPEED
 * GPIO possible Output Speeds
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MED		1
#define GPIO_SPEED_HIGH		2
#define GPIO_SPEED_VHIGH	3


/*
 * @GPIO_PUPD_CFG
 * GPIO PuPd configurations
 */
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2




/***************************************************************************************************************
 * 									APIs supported by this driver
 * 						For more info about the APIs check the function definitions
 ***************************************************************************************************************/


/*
 *  Peripheral Clock Setup
 */
void GPIO_PeriClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 *  Init & DeInit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx); // instead of resetting each individual bits/registers we can use the
										 // Peripheral Reset Register to reset everything at once. Therefore, the only
										 // parameter we need is the base address of the peripheral

/*
 *  Data read & write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 *  IRQ Config & ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi); // could also include IRQGrouping but we won't include it here
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);










#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
