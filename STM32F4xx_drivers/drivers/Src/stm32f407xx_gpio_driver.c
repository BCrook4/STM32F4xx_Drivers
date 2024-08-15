/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Jun 21, 2024
 *      Author: bento
 */


#include "stm32f407xx_gpio_driver.h"



/*
 *  Peripheral Clock Setup
 */

/*******************************************************************************************
 *
 * @fn					- GPIO_PeriClkControl
 *
 * @brief				- This fn enables or disables peripheral clock for given GPIO port
 *
 * @param[in]			- base address of the GPIO peripheral
 * @param[in]			- ENABLE or DISABLE macros
 * @param[in]			-
 *
 * @return				- none
 *
 * @Note				- none
 *
 */

//something is up with this function. it doesn't like the GPIOx_PCLK_EN() macros
void GPIO_PeriClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{

			if(pGPIOx == GPIOA)
			{
				GPIOA_PCLK_EN();
			}else if(pGPIOx == GPIOB)
			{
				GPIOB_PCLK_EN();
			}else if(pGPIOx == GPIOC)
			{
				GPIOC_PCLK_EN();
			}else if(pGPIOx == GPIOD)
			{
				GPIOD_PCLK_EN();
			}else if(pGPIOx == GPIOE)
			{
				GPIOE_PCLK_EN();
			}else if(pGPIOx == GPIOF)
			{
				GPIOF_PCLK_EN();
			}else if(pGPIOx == GPIOG)
			{
				GPIOG_PCLK_EN();
			}else if(pGPIOx == GPIOH)
			{
				GPIOH_PCLK_EN();
			}else if(pGPIOx == GPIOI)
			{
				GPIOI_PCLK_EN();
			}
		}

		else
		{
			if(pGPIOx == GPIOA)
					{
						GPIOA_PCLK_DI();
					}else if(pGPIOx == GPIOB)
					{
						GPIOB_PCLK_DI();
					}else if(pGPIOx == GPIOC)
					{
						GPIOC_PCLK_DI();
					}else if(pGPIOx == GPIOD)
					{
						GPIOD_PCLK_DI();
					}else if(pGPIOx == GPIOE)
					{
						GPIOE_PCLK_DI();
					}else if(pGPIOx == GPIOF)
					{
						GPIOF_PCLK_DI();
					}else if(pGPIOx == GPIOG)
					{
						GPIOG_PCLK_DI();
					}else if(pGPIOx == GPIOH)
					{
						GPIOH_PCLK_DI();
					}else if(pGPIOx == GPIOI)
					{
						GPIOI_PCLK_DI();
					}
		}




}


/*
 *  Init & DeInit
 */

/*******************************************************************************************
 *
 * @fn					- GPIO_Init
 *
 * @brief				- This fn initialises the given GPIO peripheral
 *
 * @param[in]			-
 * @param[in]			-
 * @param[in]			-
 *
 * @return				- none
 *
 * @Note				- none
 *
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;	// temporary register

	// 1. Configure the mode of GPIO pin

	// if GPIO pin mode is a non-interrupt mode (i.e. <= GPIO_MODE_ANALOG)
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// non-interrupt mode

		// check the GPIO port mode register to see why we do the shift by 2 * PinNumber
		// (its because each pin takes 2 bits to config so the config bits correspond to 2x the pin number)
		temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		// now temp holds the desired Pin Mode at the bit position corresponding to the desired pin number
		// now we set the actual bits in the register below then reinitialise temp to 0;
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
		pGPIOHandle->pGPIOx->MODER |= temp; // setting
		temp = 0;

	}else
	{
		// interrupt mode
		// this part is for interrupt mode
			if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
			{
				// 1. configure the FTSR (falling edge triggered service register)
				EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

				// clear corresponding RTSR bit
				EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			}
			else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
			{
				// 1. configure the RTSR
				EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

				// clear corresponding FTSR bit
				EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			}
			else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
			{
				// 1. configure both FTSR & RTSR
				EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

				EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			}

			// 2. configure the GPIO port selection in SYSCFG_EXTICR

			// to get the correct EXTICR[] register array index, divide pin number by 4
			// to get the correct bits, we need to take pin # % 4 then multiply result by 4

			uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
			uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
			uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
			SYSCFG_PCLK_EN();
			SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);


			// 3. enable the exti interrupt delivery using IMR (Interrupt Mask Register)
			EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;
	// 2. Configure the speed
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp; // setting

	temp = 0;

	// 3. Configure the PuPd settings
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp; // setting

	temp = 0;

	// 4. Configure the OPtype
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp; // setting

	temp = 0;

	// 5. configure the alt functionality
	// can only be configured if mode is set to alternate functionality mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// configure alternate function registers
		uint8_t temp1, temp2;

		// alt functionality has both high & low registers which weve stored in an array.
		// temp1 will get the index of the array for our desired pin while temp2 * 4 will give us the bit position
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2) ); // clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2) ); // setting

	}

}

/*******************************************************************************************
 *
 * @fn					- GPIO_DeInit
 *
 * @brief				- This fn De-initialises the given GPIO peripheral by writing to the Peripheral Reset Register
 *
 * @param[in]			- base address of the given GPIO peripheral
 * @param[in]			-
 * @param[in]			-
 *
 * @return				- none
 *
 * @Note				- none
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}

}

/*
 *  Data read & write
 */

/*******************************************************************************************
 *
 * @fn					- GPIO_ReadFromInputPin
 *
 * @brief				- this fn reads the data from the GPIO Pin
 *
 * @param[in]			- Base address of GPIO Peripheral
 * @param[in]			- Pin number of given GPIO Pin
 * @param[in]			-
 *
 * @return				- Value read from GPIO Pin (0 or 1)
 *
 * @Note				- none
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	uint8_t value;
	// we want the value in the bit position corresponding with the PinNumber but don't want all the trailing 0s
	// so we shift the value to the right until it is in the least significant bit position
	value = (uint8_t)( (pGPIOx->IDR >> PinNumber) & 0x00000001 );
	return value;

}

/*******************************************************************************************
 *
 * @fn					- GPIO_ReadFromInputPort
 *
 * @brief				- This fn reads the data from the GPIO Port
 *
 * @param[in]			- base address of desired GPIO Peripheral
 * @param[in]			-
 * @param[in]			-
 *
 * @return				- data read from GPIO Port
 *
 * @Note				- none
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

	// reads the whole port so this time we don't care about bit positions bc we're reading the whole port

	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;

}

/*******************************************************************************************
 *
 * @fn					- GPIO_WriteToOutputPin
 *
 * @brief				- this fn writes (0 or 1) to the GPIO Output data register at the bit field corresponding to the pin number
 *
 * @param[in]			- base address of GPIO peripheral
 * @param[in]			- Desired GPIO Pin Number
 * @param[in]			- Value to be written to Pin
 *
 * @return				- none
 *
 * @Note				- none
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

	if(Value == GPIO_PIN_SET)
	{
		// write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		// clear (write 0) to output data register at the bit field corresponding to the pin number
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}

/*******************************************************************************************
 *
 * @fn					- GPIO_WriteToOutputPort
 *
 * @brief				- this fn writes a value to the given GPIO Port
 *
 * @param[in]			- base address of GPIO peripheral
 * @param[in]			- Value to be written to Pin
 * @param[in]			-
 *
 * @return				- none
 *
 * @Note				- none
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){

	// we're writing to the whole port so we just need to copy Value into the port
	pGPIOx->ODR |= Value;

}

/*******************************************************************************************
 *
 * @fn					- GPIO_ToggleOutputPin
 *
 * @brief				- this fn toggles the state of the given GPIO Pin
 *
 * @param[in]			- base address of GPIO peripheral
 * @param[in]			- Desired GPIO Pin Number
 * @param[in]			-
 *
 * @return				- none
 *
 * @Note				- none
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	// we can use XOR ( ^= ) to toggle bit
	pGPIOx->ODR ^= (1 << PinNumber);

}

/*
 *  IRQ Config & ISR handling
 */

/*******************************************************************************************
 *
 * @fn					- GPIO_IRQITConfig
 *
 * @brief				- this fn configures the Interrupt Controller
 *
 * @param[in]			-
 * @param[in]			-
 * @param[in]			-
 *
 * @return				- none
 *
 * @Note				- none
 *
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			// program ISER0 Register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			// program ISER1 Register
			// if IRQNumber is btwn 32 & 63 (inclusive) we move to register 2 but will need to bit shift by
			// IRQ % 32 to get the correct bit field of the register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			// program ISER2 Register
			// same as above but this time take mod of 64
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			// program ICER0 Register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			// program ICER1 Register
			// if IRQNumber is btwn 32 & 63 (inclusive) we move to register 2 but will need to bit shift by
			// IRQ % 32 to get the correct bit field of the register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			// program ICER2 Register
			// same as above but this time take mod of 64
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}

}

/*******************************************************************************************
 *
 * @fn					- GPIO_IRQPriorityConfig
 *
 * @brief				- Configures the IRQ priorities by writing to the NVIC Interrupt Priority Registers
 *
 * @param[in]			- IRQPriority
 * @param[in]			-
 * @param[in]			-
 *
 * @return				- none
 *
 * @Note				- none
 *
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. 1st lets calculate the appropriate IPR Register
	uint8_t iprx = IRQNumber / 4; // gives the desired register number (4 IRQ numbers per register hence /4)
	uint8_t iprx_section = IRQNumber % 4; // gives the appropriate bit fields when multiplied by 8 (8 bits per section, 4 sections per register)
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED); // determines number of bits to shift left by

	*(NVIC_PR_BASE_ADDR + iprx) |= ( IRQPriority << shift_amount ); // sets the priority of the desired IPR Register
																		  // can't 8 bit section ignores the lowest 4 bits

}

/*******************************************************************************************
 *
 * @fn					- GPIO_IRQHandling
 *
 * @brief				- handles the IRQ - clears the PR (Pending Register)
 *
 * @param[in]			-
 * @param[in]			-
 * @param[in]			-
 *
 * @return				- none
 *
 * @Note				- none
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// clear the exti pr register corresponding to the pin number
	if(EXTI->PR & ( 1 << PinNumber))	// this is checking the relevant bit by logical ANDing it with 1
	{
		// clear
		EXTI->PR |= ( 1 << PinNumber); // write 1 to pending register (PR) to clear see section 12.3.6 Pending register (EXTI_PR) of Reference manual
	}

}
