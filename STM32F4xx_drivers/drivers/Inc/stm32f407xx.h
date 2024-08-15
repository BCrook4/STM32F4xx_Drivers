/*
 * stm32f407xx.h
 *
 *  Created on: Jun 8, 2024
 *      Author: bento
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile

/***************************** PROCESSOR SPECIFIC DETAILS ************************************************/

/*
 * ARM Cortex Mx Processor NVIC ISERx Register Addresses (Interrupt Set-Enable Registers)
 */
#define NVIC_ISER0		( (__vo uint32_t*)0xE000E100 )			// base address of ISER0 Register
#define NVIC_ISER1		( (__vo uint32_t*)0xE000E104 )			// base address of ISER1 Register
#define NVIC_ISER2		( (__vo uint32_t*)0xE000E108 )			// base address of ISER2 Register
#define NVIC_ISER3		( (__vo uint32_t*)0xE000E10C )			// base address of ISER3 Register

/*
 * ARM Cortex Mx Processor NVIC ICERx Register Addresses (Interrupt Clear-Enable Registers)
 */
#define NVIC_ICER0		( (__vo uint32_t*)0xE000E180 )			// base address of ICER0 Register
#define NVIC_ICER1		( (__vo uint32_t*)0xE000E184 )			// base address of ICER1 Register
#define NVIC_ICER2		( (__vo uint32_t*)0xE000E188 )			// base address of ICER2 Register
#define NVIC_ICER3		( (__vo uint32_t*)0xE000E18C )			// base address of ICER3 Register


/*
 * ARM Cortex Mx Processor Priority Register Address
 */
#define NVIC_PR_BASE_ADDR	((__vo uint32_t*)0xE000E400)		// Base address of Interrupt Priority Register of NVIC


#define NO_PR_BITS_IMPLEMENTED		4			// number of priority bits implemented by STM32

/*
 * base addresses of Flash, SRAM, and ROM memories
 */

#define FLASH_BASEADDR					0x08000000U // base address of Flash memory //the U at the end is to tell compiler this value is unsigned
#define SRAM1_BASEADDR					0x20000000U // base address of SRAM1
// calculation of base address of SRAM2: SRAM1 has size of 112KB = 112 * 2^10 = 112 * 1024 = 114688dec = 0x1 C000 (hex)
// therefore base address of SRAM2 = SRAM1_BASEADDR + SRAM1 size = 0x2000 0000 + 0x0001 C0000 = 0x2001 C000
#define SRAM2_BASEADDR					0x2001C000U // base address of SRAM2
#define ROM_BASEADDR					0x1FFF0000U // Base address of ROM
#define SRAM 							SRAM1_BASEADDR // base address of SRAM

/*
 * base addresses of Bus Domains (APBx and AHBx Bus Peripherals)
 */

#define PERIPH_BASE						0X40000000U // start of peripheral base addresses
#define APB1PERIPH_BASE					PERIPH_BASE // base of APB1 is same as peripheral base
#define APB2PERIPH_BASE					0x40010000U // base address of APB2 bus
#define AHB1PERIPH_BASE					0x40020000U // base address of AHB1 bus
#define AHB2PERIPH_BASE					0x50000000U // base address of AHB2 bus

/*
 * base addresses of AHB1 Peripherals (only those that were interested in
 */

#define GPIOA_BASEADDR					(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR					(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR					(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR					(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR					(AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR					(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR					(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR					(AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR					(AHB1PERIPH_BASE + 0x2000)
#define GPIOJ_BASEADDR					(AHB1PERIPH_BASE + 0x2400)
#define GPIOK_BASEADDR					(AHB1PERIPH_BASE + 0x2800)

#define RCC_BASEADDR					(AHB1PERIPH_BASE + 0x3800)




/*
 * base addresses of peripherals on APB1 Bus
 * only includes those that we are interested in
 */

#define I2C1_BASEADDR					(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR					(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR					(APB1PERIPH_BASE + 0x5C00)

#define SPI2_BASEADDR					(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR					(APB1PERIPH_BASE + 0x3C00)

#define USART2_BASEADDR					(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR					(APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR					(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR					(APB1PERIPH_BASE + 0x5000)


/*
 * base addresses of peripherals on APB1 Bus
 * only includes those that we are interested in
 */

#define SPI1_BASEADDR					(APB2PERIPH_BASE + 0x3000)
#define USART1_BASEADDR					(APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR					(APB2PERIPH_BASE + 0x1400)
#define EXTI_BASEADDR					(APB2PERIPH_BASE + 0x3C00)
#define SYSCFG_BASEADDR					(APB2PERIPH_BASE + 0x3800)



/************************ PERIPHERAL REGISTER DEFINITION STRUCTURES ********************************/

/*
 * Note: Registers of a peripheral are specific to each MCU
 * i.e. The number of registers of the SPI peripheral of STM32Fx family of MCUs may be different to that of STM32Lx...
 * Check your device Reference Manual for more info
 */

/*
 * using a struct, we can create placeholder variables to store the register values for the peripherals registers.
 * we use uint32_t bc the registers are 32 bits wide.
 * if we then create a pointer to a struct of this type and assign it the base address of the peripheral.
 * this will mean all the variables in the struct will be stored one after another so they will line up with their
 * addresses in memory
 */
typedef struct{
	// some of these register will be volatile in nature, so it is wise to define all the elements as volatile in case
	// we defined __vo as a macro for volatile

	__vo uint32_t MODER;				/*	GPIO Mode Register										Offset: 0x00	*/
	__vo uint32_t OTYPER;				/*	Output Type Register									Offset: 0x04	*/
	__vo uint32_t OSPEEDR;				/*	Output Speed Register									Offset: 0x08	*/
	__vo uint32_t PUPDR;				/*	Pull-Up / Pull-Down Register							Offset: 0x0C	*/
	__vo uint32_t IDR;					/*	Input Data Register										Offset: 0x10	*/
	__vo uint32_t ODR;					/*	Output Data Register									Offset: 0x14	*/
	__vo uint32_t BSRR;					/* Bit Set/Reset Register									Offset: 0x18	*/
	__vo uint32_t LCKR;					/* Configuration Lock Register								Offset: 0x1C	*/
	__vo uint32_t AFR[2];				/* Alternate Function Low, AFR[0], & High, AFR[1] Reg		Offset: 0x20	*/

}GPIO_RegDef_t;

/*
 * Peripheral register definition structure for RCC
 */

typedef struct{

	__vo uint32_t CR;				/* clock control register,										Offset: 0x00		*/
	__vo uint32_t PLLCFGR;			/* PLL configuration register,									Offset: 0x04		*/
	__vo uint32_t CFGR;				/* clock configuration register,								Offset: 0x08		*/
	__vo uint32_t CIR;				/* clock interrupt register,									Offset: 0x0C		*/
	__vo uint32_t AHB1RSTR;			/* AHB1 peripheral reset register,								Offset: 0x10		*/
	__vo uint32_t AHB2RSTR;			/* AHB2 peripheral reset register,								Offset: 0x14		*/
	__vo uint32_t AHB3RSTR;			/* AHB3 peripheral reset register,								Offset: 0x18		*/
	uint32_t	  RESERVED0;		/* Reserved,													Offset: 0x1C		*/
	__vo uint32_t APB1RSTR;			/* APB1 peripheral reset register,								Offset: 0x20		*/
	__vo uint32_t APB2RSTR;			/* APB2 peripheral reset register,								Offset: 0x24		*/
	uint32_t	  RESERVED1[2];		/* Reserved,													Offset: 0X28 - 0x2C	*/
	__vo uint32_t AHB1ENR;			/* AHB1 peripheral clock enable register,						Offset: 0x30		*/
	__vo uint32_t AHB2ENR;			/* AHB2 peripheral clock enable register,						Offset: 0x34		*/
	__vo uint32_t AHB3ENR;			/* AHB3 peripheral clock enable register,						Offset: 0x38		*/
	uint32_t	  RESERVED2;		/* Reserved,													Offset: 0x3C		*/
	__vo uint32_t APB1ENR;			/* APB1 peripheral clock enable register,						Offset: 0x40		*/
	__vo uint32_t APB2ENR;			/* APB2 peripheral clock enable register,						Offset: 0x44		*/
	uint32_t	  RESERVED3[2];		/* Reserved,													Offset: 0x48 - 0x4C	*/
	__vo uint32_t AHB1LPENR;		/* AHB1 peripheral clock enable in low power mode register,		Offset: 0x50		*/
	__vo uint32_t AHB2LPENR;		/* AHB2 peripheral clock enable in low power mode register,		Offset: 0x54		*/
	__vo uint32_t AHB3LPENR;		/* AHB3 peripheral clock enable in low power mode register,		Offset: 0x58		*/
	uint32_t	  RESERVED4;		/* Reserved,													Offset: 0x5C		*/
	__vo uint32_t APB1LPENR;		/* APB1 peripheral clock enable in low power mode register,		Offset: 0x60		*/
	__vo uint32_t APB2LPENR;		/* APB2 peripheral clock enabled in low power mode register,	Offset: 0x64		*/
	uint32_t	  RESERVED5[2];		/* Reserved,													Offset: 0x68 - 0x6C	*/
	__vo uint32_t RCC_BDCR;			/* Backup domain control register,								Offset: 0x70		*/
	__vo uint32_t CSR;				/* clock control & status register,								Offset: 0x74		*/
	uint32_t	  RESERVED6[2];		/* Reserved,													Offset: 0x78 - 0x7C	*/
	__vo uint32_t SSCGR;			/* spread spectrum clock generation register,					Offset: 0x80		*/
	__vo uint32_t PLLI2SCFGR;		/* PLLI2S configuration register,								Offset: 0x84		*/

	//we have to include the RESERVED variables so that the other variables will still line up with the correct addresses

}RCC_RegDef_t;


/*
 * Peripheral register definition structure for EXTI
 */

typedef struct{

	__vo uint32_t IMR;				// Interrupt mask register					address offset: 0x00
	__vo uint32_t EMR;				// Event mask register						address offset: 0x04
	__vo uint32_t RTSR;				// Rising trigger selection register		address offset: 0x08
	__vo uint32_t FTSR;				// Falling trigger selection register		address offset: 0x0C
	__vo uint32_t SWIER;			// Software interrupt event register		address offset: 0x10
	__vo uint32_t PR;				// Pending register			 				address offset: 0x14


}EXTI_RegDef_t;


/*
 * Peripheral register definition structure for SYSCFG
 */

typedef struct{

	__vo uint32_t MEMRMP;		// SYSCFG memory remap register									address offset: 0x00
	__vo uint32_t PMC;			// SYSCFG peripheral mode configuration register				address offset: 0x04
	__vo uint32_t EXTICR[4];	// SYSCFG external interrupt configuration registers 1-4		address offset: 0x08 - 0x0C
	uint32_t RESERVED1[2];		// RESERVED to keep alignment of struct variables				address offset: 0x10 - 0x1C
	__vo uint32_t CMPCR;		// Compensation cell control register							address offset: 0x20
	uint32_t RESERVED2[2];		// RESERVED to keep alignment of struct variables				address offset: 0x24 - 0x28
	__vo uint32_t CFGR;			// Don't know what this is. not in my ref manual but the instructor added it		address offset: 0x2C

}SYSCFG_RegDef_t;


/*
 * Peripheral definitions (peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG			((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH			((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI			((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC				((RCC_RegDef_t*)RCC_BASEADDR) 		//defining RCC as a struct of type RCC_RegDef_t

#define EXTI			((EXTI_RegDef_t*)EXTI_BASEADDR)		// defining external interrupt/event controller

#define SYSCFG			((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)		// defining System configuration structure macro


/*Clock enable macros for GPIOx Peripherals
the -> operator allows access to elements of struct (i.e. AHB1ENR of RCC_RegDef_t)*/

#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))		//Enable GPIO Peripheral Clock A
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))		//Enable GPIO Peripheral Clock B
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))		//Enable GPIO Peripheral Clock C
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))		//Enable GPIO Peripheral Clock D
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))		//Enable GPIO Peripheral Clock E
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))		//Enable GPIO Peripheral Clock F
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))		//Enable GPIO Peripheral Clock G
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))		//Enable GPIO Peripheral Clock H
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))		//Enable GPIO Peripheral Clock I


// Clock enable macros for I2Cx Peripherals
#define I2C1_PCLK_EN()		( RCC->APB1ENR |= (1 << 21) )		// Enable I2C1 Peripheral Clock
#define I2C2_PCLK_EN()		( RCC->APB1ENR |= (1 << 22) )		// Enable I2C2 Peripheral Clock
#define I2C3_PCLK_EN()		( RCC->APB1ENR |= (1 << 23) )		// Enable I2C3 Peripheral Clock

// Clock enable macros for SPIx Peripherals
#define SPI1_PCLK_EN()		( RCC->APB2ENR |= (1 << 12) )		// Enable SPI1 Peripheral Clock
#define SPI2_PCLK_EN()		( RCC->APB1ENR |= (1 << 14) )		// Enable SPI2 Peripheral Clock
#define SPI3_PCLK_EN()		( RCC->APB1ENR |= (1 << 15) )		// Enable SPI3 Peripheral Clock

// Clock enable macros for USARTx Peripherals
#define USART1_PCLK_EN()	( RCC->APB2ENR |= (1 << 4) )		// Enable USART1 Peripheral Clock
#define USART2_PCLK_EN()	( RCC->APB1ENR |= (1 << 17) )		// Enable USART2 Peripheral Clock
#define USART3_PCLK_EN()	( RCC->APB1ENR |= (1 << 18) )		// Enable USART3 Peripheral Clock
#define USART4_PCLK_EN()	( RCC->APB1ENR |= (1 << 19) )		// Enable USART4 Peripheral Clock
#define USART5_PCLK_EN()	( RCC->APB1ENR |= (1 << 20) )		// Enable USART5 Peripheral Clock
#define USART6_PCLK_EN()	( RCC->APB2ENR |= (1 << 5) )		// Enable USART6 Peripheral Clock

// Clock enable macros for SYSCFG Peripherals
#define SYSCFG_PCLK_EN()	( RCC->APB2ENR |= (1 << 14) )		// Enable SYSCFG Peripheral Clock


//Clock disable macros for GPIOx Peripherals
//&= is bitwise AND ~ is negation of -- ~(1 << x) is how to reset a bit w/ &=
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))		//Disable GPIO Peripheral Clock A
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))		//Disable GPIO Peripheral Clock B
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))		//Disable GPIO Peripheral Clock C
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))		//Disable GPIO Peripheral Clock D
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))		//Disable GPIO Peripheral Clock E
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))		//Disable GPIO Peripheral Clock F
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))		//Disable GPIO Peripheral Clock G
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))		//Disable GPIO Peripheral Clock H
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 8))		//Disable GPIO Peripheral Clock I

// Clock disable macros for I2Cx Peripherals
#define I2C1_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 21) )		// Disable I2C1 Peripheral Clock
#define I2C2_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 22) )		// Disable I2C2 Peripheral Clock
#define I2C3_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 23) )		// Disable I2C3 Peripheral Clock

// Clock disable macros for SPIx Peripherals
#define SPI1_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 12) )		// Disable SPI1 Peripheral Clock
#define SPI2_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 14) )		// Disable SPI2 Peripheral Clock
#define SPI3_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 15) )		// Disable SPI3 Peripheral Clock

// Clock disable macros for USARTx Peripherals
#define USART1_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 4) )		// Disable USART1 Peripheral Clock
#define USART2_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 17) )		// Disable USART2 Peripheral Clock
#define USART3_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 18) )		// Disable USART3 Peripheral Clock
#define USART4_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 19) )		// Disable USART4 Peripheral Clock
#define USART5_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 20) )		// Disable USART5 Peripheral Clock
#define USART6_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 5) )		// Disable USART6 Peripheral Clock

// Clock disable macros for SYSCFG Peripherals
#define SYSCFG_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 14) )		// Disable SYSCFG Peripheral Clock


// Macros to reset GPIOx peripherals
								// do while loop allows us to use multiple statements in a macro
#define GPIOA_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)

/*
 * macro to return port code for given GPIOx base address
 * uses C conditional (or ternary operators) google it if you forget what this is
 * (condition) ? statement1 : statement2
 * if condition is true statement1 is executed/returned
 * else statement2 is executed/returned
 *  :\ makes it continue to the next conditional
 */

// the numbers assigned to the macro correspond to the given GPIOs
// i.e. you would write that number to the the appropriate bits of the EXTICRx register to select the GPIO as the input
#define GPIO_BASEADDR_TO_CODE(x)		( (x == GPIOA) ? 0 :\
										  (x == GPIOB) ? 1 :\
										  (x == GPIOC) ? 2 :\
										  (x == GPIOD) ? 3 :\
										  (x == GPIOE) ? 4 :\
										  (x == GPIOF) ? 5 :\
										  (x == GPIOG) ? 6 :\
										  (x == GPIOH) ? 7 :\
										  (x == GPIOI) ? 8 : 0 )


/*
 * IRQ (Interrupt Request) Numbers
 */

#define IRQ_NO_EXTI0			6			//
#define IRQ_NO_EXTI1			7			//
#define IRQ_NO_EXTI2			8			//
#define IRQ_NO_EXTI3			9			//
#define IRQ_NO_EXTI4			10			//
#define IRQ_NO_EXTI9_5			23			//
#define IRQ_NO_EXTI15_10		40			//

/*
 * IRQ (Interrupt Request) Priorities
 */
#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15

// some generic macros

#define ENABLE 					1
#define DISABLE 				0
#define SET						ENABLE
#define RESET 					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET




#include "stm32f407xx_gpio_driver.h"

#endif /* INC_STM32F407XX_H_ */
