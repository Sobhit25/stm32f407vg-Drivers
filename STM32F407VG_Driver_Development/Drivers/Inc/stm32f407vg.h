/*
 * stm32f407vg.h
 *
 *  Created on: Aug 17, 2021
 *      Author: sobhit25
 */

#ifndef INC_STM32F407VG_H_
#define INC_STM32F407VG_H_

#include <stdint.h>
#include <stddef.h>


#define __vo volatile
#define __weak __attribute__((weak))


#define ENABLE 1
#define DISABLE 0
#define SET 1
#define RESET 0


/*
 * Base addresses of SRAM and Flash
 */

#define SRAM2_BASEADDR 0x2001C000U
#define SRAM1_BASEADDR 0x20000000U
#define Flash_BASEADDR 0x08000000U
#define ROM_BASEADDR   0x1FFF0000U

#define SRAM1 SRAM1_BASEADDR
#define SRAM2 SRAM2_BASEADDR
#define ROM ROM_BASEADDR
#define Flash Flash_BASEADDR


/*******************************MCU Specific Details***********************************
 * *****************										***************************
 *************************************************************************************/
#define NVIC_ISER0 	(__vo uint32_t*)0xE000E100U
#define NVIC_ISER1	(__vo uint32_t*)0xE000E104U
#define NVIC_ISER2	(__vo uint32_t*)0xE000E108U
#define NVIC_ISER3	(__vo uint32_t*)0xE000E10CU

#define NVIC_ICER0 	(__vo uint32_t*) 0xE000E180U
#define NVIC_ICER1	(__vo uint32_t*) 0xE000E184U
#define NVIC_ICER2	(__vo uint32_t*) 0xE000E188U
#define NVIC_ICER3	(__vo uint32_t*) 0xE000E18CU

#define NVIC_ISPR0 	(__vo uint32_t*) 0xE000E200U
#define NVIC_ISPR1	(__vo uint32_t*) 0xE000E204U
#define NVIC_ISPR2	(__vo uint32_t*) 0xE000E208U
#define NVIC_ISPR3	(__vo uint32_t*) 0xE000E20CU

#define NVIC_ICPR0 	(__vo uint32_t*) 0xE000E280U
#define NVIC_ICPR1	(__vo uint32_t*) 0xE000E284U
#define NVIC_ICPR2	(__vo uint32_t*) 0xE000E288U
#define NVIC_ICPR3	(__vo uint32_t*) 0xE000E28CU

#define NVIC_IPR_BASE	(__vo uint32_t*) 0xE000E400U

//No Of Priority Bits Implemented
#define NO_IPR_BITS		4

/*
 * Base addresses of various  Bus Domains(AHBx & APBx)
 */
#define APB1_BASEADDR 0x40000000U
#define APB2_BASEADDR 0x40010000U
#define AHB1_BASEADDR 0x40020000U
#define AHB2_BASEADDR 0x50000000U
#define AHB3_BASEADDR 0xA0000000U


/*
 * Base addresses of various peripherals ON APB1 BUS
 */
#define TIM2_BASEADDR 	(APB1_BASEADDR + 0x0000U)
#define TIM3_BASEADDR 	(APB1_BASEADDR + 0x0400U)
#define TIM4_BASEADDR 	(APB1_BASEADDR + 0x0800U)
#define TIM5_BASEADDR 	(APB1_BASEADDR + 0x0C00U)
#define TIM6_BASEADDR 	(APB1_BASEADDR + 0x1000U)
#define TIM7_BASEADDR 	(APB1_BASEADDR + 0x1400U)
#define TIM12_BASEADDR 	(APB1_BASEADDR + 0x1800U)
#define TIM13_BASEADDR	(APB1_BASEADDR + 0x1C00U)
#define TIM14_BASEADDR 	(APB1_BASEADDR + 0x2000U)



#define SPI2_BASEADDR 	(APB1_BASEADDR + 0x3800U)
#define SPI3_BASEADDR 	(APB1_BASEADDR + 0x3C00U)

#define USART2_BASEADDR 	(APB1_BASEADDR + 0x4400U)
#define USART3_BASEADDR 	(APB1_BASEADDR + 0x4800U)
#define UART4_BASEADDR 		(APB1_BASEADDR + 0x4C00U)
#define UART5_BASEADDR 		(APB1_BASEADDR + 0x5000U)

#define UART7_BASEADDR 		(APB1_BASEADDR + 0x7800U)
#define UART8_BASEADDR 		(APB1_BASEADDR + 0x7C00U)

#define I2C1_BASEADDR		(APB1_BASEADDR + 0x5400U)
#define I2C2_BASEADDR		(APB1_BASEADDR + 0x5800U)
#define I2C3_BASEADDR		(APB1_BASEADDR + 0x5C00U)




/*
 * Base addresses of various peripherals On APB2 BUS
 */
#define TIM1_BASEADDR		(APB2_BASEADDR + 0x0000U)
#define TIM8_BASEADDR		(APB2_BASEADDR + 0x0400U)
#define TIM9_BASEADDR		(APB2_BASEADDR + 0x4000U)
#define TIM10_BASEADDR		(APB2_BASEADDR + 0x4400U)
#define TIM11_BASEADDR		(APB2_BASEADDR + 0x4800U)


#define USART1_BASEADDR		(APB2_BASEADDR + 0x1000U)
#define USART6_BASEADDR		(APB2_BASEADDR + 0x1400U)

#define SPI1_BASEADDR		(APB2_BASEADDR + 0x3000U)
#define SPI4_BASEADDR		(APB2_BASEADDR + 0x3400U)
#define SPI5_BASEADDR		(APB2_BASEADDR + 0x5000U)
#define SPI6_BASEADDR		(APB2_BASEADDR + 0x5400U)

#define SYSCFG_BASEADDR		(APB2_BASEADDR + 0x3800U)
#define EXTI_BASEADDR		(APB2_BASEADDR + 0x3C00U)




/*
 * Base addresses of various peripherals On APB2 BUS
 */
#define GPIOA_BASEADDR		(AHB1_BASEADDR + 0x0000U)
#define GPIOB_BASEADDR		(AHB1_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR		(AHB1_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR		(AHB1_BASEADDR + 0x0C00U)
#define GPIOE_BASEADDR		(AHB1_BASEADDR + 0x1000U)

#define RCC_BASEADDR		(AHB1_BASEADDR + 0x3800U)
#define CRC_BASEADDR		(AHB1_BASEADDR + 0x3000U)



/***********************Peripheral Register DEfinitions***********************/
/*
 * Note - To create base addresses of every bit of every peripheral can be tedious
 * so we do not do that. We create a peripheral structure and initialise a pointer pointing
 * to the base address of the respective peripheral. Ok, lets do that!
 *****************************************************************************/

/*
 * Peripheral Structure for GPIO Register
 */
typedef struct{
	__vo uint32_t MODER;      	/*!< GPIO port mode register >*/
	__vo uint32_t OTYPER;		/*!< GPIO port output type register >*/
	__vo uint32_t OSPEEDR;		/*!< GPIO port output speed register >*/
	__vo uint32_t PUPDR;			/*!< GPIO port Pull-Up Pull-Down register >*/
	__vo uint32_t IDR;			/*!< GPIO port Input Data Register register >*/
	__vo uint32_t ODR;			/*!< GPIO port Output Data Register register >*/
	__vo uint32_t BSRR;			/*!< GPIO port bit set/reset register >*/
	__vo uint32_t LCKR;			/*!< GPIO port configuration lock register >*/
	__vo uint32_t AFRL;			/*!< GPIO port Alternate Functionality mode Low register >*/
	__vo uint32_t AFRH;			/*!< GPIO port Alternate Functionality mode High register >*/
}GPIO_RegDef_t;

/*
 * Peripheral Structure for RCC Register
 */
typedef struct{
	__vo uint32_t CR;			/*!< RCC clock control register >*/
	__vo uint32_t PLLCFGR;		/*!< RCC PLL configuration register >*/
	__vo uint32_t CFGR;			/*!< RCC clock configuration register >*/
	__vo uint32_t CIR;			/*!< RCC clock interrupt register >*/
	__vo uint32_t AHB1RSTR; 	/*!< RCC AHB1 peripheral reset register >*/
	__vo uint32_t AHB2RSTR;		/*!< RCC AHB2 peripheral reset register >*/
	__vo uint32_t AHB3RSTR;		/*!< RCC AHB3 peripheral reset register >*/
	__vo uint32_t reserved1;		/*!< RCC RESERVED MEMORY >*/
	__vo uint32_t APB1RSTR;		/*!< RCC APB1 peripheral reset register >*/
	__vo uint32_t APB2RSTR;		/*!< RCC APB2 peripheral reset register >*/
	__vo uint32_t reserved2[2];		/*!< RCC RESERVED MEMORY  >*/
	__vo uint32_t AHB1ENR;		/*!< RCC AHB1 peripheral clock enable register >*/
	__vo uint32_t AHB2ENR;		/*!< RCC AHB2 peripheral clock enable register >*/
	__vo uint32_t AHB3ENR;		/*!< RCC AHB3 peripheral clock enable register >*/
	__vo uint32_t reserved3;		/*!< RCC RESERVED MEMORY  >*/
	__vo uint32_t APB1ENR;		/*!< RCC APB1 peripheral clock enable register >*/
	__vo uint32_t APB2ENR;		/*!< RCC APB2 peripheral clock enable register >*/
	__vo uint32_t reserved4[2];		/*!< RCC RESERVED MEMORY  >*/
	__vo uint32_t AHB1LPENR;	/*!< RCC AHB1 peripheral clock enable in low power mode register >*/
	__vo uint32_t AHB2LPENR;	/*!< RCC AHB2 peripheral clock enable in low power mode register >*/
	__vo uint32_t AHB3LPENR;	/*!< RCC AHB3 peripheral clock enable in low power mode register >*/
	__vo uint32_t reserved5;		/*!< RCC RESERVED MEMORY  >*/
	__vo uint32_t APB1LPENR;	/*!< RCC APB1 peripheral clock enable in low power mode register >*/
	__vo uint32_t APB2LPENR;	/*!< RCC APB2 peripheral clock enable in low power mode register >*/
	__vo uint32_t reserved6[2];		/*!< RCC RESERVED MEMORY  >*/
	__vo uint32_t BDCR;			/*!< RCC Backup domain control register >*/
	__vo uint32_t CSR;			/*!< RCC clock control & status register >*/
	__vo uint32_t reserved7[2];		/*!< RCC RESERVED MEMORY  >*/
	__vo uint32_t SSCGR;		/*!< RCC spread spectrum clock generation register >*/
	__vo uint32_t PLLI2SCFGR;	/*!< RCC PLLI2S configuration register >*/
}RCC_RegDef_t;


/*
 * Peripheral Structure for EXTI Register
 */
typedef struct{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;


/*
 * Peripheral Structure for SYSCFG Register
 */
typedef struct{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t reserved[2];
	__vo uint32_t CMPCR;
}SYSCFG_RegDef_t;



/*
 * Peripheral Structure for SPI Register
 */
typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;



/*
 * Peripheral Definitions which are typecasted to xxx_RegDef_t
 */
#define GPIOA	((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB	((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC	((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD	((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE	((GPIO_RegDef_t*)GPIOE_BASEADDR)

#define RCC     ((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI     ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG   ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


#define SPI1 	((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2 	((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3 	((SPI_RegDef_t*)SPI3_BASEADDR)


/*
 * Clock Enable macros for GPIOx peripherals
 */
#define GPIOA_PClk_EN()		(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PClk_EN()		(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PClk_EN()		(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PClk_EN()		(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PClk_EN()		(RCC->AHB1ENR |= (1<<4))



/*
 * Peripheral Reset macros for GPIOx peripherals
 */
#define GPIOA_RESET()		(RCC->AHB1RSTR  |= (1<<0))
#define GPIOB_RESET()		(RCC->AHB1RSTR  |= (1<<1))
#define GPIOC_RESET()		(RCC->AHB1RSTR  |= (1<<2))
#define GPIOD_RESET()		(RCC->AHB1RSTR  |= (1<<3))
#define GPIOE_RESET()		(RCC->AHB1RSTR  |= (1<<4))




/*
 * Clock Enable macros for I2Cx peripherals
 */
#define I2C1_PClk_EN()		(RCC->APB1ENR |= (1<<21))
#define I2C2_PClk_EN()		(RCC->APB1ENR |= (1<<22))
#define I2C3_PClk_EN()		(RCC->APB1ENR |= (1<<23))


/*
 * Clock Enable macros for SPIx peripherals
 */
#define SPI2_PClk_EN()		(RCC->APB1ENR |= (1<<14))
#define SPI3_PClk_EN()		(RCC->APB1ENR |= (1<<15))
#define SPI1_PClk_EN()		(RCC->APB2ENR |= (1<<12))


/*
 * Clock Enable macros for USARTx peripherals
 */
#define USART2_PClk_EN()		(RCC->APB1ENR |= (1<<17))
#define USART3_PClk_EN()		(RCC->APB1ENR |= (1<<18))
#define UART4_PClk_EN()			(RCC->APB1ENR |= (1<<19))
#define UART5_PClk_EN()			(RCC->APB1ENR |= (1<<20))
#define USART1_PClk_EN()		(RCC->APB2ENR |= (1<<4))
#define USART6_PClk_EN()		(RCC->APB2ENR |= (1<<5))


/*
 * Clock Enable macros for SYSCGF peripherals
 */
#define SYSCGF_PClk_EN()		(RCC->APB2ENR |= (1<<14))






/*
 * Clock Disable macros for GPIOx peripherals
 */
#define GPIOA_PClk_DI()		(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PClk_DI()		(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PClk_DI()		(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PClk_DI()		(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PClk_DI()		(RCC->AHB1ENR &= ~(1<<4))


/*
 * Clock Disable macros for I2Cx peripherals
 */
#define I2C1_PClk_DI()		(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PClk_DI()		(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PClk_DI()		(RCC->APB1ENR &= ~(1<<23))


/*
 * Clock Disable macros for SPIx peripherals
 */
#define SPI2_PClk_DI()		(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PClk_DI()		(RCC->APB1ENR &= ~(1<<15))
#define SPI1_PClk_DI()		(RCC->APB2ENR &= ~(1<<12))

/*
 * Clock Disable macros for USARTx peripherals
 */
#define USART2_PClk_DI()		(RCC->APB1ENR &= ~(1<<17))
#define USART3_PClk_DI()		(RCC->APB1ENR &= ~(1<<18))
#define UART4_PClk_DI()			(RCC->APB1ENR &= ~(1<<19))
#define UART5_PClk_DI()			(RCC->APB1ENR &= ~(1<<20))
#define USART1_PClk_DI()		(RCC->APB2ENR &= ~(1<<4))
#define USART6_PClk_DI()		(RCC->APB2ENR &= ~(1<<5))


/*
 * Clock Disable macros for SYSCGF peripherals
 */
#define SYSCGF_PClk_DI()		(RCC->APB2ENR &= ~(1<<14))


#define GPIO_BASEADDR_TO_CODE(x) 	((x == GPIOA)?0:\
									 (x == GPIOB)?1:\
									 (x == GPIOC)?2:\
									 (x == GPIOD)?3:\
									 (x == GPIOE)?4:0)



/*
 * IRQ Mapping Of Various Interrupts specific to our Board
 */
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40


#define IRQ_NO_SPI1			42
#define IRQ_NO_SPI2			43
#define IRQ_NO_SPI3			58


/* ************************************************************ *
 * 	Bit Position Macros for SPI Configuration Registers
 * ************************************************************ */

// BIT POSITION MACROS FOR SPI-CR1 REGISTER
#define SPI_CR1_CPHA  		0
#define SPI_CR1_CPOL  		1
#define SPI_CR1_MSTR  		2
#define SPI_CR1_BR  		3
#define SPI_CR1_SPE  		6
#define SPI_CR1_LSBFIRST  	7
#define SPI_CR1_SSI  		8
#define SPI_CR1_SSM  		9
#define SPI_CR1_RXONLY  	10
#define SPI_CR1_DFF  		11
#define SPI_CR1_CRCNEXT  	12
#define SPI_CR1_CRCEN  		13
#define SPI_CR1_BIDIOE  	14
#define SPI_CR1_BIDIMODE  	15


// BIT POSITION MACROS FOR SPI-CR2 REGISTER
#define SPI_CR2_RXDMAEN 	0
#define SPI_CR2_TXDMAEN 	1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7


// BIT POSITION MACROS FOR SPI-SR REGISTER
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8



/*
 * Peripheral Reset macros for SPIx peripherals
 */
#define SPI1_RESET()		(RCC->APB2RSTR  |= (1<<12))
#define SPI2_RESET()		(RCC->APB1RSTR  |= (1<<14))
#define SPI3_RESET()		(RCC->APB1RSTR  |= (1<<15))



#include <stm32f407vg_gpio_driver.h>
#include <stm32f407vg_spi_driver.h>

#endif /* INC_STM32F407VG_H_ */
