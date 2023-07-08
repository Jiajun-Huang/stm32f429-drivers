/*
 * stm32f429.h
 *
 *  Created on: Jul 2, 2023
 *      Author: Jiajun
 */

#ifndef INC_STM32F429_H_
#define INC_STM32F429_H_
#include "type_def.h"
#include <stdint.h>

//** CORE PERIPHERAL BASE ADDRESSES **//
// ISER (Interrupt Set enable register) register addresses
#define NVIC_ISER (uint32_t *) 0xE000E100

// ICER (Interrupt Clear enable register) register addresses
#define NVIC_ICER (uint32_t *) 0xE000E180

// Priority register address calculation
#define NVIC_IPR (uint32_t *) 0xE000E400

#define NO_PR_BITS_IMPLEMENTED 4

// Base addresses of Flash and SRAM memories
#define FLASH_BASEADDR 0x08000000U
#define SRAM1_BASEADDR 0x20000000U
#define SRAM2_BASEADDR 0x2001C000U
#define SRAM SRAM1_BASEADDR
#define ROM_BASEADDR 0x1FFF0000U

// AHBx and APBx Bus Peripheral base addresses
#define PERIPH_BASEADDR 0x40000000U
#define APB1PERIPH_BASEADDR PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR 0x40010000U
#define AHB1PERIPH_BASEADDR 0x40020000U
#define AHB2PERIPH_BASEADDR 0x50000000U

// GPIO base addresses
#define GPIOA_BASEADDR (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR (AHB1PERIPH_BASEADDR + 0x2000)

// I2C base addresses
#define I2C1_BASEADDR (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR (APB1PERIPH_BASEADDR + 0x5C00)

// SPI base addresses
#define SPI1_BASEADDR (APB2PERIPH_BASEADDR + 0x3000)
#define SPI2_BASEADDR (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR (APB1PERIPH_BASEADDR + 0x3C00)
#define SPI4_BASEADDR (APB2PERIPH_BASEADDR + 0x3400)

// USART base addresses
#define USART1_BASEADDR (APB2PERIPH_BASEADDR + 0x1000)
#define USART2_BASEADDR (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR (APB1PERIPH_BASEADDR + 0x4800)
#define USART6_BASEADDR (APB2PERIPH_BASEADDR + 0x1400)

// UART base addresses
#define UART4_BASEADDR (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR (APB1PERIPH_BASEADDR + 0x5000)

#define EXTI_BASEADDR (APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR (APB2PERIPH_BASEADDR + 0x3800)
#define RCC_BASEADDR (AHB1PERIPH_BASEADDR + 0x3800)

#define EXTI ((EXTI_RegDef_t *)EXTI_BASEADDR)
#define SYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

#define RCC ((RCC_RegDef_t *)RCC_BASEADDR)

//** object definitions start **//
#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI ((GPIO_RegDef_t *)GPIOI_BASEADDR)

#define RCC ((RCC_RegDef_t *)RCC_BASEADDR)
#define EXTI ((EXTI_RegDef_t *)EXTI_BASEADDR)
#define SYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

#define SPI1 ((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t *)SPI3_BASEADDR)

// #define I2C1 ((I2C_RegDef_t *)I2C1_BASEADDR)
// #define I2C2 ((I2C_RegDef_t *)I2C2_BASEADDR)
// #define I2C3 ((I2C_RegDef_t *)I2C3_BASEADDR)

// #define USART1 ((USART_RegDef_t *)USART1_BASEADDR)
// #define USART2 ((USART_RegDef_t *)USART2_BASEADDR)
// #define USART3 ((USART_RegDef_t *)USART3_BASEADDR)
// #define UART4 ((USART_RegDef_t *)UART4_BASEADDR)
// #define UART5 ((USART_RegDef_t *)UART5_BASEADDR)
// #define USART6 ((USART_RegDef_t *)USART6_BASEADDR)

//** object definitions end **//

// Clock enable/disable macros for GPIOx peripherals
#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN() (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN() (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN() (RCC->AHB1ENR |= (1 << 8))
#define GPIOALL_PCLK_EN() (RCCC->AHB1ENR |= 0x000000FF)

#define GPIOA_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 8))
#define GPIOALL_PCLK_DI() (RCC->AHB1ENR &= 0xFFFFFF00)

#define GPIO_BASEADDR_TO_CODE(x) (((uintptr_t)(x) - ((uintptr_t)GPIOA)) >> 10)

#define GPIOA_REG_RESET()                                                      \
    do {                                                                       \
        RCC->AHB1RSTR |= (1 << 0);                                             \
        RCC->AHB1RSTR &= ~(1 << 0);                                            \
    } while (0)
#define GPIOB_REG_RESET()                                                      \
    do {                                                                       \
        RCC->AHB1RSTR |= (1 << 1);                                             \
        RCC->AHB1RSTR &= ~(1 << 1);                                            \
    } while (0)
#define GPIOC_REG_RESET()                                                      \
    do {                                                                       \
        RCC->AHB1RSTR |= (1 << 2);                                             \
        RCC->AHB1RSTR &= ~(1 << 2);                                            \
    } while (0)
#define GPIOD_REG_RESET()                                                      \
    do {                                                                       \
        RCC->AHB1RSTR |= (1 << 3);                                             \
        RCC->AHB1RSTR &= ~(1 << 3);                                            \
    } while (0)
#define GPIOE_REG_RESET()                                                      \
    do {                                                                       \
        RCC->AHB1RSTR |= (1 << 4);                                             \
        RCC->AHB1RSTR &= ~(1 << 4);                                            \
    } while (0)
#define GPIOF_REG_RESET()                                                      \
    do {                                                                       \
        RCC->AHB1RSTR |= (1 << 5);                                             \
        RCC->AHB1RSTR &= ~(1 << 5);                                            \
    } while (0)
#define GPIOG_REG_RESET()                                                      \
    do {                                                                       \
        RCC->AHB1RSTR |= (1 << 6);                                             \
        RCC->AHB1RSTR &= ~(1 << 6);                                            \
    } while (0)
#define GPIOH_REG_RESET()                                                      \
    do {                                                                       \
        RCC->AHB1RSTR |= (1 << 7);                                             \
        RCC->AHB1RSTR &= ~(1 << 7);                                            \
    } while (0)
#define GPIOI_REG_RESET()                                                      \
    do {                                                                       \
        RCC->AHB1RSTR |= (1 << 8);                                             \
        RCC->AHB1RSTR &= ~(1 << 8);                                            \
    } while (0)
#define GPIOALL_REG_RESET()                                                    \
    do {                                                                       \
        RCC->AHB1RSTR |= 0x000000FF;                                           \
        RCC->AHB1STR &= 0xFFFFFF00                                             \
    } while (0)

//** GPIO object end **//

//** SPI object start **//

// Clock enable/disable macros for I2Cx peripherals
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))

#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 23))

// Clock enable/disable macros for SPIx peripherals
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))

#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI() (RCC->APB2ENR &= ~(1 << 13))

#define SPI1_REG_RESET()                                                       \
    do {                                                                       \
        RCC->APB2RSTR |= (1 << 12);                                            \
        RCC->APB2RSTR &= ~(1 << 12);                                           \
    } while (0)

#define SPI2_REG_RESET()                                                       \
    do {                                                                       \
        RCC->APB1RSTR |= (1 << 14);                                            \
        RCC->APB1RSTR &= ~(1 << 14);                                           \
    } while (0)

#define SPI3_REG_RESET()                                                       \
    do {                                                                       \
        RCC->APB1RSTR |= (1 << 15);                                            \
        RCC->APB1RSTR &= ~(1 << 15);                                           \
    } while (0)

#define SPI4_REG_RESET()                                                       \
    do {                                                                       \
        RCC->APB2RSTR |= (1 << 13);                                            \
        RCC->APB2RSTR &= ~(1 << 13);                                           \
    } while (0)

// Clock enable/disable macros for USARTx peripherals
#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN() (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN() (RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN() (RCC->APB2ENR |= (1 << 5))

#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI() (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI() (RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI() (RCC->APB2ENR &= ~(1 << 5))

// Clock enable/disable macros for SYSCFG peripheral
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))
#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= ~(1 << 14))

// IRQ (Intr) numbers for EXTI obtain from vector table of position number
#define IRQ_NO_EXTI0 6
#define IRQ_NO_EXTI1 7
#define IRQ_NO_EXTI2 8
#define IRQ_NO_EXTI3 9
#define IRQ_NO_EXTI4 10
#define IRQ_NO_EXTI9_5 23
#define IRQ_NO_EXTI15_10 40

// IRQ (Intr) numbers for SPIx peripherals
#define IRQ_NO_SPI1 35
#define IRQ_NO_SPI2 36
#define IRQ_NO_SPI3 51
#define IRQ_NO_SPI4 84
#define IRQ_NO_SPI5 85
#define IRQ_NO_SPI6 86

// IRQ (Intr) priority levels
#define NVIC_IRQ_PRI0 0
#define NVIC_IRQ_PRI1 1
#define NVIC_IRQ_PRI2 2
#define NVIC_IRQ_PRI3 3
#define NVIC_IRQ_PRI4 4
#define NVIC_IRQ_PRI5 5
#define NVIC_IRQ_PRI6 6
#define NVIC_IRQ_PRI7 7
#define NVIC_IRQ_PRI8 8
#define NVIC_IRQ_PRI9 9
#define NVIC_IRQ_PRI10 10
#define NVIC_IRQ_PRI11 11
#define NVIC_IRQ_PRI12 12
#define NVIC_IRQ_PRI13 13
#define NVIC_IRQ_PRI14 14
#define NVIC_IRQ_PRI15 15

// some generic macros
#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET

/**
 * @brief Open or close IRQ interrupt for given IRQ number
 *
 * @param IRQNumber  IRQ number
 * @param EnorDi  ENABLE or DISABLE
 */
void IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

/**
 * @brief Configure priority for given IRQ number
 *
 * @param IRQNumber  IRQ number
 * @param IRQPriority  IRQ priority
 */
void IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

#include "GPIO_driver.h"
#include "SPI_driver.h"

#endif /* INC_STM32F429_H_ */
