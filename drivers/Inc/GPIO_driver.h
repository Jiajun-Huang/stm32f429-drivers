/*
 * GPIO_driver.h
 *
 *  Created on: Jul 2, 2023
 *      Author: jiajun
 */

#include "stm32f429xx.h"
#include <stdint.h>

#ifndef GPIO_DRIVER_H_
#define GPIO_DRIVER_H_

//@GPIO_PIN_NUMBERS
// GPIO pin numbers
#define GPIO_PIN_NO_0 0
#define GPIO_PIN_NO_1 1
#define GPIO_PIN_NO_2 2
#define GPIO_PIN_NO_3 3
#define GPIO_PIN_NO_4 4
#define GPIO_PIN_NO_5 5
#define GPIO_PIN_NO_6 6
#define GPIO_PIN_NO_7 7
#define GPIO_PIN_NO_8 8
#define GPIO_PIN_NO_9 9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15

// @GPIO_PIN_MODES
#define GPIO_MODE_IN 0     // input mode
#define GPIO_MODE_OUT 1    // output mode
#define GPIO_MODE_ALTFN 2  // alternate function
#define GPIO_MODE_ANALOG 3 // analog mode
#define GPIO_MODE_IT_FT 4  // interrupt falling edge trigger
#define GPIO_MODE_IT_RT 5  // interrupt rising edge trigger
#define GPIO_MODE_IT_RFT 6 // interrupt rising and falling edge trigger

// @GPIO_PIN_OUT_TYPE
// GPIO pin possible output types
#define GPIO_OP_TYPE_PP 0 // push-pull
#define GPIO_OP_TYPE_OD 1 // open-drain

// @GPIO_SPEED
// GPIO pin possible output speeds
#define GPIO_SPEED_LOW 0    // low speed
#define GPIO_SPEED_MEDIUM 1 // medium speed
#define GPIO_SPEED_FAST 2   // fast speed
#define GPIO_SPEED_HIGH 3   // high speed

// @GPIO_PIN_PUPD
// GPIO pin pull-up/pull-down configuration macros
#define GPIO_NO_PUPD 0 // no pull-up or pull-down
#define GPIO_PIN_PU 1  // pull-up
#define GPIO_PIN_PD 2  // pull-down

typedef struct {
    uint8_t GPIO_PinNumber;      // possible values from @GPIO_PIN_NUMBERS
    uint8_t GPIO_PinMode;        // possible values from @GPIO_PIN_MODES
    uint8_t GPIO_PinSpeed;       // possible values from @GPIO_PIN_SPEED
    uint8_t GPIO_PinPuPdControl; // possible values from @GPIO_PIN_PUPD
    uint8_t GPIO_PinOPType;      // possible values from @GPIO_PIN_OUT_TYPE
    uint8_t GPIO_PinAltFunMode;  // possible values from @GPIO_PIN_ALT_FUN_MODE
} GPIO_PinConfig_t;

typedef struct {
    GPIO_RegDef_t *pGPIOx; // pointer to hold the base address of the GPIO port
                           // to which the pin belongs
    GPIO_PinConfig_t GPIO_PinConfig; // holds GPIO pin configuration settings

} GPIO_Handle_t;

void GPIO_Init(GPIO_Handle_t *pGPIOHandle); // initialize the GPIO port
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);    // reset the GPIO port

/**
 * @brief Enable or disable peripheral clock for the given GPIO port
 *
 * @param pGPIOx basde address of the GPIO port
 * @param EnorDi enable or disable MACROS
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,
                           uint8_t EnorDi); // enable or disable peripheral
                                            // clock for the given GPIO port

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,
                              uint8_t PinNumber); // read from a specific pin
uint16_t
GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx); // read from the entire port
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
                           uint8_t Value); // write to a specific pin
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,
                            uint16_t Value); // write to the entire port
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,
                          uint8_t PinNumber); // toggle the output pin

/**
 * @brief IRQ configuration and ISR handling
 *
 *
 * @param IRQNumber
 * @param IRQPriority
 * @param EnorDi
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,
                             uint8_t EnorDi); // IRQNumber is the IRQ number of

/**
 * @brief IRQ handling from the pin number
 *
 * @param PinNumber
 */
void GPIO_IRQHandling(uint8_t PinNumber); // IRQ handling from the pin number

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

#endif /* GPIO_DRIVER_H_ */
