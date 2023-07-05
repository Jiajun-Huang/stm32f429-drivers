/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */


#if !defined(__SOFT_FP__) && defined(__ARM_FP)
#warning                                                                       \
    "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void GPIO_interrup_test(void);


#include <stdint.h>
#include <stm32f429.h>
// #include <GPIO_driver.h>

int main(void) {
    GPIO_interrup_test();
    for (;;)
        ;
}

void GPIO_interrup_test(void) {
    // config PB0 as input interrupt
    GPIO_Handle_t gpio = {
        .GPIO_PinConfig = {.GPIO_PinNumber = GPIO_PIN_0,
                           .GPIO_PinMode = GPIO_MODE_IT_FT,
                           .GPIO_PinSpeed = GPIO_SPEED_FAST,
                           .GPIO_PinPuPdControl = GPIO_NO_PUPD,
                           .GPIO_PinOPType = GPIO_OP_TYPE_PP,
                           .GPIO_PinAltFunMode = 0},
        .pGPIOx = GPIOB};
    
    GPIO_Init(&gpio);

    // config PG13 as output
    gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
    gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;

    GPIO_Init(&gpio);
}