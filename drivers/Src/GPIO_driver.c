/*
 * GPIO_driver.c
 *
 *  Created on: Jul 2, 2023
 *      Author: jiajun
 */

#include "GPIO_driver.h"

void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
    register uint32_t temp = 0; // temp register
    register uint8_t pinNumber = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

    GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
    // 1. configure the mode of gpio pin
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
        // non interrupt mode
        temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pinNumber);
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << 2 * pinNumber); // clear
        pGPIOHandle->pGPIOx->MODER |= temp;

    } else {
        // interrupt mode
        if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
            // configure the FTSR
            EXTI->FTSR |= (1 << pinNumber);
            EXTI->RTSR &= ~(1 << pinNumber);

        } else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==
                   GPIO_MODE_IT_RT) {
            // configure the RTSR
            EXTI->RTSR |= (1 << pinNumber);
            EXTI->FTSR &= ~(1 << pinNumber);

        } else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==
                   GPIO_MODE_IT_RFT) {
            // configure both FTSR and RTSR
            EXTI->FTSR |= (1 << pinNumber);
            EXTI->RTSR |= (1 << pinNumber);
        }

        // configure the GPIO port selection in SYSCFG_EXTICR
        uint8_t temp1 = pinNumber / 4; // which exti register
        uint8_t temp2 = pinNumber % 4; // which pin in the register
        uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
        SYSCFG_PCLK_EN(); // enable the clock for SYSCFG
        SYSCFG->EXTICR[temp1] =
            portCode << (temp2 *
                         4); // configure the port selection in SYSCFG_EXTICR

        // enable the exti interrupt delivery using IMR (interrupt mask
        // register)
        EXTI->IMR |= (1 << pinNumber);
    }
    // 2. configure the speed
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << 2 * pinNumber); // clear
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;

    // 3. configure the pupd settings
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << 2 * pinNumber); // clear
    pGPIOHandle->pGPIOx->PUPDR |= temp;

    // 4. configure the output type
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pinNumber));
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pinNumber); // clear
    pGPIOHandle->pGPIOx->OTYPER |= temp;

    // 5. configure the alternate functionality
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
        uint32_t temp1, temp2;
        temp1 = pinNumber / 8; // ALT function high or low register
        temp2 = pinNumber % 8; // which pin in the register

        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
        pGPIOHandle->pGPIOx->AFR[temp1] |=
            (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
    }
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
    if (pGPIOx == GPIOA) {
        GPIOA_REG_RESET();
    } else if (pGPIOx == GPIOB) {
        GPIOB_REG_RESET();
    } else if (pGPIOx == GPIOC) {
        GPIOC_REG_RESET();
    } else if (pGPIOx == GPIOD) {
        GPIOD_REG_RESET();
    } else if (pGPIOx == GPIOE) {
        GPIOE_REG_RESET();
    } else if (pGPIOx == GPIOF) {
        GPIOF_REG_RESET();
    } else if (pGPIOx == GPIOG) {
        GPIOG_REG_RESET();
    } else if (pGPIOx == GPIOH) {
        GPIOH_REG_RESET();
    } else if (pGPIOx == GPIOI) {
        GPIOI_REG_RESET();
    }
}
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        if (pGPIOx == GPIOA) {
            GPIOA_PCLK_EN();
        } else if (pGPIOx == GPIOB) {
            GPIOB_PCLK_EN();
        } else if (pGPIOx == GPIOC) {
            GPIOC_PCLK_EN();
        } else if (pGPIOx == GPIOD) {
            GPIOD_PCLK_EN();
        } else if (pGPIOx == GPIOE) {
            GPIOE_PCLK_EN();
        } else if (pGPIOx == GPIOF) {
            GPIOF_PCLK_EN();
        } else if (pGPIOx == GPIOG) {
            GPIOG_PCLK_EN();
        } else if (pGPIOx == GPIOH) {
            GPIOH_PCLK_EN();
        } else if (pGPIOx == GPIOI) {
            GPIOI_PCLK_EN();
        }
    } else if (EnorDi == DISABLE) {
        if (pGPIOx == GPIOA) {
            GPIOA_PCLK_DI();
        } else if (pGPIOx == GPIOB) {
            GPIOB_PCLK_DI();
        } else if (pGPIOx == GPIOC) {
            GPIOC_PCLK_DI();
        } else if (pGPIOx == GPIOD) {
            GPIOD_PCLK_DI();
        } else if (pGPIOx == GPIOE) {
            GPIOE_PCLK_DI();
        } else if (pGPIOx == GPIOF) {
            GPIOF_PCLK_DI();
        } else if (pGPIOx == GPIOG) {
            GPIOG_PCLK_DI();
        } else if (pGPIOx == GPIOH) {
            GPIOH_PCLK_DI();
        } else if (pGPIOx == GPIOI) {
            GPIOI_PCLK_DI();
        }
    }

} // enable or disable peripheral
  // clock for the given GPIO port
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
    uint8_t value;
    value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
    return value;
} // read from a specific pin
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
    uint16_t value;
    value = (uint16_t)pGPIOx->IDR;
    return value;
} // read from the entire port
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
                           uint8_t Value) {
    if (Value == GPIO_PIN_SET) {
        // write 1 to the output data register at the bit field corresponding to
        // the pin number
        pGPIOx->ODR |= (1 << PinNumber);
    } else {
        // write 0
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
} // write to a specific pin
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {
    pGPIOx->ODR = Value;
} // write to the entire port
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
    pGPIOx->ODR ^= (1 << PinNumber);
} // toggle the output pin

void GPIO_IRQHandling(uint8_t PinNumber) {
    // clear the EXTI PR register corresponding to the pin number
    if (EXTI->PR & (1 << PinNumber)) {
        // clear
        EXTI->PR |= (1 << PinNumber);
    }
} // IRQ handling from the pin number

void GPIO_RegisterIRQHandler(uint8_t PinNumber, uint32_t IRQPriority,
                             void (*function)(void)) {

    IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);
    IRQPriorityConfig(IRQ_NO_EXTI0, IRQPriority);

    if (PinNumber >= 0 && PinNumber <= 4) {
        *((void (**)(void))EXTI0_HANDLER_ADDR + PinNumber) = function;
    } else if (PinNumber >= 5 && PinNumber <= 9) {
    } else if (PinNumber >= 10 && PinNumber <= 15) {
        *((void (**)(void))EXTI15_10_HANDLER_ADDR) = function;
    }
}
