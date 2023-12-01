#include "stm32f429.h"

void IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {
    uint8_t regSel = IRQNumber / 32;  // determine which register to use
    uint8_t bitPos = IRQNumber % 32;  // determine which bit to use

    if (EnorDi == ENABLE) {
        *(NVIC_ISER + regSel) |= (1U << bitPos);
    } else {
        *(NVIC_ICER + regSel) |= (1U << bitPos);
    }
}
void IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
    // find out the IPR register
    uint8_t iprx = IRQNumber / 4;          // find out the IPR register
    uint8_t iprx_section = IRQNumber % 4;  // find out the section of the IPR
                                           // register

    *(NVIC_IPR + iprx) |= (IRQPriority << (8 * iprx_section + 4));
}

uint32_t getPCLK1Value() {
    // get the value of the APB2 bus frequency

    uint32_t pclk = RCC->CFGR.SWS;
    if (pclk == 0) {  // HSI
        pclk = 16000000;
    } else if (pclk == 1) {  // HSE
        pclk = 8000000;
    } else if (pclk == 2) {  // PLL
        // need to find out the freq of PLL
    }

    // AHB1 prescaler
    uint8_t hpre = RCC->CFGR.HPRE;
    if (hpre >= 8) {
        pclk = pclk >> (hpre - 7);
    }

    // APB1 prescaler
    uint8_t ppre1 = RCC->CFGR.PPRE1;
    if (ppre1 >= 4) {
        pclk = pclk >> (ppre1 - 3);
    }

    return pclk;
}

uint32_t getPCLK1Value() {
    uint32_t pclk = RCC->CFGR.SWS;
    if (pclk == 0) {  // HSI
        pclk = 16000000;
    } else if (pclk == 1) {  // HSE
        pclk = 8000000;
    } else if (pclk == 2) {  // PLL
        // need to find out the freq of PLL
    }

    // AHB1 prescaler
    uint8_t hpre = RCC->CFGR.HPRE;
    if (hpre >= 8) {
        pclk = pclk >> (hpre - 7);
    }

    // APB1 prescaler
    uint8_t ppre2 = RCC->CFGR.PPRE2;
    if (ppre2 >= 4) {
        pclk = pclk >> (ppre2 - 3);
    }

    return pclk;
}
