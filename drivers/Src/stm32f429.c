#include "stm32f429.h"

void IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {
    uint8_t regSel = IRQNumber / 32; // determine which register to use
    uint8_t bitPos = IRQNumber % 32; // determine which bit to use

    if (EnorDi == ENABLE) {
        *(NVIC_ISER + regSel) |= (1U << bitPos);
    } else {
        *(NVIC_ICER + regSel) |= (1U << bitPos);
    }
}
void IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
    // find out the IPR register
    uint8_t iprx = IRQNumber / 4;         // find out the IPR register
    uint8_t iprx_section = IRQNumber % 4; // find out the section of the IPR
                                          // register

    *(NVIC_IPR + iprx) |= (IRQPriority << (8 * iprx_section + 4));
}
