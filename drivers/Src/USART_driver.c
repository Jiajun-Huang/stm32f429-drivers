#include "USART_driver.h"

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi) {
    if (EnOrDi == ENABLE) {
        if (pUSARTx == USART1) {
            USART1_PCLK_EN();
        } else if (pUSARTx == USART2) {
            USART2_PCLK_EN();
        } else if (pUSARTx == USART3) {
            USART3_PCLK_EN();
        } else if (pUSARTx == UART4) {
            UART4_PCLK_EN();
        } else if (pUSARTx == UART5) {
            UART5_PCLK_EN();
        } else if (pUSARTx == USART6) {
            USART6_PCLK_EN();
        }
    }
}

void USART_Init(USART_Handle_t *pUSARTHandle) {
    USART_RegDef_t *pUSARTx = pUSARTHandle->pUSARTx;
    USART_Config_t pUSARTConfig = pUSARTHandle->USART_Config;

    // Enable the peripheral clock
    USART_PeriClockControl(pUSARTx, ENABLE);

    // Enable USART peripheral
    pUSARTx->CR1.UE = SET;

    if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX) {
        // Implement the code to enable the Receiver bit field
        pUSARTx->CR1.RE = SET;
    } else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX) {
        // Implement the code to enable the Transmitter bit field
        pUSARTx->CR1.TE = SET;

    } else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX) {
        // Implement the code to enable the both Transmitter and Receiver bit fields
        pUSARTx->CR1.RE = SET;
        pUSARTx->CR1.TE = SET;
    }

    // Implement the code to configure the Word length configuration item
    pUSARTx->CR1.M = pUSARTConfig.USART_WordLength;

    // STOP bits configuration
    pUSARTx->CR2.STOP = pUSARTConfig.USART_NoOfStopBits;

    // config hardware flow control
    if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS) {
        // Implement the code to enable CTS flow control
        pUSARTx->CR3.CTSE = SET;

    } else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS) {
        // Implement the code to enable RTS flow control
        pUSARTx->CR3.RTSE = SET;
    } else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS) {
        // Implement the code to enable both CTS and RTS Flow control
        pUSARTx->CR3.CTSE = SET;
        pUSARTx->CR3.RTSE = SET;
    }

    // Implement the code to configure the USART baud rate
    USART_SetBaudRate(pUSARTx, pUSARTConfig.USART_BaudRate);
}
void USART_DeInit(USART_Handle_t *pUSARTHandle) {
}

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len) {
    uint16_t *pdata;

    // Loop over until "Len" number of bytes are transferred
    for (uint32_t i = 0; i < Len; i++) {
        // Implement the code to wait until TXE flag is set in the SR
        while (!pUSARTHandle->pUSARTx->SR.TXE)
            ;

        // Check the USART_WordLength item for 9BIT or 8BIT in a frame
        if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
            // if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
            pdata = (uint16_t *)pTxBuffer;
            pUSARTHandle->pUSARTx->DR.DR = (*pdata & (uint16_t)0x01FF);

            // check for USART_ParityControl
            if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
                // No parity is used in this transfer , so 9bits of user data will be sent
                // Implement the code to increment pTxBuffer twice
                pTxBuffer += 2;
            } else {
                // Parity bit is used in this transfer . so 8bits of user data will be sent
                // The 9th bit will be replaced by parity bit by the hardware
                pTxBuffer++;
            }
        } else {
            // This is 8bit data transfer
            pUSARTHandle->pUSARTx->DR.DR = *pTxBuffer;

            // Implement the code to increment the buffer address
            pTxBuffer++;
        }
    }

    // Implement the code to wait till TC flag is set in the SR
    while (!pUSARTHandle->pUSARTx->SR.TC)
        ;
}

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len) {
}

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len) {
}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len) {
}

void USART_IRQHandling(USART_Handle_t *pUSARTHandle) {
}

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi) {
}

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate) {
    // Variable to hold the APB clock
    uint32_t PCLKx;

    uint32_t usartdiv;

    // variables to hold Mantissa and Fraction values
    uint32_t M_part, F_part;

    uint32_t tempreg = 0;
    if (pUSARTx == USART1 || pUSARTx == USART6) {
        // USART1 and USART6 are hanging on APB2 bus
        PCLKx = getPCLK2Value();
    } else {
        PCLKx = getPCLK1Value();
    }

    // Check for OVER8 configuration bit
    if (pUSARTx->CR1.OVER8) {
        // OVER8 = 1 , over sampling by 8
        usartdiv = ((25 * PCLKx) / (2 * BaudRate));
    } else {
        // over sampling by 16
        usartdiv = ((25 * PCLKx) / (4 * BaudRate));
    }

    // Calculate the Mantissa part
    M_part = usartdiv / 100;

    // Place the Mantissa part in appropriate bit position . refer USART_BRR
    pUSARTx->BRR.DIV_Mantissa = M_part;

    // Extract the fraction part
    F_part = (usartdiv - (M_part * 100));

    // Calculate the final fractional
    if (pUSARTx->CR1.OVER8) {
        // OVER8 = 1 , over sampling by 8
        F_part = (((F_part * 8) + 50) / 100) & ((uint8_t)0x07);

    } else {
        // over sampling by 16
        F_part = (((F_part * 16) + 50) / 100) & ((uint8_t)0x0F);
    }

    // Place the fractional part in appropriate bit position . refer USART_BRR
    pUSARTx->BRR.DIV_Fracton = F_part;