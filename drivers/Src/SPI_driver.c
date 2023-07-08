/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: June 4, 2023
 *      Author: Jiajun
 */

#include "SPI_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle,
                                     SPI_CallbackFunc *callbackFunc);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle,
                                      SPI_CallbackFunc *callbackFunc);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle,
                                         SPI_CallbackFunc *callbackFunc);

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {

    if (EnorDi == ENABLE) {
        if (pSPIx == SPI1) {
            SPI1_PCLK_EN();
        } else if (pSPIx == SPI2) {
            SPI2_PCLK_EN();
        } else if (pSPIx == SPI3) {
            SPI3_PCLK_EN();
        }
    } else {
        // TODO
    }
}

void SPI_Init(SPI_Handle_t *pSPIHandle) {

    // peripheral clock enable

    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

    // first lets configure the SPI_CR1 register

    SPI_CR1_bits_t cr1 = {0};
    SPI_Config_t SPIConfig = pSPIHandle->SPIConfig;
    // 1. configure the device mode
    cr1.MSTR = SPIConfig.SPI_DeviceMode;

    // 2. Configure the bus config
    if (SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) { // full duplex
        // BIDI mode should be cleared
        // clear by default

    } else if (SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) { // half duplex
        // bidi mode should be set
        cr1.BIDIMODE = SET;
    } else if (SPIConfig.SPI_BusConfig ==
               SPI_BUS_CONFIG_SIMPLEX_RXONLY) { // simplex rx only
        // BIDI mode should be cleared
        // clear by default

        // RXONLY bit must be set
        cr1.RXONLY = SET;
    }

    // 3. Configure the spi serial clock speed (baud rate)
    cr1.BR = SPIConfig.SPI_SclkSpeed;

    // 4.  Configure the DFF
    cr1.DFF = SPIConfig.SPI_DFF;

    // 5. configure the CPOL
    cr1.CPOL = SPIConfig.SPI_CPOL;

    // 6 . configure the CPHA
    cr1.CPHA = SPIConfig.SPI_CPHA;

    // 7. configure the SSM
    cr1.SSM = SPIConfig.SPI_SSM;

    // configure the CR1 register
    pSPIHandle->pSPIx->CR1 = cr1;

    // pSPIHandle->pSPIx->CR1.SSI = SET;
    // pSPIHandle->pSPIx->CR1.SPE = SET;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx) {
    // set the reset bit
    if (pSPIx == SPI1) {
        SPI1_REG_RESET();
    } else if (pSPIx == SPI2) {
        SPI2_REG_RESET();
    } else if (pSPIx == SPI3) {
        SPI3_REG_RESET();
    }
}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {
    while (Len > 0) {
        // 1. wait until TXE is set
        while (!(pSPIx->SR.TXE))
            ;

        // 2. check the DFF bit in CR1
        if (pSPIx->CR1.DFF) {
            // 16 bit DFF
            // 1. load the data in to the DR
            pSPIx->DR = *((uint16_t *)pTxBuffer);
            Len -= 2;
            (uint16_t *)pTxBuffer++;
        } else {
            // 8 bit DFF
            pSPIx->DR = *pTxBuffer;
            Len--;
            pTxBuffer++;
        }
    }
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {
    while (Len > 0) {
        // 1. wait until RXNE is set
        while (!(pSPIx->SR.RXNE))
            ;

        // 2. check the DFF bit in CR1
        if (pSPIx->CR1.DFF) {
            // 16 bit DFF
            // 1. load the data from DR to Rxbuffer address
            *((uint16_t *)pRxBuffer) = pSPIx->DR;
            Len -= 2;
            (uint16_t *)pRxBuffer++;
        } else {
            // 8 bit DFF
            *(pRxBuffer) = pSPIx->DR;
            Len--;
            pRxBuffer++;
        }
    }
}

void SPI_PeripheralSwich(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
    if (EnOrDi == ENABLE) {
        pSPIx->CR1.SPE = SET;
    } else {
        pSPIx->CR1.SPE = RESET;
    }
}

void SPI_SSISwich(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
    if (EnOrDi == ENABLE) {
        pSPIx->CR1.SSI = SET;
    } else {
        pSPIx->CR1.SSI = RESET;
    }
}

void SPI_SSOESwich(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {
    if (EnOrDi == ENABLE) {
        pSPIx->CR2.SSOE = SET;
    } else {
        pSPIx->CR2.SSOE = RESET;
    }
}


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer,
                       uint32_t Len) {
    uint8_t state = pSPIHandle->TxState;

    // if not busy in transmission
    if (state != SPI_BUSY_IN_TX) {
        // 1 . Save the Tx buffer address and Len information in some global
        // variables
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLen = Len;
        // 2.  Mark the SPI state as busy in transmission so that no other code
        // can take over same SPI peripheral until transmission is over
        pSPIHandle->TxState = SPI_BUSY_IN_TX;

        // 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is
        // set
        pSPIHandle->pSPIx->CR2.TXEIE = SET;
    }

    return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer,
                          uint32_t Len) {
    uint8_t state = pSPIHandle->RxState;

    if (state != SPI_BUSY_IN_RX) {
        // 1 . Save the Rx buffer address and Len information in some global
        // variables
        pSPIHandle->pRxBuffer = pRxBuffer;
        pSPIHandle->RxLen = Len;
        // 2.  Mark the SPI state as busy in reception so that no other code can
        // take over same SPI peripheral until reception is over
        pSPIHandle->RxState = SPI_BUSY_IN_RX;

        // 3. Enable the RXNEIE control bit to get interrupt whenever RXNEIE
        // flag is set in SR
        pSPIHandle->pSPIx->CR2.RXNEIE = SET;
    }

    return state;
}

void SPI_IRQHandling(SPI_Handle_t *pHandle, SPI_CallbackFunc *pCallbackFunc) {

    register uint8_t isBufferEmpty, isInterruptEnabled;

    // first lets check for TXE
    isBufferEmpty = pHandle->pSPIx->SR.TXE;
    isInterruptEnabled = pHandle->pSPIx->CR2.TXEIE;

    if (isBufferEmpty && isInterruptEnabled) {
        // handle TXE
        spi_txe_interrupt_handle(pHandle, pCallbackFunc);
    }

    // check for RXNE
    isBufferEmpty = pHandle->pSPIx->SR.RXNE;
    isInterruptEnabled = pHandle->pSPIx->CR2.RXNEIE;

    if (isBufferEmpty && isInterruptEnabled) {
        // handle RXNE
        spi_rxne_interrupt_handle(pHandle, pCallbackFunc);
    }

    // check for ovr flag
    register uint8_t isOverRuned = pHandle->pSPIx->SR.OVR;
    isInterruptEnabled = pHandle->pSPIx->CR2.ERRIE;

    if (isOverRuned && isInterruptEnabled) {
        // handle ovr error
        spi_ovr_err_interrupt_handle(pHandle, pCallbackFunc);
    }

    // CRC error

    // MODF error
}

// some helper function implementations

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle,
                                     SPI_CallbackFunc *pCallbackFunc) {
    // check the DFF bit in CR1
    if (pSPIHandle->pSPIx->CR1.DFF) {
        // 16 bit DFF
        // 1. load the data in to the DR
        pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pTxBuffer);
        pSPIHandle->TxLen -= 2;
        (uint16_t *)pSPIHandle->pTxBuffer++;
    } else {
        // 8 bit DFF
        pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
        pSPIHandle->TxLen--;
        pSPIHandle->pTxBuffer++;
    }

    if (!pSPIHandle->TxLen) {
        // TxLen is zero , so close the spi transmission and inform the
        // application that TX is over.

        // this prevents interrupts from setting up of TXE flag
        SPI_CloseTransmisson(pSPIHandle);
        (*pCallbackFunc)(pSPIHandle, SPI_EVENT_TX_CMPLT);
    }
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle,
                                      SPI_CallbackFunc *pCallbackFunc) {
    // do rxing as per the dff
    if (pSPIHandle->pSPIx->CR1.RXONLY) {
        // 16 bit
        *((uint16_t *)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen -= 2;
        pSPIHandle->pRxBuffer++;
        pSPIHandle->pRxBuffer++;

    } else {
        // 8 bit
        *(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen--;
        pSPIHandle->pRxBuffer++;
    }

    if (!pSPIHandle->RxLen) {
        // reception is complete
        SPI_CloseReception(pSPIHandle);
        (*pCallbackFunc)(pSPIHandle, SPI_EVENT_RX_CMPLT);
    }
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle,
                                         SPI_CallbackFunc *pCallbackFunc) {

    uint8_t temp;
    // 1. clear the ovr flag
    if (pSPIHandle->TxState != SPI_BUSY_IN_TX) {
        temp = pSPIHandle->pSPIx->DR;
        //        temp = pSPIHandle->pSPIx->SR;
    }
    (void)temp;
    // 2. inform the application
    (*pCallbackFunc)(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle) {
    pSPIHandle->pSPIx->CR2.TXEIE = RESET;
    pSPIHandle->pTxBuffer = RESET;
    pSPIHandle->TxLen = 0;
    pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle) {
    pSPIHandle->pSPIx->CR2.RXNEIE = RESET;
    pSPIHandle->pRxBuffer = RESET;
    pSPIHandle->RxLen = 0;
    pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) {
    // uint8_t temp;
    // temp = pSPIx->DR;
    // temp = pSPIx->SR;
    // (void)temp;
}

// __weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,
//                                          uint8_t AppEv) {

//     // This is a weak implementation . the user application may override this
//     // function.
// }
