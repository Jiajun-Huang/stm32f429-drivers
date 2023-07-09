#include "I2C_driver.h"

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
    // enable clock for given I2C peripheral
    if (EnorDi == ENABLE) {
        if (pI2Cx == I2C1) {
            I2C1_PCLK_EN();
        } else if (pI2Cx == I2C2) {
            I2C2_PCLK_EN();
        } else if (pI2Cx == I2C3) {
            I2C3_PCLK_EN();
        }
    } else if (EnorDi == DISABLE) {
        if (pI2Cx == I2C1) {
            I2C1_PCLK_DI();
        } else if (pI2Cx == I2C2) {
            I2C2_PCLK_DI();
        } else if (pI2Cx == I2C3) {
            I2C3_PCLK_DI();
        }
    }
}
void I2C_Init(I2C_Handle_t *pI2CHandle) {
    I2C_Config_t config = pI2CHandle->I2C_Config;
    I2C_RegDef_t *pI2Cx = pI2CHandle->pI2Cx;
    // cr1 config ark
    I2C_PeriClockControl(pI2Cx, ENABLE);
    pI2Cx->I2C_CR1.PE = ENABLE;
    pI2Cx->I2C_CR1.ACK = config.I2C_AckControl;


    // cr2 calculate freq
    // which syscalck is used
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

    // config CR2 freq
    pI2Cx->I2C_CR2.FREQ = pclk / 1000000;

    // configure the configure address of I2C
    uint8_t addmode = config.I2C_ADDMode;
    if (addmode) {
        // 10 bit
        pI2Cx->I2C_OAR1.ADDMODE = 1;
        pI2Cx->I2C_OAR1.ADD = config.I2C_DeviceAddress;
    } else {
        // 7 bit
        pI2Cx->I2C_OAR1.ADDMODE = 0;
        pI2Cx->I2C_OAR1.ADD = config.I2C_DeviceAddress << 1;
    }

    // configure CCR
    uint16_t ccr = 0;
    uint16_t trise = 0;
    if (config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
        // standard mode
        ccr = pclk / (2 * config.I2C_SCLSpeed);
        trise = pclk / 1000000 + 1;
    } else {
        // fast mode
        pI2Cx->I2C_CCR.FS = 1;
        pI2Cx->I2C_CCR.DUTY = config.I2C_FMDutyCycle;
        trise = (pclk / 1000000) * 300 + 1;
        if (config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
            ccr = pclk / (3 * config.I2C_SCLSpeed);
        } else {
            ccr = pclk / (25 * config.I2C_SCLSpeed);
        }
    }
    pI2Cx->I2C_TRISE.TRISE = trise;
    pI2Cx->I2C_CCR.CCR = ccr;
}
void I2C_DeInit(I2C_RegDef_t *pI2Cx) {}
void I2C_PeriferalControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        pI2Cx->I2C_CR1.PE = 1;
    } else {
        pI2Cx->I2C_CR1.PE = 0;
    }
}
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint16_t SlaveAddr) {
    I2C_RegDef_t *pI2Cx = pI2CHandle->pI2Cx;

    // generate start condition
    pI2Cx->I2C_CR1.START = 1;
    // confirm that start generation is completed by checking the SB flag in the SR1 until SB is cleared SCL will be stretched
    while (!pI2Cx->I2C_SR1.SB)
        ;

    // send the address of the slave with r/nw bit set to w(0) (total 8 bits)
    SlaveAddr = SlaveAddr << 1;
    pI2Cx->I2C_DR.DR = SlaveAddr;

    // confirm that address phase is completed by checking the ADDR flag in the SR1
    while (!pI2Cx->I2C_SR1.ADDR)
        ;
    // clear the ADDR by reading SR1 and SR2
    I2C_SR1_bits_t dummyread1 = pI2Cx->I2C_SR1;
    I2C_SR2_bits_t dummyread2 = pI2Cx->I2C_SR2;

    // send the data until Len becomes 0
    while (Len > 0) {
        while (!pI2Cx->I2C_SR1.TxE)
            ;
        pI2Cx->I2C_DR.DR = *pTxBuffer;
        pTxBuffer++;
        Len--;
    }

    // wait for TXE=1 and BTF=1 before generating the stop condition
    while (!pI2Cx->I2C_SR1.TxE)
        ;
    while (!pI2Cx->I2C_SR1.BTF)
        ;

    // generate stop condition
    pI2Cx->I2C_CR1.STOP = 1;
}
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
                           uint32_t Len, uint16_t SlaveAddr) {}
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data) {}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx) {}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle) {}
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle) {}
