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
    pI2Cx->I2C_CR1.ACK = config.I2C_AckControl;

    I2C_PeriClockControl(pI2Cx, ENABLE);

    // cr2 calculate freq
    // which syscalck is used
    uint32_t pclk = RCC->CFGR.SWS;
    if (pclk == 0) { // HSI
        pclk = 16000000;
    } else if (pclk == 1) { // HSE
        pclk = 8000000;
    } else if (pclk == 2) { // PLL
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
    if (config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
        // standard mode
        ccr = pclk / (2 * config.I2C_SCLSpeed);
    } else {
        // fast mode
        pI2Cx->I2C_CCR.FS = 1;
        pI2Cx->I2C_CCR.DUTY = config.I2C_FMDutyCycle;
        if (config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
            ccr = pclk / (3 * config.I2C_SCLSpeed);
        } else {
            ccr = pclk / (25 * config.I2C_SCLSpeed);
        }
    }
    pI2Cx->I2C_CCR.CCR = ccr;
}
void I2C_DeInit(I2C_RegDef_t *pI2Cx) {}
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr) {
    
}
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
                           uint32_t Len, uint8_t SlaveAddr) {}
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data) {}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx) {}

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {}
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {}
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle) {}
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle) {}
