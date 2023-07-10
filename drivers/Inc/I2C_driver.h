#include "stm32f429.h"

#ifndef INC_I2C_DRIVER_H_
#define INC_I2C_DRIVER_H_

typedef struct {
    uint32_t I2C_SCLSpeed;
    uint32_t I2C_DeviceAddress;
    uint32_t I2C_AckControl;
    uint32_t I2C_FMDutyCycle;
    uint32_t I2C_ADDMode;
} I2C_Config_t;

typedef struct {
    I2C_RegDef_t *pI2Cx;
    I2C_Config_t I2C_Config;
    uint8_t *pTxBuffer;
    uint8_t *pRxBuffer;
    uint32_t TxLen;
    uint32_t RxLen;
    uint8_t TxRxState;
    uint8_t DevAddr;
    uint32_t RxSize;
    uint8_t Sr;
} I2C_Handle_t;

#define I2C_SCL_SPEED_SM 100000
#define I2C_SCL_SPEED_FM4k 400000
#define I2C_SCL_SPEED_FM2k 200000

// AckControl

#define I2C_ACK_ENABLE 1
#define I2C_ACK_DISABLE 0

// FMDutyCycle
#define I2C_FM_DUTY_2 0
#define I2C_FM_DUTY_16_9 1

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
/**
 * @brief Initialize the I2C peripheral
 *
 * @param pI2CHandle address bit have to be correct
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint16_t SlaveAddr);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint16_t SlaveAddr);

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint16_t SlaveAddr);

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint16_t SlaveAddr);

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);
void I2C_PeriferalControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

#endif /* INC_I2C_DRIVER_H_ */