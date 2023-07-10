#ifndef INC_USART_DRIVER_H_
#define INC_USART_DRIVER_H_

#include <stdint.h>

#include "stm32f429.h"

typedef struct
{
    uint8_t USART_Mode;
    uint32_t USART_BaudRate;
    uint8_t USART_NoOfStopBits;
    uint8_t USART_WordLength;
    uint8_t USART_ParityControl;
    uint8_t USART_HWFlowControl;
} USART_Config_t;

typedef struct
{
    USART_RegDef_t *pUSARTx;
    USART_Config_t USART_Config;
    uint8_t *pTxBuffer;
    uint8_t *pRxBuffer;
    uint32_t TxLen;
    uint32_t RxLen;
    uint8_t TxState;
    uint8_t RxState;
} USART_Handle_t;



#endif /* INC_USART_DRIVER_H_ */
