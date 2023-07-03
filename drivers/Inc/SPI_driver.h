/*
 * SPI_driver.h
 *
 *  Created on: Jul. 3, 2023
 *      Author: jiajun
 */

#ifndef INC_SPI_DRIVER_H_
#define INC_SPI_DRIVER_H_
#include <stdint.h>
#include "stm32f429xx.h"

typedef struct {
    uint8_t SPI_DeviceMode;
    uint8_t SPI_BusConfig;
    uint8_t SPI_SclkSpeed;
    uint8_t SPI_DFF;
    uint8_t SPI_CPOL;
    uint8_t SPI_CPHA;
    uint8_t SPI_SSM;
} SPI_Config_t;

typedef struct {
    SPI_RegDef_t *pSPIx;
    SPI_Config_t SPIConfig;
} SPI_Handle_t;



#endif /* INC_SPI_DRIVER_H_ */
