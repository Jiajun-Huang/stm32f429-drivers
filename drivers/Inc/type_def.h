/*
 * type_def.h
 *
 *  Created on: Jul 4, 2023
 *      Author: jiajun
 */

#include <stdint.h>

#ifndef TYPE_DEF_H_
#define TYPE_DEF_H_

#define __vo volatile

typedef struct {
    __vo uint32_t IMR;   // Interrupt mask register
    __vo uint32_t EMR;   // Event mask register
    __vo uint32_t RTSR;  // Rising trigger selection register
    __vo uint32_t FTSR;  // Falling trigger selection register
    __vo uint32_t SWIER; // Software interrupt event register
    __vo uint32_t PR;    // Pending register
} EXTI_RegDef_t;         // External interrupt/event controller

typedef struct {
    __vo uint32_t MEMRMP; // SYSCFG memory remap register
    __vo uint32_t PMC;    // SYSCFG peripheral mode configuration register
    __vo uint32_t
        EXTICR[4];         // SYSCFG external interrupt configuration register 1
    uint32_t RESERVED1[2]; // Reserved, 0x14-0x18
    __vo uint32_t CMPCR;   // Compensation cell control register
    uint32_t RESERVED2[2]; // Reserved, 0x20-0x24
    __vo uint32_t CFGR;    // SYSCFG configuration register
} SYSCFG_RegDef_t;         // System configuration controller

typedef struct {
    __vo uint32_t CR;         // RCC clock control register
    __vo uint32_t PLLCFGR;    // RCC PLL configuration register
    __vo uint32_t CFGR;       // RCC clock configuration register
    __vo uint32_t CIR;        // RCC clock interrupt register
    __vo uint32_t AHB1RSTR;   // RCC AHB1 peripheral reset register
    __vo uint32_t AHB2RSTR;   // RCC AHB2 peripheral reset register
    __vo uint32_t AHB3RSTR;   // RCC AHB3 peripheral reset register
    uint32_t RESERVED0;       // Reserved, 0x1C
    __vo uint32_t APB1RSTR;   // RCC APB1 peripheral reset register
    __vo uint32_t APB2RSTR;   // RCC APB2 peripheral reset register
    uint32_t RESERVED1[2];    // Reserved, 0x28-0x2C
    __vo uint32_t AHB1ENR;    // RCC AHB1 peripheral clock enable register
    __vo uint32_t AHB2ENR;    // RCC AHB2 peripheral clock enable register
    __vo uint32_t AHB3ENR;    // RCC AHB3 peripheral clock enable register
    uint32_t RESERVED2;       // Reserved, 0x3C
    __vo uint32_t APB1ENR;    // RCC APB1 peripheral clock enable register
    __vo uint32_t APB2ENR;    // RCC APB2 peripheral clock enable register
    uint32_t RESERVED3[2];    // Reserved, 0x48-0x4C
    __vo uint32_t AHB1LPENR;  // RCC AHB1 peripheral clock enable in low
                              // power mode register
    __vo uint32_t AHB2LPENR;  // RCC AHB2 peripheral clock enable in low
                              // power mode register
    __vo uint32_t AHB3LPENR;  // RCC AHB3 peripheral clock enable in low
                              // power mode register
    uint32_t RESERVED4;       // Reserved, 0x5C
    __vo uint32_t APB1LPENR;  // RCC APB1 peripheral clock enable in low
                              // power mode register
    __vo uint32_t APB2LPENR;  // RCC APB2 peripheral clock enable in low
                              // power mode register
    uint32_t RESERVED5[2];    // Reserved, 0x68-0x6C
    __vo uint32_t BDCR;       // RCC Backup domain control register
    __vo uint32_t CSR;        // RCC clock control & status register
    uint32_t RESERVED6[2];    // Reserved, 0x78-0x7C
    __vo uint32_t SSCGR;      // RCC spread spectrum clock generation register
    __vo uint32_t PLLI2SCFGR; // RCC PLLI2S configuration register
} RCC_RegDef_t;               // Reset and clock control register

//** GPIO **//
typedef struct {
    __vo uint32_t MODER;   // GPIO port mode register
    __vo uint32_t OTYPER;  // GPIO port output type register
    __vo uint32_t OSPEEDR; // GPIO port output speed register
    __vo uint32_t PUPDR;   // GPIO port pull-up/pull-down register
    __vo uint32_t IDR;     // GPIO port input data register
    __vo uint32_t ODR;     // GPIO port output data register
    __vo uint32_t BSRR;    // GPIO port bit set/reset register
    __vo uint32_t LCKR;    // GPIO port configuration lock register
    __vo uint32_t AFR[2];  // GPIO alternate function low register
                           // GPIO alternate function high register
} GPIO_RegDef_t;

//** SPI **//


typedef struct {
    __vo uint32_t CPHA : 1;     // Clock phase
    __vo uint32_t CPOL : 1;     // Clock polarity
    __vo uint32_t MSTR : 1;     // Master selection
    __vo uint32_t BR : 3;       // Baud rate control
    __vo uint32_t SPE : 1;      // SPI enable
    __vo uint32_t LSBFIRST : 1; // Frame format
    __vo uint32_t SSI : 1;      // Internal slave select
    __vo uint32_t SSM : 1;      // Software slave management
    __vo uint32_t RXONLY : 1;   // Receive only
    __vo uint32_t DFF : 1;      // Data frame format
    __vo uint32_t CRCNEXT : 1;  // CRC transfer next
    __vo uint32_t CRCEN : 1;    // Hardware CRC calculation enable
    __vo uint32_t BIDIOE : 1;   // Output enable in bidirectional mode
    __vo uint32_t BIDIMODE : 1; // Bidirectional data mode enable
    __vo uint32_t RESERVED : 16;
} SPI_CR1_bits_t; // 28.5.1  SPI control register 1 (SPIx_CR1)

typedef struct {
    __vo uint32_t RXDMAEN : 1; // Rx buffer DMA enable
    __vo uint32_t TXDMAEN : 1; // Tx buffer DMA enable
    __vo uint32_t SSOE : 1;    // SS output enable
    __vo uint32_t REASERVED1 : 1;
    __vo uint32_t FRF : 1;    // Frame format
    __vo uint32_t ERRIE : 1;  // Error interrupt enable
    __vo uint32_t RXNEIE : 1; // RX buffer not empty interrupt enable
    __vo uint32_t TXEIE : 1;  // Tx buffer empty interrupt enable
    __vo uint32_t REASERVED2 : 24;
} SPI_CR2_bits_t; // 28.5.2  SPI control register 2 (SPIx_CR2)

// 28.5.3  SPI status register (SPIx_SR)
typedef struct {
    __vo uint32_t RXNE : 1;   // Receive buffer not empty
    __vo uint32_t TXE : 1;    // Transmit buffer empty
    __vo uint32_t CHSIDE : 1; // Channel side
    __vo uint32_t UDR : 1;    // Underrun flag
    __vo uint32_t CRCERR : 1; // CRC error flag
    __vo uint32_t MODF : 1;   // Mode fault
    __vo uint32_t OVR : 1;    // Overrun flag
    __vo uint32_t BSY : 1;    // Busy flag
    __vo uint32_t FRE : 1;    // Frame format error
    __vo uint32_t RESERVED1 : 23;
} SPI_SR_bits_t;

//  28.5.4  SPI data register (SPIx_DR)
typedef struct {
    __vo uint32_t DR : 16; // Data register
    __vo uint32_t RESERVED : 16;
} SPI_DR_bits_t;

// 28.5.5  SPI CRC polynomial register (SPIx_CRCPR)
typedef struct {
    __vo uint32_t CRCPOLY : 16; // CRC polynomial register
    __vo uint32_t RESERVED : 16;
} SPI_CRCPR_bits_t;

// 28.5.6  SPI RX CRC register (SPIx_RXCRCR)
typedef struct {
    __vo uint32_t RxCRC : 16; // Rx CRC register
    __vo uint32_t RESERVED : 16;
} SPI_RXCRCR_bits_t;

// 28.5.7  SPI TX CRC register (SPIx_TXCRCR)
typedef struct {
    __vo uint32_t TxCRC : 16; // Tx CRC register
    __vo uint32_t RESERVED : 16;
} SPI_TXCRCR_bits_t;

// 28.5.8  SPI_I2S configuration register (SPIx_I2SCFGR)
typedef struct {
    __vo uint32_t
        CHLEN : 1; // Channel length (number of bits per audio channel)
    __vo uint32_t DATLEN : 2;     // Data length to be transferred
    __vo uint32_t CKPOL : 1;      // Steady state clock polarity
    __vo uint32_t I2SSTD : 2;     // I2S standard selection
    __vo uint32_t RESERVED : 1;   // Reserved
    __vo uint32_t PCMSYNC : 1;    // PCM frame synchronization
    __vo uint32_t I2SCFG : 2;     // I2S configuration mode
    __vo uint32_t I2SE : 1;       // I2S Enable
    __vo uint32_t I2SMOD : 1;     // I2S mode selection
    __vo uint32_t RESERVED2 : 20; // Reserved
} SPI_I2SCFGR_bits_t;

// 28.5.9  SPI_I2S prescaler register (SPIx_I2SPR)
typedef struct {
    __vo uint32_t I2SDIV : 8; // I2S Linear prescaler
    __vo uint32_t ODD : 1;    // Odd factor for the prescaler
    __vo uint32_t MCKOE : 1;  // Master clock output enable
    __vo uint32_t RESERVED1 : 22;
} SPI_I2SPR_bits_t;

typedef struct {
    __vo SPI_CR1_bits_t CR1;         // SPI control register 1
    __vo SPI_CR2_bits_t CR2;         // SPI control register 2
    __vo SPI_SR_bits_t SR;           // SPI status register
    __vo SPI_DR_bits_t DR;           // SPI data register
    __vo SPI_CRCPR_bits_t CRCPR;     // SPI CRC polynomial register
    __vo SPI_RXCRCR_bits_t RXCRCR;   // SPI RX CRC register
    __vo SPI_TXCRCR_bits_t TXCRCR;   // SPI TX CRC register
    __vo SPI_I2SCFGR_bits_t I2SCFGR; // SPI_I2S configuration register
    __vo SPI_I2SPR_bits_t I2SPR;     // SPI_I2S prescaler register
} SPI_RegDef_t; 

//** END OF SPI **//



#endif /* TYPE_DEF_H_ */
