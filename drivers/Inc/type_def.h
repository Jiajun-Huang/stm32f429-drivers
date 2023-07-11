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
    __vo uint32_t IMR;    // Interrupt mask register
    __vo uint32_t EMR;    // Event mask register
    __vo uint32_t RTSR;   // Rising trigger selection register
    __vo uint32_t FTSR;   // Falling trigger selection register
    __vo uint32_t SWIER;  // Software interrupt event register
    __vo uint32_t PR;     // Pending register
} EXTI_RegDef_t;          // External interrupt/event controller

typedef struct {
    __vo uint32_t MEMRMP;     // SYSCFG memory remap register
    __vo uint32_t PMC;        // SYSCFG peripheral mode configuration register
    __vo uint32_t EXTICR[4];  // SYSCFG external interrupt configuration register 1
    uint32_t RESERVED1[2];    // Reserved, 0x14-0x18
    __vo uint32_t CMPCR;      // Compensation cell control register
    uint32_t RESERVED2[2];    // Reserved, 0x20-0x24
    __vo uint32_t CFGR;       // SYSCFG configuration register
} SYSCFG_RegDef_t;            // System configuration controller

typedef struct {
    __vo uint32_t SW : 2;         // System clock switch rw
    __vo uint32_t SWS : 2;        // System clock switch status r
    __vo uint32_t HPRE : 4;       // AHB prescaler rw
    __vo uint32_t RESERVED0 : 2;  // Reserved, 0x8-0x9
    __vo uint32_t PPRE1 : 3;      // APB low-speed prescaler
    __vo uint32_t PPRE2 : 3;      // APB high-speed prescaler
    __vo uint32_t RTCPRE : 5;     // HSE division factor for RTC clock
    __vo uint32_t MCO1 : 2;       // Microcontroller clock output 1
    __vo uint32_t I2SSRC : 1;     // I2S clock selection
    __vo uint32_t MCO1PRE : 3;    // MCO1 prescaler
    __vo uint32_t MCO2PRE : 3;    // MCO2 prescaler
    __vo uint32_t MCO2 : 2;       // Microcontroller clock output 2
} RCC_CFGR_bits_t;

typedef struct {
    __vo uint32_t CR;           // RCC clock control register
    __vo uint32_t PLLCFGR;      // RCC PLL configuration register
    __vo RCC_CFGR_bits_t CFGR;  // RCC clock configuration register
    __vo uint32_t CIR;          // RCC clock interrupt register
    __vo uint32_t AHB1RSTR;     // RCC AHB1 peripheral reset register
    __vo uint32_t AHB2RSTR;     // RCC AHB2 peripheral reset register
    __vo uint32_t AHB3RSTR;     // RCC AHB3 peripheral reset register
    uint32_t RESERVED0;         // Reserved, 0x1C
    __vo uint32_t APB1RSTR;     // RCC APB1 peripheral reset register
    __vo uint32_t APB2RSTR;     // RCC APB2 peripheral reset register
    uint32_t RESERVED1[2];      // Reserved, 0x28-0x2C
    __vo uint32_t AHB1ENR;      // RCC AHB1 peripheral clock enable register
    __vo uint32_t AHB2ENR;      // RCC AHB2 peripheral clock enable register
    __vo uint32_t AHB3ENR;      // RCC AHB3 peripheral clock enable register
    uint32_t RESERVED2;         // Reserved, 0x3C
    __vo uint32_t APB1ENR;      // RCC APB1 peripheral clock enable register
    __vo uint32_t APB2ENR;      // RCC APB2 peripheral clock enable register
    uint32_t RESERVED3[2];      // Reserved, 0x48-0x4C
    __vo uint32_t AHB1LPENR;    // RCC AHB1 peripheral clock enable in low
                                // power mode register
    __vo uint32_t AHB2LPENR;    // RCC AHB2 peripheral clock enable in low
                                // power mode register
    __vo uint32_t AHB3LPENR;    // RCC AHB3 peripheral clock enable in low
                                // power mode register
    uint32_t RESERVED4;         // Reserved, 0x5C
    __vo uint32_t APB1LPENR;    // RCC APB1 peripheral clock enable in low
                                // power mode register
    __vo uint32_t APB2LPENR;    // RCC APB2 peripheral clock enable in low
                                // power mode register
    uint32_t RESERVED5[2];      // Reserved, 0x68-0x6C
    __vo uint32_t BDCR;         // RCC Backup domain control register
    __vo uint32_t CSR;          // RCC clock control & status register
    uint32_t RESERVED6[2];      // Reserved, 0x78-0x7C
    __vo uint32_t SSCGR;        // RCC spread spectrum clock generation register
    __vo uint32_t PLLI2SCFGR;   // RCC PLLI2S configuration register
} RCC_RegDef_t;                 // Reset and clock control register

//** GPIO **//
typedef struct {
    __vo uint32_t MODER;    // GPIO port mode register
    __vo uint32_t OTYPER;   // GPIO port output type register
    __vo uint32_t OSPEEDR;  // GPIO port output speed register
    __vo uint32_t PUPDR;    // GPIO port pull-up/pull-down register
    __vo uint32_t IDR;      // GPIO port input data register
    __vo uint32_t ODR;      // GPIO port output data register
    __vo uint32_t BSRR;     // GPIO port bit set/reset register
    __vo uint32_t LCKR;     // GPIO port configuration lock register
    __vo uint32_t AFR[2];   // GPIO alternate function low register
                            // GPIO alternate function high register
} GPIO_RegDef_t;

//** SPI **//

typedef struct {
    __vo uint32_t CPHA : 1;      // Clock phase
    __vo uint32_t CPOL : 1;      // Clock polarity
    __vo uint32_t MSTR : 1;      // Master selection
    __vo uint32_t BR : 3;        // Baud rate control
    __vo uint32_t SPE : 1;       // SPI enable
    __vo uint32_t LSBFIRST : 1;  // Frame format
    __vo uint32_t SSI : 1;       // Internal slave select
    __vo uint32_t SSM : 1;       // Software slave management
    __vo uint32_t RXONLY : 1;    // Receive only
    __vo uint32_t DFF : 1;       // Data frame format
    __vo uint32_t CRCNEXT : 1;   // CRC transfer next
    __vo uint32_t CRCEN : 1;     // Hardware CRC calculation enable
    __vo uint32_t BIDIOE : 1;    // Output enable in bidirectional mode
    __vo uint32_t BIDIMODE : 1;  // Bidirectional data mode enable
    __vo uint32_t RESERVED : 16;
} SPI_CR1_bits_t;  // 28.5.1  SPI control register 1 (SPIx_CR1)

typedef struct {
    __vo uint32_t RXDMAEN : 1;  // Rx buffer DMA enable
    __vo uint32_t TXDMAEN : 1;  // Tx buffer DMA enable
    __vo uint32_t SSOE : 1;     // SS output enable
    __vo uint32_t REASERVED1 : 1;
    __vo uint32_t FRF : 1;     // Frame format
    __vo uint32_t ERRIE : 1;   // Error interrupt enable
    __vo uint32_t RXNEIE : 1;  // RX buffer not empty interrupt enable
    __vo uint32_t TXEIE : 1;   // Tx buffer empty interrupt enable
    __vo uint32_t REASERVED2 : 24;
} SPI_CR2_bits_t;  // 28.5.2  SPI control register 2 (SPIx_CR2)

// 28.5.3  SPI status register (SPIx_SR)
typedef struct {
    __vo uint32_t RXNE : 1;    // Receive buffer not empty
    __vo uint32_t TXE : 1;     // Transmit buffer empty
    __vo uint32_t CHSIDE : 1;  // Channel side
    __vo uint32_t UDR : 1;     // Underrun flag
    __vo uint32_t CRCERR : 1;  // CRC error flag
    __vo uint32_t MODF : 1;    // Mode fault
    __vo uint32_t OVR : 1;     // Overrun flag
    __vo uint32_t BSY : 1;     // Busy flag
    __vo uint32_t FRE : 1;     // Frame format error
    __vo uint32_t RESERVED1 : 23;
} SPI_SR_bits_t;

// 28.5.5  SPI CRC polynomial register (SPIx_CRCPR)
typedef struct {
    __vo uint32_t CRCPOLY : 16;  // CRC polynomial register
    __vo uint32_t RESERVED : 16;
} SPI_CRCPR_bits_t;

// 28.5.6  SPI RX CRC register (SPIx_RXCRCR)
typedef struct {
    __vo uint32_t RxCRC : 16;  // Rx CRC register
    __vo uint32_t RESERVED : 16;
} SPI_RXCRCR_bits_t;

// 28.5.7  SPI TX CRC register (SPIx_TXCRCR)
typedef struct {
    __vo uint32_t TxCRC : 16;  // Tx CRC register
    __vo uint32_t RESERVED : 16;
} SPI_TXCRCR_bits_t;

// 28.5.8  SPI_I2S configuration register (SPIx_I2SCFGR)
typedef struct {
    __vo uint32_t CHLEN : 1;       // Channel length (number of bits per audio channel)
    __vo uint32_t DATLEN : 2;      // Data length to be transferred
    __vo uint32_t CKPOL : 1;       // Steady state clock polarity
    __vo uint32_t I2SSTD : 2;      // I2S standard selection
    __vo uint32_t RESERVED : 1;    // Reserved
    __vo uint32_t PCMSYNC : 1;     // PCM frame synchronization
    __vo uint32_t I2SCFG : 2;      // I2S configuration mode
    __vo uint32_t I2SE : 1;        // I2S Enable
    __vo uint32_t I2SMOD : 1;      // I2S mode selection
    __vo uint32_t RESERVED2 : 20;  // Reserved
} SPI_I2SCFGR_bits_t;

// 28.5.9  SPI_I2S prescaler register (SPIx_I2SPR)
typedef struct {
    __vo uint32_t I2SDIV : 8;  // I2S Linear prescaler
    __vo uint32_t ODD : 1;     // Odd factor for the prescaler
    __vo uint32_t MCKOE : 1;   // Master clock output enable
    __vo uint32_t RESERVED1 : 22;
} SPI_I2SPR_bits_t;

typedef struct {
    __vo SPI_CR1_bits_t CR1;          // SPI control register 1
    __vo SPI_CR2_bits_t CR2;          // SPI control register 2
    __vo SPI_SR_bits_t SR;            // SPI status register
    __vo uint32_t DR;                 // SPI data register
    __vo SPI_CRCPR_bits_t CRCPR;      // SPI CRC polynomial register
    __vo SPI_RXCRCR_bits_t RXCRCR;    // SPI RX CRC register
    __vo SPI_TXCRCR_bits_t TXCRCR;    // SPI TX CRC register
    __vo SPI_I2SCFGR_bits_t I2SCFGR;  // SPI_I2S configuration register
    __vo SPI_I2SPR_bits_t I2SPR;      // SPI_I2S prescaler register
} SPI_RegDef_t;

//** END OF SPI **//

//** START OF I2C **//
typedef struct {
    __vo uint32_t PE : 1;     // Peripheral enable
    __vo uint32_t SMBUS : 1;  // SMBus mode
    __vo uint32_t RESERVED1 : 1;
    __vo uint32_t SMBTYPE : 1;    // SMBus type
    __vo uint32_t ENARP : 1;      // ARP enable
    __vo uint32_t ENPEC : 1;      // PEC enable
    __vo uint32_t ENGC : 1;       // General call enable
    __vo uint32_t NOSTRETCH : 1;  // Clock stretching disable
    __vo uint32_t START : 1;      // Start generation
    __vo uint32_t STOP : 1;       // Stop generation
    __vo uint32_t ACK : 1;        // Acknowledge enable
    __vo uint32_t POS : 1;        // Acknowledge/PEC Position (for data reception)
    __vo uint32_t PEC : 1;        // Packet error checking
    __vo uint32_t ALERT : 1;      // SMBus alert
    __vo uint32_t RESERVED2 : 1;
    __vo uint32_t SWRST : 1;  // Software reset
    __vo uint32_t RESERVED3 : 16;
} I2C_CR1_bits_t;

typedef struct {
    __vo uint32_t FREQ : 6;  // Peripheral clock frequency
    __vo uint32_t RESERVED1 : 2;
    __vo uint32_t ITERREN : 1;  // Error interrupt enable
    __vo uint32_t ITEVTEN : 1;  // Event interrupt enable
    __vo uint32_t ITBUFEN : 1;  // Buffer interrupt enable
    __vo uint32_t DMAEN : 1;    // DMA requests enable
    __vo uint32_t LAST : 1;     // DMA last transfer
    __vo uint32_t RESERVED2 : 19;
} I2C_CR2_bits_t;

typedef struct {
    __vo uint32_t ADD : 10;  // I2C address if 7-bit addressing mode is selected then only ADD[7:1] bits are used
    __vo uint32_t RESERVED1 : 5;
    __vo uint32_t ADDMODE : 1;  // Addressing mode (slave mode)
    __vo uint32_t RESERVED2 : 16;
} I2C_OAR1_bits_t;

typedef struct {
    __vo uint32_t ENDUAL : 1;  // Interface address
    __vo uint32_t ADD2 : 7;    // Dual addressing mode enable
    __vo uint32_t RESERVED1 : 24;
} I2C_OAR2_bits_t;

typedef struct {
    __vo uint32_t DR : 8;  // 8-bit data register
    __vo uint32_t RESERVED1 : 24;
} I2C_DR_bits_t;

typedef struct {
    __vo uint32_t SB : 1;     // Start bit (Master mode)
    __vo uint32_t ADDR : 1;   // Address sent (master mode)/matched (slave mode)
    __vo uint32_t BTF : 1;    // Byte transfer finished
    __vo uint32_t ADD10 : 1;  // 10-bit header sent (Master mode)
    __vo uint32_t STOPF : 1;  // Stop detection (Slave mode)
    __vo uint32_t RESERVED1 : 1;
    __vo uint32_t RxNE : 1;    // Data register not empty (receivers)
    __vo uint32_t TxE : 1;     // Data register empty (transmitters)
    __vo uint32_t BERR : 1;    // Bus error
    __vo uint32_t ARLO : 1;    // Arbitration lost (master mode)
    __vo uint32_t AF : 1;      // Acknowledge failure
    __vo uint32_t OVR : 1;     // Overrun/Underrun
    __vo uint32_t PECERR : 1;  // PEC Error in reception
    __vo uint32_t RESERVED2 : 1;
    __vo uint32_t TIMEOUT : 1;   // Timeout or Tlow error
    __vo uint32_t SMBALERT : 1;  // SMBus alert
    __vo uint32_t RESERVED3 : 16;
} I2C_SR1_bits_t;

typedef struct {
    __vo uint32_t MSL : 1;   // Master/Slave
    __vo uint32_t BUSY : 1;  // Bus busy
    __vo uint32_t TRA : 1;   // Transmitter/receiver
    __vo uint32_t RESERVED1 : 1;
    __vo uint32_t GENCALL : 1;     // General call address (Slave mode)
    __vo uint32_t SMBDEFAULT : 1;  // SMBus device default address (Slave mode)
    __vo uint32_t SMBHOST : 1;     // SMBus host header (Slave mode)
    __vo uint32_t DUALF : 1;       // Dual flag (Slave mode)
    __vo uint32_t PEC : 8;         // Packet error checking register
    __vo uint32_t RESERVED2 : 16;
} I2C_SR2_bits_t;

typedef struct {
    __vo uint32_t CCR : 12;  // Clock control register in Fast/Standard mode (Master mode)
    __vo uint32_t RESERVED1 : 2;
    __vo uint32_t DUTY : 1;  // Fast mode duty cycle
    __vo uint32_t FS : 1;    // I2C master mode selection
    __vo uint32_t RESERVED2 : 16;
} I2C_CCR_bits_t;

typedef struct {
    __vo uint32_t TRISE : 6;  // Maximum rise time in Fast/Standard mode (Master mode)
    __vo uint32_t RESERVED1 : 26;
} I2C_TRISE_bits_t;

typedef struct {
    __vo uint32_t DNF : 4;    // Digital noise filter
    __vo uint32_t ANOFF : 1;  // Analog noise filter OFF
    __vo uint32_t RESERVED1 : 27;
} I2C_FLTR_bits_t;

typedef struct {
    __vo I2C_CR1_bits_t I2C_CR1;      // I2C Control register 1
    __vo I2C_CR2_bits_t I2C_CR2;      // I2C Control register 2
    __vo I2C_OAR1_bits_t I2C_OAR1;    // I2C Own address register 1
    __vo I2C_OAR2_bits_t I2C_OAR2;    // I2C Own address register 2
    __vo I2C_DR_bits_t I2C_DR;        // I2C Data register
    __vo I2C_SR1_bits_t I2C_SR1;      // I2C Status register 1
    __vo I2C_SR2_bits_t I2C_SR2;      // I2C Status register 2
    __vo I2C_CCR_bits_t I2C_CCR;      // I2C Clock control register
    __vo I2C_TRISE_bits_t I2C_TRISE;  // I2C TRISE register
    __vo I2C_FLTR_bits_t I2C_FLTR;    // I2C FLTR register
} I2C_RegDef_t;

//* USART *//
typedef struct {
    __vo uint32_t PE : 1;    // Parity error
    __vo uint32_t FE : 1;    // Framing error
    __vo uint32_t NF : 1;    // Noise detected flag
    __vo uint32_t ORE : 1;   // Overrun error
    __vo uint32_t IDLE : 1;  // IDLE line detected
    __vo uint32_t RXNE : 1;  // Read data register not empty
    __vo uint32_t TC : 1;    // Transmission complete
    __vo uint32_t TXE : 1;   // Transmit data register empty
    __vo uint32_t LBD : 1;   // LIN break detection flag
    __vo uint32_t CTS : 1;   // CTS flag
    __vo uint32_t RESERVED1 : 22;
} USART_SR_bits_t;

typedef struct
{
    __vo uint32_t DIV_Fracton : 4;    // Fraction of USARTDIV
    __vo uint32_t DIV_Mantissa : 12;  // Mantissa of USARTDIV
    __vo uint32_t RESERVED1 : 16;
} USART_BRR_bits_t;

typedef struct
{
    __vo uint32_t SBK : 1;     // Send break
    __vo uint32_t RWU : 1;     // Receiver wakeup
    __vo uint32_t RE : 1;      // Receiver enable
    __vo uint32_t TE : 1;      // Transmitter enable
    __vo uint32_t IDLEIE : 1;  // IDLE interrupt enable
    __vo uint32_t RXNEIE : 1;  // RXNE interrupt enable
    __vo uint32_t TCIE : 1;    // Transmission complete interrupt enable
    __vo uint32_t TXEIE : 1;   // TXE interrupt enable
    __vo uint32_t PEIE : 1;    // PE interrupt enable
    __vo uint32_t PS : 1;      // Parity selection
    __vo uint32_t PCE : 1;     // Parity control enable
    __vo uint32_t WAKE : 1;    // Wakeup method
    __vo uint32_t M : 1;       // Word length
    __vo uint32_t UE : 1;      // USART enable
    __vo uint32_t RESERVED1 : 1;
    __vo uint32_t OVER8 : 1;  // Oversampling mode
    __vo uint32_t RESERVED2 : 16;
} USART_CR1_bits_t;

typedef struct {
    __vo uint32_t ADD : 4;  // Address of the USART node
    __vo uint32_t RESERVED1 : 1;
    __vo uint32_t LBDL : 1;   // LIN break detection length
    __vo uint32_t LBDIE : 1;  // LIN break detection interrupt enable
    __vo uint32_t RESERVED2 : 1;
    __vo uint32_t LBCL : 1;   // Last bit clock pulse
    __vo uint32_t CPHA : 1;   // Clock phase
    __vo uint32_t CPOL : 1;   // Clock polarity
    __vo uint32_t CLKEN : 1;  // Clock enable
    __vo uint32_t STOP : 2;   // STOP bits
    __vo uint32_t LINEN : 1;  // LIN mode enable
    __vo uint32_t RESERVED3 : 17;
} USART_CR2_bits_t;

typedef struct {
    __vo uint32_t EIE : 1;     // Error interrupt enable
    __vo uint32_t IREN : 1;    // IrDA mode enable
    __vo uint32_t IRLP : 1;    // IrDA low-power
    __vo uint32_t HDSEL : 1;   // Half-duplex selection
    __vo uint32_t NACK : 1;    // Smartcard NACK enable
    __vo uint32_t SCEN : 1;    // Smartcard mode enable
    __vo uint32_t DMAR : 1;    // DMA enable receiver
    __vo uint32_t DMAT : 1;    // DMA enable transmitter
    __vo uint32_t RTSE : 1;    // RTS enable
    __vo uint32_t CTSE : 1;    // CTS enable
    __vo uint32_t CTSIE : 1;   // CTS interrupt enable
    __vo uint32_t ONEBIT : 1;  // One sample bit method enable
    __vo uint32_t RESERVED1 : 20;
} USART_CR3_bits_t;

typedef struct {
    __vo uint32_t PSC : 8;  // Prescaler value
    __vo uint32_t GT : 8;   // Guard time value
    __vo uint32_t RESERVED1 : 16;
} USART_GTPR_bits_t;

typedef struct {
    __vo uint32_t DR : 9;  // Data value
    __vo uint32_t RESERVED1 : 23;

} USART_DR_bits_t;
typedef struct {
    __vo USART_SR_bits_t SR;      // USART Status register
    __vo USART_DR_bits_t DR;      // USART Data register
    __vo USART_BRR_bits_t BRR;    // USART Baud rate register
    __vo USART_CR1_bits_t CR1;    // USART Control register 1
    __vo USART_CR2_bits_t CR2;    // USART Control register 2
    __vo USART_CR3_bits_t CR3;    // USART Control register 3
    __vo USART_GTPR_bits_t GTPR;  // USART Guard time and prescaler register
} USART_RegDef_t;
#endif /* TYPE_DEF_H_ */
