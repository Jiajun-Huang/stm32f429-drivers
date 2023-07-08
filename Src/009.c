
/*
 * This application receives and prints the user message received from the
 * Arduino peripheral in SPI interrupt mode User sends the message through
 * Arduino IDE's serial monitor tool Monitor the message received in the SWV itm
 * data console
 */
/*
 * Note : Follow the instructions to test this code
 * 1. Download this code on to STM32 board , acts as Master
 * 2. Download Slave code (003SPISlaveUartReadOverSPI.ino) on to Arduino board
 * (Slave)
 * 3. Reset both the boards
 * 4. Enable SWV ITM data console to see the message
 * 5. Open Arduino IDE serial monitor tool
 * 6. Type anything and send the message (Make sure that in the serial monitor
 * tool line ending set to carriage return)
 */
#include "stdio.h"
#include "stm32f429.h"
#include <string.h>

SPI_Handle_t SPI1handle;

#define MAX_LEN 500

char RcvBuff[MAX_LEN];

volatile char ReadByte;

volatile uint8_t rcvStop = 0;

/*This flag will be set in the interrupt handler of the Arduino interrupt GPIO
 */
volatile uint8_t dataAvailable = 0;

void delay(void) {
    for (uint32_t i = 0; i < 500000 / 2; i++)
        ;
}

/*
 * PB14 --> SPI1_MISO
 * PB15 --> SPI1_MOSI
 * PB13 -> SPI1_SCLK
 * PB12 --> SPI1_NSS
 * ALT function mode : 5
 */

void SPI1_GPIOInits(void) {
    GPIO_Handle_t SPIPins;

    SPIPins.pGPIOx = GPIOA;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    // SCLK
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
    GPIO_Init(&SPIPins);

    // MOSI
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
    GPIO_Init(&SPIPins);

    // MISO
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
    GPIO_Init(&SPIPins);

    // NSS
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_4;
    GPIO_Init(&SPIPins);
}

void SPI1_Inits(void) {
    SPI1handle.pSPIx = SPI1;
    SPI1handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI1handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI1handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;
    SPI1handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    SPI1handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI1handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI1handle.SPIConfig.SPI_SSM =
        SPI_SSM_DI; // Hardware slave management enabled for NSS pin

    SPI_Init(&SPI1handle);
}

/*This function configures the gpio pin over which SPI peripheral issues data
 * available interrupt */
void Slave_GPIO_InterruptPinInit(void) {
    GPIO_Handle_t spiIntPin;
    memset(&spiIntPin, 0, sizeof(spiIntPin));

    // this is led gpio configuration
    spiIntPin.pGPIOx = GPIOD;
    spiIntPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
    spiIntPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
    spiIntPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    spiIntPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    GPIO_Init(&spiIntPin);

    IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
    IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
}

int main(void) {

    uint8_t dummy = 0xff;
    printf("asdfasdf");
    Slave_GPIO_InterruptPinInit();

    // this function is used to initialize the GPIO pins to behave as SPI1 pins
    SPI1_GPIOInits();

    // This function is used to initialize the SPI1 peripheral parameters
    SPI1_Inits();

    /*
     * making SSOE 1 does NSS output enable.
     * The NSS pin is automatically managed by the hardware.
     * i.e when SPE=1 , NSS will be pulled to low
     * and NSS pin will be high when SPE=0
     */
    SPI_SSOESwich(SPI1, ENABLE);

    IRQInterruptConfig(IRQ_NO_SPI1, ENABLE);

    while (1) {

        rcvStop = 0;

        while (!dataAvailable)
            ; // wait till data available interrupt from transmitter
              // device(slave)

        IRQInterruptConfig(IRQ_NO_EXTI9_5, DISABLE);

        // enable the SPI1 peripheral
        SPI_PeripheralSwich(SPI1, ENABLE);

        while (!rcvStop) {
            /* fetch the data from the SPI peripheral byte by byte in interrupt
             * mode */
            while (SPI_SendDataIT(&SPI1handle, &dummy, 1) == SPI_BUSY_IN_TX)
                ;
            while (SPI_ReceiveDataIT(&SPI1handle, &(ReadByte), 1) ==
                   SPI_BUSY_IN_RX)
                ;
        }

        // confirm SPI is not busy
        while (SPI1->SR.BSY)
            ;

        // Disable the SPI1 peripheral
        SPI_PeripheralSwich(SPI1, DISABLE);

        printf("Rcvd data = %s\n", RcvBuff);

        dataAvailable = 0;

        IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
    }

    return 0;
}

/* Runs when a data byte is received from the peripheral over SPI*/

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv) {
    static uint32_t i = 0;
    /* In the RX complete event , copy data in to rcv buffer . '\0' indicates
     * end of message(rcvStop = 1) */
    if (AppEv == SPI_EVENT_RX_CMPLT) {
        RcvBuff[i++] = ReadByte;
        if (ReadByte == '\0' || (i == MAX_LEN)) {
            rcvStop = 1;
            RcvBuff[i - 1] = '\0';
            i = 0;
        }
    }
}

void SPI1_IRQHandler(void) {
    SPI_CallbackFunc callback = &SPI_ApplicationEventCallback;
    SPI_IRQHandling(&SPI1handle, callback);
}

/* Slave data available interrupt handler */
void EXTI9_5_IRQHandler(void) {
    GPIO_IRQHandling(GPIO_PIN_6);
    dataAvailable = 1;
}
