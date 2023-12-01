#include <string.h>

#include "stdio.h"
#include "stm32f429.h"

I2C_Handle_t I2C1Handle = {
    .pI2Cx = I2C1,
    .I2C_Config.I2C_AckControl = ENABLE,
    .I2C_Config.I2C_DeviceAddress = 0x69,
    .I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2,
    .I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM,
    .I2C_Config.I2C_ADDMode = 0};

void delay(void) {
    for (uint32_t i = 0; i < 500000 / 2; i++)
        ;
}

void I2C1_GPIO_inits(void) {
    GPIO_Handle_t I2CPins = {
        .pGPIOx = GPIOB,
        .GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN,
        .GPIO_PinConfig.GPIO_PinAltFunMode = 4,
        .GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD,
        .GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD,
        .GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST,
        .GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6};

    // SCL
    GPIO_Init(&I2CPins);

    // SDA
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
    GPIO_Init(&I2CPins);
}

void I2C1_inits(void) {
    I2C_Init(&I2C1Handle);
    I2C_PeriferalControl(I2C1, ENABLE);
};

void GPIO_ButtonInit(void) {
    GPIO_Handle_t I2CPins = {
        .pGPIOx = GPIOA,
        .GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN,
        .GPIO_PinConfig.GPIO_PinAltFunMode = 0,
        .GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP,
        .GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD,
        .GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST,
        .GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0};

    GPIO_Init(&I2CPins);
    // GPIO_IRQHandling(GPIO_PIN_0);
    // IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);
    // IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI15);

    // intterupt config
}

int main(void) {
    char user_data[] = "Hello world";
    GPIO_ButtonInit();
    I2C1_GPIO_inits();
    I2C1_inits();
    while (1) {
        delay();
        I2C_MasterSendData(&I2C1Handle, user_data, strlen(user_data), 0x68);
    }
}
