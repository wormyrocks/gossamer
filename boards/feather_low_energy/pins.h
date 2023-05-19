#pragma once

#include "hal_gpio.h"


// Analog pins
HAL_GPIO_PIN(AREF, A, 4)
HAL_GPIO_PIN(A0,   A, 2)
HAL_GPIO_PIN(A1,   A, 5)
HAL_GPIO_PIN(A2,   B, 8)
HAL_GPIO_PIN(A3,   B, 9)
HAL_GPIO_PIN(A4,   A, 6)
HAL_GPIO_PIN(A5,   A, 7)

// Digital pins
HAL_GPIO_PIN(D5,   A, 16)
HAL_GPIO_PIN(D6,   A, 17)
HAL_GPIO_PIN(D9,   A, 18)
HAL_GPIO_PIN(D10,  A, 19)
HAL_GPIO_PIN(D11,  A,  8)
HAL_GPIO_PIN(D12,  A,  9)
HAL_GPIO_PIN(D13,  A, 10)

// LED
HAL_GPIO_PIN(LED,  A, 10)

// SPI
HAL_GPIO_PIN(SCK,  B, 11)
HAL_GPIO_PIN(MOSI, B, 10)
HAL_GPIO_PIN(MISO, A, 12)

// I2C
HAL_GPIO_PIN(SDA,  A, 22)
HAL_GPIO_PIN(SCL,  A, 23)
#define I2C_SERCOM SERCOM5
#define I2C_SERCOM_APBCMASK MCLK_APBDMASK_SERCOM5
#define I2C_GCLK_CLKCTRL_ID SERCOM5_GCLK_ID_CORE

// UART
HAL_GPIO_PIN(TX,   B, 2)
HAL_GPIO_PIN(RX,   A, 21)
