#pragma once
#define F_CPU 48000000

#include "hal_gpio.h"
#include "i2c.h"

/*
// Analog pins
HAL_GPIO_PIN(AREF, A, 3)
HAL_GPIO_PIN(A0,   A, 2)
HAL_GPIO_PIN(A1,   A, 4)
HAL_GPIO_PIN(A2,   A, 6)
HAL_GPIO_PIN(A3,   A, 7)

// Digital pins
HAL_GPIO_PIN(LED, A,  5)

// SPI
HAL_GPIO_PIN(SCK,  A, 16)
HAL_GPIO_PIN(MOSI, A, 22)
HAL_GPIO_PIN(MISO, A, 23)
*/
// Analog
#define HAL_GPIO_A7_ADC_CHANNEL 5
HAL_GPIO_PIN(VOL_POT, A,  7)

// Digital
HAL_GPIO_PIN(LED, A,  16)
HAL_GPIO_PIN(ESS_SHDN, A,  6)
HAL_GPIO_PIN(HPA_SHDN, A,  3)

// I2C
HAL_GPIO_PIN(SDA,  A, 22)
HAL_GPIO_PIN(SCL,  A, 23)

#define I2C_SERCOM 1

// UART
// pad 2, mux d
HAL_GPIO_PIN(UART_TX, A, 8)
// pad 3, mux d
HAL_GPIO_PIN(UART_RX, A, 9)
