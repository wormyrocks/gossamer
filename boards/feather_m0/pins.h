#pragma once

#include "hal_gpio.h"
#include "spi.h"
#include "i2c.h"
#include "uart.h"

// Analog pins
HAL_GPIO_PIN(AREF, A, 3)
HAL_GPIO_PIN(A0,   A, 2)
HAL_GPIO_PIN(A1,   B, 8)
HAL_GPIO_PIN(A2,   B, 9)
HAL_GPIO_PIN(A3,   A, 4)
HAL_GPIO_PIN(A4,   A, 5)
HAL_GPIO_PIN(A5,   B, 2)

// Digital pins
HAL_GPIO_PIN(D0,   A, 11) // also RX below
HAL_GPIO_PIN(D1,   A, 10) // also TX below
HAL_GPIO_PIN(D4,   A,  8) // SDCS on Adalogger, RST on radio
HAL_GPIO_PIN(D5,   A, 15)
HAL_GPIO_PIN(D6,   A, 20)
HAL_GPIO_PIN(D7,   A, 21) // Card Detect on Adalogger, IRQ on radio
HAL_GPIO_PIN(D8,   A,  6) // Green LED on Adalogger, CS on radio
HAL_GPIO_PIN(D9,   A,  7)
HAL_GPIO_PIN(D10,  A, 18)
HAL_GPIO_PIN(D11,  A, 16)
HAL_GPIO_PIN(D12,  A, 19)
HAL_GPIO_PIN(D13,  A, 17) // also LED
HAL_GPIO_PIN(LED,  A, 17)

// SPI
HAL_GPIO_PIN(SCK,  B, 11)
HAL_GPIO_PIN(MOSI, B, 10)
HAL_GPIO_PIN(MISO, A, 12)
#define SPI_SERCOM 4
#define SPI_SERCOM_DOPO SPI_DOPO_2_SCK_3
#define SPI_SERCOM_DIPO SPI_DIPO_0

// I2C
HAL_GPIO_PIN(SDA,  A, 22)
HAL_GPIO_PIN(SCL,  A, 23)
#define I2C_SERCOM 5

// UART
HAL_GPIO_PIN(TX,   A, 10)
HAL_GPIO_PIN(RX,   A, 11)
#define UART_SERCOM 0
#define UART_SERCOM_TXPO UART_TXPO_2
#define UART_SERCOM_RXPO UART_RXPO_3

// Express boards
HAL_GPIO_PIN(FLASH_SCK,  A,  9)
HAL_GPIO_PIN(FLASH_MOSI, A,  8)
HAL_GPIO_PIN(FLASH_MISO, A, 14)
HAL_GPIO_PIN(FLASH_CS,   A, 13)
HAL_GPIO_PIN(NEOPIXEL,   A,  6)
