/** 
 * @file i2c.h
 * @brief I2C Peripheral
 */
/*
    Cribbed from the Castor & Pollux Gemini firmware:
    https://github.com/wntrblm/Castor_and_Pollux/
    Copyright (c) 2021 Alethea Katherine Flowers.
    Published under the standard MIT License.
    Full text available at: https://opensource.org/licenses/MIT
*/

#pragma once

/* Routines for interacting with I2C devices. */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef enum {
    I2C_RESULT_SUCCESS = 0,
    I2C_RESULT_ERR_ADDR_NACK = -1,
    I2C_RESULT_ERR_BUSSTATE = -2,
    I2C_RESULT_ERR_BUSERR = -3,
    I2C_RESULT_ERR_DATA_NACK = -4,
} I2CResult;

/**
 * @brief Initializes the I2C peripheral for a board with defined SDA/SCL pins.
 *        Defaults to a bus speed of 400kHz.
 * @details This function will only work if a board has pins named SDA and SCL,
 *          and relevant definitions for I2C_SERCOM. If you don't have these
 *          definitions, the build will fail at  link time.
 */
void i2c_init(void);

/**
 * @brief Enables the I2C peripheral for a board with defined SDA/SCL pins.
 */
void i2c_enable(void);

/**
 * @brief Writes data to an I2C device at the provided address.
 * @param address The I2C address of the device.
 * @param data The data to write.
 * @param len The length of the data to write.
 */
I2CResult i2c_write(uint8_t address, uint8_t* data, size_t len);

/**
 * @brief Disables the I2C peripheral for a board with defined SDA/SCL pins.
 */
void i2c_disable(void);

/**
 * @brief Initializes an I2C peripheral on the given SERCOM.
 * @param sercom The SERCOM you wish to use, as numbered in the data sheet.
 * @param baud The baud rate you wish to use.
 * @note You are responsible for setting the appropriate pin mux for the SDA and SCL pins.
 *       This function does not know which pins you intend to use, or whether they are on
 *       the SERCOM or SERCOM_ALT pin mux.
 */
void i2c_init_custom(uint8_t sercom, uint32_t baud);

/**
 * @brief Enables the I2C peripheral on the given SERCOM.
 * @param sercom The SERCOM of the I2C peripheral, as numbered in the data sheet.
 */
void i2c_enable_custom(uint8_t sercom);

/**
 * @brief Writes data to an I2C device on the given SERCOM at the provided address.
 * @param sercom The SERCOM of the I2C peripheral, as numbered in the data sheet.
 * @param address The I2C address of the device.
 * @param data The data to write.
 * @param len The length of the data to write.
 */
I2CResult i2c_write_custom(uint8_t sercom, uint8_t address, uint8_t* data, size_t len);

/**
 * @brief Disables the I2C peripheral on the given SERCOM.
 * @param sercom The SERCOM of the I2C peripheral, as numbered in the data sheet.
 */
void i2c_disable_custom(uint8_t sercom);
