/*
    Cribbed from the Castor & Pollux Gemini firmware:
    https://github.com/wntrblm/Castor_and_Pollux/
    Copyright (c) 2021 Alethea Katherine Flowers.
    Published under the standard MIT License.
    Full text available at: https://opensource.org/licenses/MIT
*/

#include "pins.h"
#include "sam.h"
#include "system.h"
#include "sercom.h"
#include "spi.h"

#if defined(SPI_SERCOM)

void spi_init(spi_mode_t mode, uint32_t baud) {
    spi_init_custom(SPI_SERCOM, SPI_SERCOM_DOPO, SPI_SERCOM_DIPO, mode, baud);
}

void spi_enable(void) {
    spi_enable_custom(SPI_SERCOM);
}

uint8_t spi_transfer(uint8_t data) {
    return spi_transfer_custom(SPI_SERCOM, data);
}

void spi_disable(void) {
    spi_disable_custom(SPI_SERCOM);
}

#endif

void spi_init_custom(uint8_t sercom, spi_dopo_t dopo, spi_dipo_t dipo, spi_mode_t mode, uint32_t baud) {
    Sercom* SERCOM = SERCOM_Peripherals[sercom].sercom;

    _sercom_clock_setup(sercom);

    /* Reset the SERCOM. */
    SERCOM->SPI.CTRLA.bit.ENABLE = 0;
    while (SERCOM->SPI.SYNCBUSY.bit.ENABLE) {};
    SERCOM->SPI.CTRLA.bit.SWRST = 1;
    while (SERCOM->SPI.SYNCBUSY.bit.SWRST || SERCOM->SPI.SYNCBUSY.bit.ENABLE) {};

    /* Setup SPI controller. */
    if (dipo == SPI_DIPO_NONE) {
        SERCOM->SPI.CTRLB.bit.RXEN = 0;
        dipo = 0;
    } else {
        SERCOM->SPI.CTRLB.bit.RXEN = 1;
    }
    SERCOM->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_MODE(3) |
                            SERCOM_SPI_CTRLA_DOPO(dopo) |
                            SERCOM_SPI_CTRLA_DIPO(dipo);

    switch (mode) {
        case SPI_MODE_0:
            SERCOM->SPI.CTRLA.bit.CPOL = 0;
            SERCOM->SPI.CTRLA.bit.CPHA = 0;
            break;
        case SPI_MODE_1:
            SERCOM->SPI.CTRLA.bit.CPOL = 0;
            SERCOM->SPI.CTRLA.bit.CPHA = 1;
            break;
        case SPI_MODE_2:
            SERCOM->SPI.CTRLA.bit.CPOL = 1;
            SERCOM->SPI.CTRLA.bit.CPHA = 0;
            break;
        case SPI_MODE_3:
            SERCOM->SPI.CTRLA.bit.CPOL = 1;
            SERCOM->SPI.CTRLA.bit.CPHA = 1;
            break;
    }

    /* Set baud */
    uint32_t baudrate = baud;
    if (baudrate == get_cpu_frequency()) {
        SERCOM->SPI.BAUD.reg = 0x1;
    } else {
        SERCOM->SPI.BAUD.reg = get_cpu_frequency() / (2 * baudrate) - 1;
    }
}

void spi_enable_custom(uint8_t sercom) {
    _sercom_enable(sercom);
}

uint8_t spi_transfer_custom(uint8_t sercom, uint8_t data) {
    while (!SERCOM_Peripherals[sercom].sercom->SPI.INTFLAG.bit.DRE) {}

    SERCOM_Peripherals[sercom].sercom->SPI.DATA.bit.DATA = data;

    while (!SERCOM_Peripherals[sercom].sercom->SPI.INTFLAG.bit.RXC) {}

    return SERCOM_Peripherals[sercom].sercom->SPI.DATA.bit.DATA;
}

void spi_disable_custom(uint8_t sercom) {
    _sercom_disable(sercom);
}
