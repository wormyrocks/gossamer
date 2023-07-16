#include "stdint.h"
#include "string.h"
#include "es901xk.h"
#include "../peripherals/i2c.h"

// void ess_init(uint8_t addr) {
//     return;
// }

uint8_t i2c_buf[10] = {0};
I2CResult i2cwrite(uint8_t target, uint8_t *data, size_t len)
{
    i2c_buf[0] = target;
    memcpy(i2c_buf+1, data, len);
    return i2c_write(ESS_ADDR2, i2c_buf, len+1);
}

// helper function
uint8_t reg_swizzle(uint8_t initval, uint8_t newval, uint8_t shift, uint8_t mask)
{
    initval &= ~(mask << shift);
    return (initval | (newval << shift));
}

// datasheet p12
void ess_set_input_type(ess_input_type input_type)
{
    uint8_t newval = reg_swizzle(0x8c, (uint8_t)ESS_INPUT_SELECT_NOAUTO, 2, 0b11);
    newval = reg_swizzle(newval, (uint8_t)input_type, 0, 0b11);
    i2cwrite(ESS_INPUT_CONFIG, &newval, 1);
}

// datasheet p14
void ess_toggle_mute (uint8_t do_mute)
{
    uint8_t newval = reg_swizzle(0x80, (do_mute ? 0b11 : 0), 0, 0b11);
    i2cwrite(ESS_GEN_SETTINGS, &newval, 1);
}

// datasheet p17
void ess_set_input_pin(ess_input_pin input)
{
    uint8_t newval = reg_swizzle(0x02, (uint8_t)input, 4, 0b111);
    i2cwrite(ESS_CHANNEL_MAPPING, &newval, 1);
}

// datasheet p19
void ess_set_volume(uint8_t vol)
{
    i2cwrite(ESS_VOLUME1, &vol, 1);
    i2cwrite(ESS_VOLUME2, &vol, 1);
}

// datasheet p20
void ess_set_gpio_inputsel(ess_input_type gpio_input_sel1, ess_input_type gpio_input_sel2)
{
    uint8_t newval = reg_swizzle(0x00, (uint8_t)gpio_input_sel1, 4, 0b11);
    newval = reg_swizzle(newval, (uint8_t)gpio_input_sel2, 6, 0b11);
    i2cwrite(ESS_GPIO_INPUT_SEL_OSF, &newval, 1);
}

/*
// datasheet p22
ess_status ess_get_chip_status () {
    uint8_t reg = 0;
    // someoen needs to write i2c_read
    //reg = i2c_read (ESS_CHIP_STATUS);
    if (((reg >> 2) & 0x7) == 0x7){
        if ()
    }
    return ess_status_NOTFOUND;
}
*/
