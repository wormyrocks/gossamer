//echo "LoadFile build/firmware.hex\nreset" | JLinkExe -autoconnect 1 -device ATSAMD11D14A -if SWD -speed 4000
#include "app.h"
#include "adc.h"
#include "tusb.h"
#include "tcc.h"
#include "usb.h"
#include "system.h"
#include "delay.h"
#include <ctype.h>
#include <stdarg.h>
#include "drivers/es901xk.h"

static void cdc_task(void);

void app_init(void) {
    set_cpu_frequency(48000000);
}

void app_setup(void) {
	/*
    usb_init();
    usb_enable();
    */
    i2c_init();
    i2c_enable();
    HAL_GPIO_HPA_SHDN_out();
    HAL_GPIO_HPA_SHDN_clr();
    HAL_GPIO_SDA_out();
    HAL_GPIO_SCL_out();
    HAL_GPIO_LED_out();
    HAL_GPIO_UART_TX_in();
    HAL_GPIO_UART_RX_out();
    HAL_GPIO_SCL_pmuxen(HAL_GPIO_PMUX_SERCOM);
    HAL_GPIO_SDA_pmuxen(HAL_GPIO_PMUX_SERCOM);
    adc_init();
    adc_enable();
    HAL_GPIO_ESS_SHDN_out();
    HAL_GPIO_ESS_SHDN_clr();
    delay_ms(250);
    HAL_GPIO_ESS_SHDN_set();
    delay_ms(100);
    delay_ms(10);
    ess_set_input_type(ESS_INPUT_SPDIF);
    ess_set_input_pin(ESS_PIN_DATA_CLK);
    HAL_GPIO_LED_set();
    delay_ms(100);
    HAL_GPIO_HPA_SHDN_set();
}

bool app_loop(void) {
    //tud_task();
    //cdc_task();

    /*
    if (i==5000) {
        echo_serial_port(0, "hi\r\n", 4);
        i=0;
    }
        i++;
        */
    //HAL_GPIO_LED_toggle();
    delay_ms(20);
    uint16_t adc_val = adc_get_analog_value(5);
    uint8_t new_volume = 0xff - ((adc_val >> 8) & 0xff);
    //ess_set_volume(new_volume);
    //tud_cdc_n_write_char(0, 'a');
    //tud_cdc_n_write_flush(0);
    return false;
}

// echo to either Serial0 or Serial1
// with Serial0 as all lower case, Serial1 as all upper case
static void echo_serial_port(uint8_t itf, uint8_t buf[], uint32_t count) {
    uint8_t const case_diff = 'a' - 'A';

    for(uint32_t i=0; i<count; i++) {
        if (itf == 0) {
            // echo back 1st port as lower case
            if (isupper(buf[i])) buf[i] += case_diff;
        } else {
            // echo back 2nd port as upper case
            if (islower(buf[i])) buf[i] -= case_diff;
        }

        tud_cdc_n_write_char(itf, buf[i]);
    }
    tud_cdc_n_write_flush(itf);
}
//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
static void cdc_task(void) {
    uint8_t itf;

    for (itf = 0; itf < CFG_TUD_CDC; itf++) {
        if (tud_cdc_n_available(itf)) {
            uint8_t buf[64];

            uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));

            // echo back to both serial ports
            //echo_serial_port(0, buf, count);
//            echo_serial_port(1, buf, count);
        }
    }
}

