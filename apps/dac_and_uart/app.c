//echo "LoadFile build/firmware.hex\nreset" | JLinkExe -autoconnect 1 -device ATSAMD11D14A -if SWD -speed 4000
#include "app.h"
#include "adc.h"
#include "tcc.h"
#include "system.h"
#include "delay.h"
#include <ctype.h>
#include <stdarg.h>
#include <stdalign.h>
#include "drivers/usb_uart.h"
#include "drivers/es901xk.h"

void app_init(void) {
    set_cpu_frequency(F_CPU);
}

void app_setup(void) {
    usb_init();
    usb_enable();
}

bool app_loop(void) {
    tud_task();
    //usb_task();
    tx_task();
    rx_task();
    break_task();
    uart_timer_task();
    return false;
}