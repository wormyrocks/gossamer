#include "app.h"
#include "dac.h"
#include "opamp.h"
#include "i2c.h"
#include "delay.h"
#include "gfx/sh1107.h"

typedef struct opamp_state_t { 
    uint8_t muxneg;
    uint8_t muxpos;
    uint8_t potmux;
    uint8_t res1mux;
    uint8_t res2mux;
} opamp_state_t;

opamp_state_t opamps[3];
uint8_t selected_row = 0;
uint8_t selected_column = 0;

void draw_opamp(int instance);
void draw_menu(int instance);

void app_init(void) {
    opamps[0].res1mux = opamps[1].res1mux = opamps[2].res1mux = OPAMP_RES1MUX_NC;
    opamps[0].res2mux = opamps[1].res2mux = opamps[2].res2mux = OPAMP_RES2MUX_NC;
}

void app_setup(void) {
    HAL_GPIO_LED_out();
    HAL_GPIO_D10_in();
    HAL_GPIO_D10_pullup();
    HAL_GPIO_D6_in();
    HAL_GPIO_D6_pullup();
    HAL_GPIO_D5_in();
    HAL_GPIO_D5_pullup();

    HAL_GPIO_SDA_out();
    HAL_GPIO_SCL_out();
    HAL_GPIO_SDA_pmuxen(HAL_GPIO_PMUX_D);
    HAL_GPIO_SCL_pmuxen(HAL_GPIO_PMUX_D);
    i2c_init();

    gfx_init(64, 128, 1);
    gfx_set_rotation(1);

    // gfx_draw_string(0, 40, "0 IN+ IN- NC  P0  NC", 1, 0, 1);
    // gfx_draw_string(0, 48, "1 IN+ IN- NC  P0  NC", 1, 0, 1);
    // gfx_draw_string(0, 56, "2 IN+ IN- NC  P0  NC", 1, 0, 1);

    sh1107_begin();
    sh1107_update();

    HAL_GPIO_AREF_pmuxen(HAL_GPIO_PMUX_B);
    HAL_GPIO_A0_pmuxen(HAL_GPIO_PMUX_B);
    HAL_GPIO_A1_pmuxen(HAL_GPIO_PMUX_B);
    HAL_GPIO_A2_pmuxen(HAL_GPIO_PMUX_B);
    HAL_GPIO_A3_pmuxen(HAL_GPIO_PMUX_B);
    HAL_GPIO_A4_pmuxen(HAL_GPIO_PMUX_B);
    HAL_GPIO_A5_pmuxen(HAL_GPIO_PMUX_B);

    opamp_init();

    opamp_set_muxpos(0, OPAMP0_MUXPOS_POS_0);
    opamp_set_muxneg(0, OPAMP0_MUXNEG_LADDER_0);
    opamp_set_res1mux(0, OPAMP_RES1MUX_GND);
    opamp_set_res2mux(0, OPAMP0_RES2MUX_OUT_0);
    opamp_set_potmux(0, OPAMP_POTMUX_RATIO_1_15);

    opamp_set_muxpos(1, OPAMP1_MUXPOS_OUT_0);
    opamp_set_muxneg(1, OPAMP1_MUXNEG_LADDER_1);
    opamp_set_res1mux(1, OPAMP_RES1MUX_GND);
    opamp_set_res2mux(1, OPAMP1_RES2MUX_OUT_1);
    opamp_set_potmux(1, OPAMP_POTMUX_RATIO_8_8);

    opamp_set_muxpos(2, OPAMP2_MUXPOS_OUT_1);
    opamp_set_muxneg(2, OPAMP2_MUXNEG_LADDER_2);
    opamp_set_res1mux(2, OPAMP_RES1MUX_GND);
    opamp_set_res2mux(2, OPAMP2_RES2MUX_OUT_2);
    opamp_set_potmux(2, OPAMP_POTMUX_RATIO_8_8);

    opamp_enable(0);
    opamp_enable(1);
    opamp_enable(2);
}

bool app_loop(void) {
    uint8_t updown = 0;

    if (!HAL_GPIO_D10_read()) updown = -1;
    if (!HAL_GPIO_D5_read()) updown = 1;

    if (updown) {
        switch (selected_column) {
            case 0:
                selected_row = (selected_row + updown) % 3;
                break;
            case 1:
                opamps[selected_row].muxpos = ((opamps[selected_row].muxpos + updown * 2) - 1) % 7;
                if (opamps[selected_row].muxpos > 3 && selected_row != 2) {
                    opamps[selected_row].muxpos = 0;
                }
                break;
            case 2:
                opamps[selected_row].muxneg = ((opamps[selected_row].muxneg + updown * 2) - 1) % 6;
                if (opamps[selected_row].muxneg > 3 && selected_row != 2) {
                    opamps[selected_row].muxneg = 0;
                }
                break;
            case 3:
                opamps[selected_row].res1mux = ((opamps[selected_row].res1mux + updown * 2) - 1) % 5;
                break;
            case 4:
                opamps[selected_row].potmux = (opamps[selected_row].potmux + updown) % 8;
                break;
            case 5:
                opamps[selected_row].res2mux = ((opamps[selected_row].res2mux + updown * 2) - 1) % 3;
                break;
        }
        HAL_GPIO_LED_set();
        delay_ms(100);
        HAL_GPIO_LED_clr();
    }
    if (!HAL_GPIO_D6_read()) {
        selected_column = (selected_column + 1) % 6;
    }

    gfx_fill_screen(0);
    gfx_draw_string(0, 33, "  POS NEG R1  POT R2", 1, 0, 1);
    for (size_t i = 0; i < 3; i++) {
        draw_opamp(i);
        draw_menu(i);
    }
    
    sh1107_update();

    return false;
}

void draw_opamp(int instance) {
    int x = (instance == 0) ? 0 : (instance == 1) ? 43 : 86;
    int y = 2;
    gfx_draw_triangle(17 + x, 0 + y, 34 + x, 8 + y, 17 + x, 16 + y, 1);
    gfx_draw_fast_hline(14 + x, 5 + y, 3, 1);
    gfx_draw_fast_hline(14 + x, 11 + y, 3, 1);
    gfx_draw_fast_hline(35 + x, 8 + y, 2, 1);

    gfx_draw_small_char(19 + x, 3 + y, '+', 1, 1);
    gfx_draw_small_char(19 + x, 9 + y, '-', 1, 1);
    gfx_draw_small_char(24 + x, 6 + y, '0' + instance, 1, 1);

    gfx_draw_fast_hline(7 + x, 22 + y, 27, 1);
    gfx_draw_pixel(10 + x, 22 + y, 0);
    gfx_draw_pixel(12 + x, 22 + y, 0);
    gfx_draw_pixel(14 + x, 22 + y, 0);
    gfx_draw_pixel(16 + x, 22 + y, 0);
    gfx_draw_pixel(24 + x, 22 + y, 0);
    gfx_draw_pixel(26 + x, 22 + y, 0);
    gfx_draw_pixel(28 + x, 22 + y, 0);
    gfx_draw_pixel(30 + x, 22 + y, 0);

    gfx_draw_pixel(10 + x, 21 + y, 1);
    gfx_draw_pixel(12 + x, 23 + y, 1);
    gfx_draw_pixel(14 + x, 21 + y, 1);
    gfx_draw_pixel(16 + x, 23 + y, 1);

    gfx_draw_pixel(24 + x, 21 + y, 1);
    gfx_draw_pixel(26 + x, 23 + y, 1);
    gfx_draw_pixel(28 + x, 21 + y, 1);
    gfx_draw_pixel(30 + x, 23 + y, 1);

    switch (opamps[instance].res1mux) {
        case 0: // OAxPOS
            gfx_draw_fast_hline(0 + x, 5 + y, 14, 1);
            gfx_draw_fast_vline(0 + x, 6 + y, 16, 1);
            gfx_draw_fast_hline(0 + x, 22 + y, 8, 1);
            break;
        case 1: // OAxNEG
            gfx_draw_fast_hline(0 + x, 11 + y, 14, 1);
            gfx_draw_fast_vline(0 + x, 12 + y, 10, 1);
            gfx_draw_fast_hline(0 + x, 22 + y, 8, 1);
            break;
        case 2: // DAC or previous output
            if (instance) {
                gfx_draw_fast_vline(6 + x, 8 + y, 15, 1);
                gfx_draw_fast_hline(-7 + x, 8 + y, 13, 1);
            } else {
                gfx_draw_fast_vline(5 + x, 21 + y, 2, 1);
                gfx_draw_fast_hline(5 + x, 22 + y, 2, 1);
                gfx_draw_string(0 + x, 15 + y, "DAC", 1, 1, 0);
            }
            break;
        case 3: // GND
            gfx_draw_fast_vline(6 + x, 22 + y, 3, 1);
            gfx_draw_fast_hline(4 + x, 25 + y, 5, 1);
            gfx_draw_fast_hline(5 + x, 27 + y, 3, 1);
            break;
        default:
            break;
    }
    switch (opamps[instance].res2mux) {
        case 0: // VCC
            gfx_draw_fast_hline(31 + x, 22 + y, 3, 1);
            gfx_draw_fast_vline(34 + x, 19 + y, 4, 1);
            gfx_draw_fast_hline(33 + x, 20 + y, 3, 1);
            break;
        case 1: // OUT
            gfx_draw_fast_hline(31 + x, 22 + y, 10, 1);
            gfx_draw_fast_vline(40 + x, 9 + y, 13, 1);
            gfx_draw_fast_hline(37 + x, 8 + y, 4, 1);
            break;
        default:
            break;
    }
    char r1[3] = {0};
    char r2[3] = {0};
    
    switch (opamps[instance].potmux) {
        case 0:
            r1[0] = '1';
            r1[1] = '4';
            r2[0] = ' ';
            r2[1] = '2';
            break;
        case 1:
            r1[0] = '1';
            r1[1] = '2';
            r2[0] = ' ';
            r2[1] = '4';
            break;
        case 2:
            r1[0] = ' ';
            r1[1] = '8';
            r2[0] = ' ';
            r2[1] = '8';
            break;
        case 3:
            r1[0] = ' ';
            r1[1] = '6';
            r2[0] = '1';
            r2[1] = '0';
            break;
        case 4:
            r1[0] = ' ';
            r1[1] = '4';
            r2[0] = '1';
            r2[1] = '2';
            break;
        case 5:
            r1[0] = ' ';
            r1[1] = '3';
            r2[0] = '1';
            r2[1] = '3';
            break;
        case 6:
            r1[0] = ' ';
            r1[1] = '2';
            r2[0] = '1';
            r2[1] = '4';
            break;
        case 7:
            r1[0] = ' ';
            r1[1] = '1';
            r2[0] = '1';
            r2[1] = '5';
            break;
    }
    gfx_draw_string(10 + x, 24 + y, r1, 1, 1, 0);
    gfx_draw_string(24 + x, 24 + y, r2, 1, 1, 0);

    switch (opamps[instance].muxpos) {
        case 0: // OAxPOS
            gfx_draw_string(2 + x, 3 + y, "IN+", 1, 1, 0);
            break;
        case 1: // OAxTAP
            gfx_draw_fast_hline(12 + x, 5 + y, 3, 1);
            gfx_draw_fast_vline(12 + x, 5 + y, 14, 1);
            gfx_draw_fast_hline(12 + x, 19 + y, 9, 1);
            gfx_draw_fast_vline(20 + x, 19 + y, 4, 1);
            break;
        case 2: // DAC or output
            if (instance == 0) {
                gfx_draw_string(2 + x, 3 + y, "DAC", 1, 1, 0);
            } else {
                gfx_draw_fast_hline(x - 6, 8 + y, 1, 1);
                gfx_draw_fast_vline(x - 5, 5 + y, 4, 1);
                gfx_draw_fast_hline(x - 5, 5 + y, 20, 1);
            }
            break;
        case 3: // GND
            gfx_draw_fast_vline(13 + x, 2 + y, 4, 1);
            gfx_draw_fast_hline(6 + x, 2 + y, 8, 1);
            gfx_draw_fast_vline(6 + x, 2 + y, 2, 1);
            gfx_draw_fast_hline(4 + x, 4 + y, 5, 1);
            gfx_draw_fast_hline(5 + x, 6 + y, 3, 1);
            break;
        case 4: // OA0 positive (second instance only)
            break;
        case 5: // OA1 positive (second instance only)
            break;
        case 6: // OA0 tap (second instance only)
            break;
        default:
            break;
    }

    switch (opamps[instance].muxneg) {
        case 0: // OAxNEG
            gfx_draw_string(2 + x, 9 + y, "IN-", 1, 1, 0);
            break;
        case 1: // OAxTAP
            gfx_draw_fast_hline(13 + x, 11 + y, 3, 1);
            gfx_draw_fast_vline(13 + x, 11 + y, 8, 1);
            gfx_draw_fast_hline(13 + x, 18 + y, 8, 1);
            gfx_draw_fast_vline(20 + x, 18 + y, 4, 1);
            break;
        case 2: // OAxOUT
            break;
        case 3: // DAC (0, 1) or OA0 negative (2)
            if (instance == 2) {
            } else {
                gfx_draw_string(2 + x, 9 + y, "DAC", 1, 1, 0);
            }
            break;
        case 4: // OA1 negative (second instance only)
            if (instance == 2) {
            }
            break;
        case 5: // DAC (second instance only)
            if (instance == 2) {
                gfx_draw_string(2 + x, 9 + y, "DAC", 1, 1, 0);
            }
            break;
        default:
            break;
    }
}

void draw_menu(int instance) {
    bool row_active = (selected_row == instance);
    bool highlighted;
    char buf[4] = {0};

    if (row_active && selected_column == 0) gfx_draw_char(0, 41 + 8 * instance, '0' + instance, 0, 1, 1);
    else gfx_draw_char(0, 41 + 8 * instance, '0' + instance, 1, 0, 1);
    
    highlighted = (row_active && selected_column == 1) ? 1 : 0;
    switch (opamps[instance].muxpos) {
        case 0: // OAxPOS
            gfx_draw_string(12, 41 + 8 * instance, "IN+", !highlighted, highlighted, 1);
            break;
        case 1: // OAxTAP
            buf[0] = 'R';
            buf[1] = 'L';
            buf[2] = '0' + instance;
            gfx_draw_string(12, 41 + 8 * instance, buf, !highlighted, highlighted, 1);
            break;
        case 2: // DAC or output
            if (instance == 0) {
                gfx_draw_string(12, 41 + 8 * instance, "DAC", !highlighted, highlighted, 1);
            } else if (instance == 1) {
                gfx_draw_string(12, 41 + 8 * instance, "OA0", !highlighted, highlighted, 1);
            } else {
                gfx_draw_string(12, 41 + 8 * instance, "OA1", !highlighted, highlighted, 1);
            }
            break;
        case 3: // GND
            gfx_draw_string(12, 41 + 8 * instance, "GND", !highlighted, highlighted, 1);
            break;
        case 4: // OA0 positive (second instance only)
            if (instance == 2) {
                gfx_draw_string(12, 41 + 8 * instance, "0+ ", !highlighted, highlighted, 1);
            } else {
                gfx_draw_string(12, 41 + 8 * instance, "Err", !highlighted, highlighted, 1);
            }
            break;
        case 5: // OA1 positive (second instance only)
            if (instance == 2) {
                gfx_draw_string(12, 41 + 8 * instance, "1+ ", !highlighted, highlighted, 1);
            } else {
                gfx_draw_string(12, 41 + 8 * instance, "Err", !highlighted, highlighted, 1);
            }
            break;
        case 6: // OA0 tap (second instance only)
            if (instance == 2) {
                gfx_draw_string(12, 41 + 8 * instance, "RL0", !highlighted, highlighted, 1);
            } else {
                gfx_draw_string(12, 41 + 8 * instance, "Err", !highlighted, highlighted, 1);
            }
            break;
        default:
            gfx_draw_string(12, 41 + 8 * instance, "Err", !highlighted, highlighted, 1);
    }

    highlighted = (row_active && selected_column == 2) ? 1 : 0;
    switch (opamps[instance].muxneg) {
        case 0: // OAxNEG
            gfx_draw_string(36, 41 + 8 * instance, "IN-", !highlighted, highlighted, 1);
            break;
        case 1: // OAxTAP
            buf[0] = 'R';
            buf[1] = 'L';
            buf[2] = '0' + instance;
            gfx_draw_string(36, 41 + 8 * instance, buf, !highlighted, highlighted, 1);
            break;
        case 2: // OAxOUT
            gfx_draw_string(36, 41 + 8 * instance, "OUT", !highlighted, highlighted, 1);
            break;
        case 3: // DAC (0, 1) or OA0 negative (2)
            if (instance == 2) {
                gfx_draw_string(36, 41 + 8 * instance, "0- ", !highlighted, highlighted, 1);
            } else {
                gfx_draw_string(36, 41 + 8 * instance, "DAC", !highlighted, highlighted, 1);
            }
            break;
        case 4: // OA1 negative (second instance only)
            if (instance == 2) {
                gfx_draw_string(36, 41 + 8 * instance, "1- ", !highlighted, highlighted, 1);
            } else {
                gfx_draw_string(36, 41 + 8 * instance, "Err", !highlighted, highlighted, 1);
            }
            break;
        case 5: // DAC (second instance only)
            if (instance == 2) {
                gfx_draw_string(36, 41 + 8 * instance, "DAC", !highlighted, highlighted, 1);
            } else {
                gfx_draw_string(36, 41 + 8 * instance, "Err", !highlighted, highlighted, 1);
            }
            break;
        default:
            gfx_draw_string(36, 41 + 8 * instance, "Err", !highlighted, highlighted, 1);
    }


    highlighted = (row_active && selected_column == 3) ? 1 : 0;
    switch(opamps[instance].res1mux) {
        case 0: // OAxPOS
            gfx_draw_string(60, 41 + 8 * instance, "IN+", !highlighted, highlighted, 1);
            break;
        case 1: // OAxNEG
            gfx_draw_string(60, 41 + 8 * instance, "IN-", !highlighted, highlighted, 1);
            break;
        case 2: // DAC or previous output
            if (instance) {
                buf[0] = 'O';
                buf[1] = 'A';
                buf[2] = '0' + instance - 1;
                gfx_draw_string(60, 41 + 8 * instance, buf, !highlighted, highlighted, 1);
            } else {
                gfx_draw_string(60, 41 + 8 * instance, "DAC", !highlighted, highlighted, 1);
            }
            break;
        case 3: // GND
            gfx_draw_string(60, 41 + 8 * instance, "GND", !highlighted, highlighted, 1);
            break;
        default:
            gfx_draw_string(60, 41 + 8 * instance, "NC ", !highlighted, highlighted, 1);
    }
    
    highlighted = (row_active && selected_column == 4) ? 1 : 0;
    buf[0] = 'P';
    buf[1] = '0' + opamps[instance].potmux;
    buf[2] = ' ';
    gfx_draw_string(84, 41 + 8 * instance, buf, !highlighted, highlighted, 1);

    highlighted = (row_active && selected_column == 5) ? 1 : 0;
    switch(opamps[instance].res2mux) {
        case 0: // VCC
            gfx_draw_string(108, 41 + 8 * instance, "VCC", !highlighted, highlighted, 1);
            break;
        case 1: // OUT
            buf[0] = 'O';
            buf[1] = 'A';
            buf[2] = '0' + instance;
            gfx_draw_string(108, 41 + 8 * instance, buf, !highlighted, highlighted, 1);
            break;
        default:
            gfx_draw_string(108, 41 + 8 * instance, "NC ", !highlighted, highlighted, 1);
    }
}
