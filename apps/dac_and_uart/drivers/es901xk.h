#pragma once
#include "../peripherals/i2c.h"

#define ESS_ADDR1 (0x90>>1)
#define ESS_ADDR2 (0x92>>1)

// https://www.esstech.com/wp-content/uploads/2021/04/ES9010K2M-Datasheet-v3.2.pdf
// Datasheet p.11
enum ess_regs {
  // Read/write registers
  // we're just going to hardcode the default values now
  // so we don't have to worry about reading the reg vals and
  // writing them back later
  ESS_SYS_SETTINGS = 0x0,   // Default: 0x00
  ESS_INPUT_CONFIG = 0x1,   // Default: 0x8C
  ESS_AUTOMUTE_TIME = 0x4,  // Default: 0x00
  ESS_AUTOMUTE_LEVEL = 0x5, // Default: 0x68
  ESS_MISC1 = 0x6,          // Default: 0x4A
  ESS_GEN_SETTINGS = 0x7,   // Default: 0x80
  ESS_GPIO_CONFIG = 0x8,    // Default: 0x10
  ESS_MASTER_CTRL = 0xA,    // Default: 0x05
  ESS_CHANNEL_MAPPING = 0xB,// Default: 0x02
  ESS_DPLL_ASRC_SETTINGS = 0xC, // Default: 0x5A
  ESS_THD_COMP = 0xD, // Default: 0x40
  ESS_SOFT_START_SETTINGS = 0xE, // Default: 0x8A
  ESS_VOLUME1 = 0xF, // 0x00
  ESS_VOLUME2 = 0x10, // 0x00
  ESS_MASTER_TRIM_U32 = 0x11, // 0xffffffff
  ESS_GPIO_INPUT_SEL_OSF = 0x15, // 0x00
  ESS_THD_COMP_C2_U16 = 0x16, // 0x00
  ESS_THD_COMP_C3_U16 = 0x18, // 0x00
  ESS_PROG_FIR_ADDR = 0x1A, // 0x00
  ESS_PROG_FIR_COEFF_U24 = 0x1B, // 0x000000
  ESS_PROG_FIR_CTRL = 0x1E, // 0x00
  // Read only
  ESS_CHIP_STATUS = 0x40,
  ESS_GPIO_STATUS = 0x41,
  ESS_DPLL_RATIO_U32 = 0x42,
  ESS_CHANNEL_STATUS_BEGIN = 0x46
};

typedef enum ess_input_type_ {
  ESS_INPUT_I2S = 0,
  ESS_INPUT_SPDIF = 1,
  ESS_INPUT_DSD = 3,
} ess_input_type;

typedef enum ess_auto_input_select_type_ {
  ESS_INPUT_SELECT_NOAUTO,
  ESS_INPUT_SELECT_I2S_DSD,
  ESS_INPUT_SELECT_I2S_SPDIF,
  ESS_INPUT_SELECT_I2S_SPDIF_DSD,
} ess_auto_input_select_type;

typedef enum ess_input_pin_ { 
  ESS_PIN_DATA_CLK = 0,
  ESS_PIN_DATA2 = 1,
  ESS_PIN_DATA1 = 2,
  ESS_PIN_GPIO1 = 3,
} ess_input_pin;

typedef enum ess_status_ {
  ess_status_NOTFOUND,
  ess_status_AUTOMUTE_ACTIVE,
  ess_status_PLL_LOCKED
} ess_status;

// void ess_init(uint8_t addr);
uint8_t reg_swizzle(uint8_t initval, uint8_t newval, uint8_t shift, uint8_t mask);
void ess_set_input_type(ess_input_type input_type);
void ess_set_input_pin(ess_input_pin input);
void ess_set_gpio_inputsel(ess_input_type gpio_input_sel1, ess_input_type gpio_input_sel2);
void ess_set_volume(uint8_t vol);
void ess_toggle_mute(uint8_t do_mute);
I2CResult i2cwrite(uint8_t target, uint8_t *data, size_t len);
