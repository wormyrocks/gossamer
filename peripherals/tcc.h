/** 
 * @file tcc.h
 * @brief Timer/Counter for Control Applications (TCC) Peripheral
 */
/*
 * MIT License
 *
 * Copyright (c) 2022 Joey Castillo
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "sam.h"

/** @brief Waits for the TCC to synchronize.
  * @details 
  * @param instance The TCC peripheral instance as numbered in the data sheet (or
  *                 0 if there is only one TCC, as on the SAM D11 or L22).
  */
void tcc_sync(uint8_t instance);

/** @brief Enables the peripheral clock for the TCC and clocks it with the selected
 *         clock source. Also resets the TCC to its starting configuration.
  * @details This just sets up the TCC in its reset state. You are still responsible
  *          for configuring it with the options you need for things like clock
  *          prescaling and waveform output.
  * @param instance The TCC peripheral instance as numbered in the data sheet, or 0.
  * @return true if the TCC was set up, false if instance was out of range (i.e. you
  *         asked for TCC2, but the microcontroller only has one TCC).
  * @note if this function returns false, you shouldn't interact with this TCC instance
  *       with any other functions; they don't do the bounds check that this does.
  */
bool tcc_setup(uint8_t instance, uint8_t clocksource);

/** @brief Enables the TCC. Make sure to call tcc_setup first to set it up.
  * @param instance The TCC peripheral instance as numbered in the data sheet, or 0.
  */
void tcc_enable(uint8_t instance);

/** @brief Disables the TCC, but retains all its settings.
  * @param instance The TCC peripheral instance as numbered in the data sheet, or 0.
  */
void tcc_disable(uint8_t instance);
