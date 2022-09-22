/*
 * MIT License
 *
 * Copyright (c) 2020 Joey Castillo
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

#define RTC_REFERENCE_YEAR (2020)

typedef union {
    struct {
        uint32_t second : 6;    // 0-59
        uint32_t minute : 6;    // 0-59
        uint32_t hour : 5;      // 0-23
        uint32_t day : 5;       // 1-31
        uint32_t month : 4;     // 1-12
        uint32_t year : 6;      // 0-63 (representing 2020-2083)
    } unit;
    uint32_t reg;               // the bit-packed value as expected by the RTC peripheral's CLOCK register.
} rtc_date_time;

typedef enum rtc_alarm_match {
    ALARM_MATCH_DISABLED = 0,
    ALARM_MATCH_SS,
    ALARM_MATCH_MMSS,
    ALARM_MATCH_HHMMSS,
} rtc_alarm_match;

/** @brief Initializes the RTC. Assumes a 1024 Hz clock on GCLK3.
 */
void rtc_init(void);

/** @brief Checks if the RTC is enabled. 
  * @return true if the RTC is enabled; false if not.
  */
bool rtc_is_enabled(void);

/** @brief Sets the date and time.
  * @param date_time The date and time you wish to set, with a year value from 0-63 representing 2020-2083.
  * @note The RTC stores the year as six bits representing a value from 0 to 63. It treats this as a year
  *       offset from a reference year, which must be a leap year. Since 2020 was a leap year, and it allows
  *       useful dates through 2083, it is assumed that apps will use 2020 as the reference year; thus
  *       1 means 2021, 2 means 2022, etc. **You will be responsible for handling this offset in your code**,
  *       if the calendar year is needed for timestamp calculation logic or display purposes.
  */
void rtc_set_date_time(rtc_date_time date_time);

/** @brief Returns the date and time.
  * @return A rtc_date_time with the current date and time, with a year value from 0-63 representing 2020-2083.
  * @see rtc_set_date_time for notes about how the year is stored.
  */
rtc_date_time rtc_get_date_time(void);

/** @brief Enables the alarm interrupt.
  * @param alarm_time The time that you wish to match. The date is currently ignored.
  * @param mask One of the values in rtc_alarm_match indicating which values to check.
  */
void rtc_enable_alarm_interrupt(rtc_date_time alarm_time, rtc_alarm_match mask);

/** @brief Disables the alarm callback.
  */
void rtc_disable_alarm_interrupt(void);
