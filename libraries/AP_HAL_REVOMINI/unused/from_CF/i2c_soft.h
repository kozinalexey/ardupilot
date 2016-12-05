/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stm32f4xx.h>
#include <hal.h>
#include "gpiopins.h"
#include "systick.h"

#ifdef __cplusplus
  extern "C" {
#endif

void s_i2cInit(const gpio_dev *sda_dev, uint8_t sda_bit, const gpio_dev *scl_dev, uint8_t scl_bit);

uint32_t s_i2cWriteBuffer( uint8_t addr_, uint8_t reg_, uint8_t len_, const uint8_t *data);
uint32_t s_i2cWrite( uint8_t addr_, uint8_t reg, uint8_t data);
uint32_t s_i2cRead( uint8_t addr_, uint8_t reg, uint8_t len, uint8_t* buf);
uint32_t s_i2cTransfer(uint8_t  addr, uint8_t  send_len, const uint8_t *send, uint8_t len, uint8_t *buf);

uint16_t s_i2cGetErrorCounter(void);
void s_i2c_bus_reset(void);

#ifdef __cplusplus
  }
#endif
