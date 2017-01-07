/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "Display_SH1106_I2C.h"

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

#define SH1106_I2C_ADDR 0x3C    // default I2C bus address

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

bool Display_SH1106_I2C::hw_init()
{
    struct {
        uint8_t reg;
        uint8_t seq[26];
    } init_seq = { 0x0,  {
            0xAE,         // Display OFF
            0xA1,         // Segment re-map
            0xC8,         // COM Output Scan Direction
            0xA8, 0x3F,   // MUX Ratio
            0xD5, 0x50,   // Display Clock Divide Ratio and Oscillator Frequency: (== +0%)
            0xD3, 0x00,   // Display Offset
            0xDB, 0x40,   // VCOMH Deselect Level
            0x81, 0xCF,   // Contrast Control
            0xAD, 0x8B,   // DC-DC Control Mode: 1b (== internal DC-DC enabled) (AKA: Enable charge pump regulator)
            0x40,         // Display Start Line
            0xDA, 0x12,   // +++ COM Pins hardware configuration
            0xD9, 0xF1,   // +++ Pre-charge Period
            0xA4,         // +++ Entire Display ON (ignoring RAM): 0b (== OFF)
            0xA6,         // +++ Normal/Inverse Display: 0b (== Normal)
            0xAF,         // Display ON
            0xB0,         // Page Address
            0x02, 0x10    // Column Address
    } };

    _dev = std::move(hal.i2c_mgr->get_device(OLED_I2C_BUS, SH1106_I2C_ADDR));
    memset(_displaybuffer, 0, SH1106_COLUMNS * SH1106_ROWS_PER_PAGE);

    // take i2c bus semaphore
    if (!_dev || !_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    // init display
    _dev->transfer((uint8_t *)&init_seq, sizeof(init_seq), nullptr, 0);

    // give back i2c semaphore
    _dev->get_semaphore()->give();


    return true;
}

bool Display_SH1106_I2C::hw_update()
{
    struct PACKED {
        uint8_t reg;
        uint8_t cmd[3];
    } command = { 0x0, {0x02 /* |= column[0:3] */, 0x10 /* |= column[4:7] */, 0xB0 /* |= page */} };

    struct PACKED {
        uint8_t reg;
        uint8_t db[SH1106_COLUMNS/2];
    } display_buffer = { 0x40, {} };

    if (!_dev || !_dev->get_semaphore()->take(5)) {
        return false;
    }
    // write buffer to display
    for (uint8_t i = 0; i < (SH1106_ROWS / SH1106_ROWS_PER_PAGE); i++) {
        command.cmd[2] = 0xB0 | (i & 0x0F);
        _dev->transfer((uint8_t *)&command, sizeof(command), nullptr, 0);

        memcpy(&display_buffer.db[0], &_displaybuffer[i * SH1106_COLUMNS], SH1106_COLUMNS/2);
        _dev->transfer((uint8_t *)&display_buffer, SH1106_COLUMNS/2 + 1, nullptr, 0);

        memcpy(&display_buffer.db[0], &_displaybuffer[i * SH1106_COLUMNS + SH1106_COLUMNS/2 ], SH1106_COLUMNS/2);
        _dev->transfer((uint8_t *)&display_buffer, SH1106_COLUMNS/2 + 1, nullptr, 0);
    }
    // give back i2c semaphore
    _dev->get_semaphore()->give();

    return true;
}

bool Display_SH1106_I2C::set_pixel(uint16_t x, uint16_t y)
{
    // check x, y range
    if ((x >= SH1106_COLUMNS) || (y >= SH1106_ROWS)) {
        return false;
    }
    // set pixel in buffer
    _displaybuffer[x + (y / 8 * SH1106_COLUMNS)] |= 1 << (y % 8);

    return true;
}

bool Display_SH1106_I2C::clear_pixel(uint16_t x, uint16_t y)
{
    // check x, y range
    if ((x >= SH1106_COLUMNS) || (y >= SH1106_ROWS)) {
        return false;
    }
    // clear pixel in buffer
    _displaybuffer[x + (y / 8 * SH1106_COLUMNS)] &= ~(1 << (y % 8));

    return true;
}
bool Display_SH1106_I2C::clear_screen()
{
     memset(_displaybuffer, 0, SH1106_COLUMNS * SH1106_ROWS_PER_PAGE);
     return true;
}
