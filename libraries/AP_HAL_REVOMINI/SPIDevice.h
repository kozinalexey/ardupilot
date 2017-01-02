/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * Copyright (C) 2015  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <inttypes.h>
#include <vector>

#include <AP_HAL/HAL.h>
#include <AP_HAL/SPIDevice.h>
#include "Semaphores.h"
#include "Scheduler.h"
#include <spi.h>
#include <boards.h>

namespace REVOMINI {


typedef enum SPIFrequency {
    SPI_18MHZ       = 0, /**< 18 MHz */
    SPI_9MHZ        = 1, /**< 9 MHz */
    SPI_4_5MHZ      = 2, /**< 4.5 MHz */
    SPI_2_25MHZ     = 3, /**< 2.25 MHz */
    SPI_1_125MHZ    = 4, /**< 1.125 MHz */
    SPI_562_500KHZ  = 5, /**< 562.500 KHz */
    SPI_281_250KHZ  = 6, /**< 281.250 KHz */
    SPI_140_625KHZ  = 7, /**< 140.625 KHz */
} SPIFrequency;


struct spi_pins {
    uint8_t nss;
    uint8_t sck;
    uint8_t miso;
    uint8_t mosi;
};

static const spi_pins board_spi_pins[] __FLASH__ = {
    { // 0
        BOARD_SPI1_NSS_PIN,
        BOARD_SPI1_SCK_PIN,
        BOARD_SPI1_MISO_PIN,
        BOARD_SPI1_MOSI_PIN
    },
    { // 1
        BOARD_SPI2_NSS_PIN,
        BOARD_SPI2_SCK_PIN,
        BOARD_SPI2_MISO_PIN,
        BOARD_SPI2_MOSI_PIN
    },
    { //2
        BOARD_SPI3_NSS_PIN,
        BOARD_SPI3_SCK_PIN,
        BOARD_SPI3_MISO_PIN,
        BOARD_SPI3_MOSI_PIN
    }
};

struct SPIDesc {
    SPIDesc(const char *name_, 
            const spi_dev  *dev_, 
            uint8_t bus_,
            spi_mode mode_,
            int16_t cs_pin_, 
            SPIFrequency lowspeed_,
            SPIFrequency highspeed_)
        : name(name_), 
        dev(dev_), 
        bus(bus_),
        mode(mode_),
        cs_pin(cs_pin_),
        lowspeed(lowspeed_), 
        highspeed(highspeed_)
    {
    }

    const char *name;
    const spi_dev  *dev;
    uint8_t bus;
    spi_mode mode;
    int16_t cs_pin;
    SPIFrequency lowspeed;
    SPIFrequency highspeed;
};

class SPIDevice : public AP_HAL::SPIDevice {
public:
    SPIDevice(SPIDesc &device_desc);

    ~SPIDevice() {}

    /* AP_HAL::SPIDevice implementation */

    /* See AP_HAL::Device::set_speed() */
    bool set_speed(AP_HAL::Device::Speed speed) override;

    /* See AP_HAL::Device::transfer() */
    bool transfer(const uint8_t *send, uint32_t send_len,
                        uint8_t *recv, uint32_t recv_len) override;

    /* See AP_HAL::SPIDevice::transfer_fullduplex() */
    bool transfer_fullduplex(const uint8_t *send, uint8_t *recv,
                                   uint32_t len) override;

    /* See AP_HAL::Device::get_semaphore() */
    AP_HAL::Semaphore *get_semaphore() { return &_semaphores[_desc.bus]; }

    /* See AP_HAL::Device::register_periodic_callback() */
    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, AP_HAL::Device::PeriodicCb proc) override
    {
        return REVOMINIScheduler::register_timer_task(period_usec, proc, &_semaphores[_desc.bus]);
    }


    bool adjust_periodic_callback(
        AP_HAL::Device::PeriodicHandle h, uint32_t period_usec) override
    {
        return REVOMINIScheduler::adjust_timer_task(h, period_usec);
    }

    bool unregister_callback(PeriodicHandle h) { return REVOMINIScheduler::unregister_timer_task(h); }



protected:
    SPIDesc &_desc;

    AP_HAL::DigitalSource *_cs;
    SPIFrequency _speed;

    static REVOMINI::Semaphore _semaphores[3]; // per bus

    bool _initialized;
    void init(void);

    void _cs_assert(){    _cs->write(0); } // Select device 
    void _cs_release(){   _cs->write(1); } // Deselect device

    const spi_pins* dev_to_spi_pins(const spi_dev *dev);
    void  configure_gpios(const spi_dev *dev, bool as_master);
    spi_baud_rate determine_baud_rate(SPIFrequency freq);


    uint8_t _transfer(uint8_t data);
    
};

class SPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    friend class SPIDevice;


    SPIDeviceManager()
    {
    }

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> get_device(const char *name);

protected:

    static const uint8_t _n_device_desc;
    static SPIDesc _device[];
};

}
