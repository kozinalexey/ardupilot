/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * Copyright (C) 2015-2016  Intel Corporation. All rights reserved.
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

#include <AP_HAL/HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/utility/OwnPtr.h>
#include "Semaphores.h"
#include "Scheduler.h"
#include <i2c.h>
//#include <i2c_soft.h>
#include "i2c_soft.h"

//#define I2C_DEBUG

// use soft I2C driver instead hardware
#define SOFT_I2C

// bus 2 (soft) pins
#define SOFT_SCL 14
#define SOFT_SDA 15

using namespace REVOMINI;

class REVOMINI::REVOI2CDevice : public AP_HAL::I2CDevice {
public:
REVOI2CDevice(uint8_t bus, uint8_t address)
        : _bus(bus)
        , _address(address)
        , _retries(1)
        , _lockup_count(0)
        , _initialized(false)
        , _dev(NULL)
    {

        const i2c_dev *dev=NULL;

	switch(bus) {
	case 0:
	    _offs =0;
#ifdef SOFT_I2C
            s_i2c = Soft_I2C( 
                _I2C1->gpio_port, _I2C1->scl_pin,
                _I2C1->gpio_port, _I2C1->sda_pin
            );
#else
	    dev = _I2C1;
#endif
	    break;
	case 1:
	    _offs = 2;
#ifdef SOFT_I2C
            s_i2c = Soft_I2C( 
                _I2C2->gpio_port, _I2C2->scl_pin,
                _I2C2->gpio_port, _I2C2->sda_pin
            );
#else
	    dev = _I2C2;
#endif
	    break;
	case 2:         // this bus can use only soft I2C driver
	    s_i2c = Soft_I2C( 
                PIN_MAP[SOFT_SCL].gpio_device,     PIN_MAP[SOFT_SCL].gpio_bit,
                PIN_MAP[SOFT_SDA].gpio_device,     PIN_MAP[SOFT_SDA].gpio_bit
            );
	}
        _dev = dev; // remember


    }
    
    ~REVOI2CDevice() { }

    /* AP_HAL::I2CDevice implementation */

    /* See AP_HAL::I2CDevice::set_address() */
    void set_address(uint8_t address) override { _address = address; }

    /* See AP_HAL::I2CDevice::set_retries() */
    void set_retries(uint8_t retries) override { _retries = retries; }


    /* AP_HAL::Device implementation */

    /* See AP_HAL::Device::transfer() */
    bool transfer(const uint8_t *send, uint32_t send_len,
                  uint8_t *recv, uint32_t recv_len) override;


    bool read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                 uint32_t recv_len, uint8_t times);


    /* See AP_HAL::Device::set_speed() */
    bool set_speed(enum AP_HAL::Device::Speed speed) override { return true; };

    /* See AP_HAL::Device::get_semaphore() */
    AP_HAL::Semaphore *get_semaphore() override { return &_semaphores[_bus]; }

    /* See AP_HAL::Device::register_periodic_callback() */
    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec, Device::PeriodicCb proc) override
    {
        return REVOMINIScheduler::register_timer_task(period_usec, proc, &_semaphores[_bus] );
    }


    bool adjust_periodic_callback(
        AP_HAL::Device::PeriodicHandle h, uint32_t period_usec) override 
    {
        return REVOMINIScheduler::adjust_timer_task(h, period_usec);
    }
    
    bool unregister_callback(PeriodicHandle h) override  { return REVOMINIScheduler::unregister_timer_task(h); }

    /* See AP_HAL::Device::get_fd() */
    int get_fd() { return 0; }

private:
    void init(){
	if(!_initialized)
//	    i2c_init(_dev, _offs, I2C_400KHz_SPEED);
	    i2c_init(_dev, _offs, I2C_250KHz_SPEED);
	_initialized=true;
    }

    void reset();

    uint8_t _bus;
    uint16_t _offs;
    uint8_t _address;
    uint8_t _retries;
    uint32_t _lockup_count;
    bool _initialized;

    const i2c_dev *_dev;
    Soft_I2C s_i2c; // per-bus instances

    static REVOMINI::Semaphore _semaphores[3]; // individual for each bus + softI2C
    
#ifdef I2C_DEBUG
    static uint8_t last_addr, last_op, last_send_len, last_recv_len, busy, last_status;
#endif
};

class REVOMINI::I2CDeviceManager : public AP_HAL::I2CDeviceManager {
    friend class REVOMINI::REVOI2CDevice;
    
public:
    I2CDeviceManager() { }

    /* AP_HAL::I2CDeviceManager implementation */
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> get_device(uint8_t bus, uint8_t address) {
        return AP_HAL::OwnPtr<AP_HAL::I2CDevice>(new REVOI2CDevice(bus, address));
    }
};

