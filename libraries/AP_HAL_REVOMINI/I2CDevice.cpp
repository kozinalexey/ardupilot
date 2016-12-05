/* -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*
 * I2CDriver.cpp --- AP_HAL_REVOMINI I2C driver.
 *
 * Copyright (C) 2013, Virtualrobotix.com Roberto Navoni , Emile 
 * All Rights Reserved.R Written by Roberto Navoni  <info@virtualrobotix.com>, 11 January 2013
 */

#include <AP_HAL/AP_HAL.h>
#include "I2CDevice.h"
#include <i2c.h>
#include "i2c_soft.h"

using namespace REVOMINI;

REVOMINI::Semaphore REVOI2CDevice::_semaphores[3]; // 2 HW and 1 SW

#ifdef I2C_DEBUG
uint8_t REVOI2CDevice::last_addr, REVOI2CDevice::last_op, REVOI2CDevice::last_send_len, REVOI2CDevice::last_recv_len, REVOI2CDevice::busy, REVOI2CDevice::last_status;
#endif

bool REVOI2CDevice::transfer(const uint8_t *send, uint32_t send_len, uint8_t *recv, uint32_t recv_len)
{

    uint16_t retries=_retries;
    
again:

#ifdef I2C_DEBUG
    uint8_t was_addr=last_addr, was_send_len=last_send_len, was_recv_len=last_recv_len, was_busy=busy, was_status=last_status;

    if(busy){
        return false; // bus is busy, semaphores fails
    }

    last_addr=_address; last_send_len=send_len; last_recv_len=recv_len; busy=1;
#endif

	uint8_t numbytes=0;
	uint32_t ret=0;

        if(!_dev){ // no hardware so use soft I2C

            if(!_initialized) {
                s_i2c.init( ); 
                _initialized=true;
            }
            
            if(recv_len==0){ // only write
                numbytes = send_len-1;
                //                 uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
                ret=s_i2c.writeBuffer( _address, *send, send_len-1, &send[1] );
            
            }else if(send_len==1){ // only read - send byte is address
                //            uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf
                ret=s_i2c.read(_address, *send, recv_len, recv);                
            } else {
                ret=s_i2c.transfer(_address, send_len, send, recv_len, recv);
            }

#ifdef I2C_DEBUG
            busy=false;
#endif
            
            if(ret == I2C_NO_DEVICE) 
                return false;

            if(ret == I2C_OK) 
                return true;
            
            s_i2c.bus_reset();    

            if(retries--) goto again;

            return false;
        }

	if(!_initialized) init();

        
	if(recv_len==0) { // only write
#ifdef I2C_DEBUG
    last_op=1;
#endif

	    numbytes = send_len;
	    ret = i2c_write(_dev,  _address, send, &numbytes);

	} else if(send_len==1) { // only read - send byte is address
#ifdef I2C_DEBUG
    last_op=0;
#endif
	    numbytes=recv_len;
	    ret = i2c_read(_dev,  _address, send, 1, recv, &numbytes);
	} else {
#ifdef I2C_DEBUG
    last_op=0;
#endif
	    numbytes=recv_len;
	    ret = i2c_read(_dev,  _address, send, send_len, recv, &numbytes);
	}

#ifdef I2C_DEBUG
    busy=0; last_status=ret;
#endif


    if(ret == I2C_NO_DEVICE) 
        return false;
    if(ret == I2C_OK) 
        return true;


// all other errors
    _lockup_count ++;  
	    
    reset();

    if(retries--) goto again;

    return false;
}


bool REVOI2CDevice::read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                 uint32_t recv_len, uint8_t times){

    while(times--) {
	bool ret = read_registers(first_reg, recv, recv_len);
	if(!ret) return false;
	recv += recv_len;
    }

    return true;
}



void REVOI2CDevice::reset(){
    i2c_deinit(_dev); // disable I2C hardware
    i2c_bus_reset(_dev);
    _initialized=false;
    REVOMINIScheduler::_delay_microseconds(50);
}

