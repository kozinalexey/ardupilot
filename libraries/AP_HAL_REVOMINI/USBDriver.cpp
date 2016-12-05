/*
 * UARTDriver.cpp --- AP_HAL_REVOMINI UART driver.
 *
 * Copyright (C) 2013, Virtualrobotix.com Roberto Navoni , Emile 
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_REVOMINI
#include "USBDriver.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>

#include <usb.h>
#include <usart.h>
#include <gpio_hal.h>


using namespace REVOMINI;

extern const AP_HAL::HAL& hal;

extern void delay(uint32_t ms);


USBDriver::USBDriver(bool usb):
    _usb_present(usb),
    _initialized(false)
{
}

void USBDriver::begin(uint32_t baud) {

    _usb_present = gpio_read_bit(_GPIOC,5);

    _initialized = true;
}

void USBDriver::begin(uint32_t baud, uint16_t rxS, uint16_t txS) {
    begin(baud);
}

void USBDriver::end() {
    if(_usb_present)
	usb_close();
}

void USBDriver::flush() {
    if(_usb_present)
	usb_reset_rx();
}

void USBDriver::set_blocking_writes(bool blocking) {
}

bool USBDriver::tx_pending() {
    return false;
}


/* REVOMINI implementations of Stream virtual methods */
uint32_t USBDriver::available() {
    return usb_data_available();
}

uint32_t USBDriver::txspace() {
    return 255;
}

int16_t USBDriver::read() {
    if(_usb_present){
	if (usb_data_available() <= 0)
	    return (-1);
	return usb_getc();
    }
    return 0;
}

/* REVOMINI implementations of Print virtual methods */
size_t USBDriver::write(uint8_t c) {

    if(_usb_present == 1){
	usb_putc(c);
    }
    return 1;
}

size_t USBDriver::write(const uint8_t *buffer, size_t size)
{
    size_t n = 0;
    while (size--) {
        n += write(*buffer++);
    }
    return n;
}

#endif // CONFIG_HAL_BOARD

