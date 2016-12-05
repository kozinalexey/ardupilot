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
#include "UARTDriver.h"

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



REVOMINIUARTDriver::REVOMINIUARTDriver(const struct usart_dev *usart):
    _usart_device(usart),
    _initialized(false)
{
}

void REVOMINIUARTDriver::begin(uint32_t baud) {

	uint32_t mode=0;

        if(_usart_device->tx_pin < BOARD_NR_GPIO_PINS){
            const stm32_pin_info *txi = &PIN_MAP[_usart_device->tx_pin];
            gpio_set_af_mode(txi->gpio_device, txi->gpio_bit, _usart_device->gpio_af);
            gpio_set_mode(txi->gpio_device, txi->gpio_bit, GPIO_AF_OUTPUT_PP);
            mode |= USART_Mode_Tx;
        } 
	
	if(_usart_device->rx_pin < BOARD_NR_GPIO_PINS){
	    const stm32_pin_info *rxi = &PIN_MAP[_usart_device->rx_pin];
            gpio_set_af_mode(rxi->gpio_device, rxi->gpio_bit, _usart_device->gpio_af);
            gpio_set_mode(rxi->gpio_device, rxi->gpio_bit, GPIO_AF_OUTPUT_PP);
            mode |= USART_Mode_Rx;
        }

        if(!mode) return;
        
	usart_init(_usart_device);
	usart_setup(_usart_device, (uint32_t)baud, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No, mode , USART_HardwareFlowControl_None, DEFAULT_TX_TIMEOUT);
	usart_enable(_usart_device);

    _initialized = true;
}

void REVOMINIUARTDriver::begin(uint32_t baud, uint16_t rxS, uint16_t txS) {
    begin(baud);
}

void REVOMINIUARTDriver::end() {
    usart_disable(_usart_device);
}

void REVOMINIUARTDriver::flush() {
    usart_reset_rx(_usart_device);
    usart_reset_tx(_usart_device);
}

void REVOMINIUARTDriver::set_blocking_writes(bool blocking) {
	usart_reset_tx(_usart_device);
	_usart_device->state->usetxrb = !blocking;
}

bool REVOMINIUARTDriver::tx_pending() {
    if (usart_txfifo_nbytes(_usart_device) > 0)   {
        return true;
    }
    return false;
}


/* REVOMINI implementations of Stream virtual methods */
uint32_t REVOMINIUARTDriver::available() {
    return usart_data_available(_usart_device);
}

uint32_t REVOMINIUARTDriver::txspace() {
    return usart_txfifo_freebytes(_usart_device);
}

int16_t REVOMINIUARTDriver::read() {
    if (available() <= 0)
        return (-1);
    return usart_getc(_usart_device);
}

/* REVOMINI implementations of Print virtual methods */
size_t REVOMINIUARTDriver::write(uint8_t c) {

    if (REVOMINIScheduler::_in_timerprocess()) {
        // not allowed from timers
        return 0;
    }

    usart_putc(_usart_device, c);
    return 1;
}

size_t REVOMINIUARTDriver::write(const uint8_t *buffer, size_t size)
{
    size_t n = 0;
    while (size--) {
        n += write(*buffer++);
    }
    return n;
}

#endif // CONFIG_HAL_BOARD

