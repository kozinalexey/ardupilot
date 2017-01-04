
#include "io.h" // in wirish folder
#include "gpio_hal.h"
#include <boards.h>



#include <ext_interrupts.h>
#include <exti.h>

#include "GPIO.h"




using namespace REVOMINI;

REVOMINIGPIO::REVOMINIGPIO()
{
}

void REVOMINIGPIO::init() 
{
    gpio_init_all();

    afio_init(); // empty
}


#pragma GCC push_options
#pragma GCC optimize ("O0")

void REVOMINIGPIO::_pinMode(uint8_t pin, uint8_t output)
{
    gpio_pin_mode outputMode;
    bool pwm = false;

    switch(output) {
    case OUTPUT:
        outputMode = GPIO_OUTPUT_PP;
        break;
    case OUTPUT_OPEN_DRAIN:
        outputMode = GPIO_OUTPUT_OD;
        break;
    case INPUT:
    case INPUT_FLOATING:
        outputMode = GPIO_INPUT_FLOATING;
        break;
    case INPUT_ANALOG:
        outputMode = GPIO_INPUT_ANALOG;
        break;
    case INPUT_PULLUP:
        outputMode = GPIO_INPUT_PU;
        break;
    case INPUT_PULLDOWN:
        outputMode = GPIO_INPUT_PD;
        break;

    case PWM:
        outputMode = GPIO_AF_OUTPUT_PP;
        pwm = true;
        break;

    case PWM_OPEN_DRAIN:
        outputMode = GPIO_AF_OUTPUT_OD;
        pwm = true;
        break;

    default:
        assert_param(0);
        return;
    }

    gpio_set_mode(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit, outputMode);

    if (pwm && PIN_MAP[pin].timer_device != NULL) {
	GPIO_PinAFConfig(PIN_MAP[pin].gpio_device->GPIOx, PIN_MAP[pin].gpio_bit, PIN_MAP[pin].timer_device->af);
	timer_set_mode(PIN_MAP[pin].timer_device, PIN_MAP[pin].timer_channel, TIMER_PWM);
    }
}


void REVOMINIGPIO::pinMode(uint8_t pin, uint8_t output){

    if ((pin >= BOARD_NR_GPIO_PINS))   return;

    _pinMode(pin, output);
}


uint8_t REVOMINIGPIO::read(uint8_t pin) {
    if (pin >= BOARD_NR_GPIO_PINS)     return 0;

    return _read(pin);
}


void REVOMINIGPIO::write(uint8_t pin, uint8_t value) {
    if ((pin >= BOARD_NR_GPIO_PINS))   return;

    _write(pin, value);
}


void REVOMINIGPIO::toggle(uint8_t pin)
{
    if ((pin >= BOARD_NR_GPIO_PINS))  return;
        
    gpio_toggle_bit(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit);
}



/* Alternative interface: */
AP_HAL::DigitalSource* REVOMINIGPIO::channel(uint16_t pin) {

    if ((pin >= BOARD_NR_GPIO_PINS)) return NULL;

    return  get_channel(pin); // new REVOMINIDigitalSource(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit);
}

/* Interrupt interface: */
bool REVOMINIGPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode)
{
    if ( (interrupt_num >= BOARD_NR_GPIO_PINS) || !p) return false;

    exti_attach_interrupt((afio_exti_num)(PIN_MAP[interrupt_num].gpio_bit),
                           gpio_exti_port(PIN_MAP[interrupt_num].gpio_device),
                           p, exti_out_mode((ExtIntTriggerMode)mode));

    return true;
}

bool REVOMINIGPIO::usb_connected(void)
{
    return gpio_read_bit(PIN_MAP[BOARD_USB_SENSE].gpio_device,PIN_MAP[BOARD_USB_SENSE].gpio_bit);
}

void REVOMINIDigitalSource::mode(uint8_t output)
{
    gpio_pin_mode outputMode;

    switch(output) {
    case OUTPUT:
        outputMode = GPIO_OUTPUT_PP;
        break;
    case OUTPUT_OPEN_DRAIN:
        outputMode = GPIO_OUTPUT_OD;
        break;
    case INPUT:
    case INPUT_FLOATING:
        outputMode = GPIO_INPUT_FLOATING;
        break;
    case INPUT_ANALOG:
        outputMode = GPIO_INPUT_ANALOG;
        break;
    case INPUT_PULLUP:
        outputMode = GPIO_INPUT_PU;
        break;
    case INPUT_PULLDOWN:
        outputMode = GPIO_INPUT_PD;
        break;
    // no PWM via this interface!
    default:
        assert_param(0);
        return;
    }

    gpio_set_mode(_device, _bit, outputMode);
}


