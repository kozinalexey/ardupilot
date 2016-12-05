
#ifndef __AP_HAL_REVOMINI_GPIO_H__
#define __AP_HAL_REVOMINI_GPIO_H__

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_REVOMINI_Namespace.h"
#include "gpio_hal.h"
#include <boards.h>

#ifndef HIGH
 #define HIGH 0x1
#endif

#ifndef LOW
 #define LOW  0x0
#endif


#ifndef HAL_GPIO_A_LED_PIN
# define HAL_GPIO_A_LED_PIN        36  // BLUE
#endif
#ifndef HAL_GPIO_B_LED_PIN
# define HAL_GPIO_B_LED_PIN        106 //  LED PA13
#endif
#ifndef HAL_GPIO_C_LED_PIN
 # define HAL_GPIO_C_LED_PIN        105 // RED
#endif

#ifndef HAL_GPIO_LED_ON
 # define HAL_GPIO_LED_ON           LOW
 # define HAL_GPIO_LED_OFF          HIGH
#endif

class REVOMINI::REVOMINIDigitalSource : public AP_HAL::DigitalSource {
public:
    REVOMINIDigitalSource(const gpio_dev *device, uint8_t bit): _device(device), _bit(bit){}
    void    mode(uint8_t output);
    uint8_t read() {                 return gpio_read_bit(_device, _bit); }
    void    write(uint8_t value) {   gpio_write_bit(_device, _bit, value); }
    void    toggle() {               gpio_write_bit(_device, _bit, !gpio_read_bit(_device, _bit)); }

private:
    const gpio_dev *_device;
    uint8_t _bit;
};



class REVOMINI::REVOMINIGPIO : public AP_HAL::GPIO {
public:
    REVOMINIGPIO();
    void    init();
    void    pinMode(uint8_t pin, uint8_t output);
    uint8_t read(uint8_t pin);
    void    write(uint8_t pin, uint8_t value);
    void    toggle(uint8_t pin);

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n);

    /* Interrupt interface: */
    bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p, uint8_t mode);

    /* return true if USB cable is connected */
    bool    usb_connected(void);
    
    inline int8_t  analogPinToDigitalPin(uint8_t pin){ return pin; }
    static inline uint8_t analogPinToDigital(uint8_t pin){ return pin; }
    
// internal usage static versions
    static void           _pinMode(uint8_t pin, uint8_t output);
    static inline uint8_t _read(uint8_t pin) { return gpio_read_bit(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit) ?  HIGH : LOW; }
    static inline void    _write(uint8_t pin, uint8_t value) { gpio_write_bit(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit, value); }
    static inline AP_HAL::DigitalSource* get_channel(uint16_t pin) { return new REVOMINIDigitalSource(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit); }
};



#endif // __AP_HAL_REVOMINI_GPIO_H__
