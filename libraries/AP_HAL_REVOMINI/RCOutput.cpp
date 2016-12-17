#include "RCOutput.h"
#include "GPIO.h"

extern const AP_HAL::HAL& hal;
using namespace REVOMINI;

/*
    OpenPilot motor order (nose up, from top)
    
    1 2
    4 3
    
    ArduCopter motor order
    
    3 1
    2 4

    Cleanflight motor order
    
    4 2 
    3 1
    

so to mimics Arducopter motors should be changed as 
    1 -> 3
    2 -> 1
    3 -> 4
    4 -> 2

so to mimics Cleanflight motors should be changed as 
    1 -> 4
    2 -> 2
    3 -> 1
    4 -> 3


*/

#define REVOMINI_OUT_CHANNELS 6

// #if FRAME_CONFIG == QUAD_FRAME // this is only QUAD layouts

// ArduCopter
static const uint8_t output_channels_arducopter[]= {  // pin assignment
    50, //Timer2/3  - 3
    46, //Timer3/3  - 1
    49, //Timer2/2  - 4
    45, //Timer3/4  - 2
    48, //Timer2/1
    47, //Timer2/0
};

// OpenPilot
static const uint8_t output_channels_openpilot[]= {  // pin assignment
    46, //Timer3/3  - 1
    45, //Timer3/4  - 2
    50, //Timer2/3  - 3
    49, //Timer2/2  - 4
    48, //Timer2/1
    47, //Timer2/0
};

// Cleanflight
static const uint8_t output_channels_cleanflight[]= {  // pin assignment
    49, //Timer2/2  - 4
    45, //Timer3/4  - 2
    46, //Timer3/3  - 1
    50, //Timer2/3  - 3
    48, //Timer2/1
    47, //Timer2/0
};

// Arducopter,shifted 2 pins right to use up to 2 servos
static const uint8_t output_channels_servo[]= {  // pin assignment
    48, //Timer2/1  - 3
    50, //Timer2/3  - 1
    47, //Timer2/0  - 4
    49, //Timer2/2  - 2
    46, //Timer3/3      servo1
    45, //Timer3/4      servo2
};

// #endif


static const uint8_t * const revo_motor_map[]={
    output_channels_arducopter,
    output_channels_openpilot,
    output_channels_cleanflight,
    output_channels_servo,
};

static const uint8_t *output_channels = output_channels_arducopter;  // current pin assignment




#define _BV(bit) (1U << (bit))

void REVOMINIRCOutput::init()
{
    setupTimers();

    memset(&_period[0], 0, sizeof(_period));

    _used_channels=0;
}


void REVOMINIRCOutput::lateInit(uint8_t map){ // 2nd stage with loaded parameters
    
    if(map >= ARRAY_SIZE(revo_motor_map)) return; // don't initialize if parameter is wrong
    
    output_channels = revo_motor_map[map];
    
    InitPWM();
}

void REVOMINIRCOutput::InitPWM()
{
    for(uint8_t i = 0; i < REVOMINI_MAX_OUTPUT_CHANNELS && i < REVOMINI_OUT_CHANNELS; i++) {
        REVOMINIGPIO::_pinMode(output_channels[i], PWM);
    }
}

uint32_t inline REVOMINIRCOutput::_timer_period(uint16_t speed_hz) {
    return (uint32_t)(2000000UL / speed_hz);
}


// channels 1&2, 3&4&5&6 can has a different rates
void REVOMINIRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {          
    uint32_t mask=1;
    
    for(uint8_t i=0; i< REVOMINI_OUT_CHANNELS; i++) { // кто последний тот и папа
        if(chmask & mask) {
            const timer_dev *dev = PIN_MAP[output_channels[i]].timer_device;
            (dev->regs)->ARR =  _timer_period(freq_hz); 
        }
        mask <<= 1;
    }
}

uint16_t REVOMINIRCOutput::get_freq(uint8_t ch) {
    if(ch >= REVOMINI_OUT_CHANNELS) return 0;
    
    const timer_dev *dev = PIN_MAP[output_channels[ch]].timer_device;

    uint32_t icr = (dev->regs)->ARR;

    /* transform to period by inverse of _time_period(icr) */
    return (uint16_t)(2000000UL / icr);
}

/* constrain pwm to be between min and max pulsewidth. */
static inline uint16_t constrain_period(uint16_t p) {
    if (p > RC_INPUT_MAX_PULSEWIDTH) return RC_INPUT_MAX_PULSEWIDTH;
    if (p < RC_INPUT_MIN_PULSEWIDTH) return RC_INPUT_MIN_PULSEWIDTH;
    return p;
}

void REVOMINIRCOutput::set_pwm(uint8_t ch, uint16_t pwm){
    if(ch>=REVOMINI_OUT_CHANNELS) return;

    if (!(_enabled_channels & _BV(ch))) return;      // not enabled

    uint8_t pin = output_channels[ch];
    if (pin >= BOARD_NR_GPIO_PINS) return;
    
    const timer_dev *dev = PIN_MAP[pin].timer_device;
    timer_set_compare(dev, PIN_MAP[pin].timer_channel, pwm);
    TIM_Cmd(dev->regs, ENABLE);
}

void REVOMINIRCOutput::write(uint8_t ch, uint16_t period_us)
{
    if(ch>=REVOMINI_OUT_CHANNELS) return;

    if(_used_channels<ch) _used_channels=ch+1;

    
    uint16_t pwm = constrain_period(period_us) << 1;

    if(_period[ch]==pwm) return; // already so

    _period[ch]=pwm;
    
    _need_update=true;

    if(_corked) return;

    set_pwm(ch, pwm);
}


void REVOMINIRCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
    if(ch>=REVOMINI_OUT_CHANNELS) return;
    
    for (int i = 0; i < len; i++) {
        write(i + ch, period_us[i]);
    }
}

uint16_t REVOMINIRCOutput::read(uint8_t ch)
{
    if(ch>=REVOMINI_OUT_CHANNELS) return RC_INPUT_MIN_PULSEWIDTH;

    uint16_t pin = output_channels[ch];

    if (pin >= BOARD_NR_GPIO_PINS)
        return RC_INPUT_MIN_PULSEWIDTH;
    
    const timer_dev *dev = PIN_MAP[pin].timer_device;

    uint16_t pwm = timer_get_compare(dev, PIN_MAP[pin].timer_channel);
    return pwm >> 1;
}

void REVOMINIRCOutput::read(uint16_t* period_us, uint8_t len)
{
// here we don't need to limit channel - all unsupported will be read as RC_INPUT_MIN_PULSEWIDTH
    for (int i = 0; i < len; i++) {
        period_us[i] = read(i);
    }
}


void REVOMINIRCOutput::enable_ch(uint8_t ch)
{
    if (ch >= REVOMINI_OUT_CHANNELS) 
        return;
    
    if (ch >= 8 && !(_enabled_channels & (1U<<ch))) {
        // this is the first enable of an auxiliary channel - setup
        // aux channels now. This delayed setup makes it possible to
        // use BRD_PWM_COUNT to setup the number of PWM channels.
        _init_alt_channels();
    }
    _enabled_channels |= (1U<<ch);
    if (_period[ch] == PWM_IGNORE_THIS_CHANNEL) {
        _period[ch] = 0;
    }
    REVOMINIGPIO::_pinMode(output_channels[ch], PWM);
}

void REVOMINIRCOutput::disable_ch(uint8_t ch)
{
    if (ch >= REVOMINI_OUT_CHANNELS) {
        return;
    }
    
    _enabled_channels &= ~(1U<<ch);
    _period[ch] = PWM_IGNORE_THIS_CHANNEL;
    

/*    uint8_t pin = output_channels[ch];
    if (pin >= BOARD_NR_GPIO_PINS) return;
    
    const timer_dev *dev = PIN_MAP[pin].timer_device;
    TIM_Cmd(dev->regs, DISABLE);
*/  // we shouldn't disable ALL timer but only one pin, it will be better to change it's mode from PWM to output

    REVOMINIGPIO::_pinMode(output_channels[ch], OUTPUT);
    REVOMINIGPIO::_write(output_channels[ch], 0);

}


void REVOMINIRCOutput::push()
{
    _corked = false;

    for (uint16_t ch = 0; ch < _used_channels; ch++) {
        set_pwm(ch, _period[ch]);
    }
}

