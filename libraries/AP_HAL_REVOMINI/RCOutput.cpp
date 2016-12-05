#include "RCOutput.h"
#include "GPIO.h"

extern const AP_HAL::HAL& hal;
using namespace REVOMINI;

static const uint8_t output_channels[]= {  // pin assignment
    46, //Timer3/3
    45, //Timer3/4
    50, //Timer2/3
    49, //Timer2/2
    48, //Timer2/1
    47  //Timer2/0
};

#define REVOMINI_OUT_CHANNELS (sizeof(output_channels)/sizeof(uint8_t))

#define _BV(bit) (1U << (bit))

void REVOMINIRCOutput::init()
{
    setupTimers();

    memset(&_period[0], 0, sizeof(_period));

    InitPWM();
    _used_channels=0;
}


void REVOMINIRCOutput::InitPWM()
{
    for(uint8_t i = 0; i < REVOMINI_MAX_OUTPUT_CHANNELS && i<sizeof(output_channels); i++) {
        REVOMINIGPIO::_pinMode(output_channels[i], PWM);
    }
}

uint32_t inline REVOMINIRCOutput::_timer_period(uint16_t speed_hz) {
    return (uint32_t)(2000000UL / speed_hz);
}


// channels 1&2, 3&4&5&6 can has a different rates
void REVOMINIRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {          
    uint32_t icr = _timer_period(freq_hz);

    if ((chmask & ( _BV(CH_1) | _BV(CH_2) )) != 0) {
	TIM3->ARR = icr;
    }

    if ((chmask & ( _BV(CH_3) | _BV(CH_4) |  _BV(CH_5) | _BV(CH_6))) != 0) {
	TIM2->ARR = icr;
    }

}

uint16_t REVOMINIRCOutput::get_freq(uint8_t ch) {
    uint32_t icr;
    switch (ch) {
    case CH_1:
    case CH_2:
        icr = (TIMER3->regs)->ARR;
        break;
    case CH_3:
    case CH_4:
    case CH_5:
    case CH_6:
        icr = (TIMER2->regs)->ARR;
        break;
    default:
        return 0;
    }
    /* transform to period by inverse of _time_period(icr). */
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

    // if(dev == NULL || dev->type == TIMER_BASIC) return; we init const structure at compile time

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

//    if(dev == NULL || dev->type == TIMER_BASIC) return RC_INPUT_MIN_PULSEWIDTH; this can't be

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

