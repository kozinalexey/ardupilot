/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_F4BY
#include "RCOutput.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_pwm_output.h>
#include <uORB/topics/actuator_direct.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_sbus.h>

extern const AP_HAL::HAL& hal;

using namespace F4BY;

/*
  enable RCOUT_DEBUG_LATENCY to measure output latency using a logic
  analyser. AUX6 will go high during the cork/push output.
 */
#define RCOUT_DEBUG_LATENCY 0

void F4BYRCOutput::init()
{
    _perf_rcout = perf_alloc(PC_ELAPSED, "APM_rcout");
    _pwm_fd = open(PWM_OUTPUT0_DEVICE_PATH, O_RDWR);
    if (_pwm_fd == -1) {
        AP_HAL::panic("Unable to open " PWM_OUTPUT0_DEVICE_PATH);
    }
    if (ioctl(_pwm_fd, PWM_SERVO_ARM, 0) != 0) {
        hal.console->printf("RCOutput: Unable to setup IO arming\n");
    }
    if (ioctl(_pwm_fd, PWM_SERVO_SET_ARM_OK, 0) != 0) {
        hal.console->printf("RCOutput: Unable to setup IO arming OK\n");
    }

    _rate_mask = 0;
    _alt_fd = -1;
    _servo_count = 0;
    _alt_servo_count = 0;

    if (ioctl(_pwm_fd, PWM_SERVO_GET_COUNT, (unsigned long)&_servo_count) != 0) {
        hal.console->printf("RCOutput: Unable to get servo count\n");
        return;
    }

    for (uint8_t i=0; i<ORB_MULTI_MAX_INSTANCES; i++) {
        _outputs[i].pwm_sub = orb_subscribe_multi(ORB_ID(actuator_outputs), i);
    }

#if !defined(CONFIG_ARCH_BOARD_PX4FMU_V4)
    _alt_fd = open("/dev/f4byfmu", O_RDWR);
    if (_alt_fd == -1) {
        hal.console->printf("RCOutput: failed to open /dev/f4byfmu");
        return;
    }
#endif

    // ensure not to write zeros to disabled channels
    _enabled_channels = 0;
    for (uint8_t i=0; i < F4BY_NUM_OUTPUT_CHANNELS; i++) {
        _period[i] = PWM_IGNORE_THIS_CHANNEL;
    }

    // publish actuator vaules on demand
    _actuator_direct_pub = NULL;

    // and armed state
    _actuator_armed_pub = NULL;
}


void F4BYRCOutput::_init_alt_channels(void)
{
    if (_alt_fd == -1) {
        return;
    }
    if (ioctl(_alt_fd, PWM_SERVO_ARM, 0) != 0) {
        hal.console->printf("RCOutput: Unable to setup alt IO arming\n");
        return;
    }
    if (ioctl(_alt_fd, PWM_SERVO_SET_ARM_OK, 0) != 0) {
        hal.console->printf("RCOutput: Unable to setup alt IO arming OK\n");
        return;
    }
    if (ioctl(_alt_fd, PWM_SERVO_GET_COUNT, (unsigned long)&_alt_servo_count) != 0) {
        hal.console->printf("RCOutput: Unable to get servo count\n");
    }
}

/*
  set output frequency on outputs associated with fd
 */
void F4BYRCOutput::set_freq_fd(int fd, uint32_t chmask, uint16_t freq_hz)
{
    // we can't set this per channel
    if (freq_hz > 50 || freq_hz == 1) {
        // we're being asked to set the alt rate
        if (_output_mode == MODE_PWM_ONESHOT || _output_mode == MODE_PWM_ONESHOT125) {
            /*
              set a 1Hz update for oneshot. This periodic output will
              never actually trigger, instead we will directly trigger
              the pulse after each push()
             */
            freq_hz = 1;
        }
        //::printf("SET_UPDATE_RATE %u\n", (unsigned)freq_hz);
        if (ioctl(fd, PWM_SERVO_SET_UPDATE_RATE, (unsigned long)freq_hz) != 0) {
            hal.console->printf("RCOutput: Unable to set alt rate to %uHz\n", (unsigned)freq_hz);
            return;
        }
        _freq_hz = freq_hz;
    }

    /* work out the new rate mask. The outputs have 3 groups of servos.

       Group 0: channels 0 1 2 3
       Group 1: channels 4 5 6 7
       Group 2: channels 8 9 10 11

       Channels within a group must be set to the same rate.

       For the moment we never set the channels above 8 to more than
       50Hz
     */
    if (freq_hz > 50 || freq_hz == 1) {
        // we are setting high rates on the given channels
        _rate_mask |= chmask & 0xFFF;
        if (_rate_mask & 0xF) {
            _rate_mask |= 0xF;
        }
        if (_rate_mask & 0xF0) {
            _rate_mask |= 0xF0;
        }
        if (_rate_mask & 0xF00) {
            _rate_mask |= 0xF00;
        }
    } else {
        // we are setting low rates on the given channels
        if (chmask & 0xF) {
            _rate_mask &= ~0xF;
        }
        if (chmask & 0xc) {
            _rate_mask &= ~0xF0;
        }
        if (chmask & 0xf0) {
            _rate_mask &= ~0xf00;
        }
    }

    //::printf("SELECT_UPDATE_RATE 0x%02x\n", (unsigned)_rate_mask);
    if (ioctl(fd, PWM_SERVO_SET_SELECT_UPDATE_RATE, _rate_mask) != 0) {
        hal.console->printf("RCOutput: Unable to set alt rate mask to 0x%x\n", (unsigned)_rate_mask);
    }
}

/*
  set output frequency
 */
void F4BYRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    if (freq_hz != 50 && (_output_mode == MODE_PWM_ONESHOT || _output_mode == MODE_PWM_ONESHOT125 )
    		    ) {
        // rate is irrelevent in oneshot
        return;
    }

    // re-fetch servo count as it might have changed due to a change in BRD_PWM_COUNT
    if (_pwm_fd != -1 && ioctl(_pwm_fd, PWM_SERVO_GET_COUNT, (unsigned long)&_servo_count) != 0) {
        hal.console->printf("RCOutput: Unable to get servo count\n");
        return;
    }
    if (_alt_fd != -1 && ioctl(_alt_fd, PWM_SERVO_GET_COUNT, (unsigned long)&_alt_servo_count) != 0) {
        hal.console->printf("RCOutput: Unable to get alt servo count\n");
        return;
    }

    // greater than 400 doesn't give enough room at higher periods for
    // the down pulse
    if (freq_hz > 400) {
        freq_hz = 400;
    }
    uint32_t primary_mask = chmask & ((1UL<<_servo_count)-1);
    uint32_t alt_mask = chmask >> _servo_count;
    if (primary_mask && _pwm_fd != -1) {
        set_freq_fd(_pwm_fd, primary_mask, freq_hz);
    }
    if (alt_mask && _alt_fd != -1) {
        set_freq_fd(_alt_fd, alt_mask, freq_hz);
    }
}

uint16_t F4BYRCOutput::get_freq(uint8_t ch)
{
    if (_rate_mask & (1U<<ch)) {
        return _freq_hz;
    }
    return 50;
}

void F4BYRCOutput::enable_ch(uint8_t ch)
{
    if (ch >= F4BY_NUM_OUTPUT_CHANNELS) {
        return;
    }
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
}

void F4BYRCOutput::disable_ch(uint8_t ch)
{
    if (ch >= F4BY_NUM_OUTPUT_CHANNELS) {
        return;
    }

    _enabled_channels &= ~(1U<<ch);
    _period[ch] = PWM_IGNORE_THIS_CHANNEL;
}

void F4BYRCOutput::set_safety_pwm(uint32_t chmask, uint16_t period_us)
{
    struct pwm_output_values pwm_values;
    memset(&pwm_values, 0, sizeof(pwm_values));
    for (uint8_t i=0; i<_servo_count; i++) {
        if ((1UL<<i) & chmask) {
            pwm_values.values[i] = period_us;
        }
        pwm_values.channel_count++;
    }
    int ret = ioctl(_pwm_fd, PWM_SERVO_SET_DISARMED_PWM, (long unsigned int)&pwm_values);
    if (ret != OK) {
        hal.console->printf("Failed to setup disarmed PWM for 0x%08x to %u\n", (unsigned)chmask, period_us);
    }
}

void F4BYRCOutput::set_failsafe_pwm(uint32_t chmask, uint16_t period_us)
{
    struct pwm_output_values pwm_values;
    memset(&pwm_values, 0, sizeof(pwm_values));
    for (uint8_t i=0; i<_servo_count; i++) {
        if ((1UL<<i) & chmask) {
            pwm_values.values[i] = period_us;
        }
        pwm_values.channel_count++;
    }
    int ret = ioctl(_pwm_fd, PWM_SERVO_SET_FAILSAFE_PWM, (long unsigned int)&pwm_values);
    if (ret != OK) {
        hal.console->printf("Failed to setup failsafe PWM for 0x%08x to %u\n", (unsigned)chmask, period_us);
    }
}

bool F4BYRCOutput::force_safety_on(void)
{
    _safety_state_request = AP_HAL::Util::SAFETY_DISARMED;
    _safety_state_request_last_ms = 1;
    return true;
}

void F4BYRCOutput::force_safety_off(void)
{
    _safety_state_request = AP_HAL::Util::SAFETY_ARMED;
    _safety_state_request_last_ms = 1;
}

void F4BYRCOutput::force_safety_pending_requests(void)
{
    // check if there is a pending saftey_state change. If so (timer != 0) then set it.
    if (_safety_state_request_last_ms != 0 &&
            AP_HAL::millis() - _safety_state_request_last_ms >= 100) {
        if (hal.util->safety_switch_state() == _safety_state_request) {
            // current switch state matches request, stop attempting
            _safety_state_request_last_ms = 0;
        } else if (_safety_state_request == AP_HAL::Util::SAFETY_DISARMED) {
            // current != requested, set it
            ioctl(_pwm_fd, PWM_SERVO_SET_FORCE_SAFETY_ON, 0);
            _safety_state_request_last_ms = AP_HAL::millis();
        } else if (_safety_state_request == AP_HAL::Util::SAFETY_ARMED) {
            // current != requested, set it
            ioctl(_pwm_fd, PWM_SERVO_SET_FORCE_SAFETY_OFF, 0);
            _safety_state_request_last_ms = AP_HAL::millis();
        }
    }
}

void F4BYRCOutput::force_safety_no_wait(void)
{
    if (_safety_state_request_last_ms != 0) {
        _safety_state_request_last_ms = 1;
        force_safety_pending_requests();
    }
}

void F4BYRCOutput::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= F4BY_NUM_OUTPUT_CHANNELS) {
        return;
    }
    if (!(_enabled_channels & (1U<<ch))) {
        // not enabled
        return;
    }
    if (ch >= _max_channel) {
        _max_channel = ch + 1;
    }
    /*
      only mark an update is needed if there has been a change, or we
      are in oneshot mode. In oneshot mode we always need to send the
      output
     */
    if (period_us != _period[ch] ||
        _output_mode == MODE_PWM_ONESHOT||
        _output_mode == MODE_PWM_ONESHOT125) {
        _period[ch] = period_us;
        _need_update = true;
    }
}

uint16_t F4BYRCOutput::read(uint8_t ch)
{
    if (ch >= F4BY_NUM_OUTPUT_CHANNELS) {
        return 0;
    }
    // if px4io has given us a value for this channel use that,
    // otherwise use the value we last sent. This makes it easier to
    // observe the behaviour of failsafe in px4io
    for (uint8_t i=0; i<ORB_MULTI_MAX_INSTANCES; i++) {
        if (_outputs[i].pwm_sub >= 0 &&
            ch < _outputs[i].outputs.noutputs &&
            _outputs[i].outputs.output[ch] > 0) {
            return _outputs[i].outputs.output[ch];
        }
    }
    return _period[ch];
}

void F4BYRCOutput::read(uint16_t* period_us, uint8_t len)
{
    for (uint8_t i=0; i<len; i++) {
        period_us[i] = read(i);
    }
}

uint16_t F4BYRCOutput::read_last_sent(uint8_t ch)
{
    if (ch >= F4BY_NUM_OUTPUT_CHANNELS) {
        return 0;
    }

    return _period[ch];
}

void F4BYRCOutput::read_last_sent(uint16_t* period_us, uint8_t len)
{
    for (uint8_t i=0; i<len; i++) {
        period_us[i] = read_last_sent(i);
    }
}

/*
  update actuator armed state
 */
void F4BYRCOutput::_arm_actuators(bool arm)
{
    if (_armed.armed == arm) {
        // already armed;
        return;
    }

	_armed.timestamp = hrt_absolute_time();
    _armed.armed = arm;
    _armed.ready_to_arm = arm;
    _armed.lockdown = false;
    _armed.force_failsafe = false;

    if (_actuator_armed_pub == NULL) {
        _actuator_armed_pub = orb_advertise(ORB_ID(actuator_armed), &_armed);
    } else {
        orb_publish(ORB_ID(actuator_armed), _actuator_armed_pub, &_armed);
    }
}

/*
  publish new outputs to the actuator_direct topic
 */
void F4BYRCOutput::_publish_actuators(void)
{
	struct actuator_direct_s actuators;

    if (_esc_pwm_min == 0 ||
        _esc_pwm_max == 0) {
        // not initialised yet
        return;
    }

	actuators.nvalues = _max_channel;
    if (actuators.nvalues > actuators.NUM_ACTUATORS_DIRECT) {
        actuators.nvalues = actuators.NUM_ACTUATORS_DIRECT;
    }
    // don't publish more than 8 actuators for now, as the uavcan ESC
    // driver refuses to update any motors if you try to publish more
    // than 8
    if (actuators.nvalues > 12) {
        actuators.nvalues = 12;
    }
    bool armed = hal.util->get_soft_armed();
	actuators.timestamp = hrt_absolute_time();
    for (uint8_t i=0; i<actuators.nvalues; i++) {
        if (!armed) {
            actuators.values[i] = 0;
        } else {
            actuators.values[i] = (_period[i] - _esc_pwm_min) / (float)(_esc_pwm_max - _esc_pwm_min);
        }
        // actuator values are from -1 to 1
        actuators.values[i] = actuators.values[i]*2 - 1;
    }

    if (_actuator_direct_pub == NULL) {
        _actuator_direct_pub = orb_advertise(ORB_ID(actuator_direct), &actuators);
    } else {
        orb_publish(ORB_ID(actuator_direct), _actuator_direct_pub, &actuators);
    }
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        _arm_actuators(true);
    }
}

void F4BYRCOutput::_send_outputs(void)
{
    uint32_t now = AP_HAL::micros();

    if ((_enabled_channels & ((1U<<_servo_count)-1)) == 0) {
        // no channels enabled
        _arm_actuators(false);
        goto update_pwm;
    }

    // always send at least at 20Hz, otherwise the IO board may think
    // we are dead
    if (now - _last_output > 50000) {
        _need_update = true;
    }

    // check for PWM count changing. This can happen then the user changes BRD_PWM_COUNT
    if (now - _last_config_us > 1000000) {
        if (_pwm_fd != -1) {
            ioctl(_pwm_fd, PWM_SERVO_GET_COUNT, (unsigned long)&_servo_count);
        }
        if (_alt_fd != -1) {
            ioctl(_alt_fd, PWM_SERVO_GET_COUNT, (unsigned long)&_alt_servo_count);
        }
        _last_config_us = now;
    }

    if (_need_update && _pwm_fd != -1) {
        _need_update = false;
        perf_begin(_perf_rcout);
        uint8_t to_send = _max_channel<_servo_count?_max_channel:_servo_count;
        if (_sbus_enabled) {
            to_send = _max_channel;
        }
        if (to_send > 0) {
            for (int i=to_send-1; i >= 0; i--) {
                if (_period[i] == 0 || _period[i] == PWM_IGNORE_THIS_CHANNEL) {
                    to_send = i;
                } else {
                    break;
                }
            }
        }
        if (to_send > 0) {
            ::write(_pwm_fd, _period, to_send*sizeof(_period[0]));
        }
        if (_max_channel > _servo_count) {
            // maybe send updates to alt_fd
            if (_alt_fd != -1 && _alt_servo_count > 0) {
                uint8_t n = _max_channel - _servo_count;
                if (n > _alt_servo_count) {
                    n = _alt_servo_count;
                }
                if (n > 0) {
                    ::write(_alt_fd, &_period[_servo_count], n*sizeof(_period[0]));
                }
            }
        }

        // also publish to actuator_direct
        _publish_actuators();

        perf_end(_perf_rcout);
        _last_output = now;
    }

update_pwm:
    for (uint8_t i=0; i<ORB_MULTI_MAX_INSTANCES; i++) {
        bool rc_updated = false;
        if (_outputs[i].pwm_sub >= 0 &&
            orb_check(_outputs[i].pwm_sub, &rc_updated) == 0 &&
            rc_updated) {
            orb_copy(ORB_ID(actuator_outputs), _outputs[i].pwm_sub, &_outputs[i].outputs);
        }
    }

}

void F4BYRCOutput::cork()
{
#if RCOUT_DEBUG_LATENCY
    hal.gpio->pinMode(55, HAL_GPIO_OUTPUT);
    hal.gpio->write(55, 1);
#endif
    _corking = true;
}

void F4BYRCOutput::push()
{
#if RCOUT_DEBUG_LATENCY
    hal.gpio->pinMode(55, HAL_GPIO_OUTPUT);
    hal.gpio->write(55, 0);
#endif
    _corking = false;
    if (_output_mode == MODE_PWM_ONESHOT || _output_mode == MODE_PWM_ONESHOT125) {
        // run timer immediately in oneshot mode
        _send_outputs();
    }
}

void F4BYRCOutput::_timer_tick(void)
{
    if (_output_mode != MODE_PWM_ONESHOT && _output_mode != MODE_PWM_ONESHOT125) {
        /* in oneshot mode the timer does nothing as the outputs are
         * sent from push() */
        _send_outputs();
    }

    force_safety_pending_requests();
}

/*
  enable sbus output
 */
bool F4BYRCOutput::enable_sbus_out(uint16_t rate_hz)
{
    int fd = open("/dev/px4io", 0);
    if (fd == -1) {
        return false;
    }
    for (uint8_t tries=0; tries<10; tries++) {
        if (ioctl(fd, SBUS_SET_PROTO_VERSION, 1) != 0) {
            continue;
        }
        if (ioctl(fd, PWM_SERVO_SET_SBUS_RATE, rate_hz) != 0) {
            continue;
        }
        close(fd);
        _sbus_enabled = true;
        return true;
    }
    close(fd);
    return false;
}

/*
  setup output mode
 */
void F4BYRCOutput::set_output_mode(enum output_mode mode)
{
unsigned int         pwm_clock = 1U;

    if (_output_mode == mode) {
        // no change
        return;
    }
    if (mode == MODE_PWM_ONESHOT || mode == MODE_PWM_ONESHOT125) {
        // when using oneshot we don't want the regular pulses. The
        // best we can do with the current PX4Firmware code is ask for
        // 1Hz. This does still produce pulses, but the trigger calls
        // mean the timer is constantly reset, so no pulses are
        // produced except when triggered by push() when the main loop
        // is running
        set_freq(_rate_mask, 1);
    }
    _output_mode = mode;
    if (_output_mode == MODE_PWM_ONESHOT  || mode == MODE_PWM_ONESHOT125) {
        //::printf("enable oneshot\n");
        ioctl(_pwm_fd, PWM_SERVO_SET_ONESHOT, 1);
        if (_alt_fd != -1) {
            ioctl(_alt_fd, PWM_SERVO_SET_ONESHOT, 1);
        }
    } else {
        ioctl(_pwm_fd, PWM_SERVO_SET_ONESHOT, 0);
        if (_alt_fd != -1) {
            ioctl(_alt_fd, PWM_SERVO_SET_ONESHOT, 0);
        }
    }

       switch (_output_mode) {

       case MODE_PWM_BRUSHED16KHZ:
       case MODE_PWM_ONESHOT125:
       case MODE_PWM_PWMFASTX8:
    	   	   pwm_clock = 8U;
        	break;
       default:
    	   break;
       }
       // setup an 8MHz clock. This has the effect of scaling all outputs by 8x
       ioctl(_pwm_fd, PWM_SERVO_SET_UPDATE_CLOCK, pwm_clock);
       if (_alt_fd != -1) {
           ioctl(_alt_fd, PWM_SERVO_SET_UPDATE_CLOCK, pwm_clock);
       }


}


#endif // CONFIG_HAL_BOARD
