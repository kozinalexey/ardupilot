/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Flymaple port by Mike McCauley
 */

#ifndef __AP_HAL_REVOMINI_ANALOGIN_H__
#define __AP_HAL_REVOMINI_ANALOGIN_H__

#include "AP_HAL_REVOMINI.h"
extern void setupADC(void);

#define REVOMINI_INPUT_MAX_CHANNELS 12


/* Generic memory-mapped I/O accessor functions */
#define MMIO8(addr)             (*(volatile uint8_t *)(addr))
#define MMIO16(addr)            (*(volatile uint16_t *)(addr))
#define MMIO32(addr)            (*(volatile uint32_t *)(addr))
#define MMIO64(addr)            (*(volatile uint64_t *)(addr))

// from libopencm3/include/libopencm3/stm32/f4/memorymap.h
/* ST provided factory calibration values @ 3.3V  - this is WRONG! BusFault on this addresses */
#define ST_VREFINT_CAL          MMIO16(0x1FFF7A2A)
#define ST_TSENSE_CAL1_30C      MMIO16(0x1FFF7A2C)
#define ST_TSENSE_CAL2_110      MMIO16(0x1FFF7A2E)




class REVOMINI::REVOMINIAnalogSource : public AP_HAL::AnalogSource {
public:
    friend class REVOMINI::REVOMINIAnalogIn;

    REVOMINIAnalogSource(uint8_t pin);
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
    void set_stop_pin(uint8_t p);
    void set_settle_time(uint16_t settle_time_ms);
    float voltage_average();
    float voltage_latest();
    float voltage_average_ratiometric();

    /* implementation specific interface: */

    /* new_sample(): called with value of ADC measurments, from interrput */
    void new_sample(uint16_t);

    /* setup_read(): called to setup ADC registers for next measurment,
     * from interrupt */
    void setup_read();

    /* stop_read(): called to stop device measurement */
    void stop_read();

    /* reading_settled(): called to check if we have read for long enough */
    bool reading_settled();

    /* read_average: called to calculate and clear the internal average.
     * implements read_average(), unscaled. */
    float _read_average();

    int16_t get_pin() { return _pin; };
protected:
    const adc_dev* _find_device();
    inline bool initialized() { return _init_done;}
private:
    /* following three are used from both an interrupt and normal thread */
    volatile uint32_t _sum_count;
    volatile uint32_t _sum;
    volatile uint16_t _latest;
    float _last_average;

    /* _pin designates the ADC input mux for the sample */
    uint8_t _pin;

    /* _stop_pin designates a digital pin to use for
       enabling/disabling the analog device */
    uint8_t _stop_pin;
    uint16_t _settle_time_ms;
    uint32_t _read_start_time_ms;
    bool _init_done;

};

/*
    voltage pin is PC2 - 12
    current pin is PC1 - 11

*/

class REVOMINI::REVOMINIAnalogIn : public AP_HAL::AnalogIn {
public:
    REVOMINIAnalogIn();
    void init();
    AP_HAL::AnalogSource* channel(int16_t n);
    float board_voltage(void) { 
        return ( /*ST_VREFINT_CAL / 1000.0*/ 1.2 * 4096  / _vcc.read_average()) * 5.0/3.3; /*_board_voltage;*/ 
    }
    float servorail_voltage(void) { return 0; }
    uint16_t power_status_flags(void) { return 0; }

protected: 
    REVOMINIAnalogSource* _create_channel(uint8_t num);
    void _register_channel(REVOMINIAnalogSource*);
    void _timer_event(void);
    REVOMINIAnalogSource* _channels[REVOMINI_INPUT_MAX_CHANNELS];
    int16_t _num_channels;
    int16_t _active_channel;
    uint16_t _channel_repeat_count;

private:
    REVOMINIAnalogSource _vcc;

    bool cnv_started;
};

#define ANALOG_INPUT_REVOMINI_VCC 253

#endif // __AP_HAL_REVOMINI_ANALOGIN_H__

