
#ifndef __AP_HAL_REVOMINI_RCINPUT_H__
#define __AP_HAL_REVOMINI_RCINPUT_H__

#include <AP_HAL_REVOMINI/AP_HAL_REVOMINI.h>
#include "UARTDriver.h"
#include <usart.h>

#define REVOMINI_RC_INPUT_MIN_CHANNELS 4
#define REVOMINI_RC_INPUT_NUM_CHANNELS 20

enum BOARD_RC_MODE {
    BOARD_RC_NONE=0,
    BOARD_RC_SBUS,
    BOARD_RC_DSM,
};

class REVOMINI::REVOMINIRCInput : public AP_HAL::RCInput {
public:
    REVOMINIRCInput();
    void init()  override;
    uint8_t  valid_channels();

    uint16_t read(uint8_t ch) override;
    uint8_t read(uint16_t* periods, uint8_t len) override;
    
    bool new_input() override;
    uint8_t num_channels() override;

    bool set_overrides(int16_t *overrides, uint8_t len) override;
    bool set_override(uint8_t channel, int16_t override) override;
    void clear_overrides() override;
    
    bool rc_bind(int dsmMode) override;
    
private:
    static void rxIntRC(uint16_t value0, uint16_t value1, bool state);
    static bool _process_ppmsum_pulse(uint16_t value);
    static void _process_dsm_pulse(uint16_t width_s0, uint16_t width_s1);
    static void _process_sbus_pulse(uint16_t width_s0, uint16_t width_s1);

    static void parse_pulses();
    
    static unsigned int ppm_sum_channel;

    static bool is_PPM;
    
    static volatile uint8_t  _valid;

    /* override state */
    static uint16_t _override[8];

    static uint64_t _last_read;
    static bool _override_valid;

    static volatile uint64_t _ppm_last_signal;
    /* private variables to communicate with input capture isr */
    static volatile uint16_t _pulse_capt[REVOMINI_RC_INPUT_NUM_CHANNELS];

    static volatile uint8_t  _valid_channels;
    
    static REVOMINIUARTDriver uartSDriver; 

    static bool _new_ppm_input();
    static bool _new_dsm_input();

    static bool _got_ppm;
    static bool _got_dsm;

    static bool _was_ppm;
    static bool _was_dsm;

    static volatile uint64_t _dsm_last_signal;
    static volatile uint16_t _dsm_val[REVOMINI_RC_INPUT_NUM_CHANNELS];

    static void add_dsm_uart_input(); // add some DSM input bytes, for RCInput over a serial port
    
    static void add_dsm_input();  // add some DSM input bytes, for RCInput over a PPMSUM line
    static void add_sbus_input(); // add some SBUS input bytes, for RCInput over a PPMSUM line

    static void dsm_bind(uint16_t cmd, int pulses);
    
    // state of add_dsm_input
    static struct DSM {
        uint8_t frame[16];
        uint8_t partial_frame_count;
        uint64_t last_input_ms;
    } dsm;
    
    // state of SBUS bit decoder
    static struct SbusState {
        uint16_t bytes[25]; // including start bit, parity and stop bits
        uint16_t bit_ofs;
    } sbus_state;

    // state of DSM decoder
    static struct DSM_State {
        uint16_t bytes[16]; // including start bit and stop bit
        uint16_t bit_ofs;
    } dsm_state;

    // state of add_sbus_input
    static struct SBUS {
        uint8_t frame[26];
        uint8_t partial_frame_count;
        uint32_t last_input_ms;
    } sbus;
    
    static enum BOARD_RC_MODE _rc_mode;    
};

#endif // __AP_HAL_REVOMINI_RCINPUT_H__
