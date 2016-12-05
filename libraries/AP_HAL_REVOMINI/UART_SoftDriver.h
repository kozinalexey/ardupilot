// inspired by AN4655 - http://www.st.com/content/ccc/resource/technical/document/application_note/3d/80/91/f9/4c/25/4a/26/DM00160482.pdf/files/DM00160482.pdf/jcr:content/translations/en.DM00160482.pdf
/*

This example shows how to control both transmit and receive processes by software, based on HW events. 

Overflow and IO capture events of a single Capture & Compare timer is used to control timing of emulation of UART input sampling and
 output handling of the Rx and Tx signals. The associated input capture pin is dedicated to receive data. Any general-purpose output 
pin or associated output compare pin can be used for data transmission. The timer overflow period is set to the channel half bit rate 
by setting the timer auto reload register. 

illustrates the timing diagram.

Figure 3. UART duplex channel based on timer capture events

Transmission process control is based on regular interrupts from the timer overflow events. 
After data for transmission is stored into dedicated variable (transmit buffer simulated by SW), an internal transmission request appears and
 the nearest overflow interrupt is used to start a new transmission (maximum latency for transmission start could be half a bit period). 
This is because the same timer is used for control both transmit and receive processes not synchronized normally. Every even timer overflow
 event interrupt controls the consecutive edge changes of the transmitted signal on the dedicated output pin until the end of the frame 
transmission. Odd overflow interrupts are discarded.

The receive process uses both input capture and output compare feature of the same timer and its dedicated pin. 
Initially, the input capture is performed at the pin. After detecting the first falling edge, the value captured in the 
capture register is then used for compare purposes because the input pin functionality is switched to output compare mode (without 
affecting any output GPIO capability since the pin still stays in input mode). Due to the half-a-bit overflow period of the timer, the nearest
 output compare event points to the middle of the first receive bit (start bit). Sampling can be simulated by three consecutive reading of the 
input pin level at that moment and if, for each of them, a low level is detected, the correctly received start bit is evaluated. The receive 
process then continues by watching every next odd output compare event. The same three point sampling method can be performed with noise 
detection logic here and for all the other received data bits until the end of the current receive frame. All even compare interrupts are 
discarded. After the stop bits of the frame are sampled, the Rx pin is switched back to input capture mode and waits for the next frame 
start condition. The detection of noise while the stop bits are being sampled should cause a frame error. If a noise is detected during start 
bit, the receive process should be aborted and the Rx pin switched back to input capture mode while waiting for the next falling edge capture.

User can build an API interface upon this low level HW abstraction level. It can include the UART channel initialization, enable and disabl
e Rx and Tx channels, read and write the data flow (e.g. control the data buffers) and check the transactions’ status. Size of data, parity 
control number of stop bits or other control can be solved on pre-compilation level by conditional compilation to speed up the code when 
these features are not used.


(C)

in case of RevoMini we have pins

    14, // PC8  T8/3  - Soft_scl or soft_tx
    15, // PC9  T8/4  - Soft_sda or soft_rx

*/


#ifndef __AP_HAL_REVOMINI_SERIALDRIVER_H__
#define __AP_HAL_REVOMINI_SERIALDRIVER_H__

#include <AP_HAL_REVOMINI/AP_HAL_REVOMINI.h>

#include <usart.h>
#include <usb.h>
#include <gpio_hal.h>
#include <pwm_in.h>
#include "Scheduler.h"
#include "GPIO.h"

#define DEFAULT_TX_TIMEOUT 10000

#define SS_DEBUG

#define  SSI_RX_BUFF_SIZE 64
#define  SSI_TX_BUFF_SIZE 64
#define  SS_MAX_RX_BUFF (SSI_RX_BUFF_SIZE - 1)  // RX buffer size
#define  SS_MAX_TX_BUFF (SSI_TX_BUFF_SIZE - 1)   // TX buffer size

//#define SERIAL_TIMER TIMER8
#define TX_PIN 14
#define RX_PIN 15

#define RX_TIMER_CHANNEL   TIMER_CH4
#define RX_TIMER_MASK      TIMER_DIER_CC4IE_BIT
#define RX_TIMER_PENDING   TIMER_SR_CC4IF_BIT
#define RX_CCR             CCR4
#define TIMER_RX_INTERRUPT TIMER_CC4_INTERRUPT


extern const struct TIM_Channel PWM_Channels[];

class REVOMINI::SerialDriver : public AP_HAL::UARTDriver  {
public:
    SerialDriver(bool inverseLogic)
    { 
        _inverse = inverseLogic;
    }


  /* REVOMINI implementations of UARTDriver virtual methods */
  void begin(uint32_t b);
  void begin(uint32_t b, uint16_t rxS, uint16_t txS);
  void end();
  void flush();

  bool is_initialized(){ return _initialized; }

  void set_blocking_writes(bool blocking);

  bool tx_pending();

  /* REVOMINI implementations of Stream virtual methods */
  uint32_t available() override;
  uint32_t txspace() override;
  int16_t read() override;

  /* Empty implementations of Print virtual methods */
  size_t write(uint8_t c);
  size_t write(const uint8_t *buffer, size_t size);

private:

    static bool _inverse;
    static bool _initialized;
    static bool _blocking;
    
    static const struct TIM_Channel *channel;
    
    static uint16_t                  bitPeriod;
//    static uint16_t                  startBitPeriod;
/*    static volatile uint16_t         rxTimingCount;
    static volatile uint16_t         txTimingCount;

*/
#ifdef SS_DEBUG
    static volatile uint8_t          bufferOverflow;
#endif

    static volatile int8_t           rxBitCount;
    static volatile uint16_t         receiveBufferWrite;
    static volatile uint16_t         receiveBufferRead;
    static volatile uint8_t          receiveBuffer[SSI_RX_BUFF_SIZE];
    static uint8_t receiveByte;

    static volatile int8_t           txBitCount;
    static volatile uint16_t         transmitBufferWrite;
    static volatile uint16_t         transmitBufferRead;
    static volatile uint8_t          transmitBuffer[SSI_RX_BUFF_SIZE];
    
    static bool txSkip;
    static bool rxSkip;
    
    static bool activeRX;
    static bool activeTX;
    

    // Clear pending interrupt and enable receive interrupt
    // Note: Clears pending interrupt
    static inline void txEnableInterrupts() {

        channel->tim->SR &= ~TIMER_SR_UIF;
        timer_enable_irq(channel->timer, TIMER_UPDATE_INTERRUPT);
        
//          *bb_perip(&(channel->tim->SR), TX_TIMER_PENDING) = 0; // Clear int pending
//          *bb_perip(&(channel->tim->DIER), TX_TIMER_MASK) = 1; // enable
    }

    // Mask transmit interrupt
    static inline void txDisableInterrupts() {
//          *bb_perip(&(channel->tim->DIER), TX_TIMER_MASK) = 0;
        timer_disable_irq(channel->timer, TIMER_UPDATE_INTERRUPT);
    }
    
    static void rxSetCapture();
    static void rxSetCompare();
    
    static void txNextBit(TIM_TypeDef *tim);
    static void rxNextBit(TIM_TypeDef *tim);
    
};

#endif
