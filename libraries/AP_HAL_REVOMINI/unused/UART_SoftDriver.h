/******************************************************************************

RevoMini SoftwareSerial driver

based on:

* SoftSerialIntAP.cpp
* Multi-instance Software Serial Library for STM32Duino
* Using Timers and Interrupts enabling use of any GPIO pins for RX/TX
* 
* Copyright 2015 Ron Curry, InSyte Technologies
* 
 
Features:
- Fully interrupt driven - no delay routines.
- Any GPIO pin may be used for tx or rx (hence the AP libray name suffix)
- Circular buffers on both send and receive.
- Works up to 115,200 baud.
- Member functions compatible with Hardware Serial and Arduino NewSoftSerial Libs
- Extensions for non-blocking read and transmit control
- Supports up to 4 ports (one timer used per port) without modification.
- Easily modified for more ports or different timers on chips with more timers.
- Can do full duplex under certain circumstances at lower baud rates.
- Can send/receive simultaneously with other ports/instantiatious at some baud
  rates and certain circumstances.

Notes:

Performance
- More than two ports use has not been extensively tested. High ISR latencies in the 
low-level Maple timer ISR code, C++ ISR wranglings, and the STM32 sharing interrupt 
architecture (not in that order) restrict simultaneous port use and speeds. Two ports
sending simultaniously at 115,200. As well, two ports simultansiously receiving at 57,600
simultaneously have been successfully tested. Various other situations have been 
tested. Results are dependent on various factors - you'll need to experiment.

Reliability
- Because of the way STM32 shares interrupts and the way STM32Arduino low level ISRs
processes them latencies can be very high for interrupts on a given timer. I won't go
into all the details of why but some interrupts can be essentially locked out. Causing
extremely delayed interrupt servicing. Some of this could be alleviated with more
sophisticated low-level ISRs that make sure that all shared interrupt sources get
serviced on an equal basis but that's not been done yet. 
This impacts the ability to do full duplex on a single port even at medium bit rates.
I've done some experimentation with two ports/instantiations with RX on one port
and TX on the other with good results for full-duplex. In any case, to be sure
that the serial data streams are not corrupted at higher data rates it's best to
write the application software such that rx and tx are not sending/receiving
simultaneously in each port and that other ports are not operating simultaneously
as well. This is, effectively, how the existing NewSoftSerial library on Arduino
operates so not a new concept. Again, experiment and find what works in your
application.

Improvements
No doubt the code can be improved upon. If you use the code PLEASE give back by
providing soure code for improvements or modifications you've made!
- Specific improvements that come to mind and I'd like to explore are:
  o Replacing the STM32/Maple timer interrupt handlers with something more streamlined
    and lower latency and overhead.
  o A better way to implement the high level C++ ISR's to reduce latency/overhead
  o Minor improvements that can save cycles in the C++ ISR's such as using bit-banding
  o Possibly a way to coordinate RX/TX to increase full-duplex capability.

License
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use, copy,
* modify, merge, publish, distribute, sublicense, and/or sell copies
* of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
 *****************************************************************************/

#ifndef __AP_HAL_REVOMINI_SOFTDRIVER_H__
#define __AP_HAL_REVOMINI_SOFTDRIVER_H__

#include <AP_HAL/AP_HAL.h>

#include <AP_HAL_REVOMINI/AP_HAL_REVOMINI.h>

#include <gpio_hal.h>
#include <timer.h>
#include <bitband.h>

#include "GPIO.h"
#include "Scheduler.h"

/******************************************************************************
* Definitions
******************************************************************************/
#define DEBUG_DELAY 1
#define DEBUG_PIN  17
#define DEBUG_PIN1 18

#define  SSI_RX_BUFF_SIZE 64
#define  SSI_TX_BUFF_SIZE 64
#define  SS_MAX_RX_BUFF (SSI_RX_BUFF_SIZE - 1)  // RX buffer size
#define  SS_MAX_TX_BUFF (SSI_TX_BUFF_SIZE - 1)   // TX buffer size
#define  _SSI_VERSION   1.1  // Library Version

#define DEFAULT_TX_TIMEOUT 10000

#ifdef __cplusplus



class REVOMINI::UART_softDriver : public AP_HAL::UARTDriver  {
public:
    UART_softDriver(const int rx_pin, const int tx_pin,  const timer_dev * timer, bool inverseLogic );
    ~UART_softDriver();

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

// implementation

    static inline int library_version() { return _SSI_VERSION; }
    bool            overflow() { bool ret = bufferOverflow; if (ret) bufferOverflow = false; return ret; }
//    int             readnb(); // Non-blocking read

    operator bool() { return true; }

    // Debug use only
#if DEBUG_DELAY
    uint16_t        getBitPeriod() { return bitPeriod; }
    void            setBitPeriod(uint16_t period) { bitPeriod = period; }
    uint16_t        getBitCentering() { return startBitPeriod; }
    void            setBitCentering(uint16_t period) { startBitPeriod = period; }
    uint8_t         getOverFlowTail() { return overFlowTail; }
    uint8_t         getOverFlowHead() { return overFlowHead; }
    uint8_t         getTXHead(){ return transmitBufferRead; }
    uint8_t         getTXTail(){ return transmitBufferWrite; }
    uint8_t         getRXHead(){ return receiveBufferRead; }
    uint8_t         getRXTail(){ return receiveBufferWrite; }
#endif
     
private:

    const uint8_t _rx_pin;
    const uint8_t _tx_pin;
    const AP_HAL::DigitalSource * _rx_;
    const AP_HAL::DigitalSource * _tx_;


    const timer_dev *dev;

    // Per object data
    bool                      activeRX;
    bool                      activeTX;

#if DEBUG_DELAY
    volatile uint8_t          overFlowTail;
    volatile uint8_t          overFlowHead;    
#endif
    
    uint16_t                  bitPeriod;
    uint16_t                  startBitPeriod;
    volatile uint16_t         rxTimingCount;
    volatile uint16_t         txTimingCount;
    volatile uint8_t          bufferOverflow;

    volatile int8_t           rxBitCount;
    volatile uint16_t         receiveBufferWrite;
    volatile uint16_t         receiveBufferRead;
    volatile uint8_t          receiveBuffer[SS_MAX_RX_BUFF]; 
    
    volatile int8_t           txBitCount;
    volatile uint16_t         transmitBufferWrite;
    volatile uint16_t         transmitBufferRead;
    volatile uint8_t          transmitBuffer[SS_MAX_TX_BUFF];
    
    // Static Data
    static UART_softDriver      *interruptObject1; // This looks inefficient but it reduces
    static UART_softDriver      *interruptObject2; // interrupt latency a small amount
    static UART_softDriver      *interruptObject3;
    static UART_softDriver      *interruptObject4;

    static voidFuncPtr        handleRXEdgeInterruptP[4];
    static voidFuncPtr        handleRXBitInterruptP[4];
    static voidFuncPtr        handleTXBitInterruptP[4];

    bool _inverse;

    bool _initialized;


    bool            listen();
    bool            isListening() { return activeRX; }
    bool            stopListening();
    bool            talk();
    bool            isTalkingT() { return activeTX; }
    bool            stopTalking();

    
    // Static Methods
    // Better way to do this?
    static inline void handleRXBitInterrupt1();
    static inline void handleRXEdgeInterrupt1();
    static inline void handleTXBitInterrupt1(); 

    static inline void handleRXBitInterrupt2();
    static inline void handleRXEdgeInterrupt2();
    static inline void handleTXBitInterrupt2();

    static inline void handleRXBitInterrupt3();
    static inline void handleRXEdgeInterrupt3();
    static inline void handleTXBitInterrupt3();

    static inline void handleRXBitInterrupt4();
    static inline void handleRXEdgeInterrupt4();
    static inline void handleTXBitInterrupt4();

    // Private Methods
    inline void     noTXInterrupts();
    inline void     txInterrupts();
    inline uint16_t isTXInterruptEnabled();
    inline void     txInterruptsClr();    
    inline void     noRXStartInterrupts();
    inline void     rxStartInterrupts();     
    inline void     noRXInterrupts();       
    inline void     rxInterrupts();          
    inline void     rxInterruptsClr();
    inline void     onRXPinChange(void);
    inline void     rxNextBit(void);       
    inline void     txNextBit(void);           
    void            setInterruptObject(const timer_dev *timer);


    void setEXTIntPriority(uint8_t pin, uint8_t priority);
    void setTimerIntPriority(const timer_dev *timer, uint8_t priority);

};

typedef void (REVOMINI::UART_softDriver::*memberFunction)(void);


union Handle {
    memberFunction fp;
    uint64_t h;
    uint32_t w[2];
};



inline uint64_t makeHandle(/* REVOMINI::UART_softDriver *obj,  */ memberFunction m) {

    Handle h = { .fp = m };
    
//    h.w[0] = (uint32_t)obj;

    return h.h;
}

// uses C calling convention
extern "C" {
#endif

void call_method( void* obj, void *ptr );

#ifdef __cplusplus
}
#endif



#endif // __AP_HAL_EMPTY_UARTDRIVER_H__

