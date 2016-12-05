/*
 * UART_SoftDriver.cpp --- AP_HAL_REVOMINI SoftwareSerial driver

we can use TIM8 all channels

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

#if CONFIG_HAL_BOARD == HAL_BOARD_REVOMINI
#include "UART_SoftDriver.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>

#include <gpio_hal.h>
#include <nvic.h>



/******************************************************************************
* Timer Definitions
* Change these if you wish to use different timer channels
******************************************************************************/
#define TIMER_MAX_COUNT   0xffff

#define TX_TIMER_CHANNEL  TIMER_CH3
#define TX_TIMER_MASK     TIMER_DIER_CC3IE_BIT
#define TX_TIMER_PENDING  TIMER_SR_CC3IF_BIT
#define TX_CCR            CCR3

#define RX_TIMER_CHANNEL  TIMER_CH4
#define RX_TIMER_MASK     TIMER_DIER_CC4IE_BIT
#define RX_TIMER_PENDING  TIMER_SR_CC4IF_BIT
#define RX_CCR            CCR4


using namespace REVOMINI;

extern const AP_HAL::HAL& hal;

extern void delay(uint32_t ms);


// API
void call_method( void* obj, void *ptr ) {

    Handle h;
    
    h.w[0] = (uint32_t) ptr;
    h.w[1] = (uint32_t) obj;
    
    UART_softDriver * t = static_cast<UART_softDriver *>(obj);
    (t->*h.fp)( );
}


/******************************************************************************
* Convenience functions to disable/enable tx and rx interrupts
******************************************************************************/
// Mask transmit interrupt
inline void UART_softDriver::noTXInterrupts() {
  *bb_perip(&(dev->regs->DIER), TX_TIMER_MASK) = 0;
}


// Enable transmit interrupt
// Note: Purposely does not clear pending interrupt
inline void UART_softDriver::txInterrupts() {
  *bb_perip(&(dev->regs->DIER), TX_TIMER_MASK) = 1;
}


// Test if transmit interrupt is enabled
inline uint16_t UART_softDriver::isTXInterruptEnabled() {
  return (*bb_perip(&(dev->regs->DIER), TX_TIMER_MASK));  
}


// Clear pending interrupt and enable receive interrupt
// Note: Clears pending interrupt
inline void UART_softDriver::txInterruptsClr() {
  *bb_perip(&(dev->regs->SR), TX_TIMER_PENDING) = 0; // Clear int pending
  *bb_perip(&(dev->regs->DIER), TX_TIMER_MASK) = 1;
}


// Mask receive start bit interrupt
inline void UART_softDriver::noRXStartInterrupts() {
    bb_peri_set_bit(&(EXTI->FTSR), PIN_MAP[_tx_pin].gpio_bit, 0);
}


// Enable receive start bit interrupt
// Note: Purposely does not clear pending interrupt
inline void UART_softDriver::rxStartInterrupts() {
    bb_peri_set_bit(&(EXTI->FTSR), PIN_MAP[_tx_pin].gpio_bit, 1);
}


// Mask receive interrupt
inline void UART_softDriver::noRXInterrupts() {
  *bb_perip(&(dev->regs->DIER), RX_TIMER_MASK) = 0;
}


// Enable receive interrupt
// Note: Purposely does not clear pending interrupt
inline void UART_softDriver::rxInterrupts() {
  *bb_perip(&(dev->regs->DIER), RX_TIMER_MASK) = 1;
}


// Clear pending interrupt and enable receive interrupt
// Note: Clears pending interrupt
inline void UART_softDriver::rxInterruptsClr() {
  *bb_perip(&(dev->regs->SR), RX_TIMER_PENDING) = 0; // Clear int pending
  *bb_perip(&(dev->regs->DIER), RX_TIMER_MASK) = 1;

}


/******************************************************************************
* Specialized functions to set interrupt priorities and assign object ointers
* These are needed due to the gyrations required to support per instance ISRs
******************************************************************************/
// Set Interrupt Priority for EXTInt line
void UART_softDriver::setEXTIntPriority(uint8_t pin, uint8_t priority) {

  switch((PIN_MAP[pin].gpio_bit)) {
    case 0:
      nvic_irq_set_priority(NVIC_EXTI0, priority);
      break;
    case 1:
      nvic_irq_set_priority(NVIC_EXTI1, priority);
      break;
    case 2:
      nvic_irq_set_priority(NVIC_EXTI2, priority);
      break;
    case 3:
      nvic_irq_set_priority(NVIC_EXTI3, priority);
      break;
    case 4:
      nvic_irq_set_priority(NVIC_EXTI4, priority);
      break;
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
      nvic_irq_set_priority(NVIC_EXTI_9_5, priority);
      break;
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15: 
      nvic_irq_set_priority(NVIC_EXTI_15_10, priority);
      break;
  }
  
}


// Set Interrupt Priority for Timer Interrupts
void UART_softDriver::setTimerIntPriority(const timer_dev *timer, uint8_t priority) {

  if(timer == TIMER1){
      nvic_irq_set_priority(NVIC_TIMER1_UP, priority);
      nvic_irq_set_priority(NVIC_TIMER1_CC, priority);
  } else  if(timer == TIMER2){
      nvic_irq_set_priority(NVIC_TIMER2, priority);
  } else  if(timer == TIMER3){
      nvic_irq_set_priority(NVIC_TIMER3, priority);
  } else  if(timer == TIMER4){
      nvic_irq_set_priority(NVIC_TIMER4, priority);
  }
  
}


// Set the correct interruptObject for this instance
void UART_softDriver::setInterruptObject(const timer_dev *timer) {

  if(timer == TIMER1){
      interruptObject1 = this;
  } else if(timer == TIMER2){
      interruptObject2 = this;
  } else if(timer == TIMER3){
      interruptObject3 = this;
  } else if(timer == TIMER4){
      interruptObject4 = this;
  }
}



/******************************************************************************
* Constructor / Destructor
******************************************************************************/
// Constructor
UART_softDriver::UART_softDriver(const int rx_pin, const int tx_pin,  const timer_dev * timer, bool inverseLogic):
    _rx_pin(rx_pin),
    _tx_pin(tx_pin),
    _rx_(REVOMINIGPIO::get_channel(rx_pin)),
    _tx_(REVOMINIGPIO::get_channel(tx_pin)),
    dev(timer),
    _inverse(inverseLogic),
    _initialized(false)
{

  // Setup ISR pointer for this instance and timer (one timer per instance)
  // This is a workaround for c++
//  setInterruptObject(rxtxTimer);

}


// Destructor
UART_softDriver::~UART_softDriver() {
  end();
}


/*
void UART_softDriver::begin(uint32_t baud) {


    _initialized = true;
}

void UART_softDriver::begin(uint32_t baud, uint16_t rxS, uint16_t txS) {
}

void UART_softDriver::end() {
}

void UART_softDriver::flush() {
}

void UART_softDriver::set_blocking_writes(bool blocking) {
}

bool UART_softDriver::tx_pending() {
    return false;
}


// REVOMINI implementations of Stream virtual methods 
uint32_t UART_softDriver::available() {
    return false;
}

uint32_t UART_softDriver::txspace() {
    return 0;
}

int16_t UART_softDriver::read() {
    return -1;
}
*/

/* REVOMINI implementations of Print virtual methods */

// Virtual write
// Saves tx byte in buffer and restarts transmit delay timer
// 1 bit time latency prior to transmit start if buffer was empty

size_t UART_softDriver::write(uint8_t c) {

    if (REVOMINIScheduler::_in_timerprocess()) {    // not allowed from timers
        return 0;
    }

    // Check if transmit timer interrupt enabled and if not unmask it
    // transmit timer interrupt will get masked by transmit ISR when buffer becomes empty
    if (!isTXInterruptEnabled()) {
  
      // Save new data in buffer
      transmitBuffer[transmitBufferWrite] = c;
//      transmitBufferWrite = (transmitBufferWrite + 1) % (SS_MAX_TX_BUFF - 1);
      transmitBufferWrite = (transmitBufferWrite == SS_MAX_TX_BUFF) ? 0 : transmitBufferWrite + 1;
      
      // Set state to 10 (send start bit) and re-enable transmit interrupt
      txBitCount = 10;
//     timerSerial.setCompare(TX_TIMER_CHANNEL, (int16_t)((dev->regs).bas->CNT) + 1); 
      timer_set_compare(dev, TX_TIMER_CHANNEL, (int16_t)(dev->regs->CNT) + 1); 
      txInterruptsClr();
      
    } else {

      // Blocks if buffer full
      bool i;
      do {
//        noTXInterrupts();
        i = (((transmitBufferWrite + 1) % SS_MAX_TX_BUFF) == transmitBufferRead);
//        txInterrupts();
      } while (i);
  
      // Save new data in buffer and bump the write pointer
      transmitBuffer[transmitBufferWrite] = c;
//      noTXInterrupts();
      transmitBufferWrite = (transmitBufferWrite == SS_MAX_TX_BUFF) ? 0 : transmitBufferWrite + 1;
//      txInterrupts();
    }
    
    return 1;
}

size_t UART_softDriver::write(const uint8_t *buffer, size_t size)
{
    size_t n = 0;
    while (size--) {
        n += write(*buffer++);
    }
    return n;
}
/******************************************************************************
* TX and RX Interrupt Service Routines
******************************************************************************/

#define bitRead(value, bit)            (((value) >> (bit)) & 0x01)

// Transmits next bit. Called by timer ch1 compare interrupt
void UART_softDriver::txNextBit(TIM_TypeDef *tim) { // ISR

  // State 0 through 7 - receive bits
  if (txBitCount <= 7) {
    if (bitRead(transmitBuffer[transmitBufferRead], txBitCount) == 1) {
      REVOMINIGPIO::_write(_tx_pin,HIGH); 
    } else {
      REVOMINIGPIO::_write(_tx_pin,LOW);
    }

//    txTimingCount = ((uint16_t)(timerSerialDEV->regs).gen->TX_CCR) + bitPeriod;
    //timerSerial.setCompare(TX_TIMER_CHANNEL, ((uint16_t)(timerSerialDEV->regs).gen->TX_CCR) + bitPeriod); 
    timer_set_compare(dev, TX_TIMER_CHANNEL, (uint16_t)(dev->regs->TX_CCR) + bitPeriod); 


    interrupts();
    // Bump the bit/state counter to state 8
    txBitCount++; 

#if DEBUG_DELAY
    REVOMINIGPIO::_write(DEBUG_PIN1,1);
    REVOMINIGPIO::_write(DEBUG_PIN1,0);
#endif

  // State 8 - Send the stop bit and reset state to state -1
  //          Shutdown timer interrupt if buffer empty
  } else if (txBitCount == 8) {

    // Send the stop bit
    REVOMINIGPIO::_write(_tx_pin, HIGH); 

    interrupts();

      transmitBufferRead = (transmitBufferRead == SS_MAX_TX_BUFF ) ? 0 : transmitBufferRead + 1;
//      transmitBufferRead = (transmitBufferRead + 1) % (SS_MAX_TX_BUFF - 1);

    if ((transmitBufferRead != transmitBufferWrite) && activeTX) {
      
      //timerSerial.setCompare(TX_TIMER_CHANNEL, ((uint16_t)(timerSerialDEV->regs).gen->TX_CCR) + bitPeriod); 
        timer_set_compare(dev, TX_TIMER_CHANNEL, (uint16_t)(dev->regs->TX_CCR) + bitPeriod);       
    
        txBitCount = 10;
      
    } else {
      
      // Buffer empty so shutdown delay/timer until "write" puts data in
      noTXInterrupts();
    
    }

  // Send  start bit for new byte
  } else if (txBitCount == 10) {
    REVOMINIGPIO::_write(_tx_pin, 0);
    interrupts();

//    txTimingCount = ((uint16_t)(timerSerialDEV->regs).gen->TX_CCR) + bitPeriod;
    //timerSerial.setCompare(TX_TIMER_CHANNEL, ((uint16_t)(timerSerialDEV->regs).gen->TX_CCR) + bitPeriod); 
      timer_set_compare(dev, TX_TIMER_CHANNEL, (uint16_t)(dev->regs->TX_CCR) + bitPeriod); 
    
    txBitCount = 0;                    
  }
  
}


// Start Bit Receive ISR
void UART_softDriver::onRXPinChange(void){ // ISR

  // Test if this is really the start bit and not a spurious edge
  if ((rxBitCount == 9) && activeRX) {  

    // Receive Timer/delay interrupt should be off now - unmask it and center the sampling time
//    rxTimingCount = ((uint16_t)(timerSerialDEV->regs).adv->CCR4 + startBitPeriod);
    //timerSerial.setCompare(RX_TIMER_CHANNEL, ((uint16_t)(timerSerialDEV->regs.gen->CNT) + startBitPeriod)); 
      timer_set_compare(dev, RX_TIMER_CHANNEL, ((uint16_t)(dev->regs->CNT) + startBitPeriod)); 
    
    rxInterruptsClr();
    
    // Mask pinchange interrupt to reduce needless interrupt
    //      overhead while receiving this byte
    noRXStartInterrupts();
    interrupts();

    // Set state/bit to first bit
    rxBitCount = 0;

  } else
    interrupts();
}


// Receive next bit. Called by timer ch2 interrupt
void UART_softDriver::rxNextBit(TIM_TypeDef *tim) { // ISR

  if (rxBitCount < 8) {

    //timerSerial.setCompare(RX_TIMER_CHANNEL, ((uint16_t)(timerSerialDEV->regs.gen->RX_CCR) + bitPeriod)); 
      timer_set_compare(dev, RX_TIMER_CHANNEL, ((uint16_t)(dev->regs->RX_CCR) + bitPeriod)); 

    receiveBuffer[receiveBufferWrite] >>= 1;  
    if (REVOMINIGPIO::_read(_rx_pin)) 
      receiveBuffer[receiveBufferWrite] |= 0x80;

#if DEBUG_DELAY
    REVOMINIGPIO::_write(DEBUG_PIN,1);
    REVOMINIGPIO::_write(DEBUG_PIN,0);
#endif
    
    interrupts();
      
     rxBitCount++;  

  // State 8 - Save incoming byte and update buffer
  } else if (rxBitCount == 8) {

    // Finish out stop bit while we...  
    //  Calculate location in buffer for next incoming byte
    //  Test if buffer full
    //  If the buffer isn't full update the tail pointer to point to next location
    //  Else if it is now full set the buffer overflow flag 
    // FYI - With this logic we effectively only have an (SS_MAX_RX_BUFF - 1) buffer size
    
    interrupts();
    uint8_t next = (receiveBufferWrite + 1) % SS_MAX_RX_BUFF;
    if (next != receiveBufferRead) {
      receiveBufferWrite = next;

    } else {
      bufferOverflow = true;

      
      #if DEBUG_DELAY
      overFlowTail = receiveBufferWrite;
      overFlowHead = receiveBufferRead;

      REVOMINIGPIO::_write(DEBUG_PIN1, 1);
      REVOMINIGPIO::_write(DEBUG_PIN1, 0);
      #endif
 
    }
    
    // Re-enable start bit detection
    rxStartInterrupts();
        
    // Shutdown nextbit timer interrupt until next start bit detected
    noRXInterrupts();

    // Set for state 9 to receive next byte
    rxBitCount = 9;
 
  } else {
    interrupts();
  }
  
}


/******************************************************************************
* Begin - Instance setup
******************************************************************************/
void UART_softDriver::begin(uint32_t tBaud) {
  
  REVOMINIGPIO::_write(_tx_pin, 1);
  REVOMINIGPIO::_pinMode(_rx_pin, INPUT_PULLUP);
  REVOMINIGPIO::_pinMode(_tx_pin, OUTPUT);

#if DEBUG_DELAY
  REVOMINIGPIO::_pinMode(DEBUG_PIN, OUTPUT);
  REVOMINIGPIO::_write(DEBUG_PIN, 0);
  REVOMINIGPIO::_pinMode(DEBUG_PIN1, OUTPUT);
  REVOMINIGPIO::_write(DEBUG_PIN1, 0);
#endif

  // Initialize the timer
  noInterrupts();
  
  timer_pause(dev); //timerSerial.pause();

  if (tBaud > 2400) {
    bitPeriod = (uint16_t)((uint32_t)(CYCLES_PER_MICROSECOND * 1000000) / tBaud);
    startBitPeriod = bitPeriod + (bitPeriod / 2) - 300;
    //timerSerial.setPrescaleFactor(1);
    timer_set_prescaler(dev,1-1);
  } else if (tBaud > 300) {
    bitPeriod = (uint16_t)(((uint32_t)(CYCLES_PER_MICROSECOND * 1000000) / 16) / tBaud);
    startBitPeriod = bitPeriod + (bitPeriod / 2);
    //timerSerial.setPrescaleFactor(16); 
    timer_set_prescaler(dev,16-1); 
  } else {
    bitPeriod = (uint16_t)(((uint32_t)(CYCLES_PER_MICROSECOND * 1000000) / 16) / tBaud) / 2;
    bitPeriod -= 600;
    startBitPeriod = bitPeriod + (bitPeriod / 2);
    //timerSerial.setPrescaleFactor(16);     
    timer_set_prescaler(dev,16-1);     
  }

//  timerSerial.setOverflow(TIMER_MAX_COUNT); 
  timer_set_reload(dev, TIMER_MAX_COUNT);

  // Set transmit bit timer  
  // Compare value set later
  //timerSerial.setMode(TX_TIMER_CHANNEL, TIMER_OUTPUT_COMPARE);
  timer_set_mode(dev,TX_TIMER_CHANNEL, TIMER_OUTPUT_COMPARE);

  // State tx machine start state, attach bit interrupt, and mask it until a byte is sent
  transmitBufferRead = transmitBufferWrite = 0;
  txBitCount = 9;  
  //timerSerial.attachInterrupt(TX_TIMER_CHANNEL, handleTXBitInterruptP[rxtxTimer - 1]);
  timer_attach_interrupt(dev, TX_TIMER_CHANNEL, makeHandle( &UART_softDriver::txNextBit ), 0);
  noTXInterrupts();
  
  // Set receive bit timer  
  // Compare value set later
  //timerSerial.setMode(RX_TIMER_CHANNEL, TIMER_OUTPUT_COMPARE);
  timer_set_mode(dev,RX_TIMER_CHANNEL, TIMER_OUTPUT_COMPARE); // capture mode by hands

  // Set rx State machine start state, attach the bit interrupt and mask it until start bit is received
  receiveBufferRead = receiveBufferWrite = 0;
  rxBitCount = 9;
  //timerSerial.attachInterrupt(RX_TIMER_CHANNEL, handleRXBitInterruptP[rxtxTimer - 1]);
  timer_attach_interrupt(dev, TX_TIMER_CHANNEL, makeHandle( &UART_softDriver::rxNextBit ), 0);

  noRXInterrupts();
      
  // Make the timer we are using a high priority interrupt
  setTimerIntPriority(dev, 0);

  // Load the timer values and start it
  timer_generate_update(dev); //timerSerial.refresh();     
  timer_resume(dev);  //timerSerial.resume();

  // Set start bit interrupt and priority and leave it enabled to rx first byte
  
//  attachInterrupt(_rx_pin, handleRXEdgeInterruptP[rxtxTimer - 1], FALLING);
  attachInterrupt(_rx_pin, makeHandle( &UART_softDriver::onRXPinChange ), FALLING);
  setEXTIntPriority(_rx_pin, 0);

  bufferOverflow = false;
  receiveBufferRead = receiveBufferWrite = 0;
  transmitBufferRead = transmitBufferWrite = 0;

  interrupts();
  
  listen();
  talk();

}


/******************************************************************************
* RX Related Public Methods
******************************************************************************/
// Sets current instance listening. Transmit is always enabled
// If his instance was already activeRX does nothing and returns false 
bool UART_softDriver::listen() {

  // If receive not activeRX then re-init and set activeRX
  if (!activeRX) {

    // Reset receieve buffer and mark activeRX
    bufferOverflow = false;
    receiveBufferRead = receiveBufferWrite = 0;
    activeRX = true;

    // Turn the receive start bit detection on
    rxStartInterrupts();
      
    return true;
  }
  return false;
}


// Stop Listening - Shuts down only RX - Use end() to stop both rx and tx
// Returns true if was listening when called
// This instance will stop all RX interrupts after current in-process
// byte is finished receiving (if any).
// If no in-process receive byte it stops immediately
bool UART_softDriver::stopListening() {

  if (activeRX) {
    
    noRXStartInterrupts();
    activeRX = false;
    return true;

  } else
    return false;
    
}


// Completely shutsdown this instance
// Not an RX related method but needs to be after stopListening
void UART_softDriver::end() {

  stopListening();
  timer_pause(dev);//timerSerial.pause();
  detachInterrupt(_rx_pin);   
  timer_detach_interrupt(dev,RX_TIMER_CHANNEL);
  timer_detach_interrupt(dev,TX_TIMER_CHANNEL);
  timer_set_mode(dev,TX_TIMER_CHANNEL, TIMER_DISABLED); 
  timer_set_mode(dev,RX_TIMER_CHANNEL, TIMER_DISABLED); 
  REVOMINIGPIO::_write(_tx_pin, 1);
}


// Returns number of bytes in the RX buffer
uint32_t UART_softDriver::available() { 
  int i;

  if (!activeRX)
    return 0;

  noRXInterrupts();
  i = (receiveBufferWrite + SS_MAX_RX_BUFF - receiveBufferRead) % SS_MAX_RX_BUFF;
  rxInterrupts();

  return i;
}

// Non-blocking read.
// Returns -1 if this instance isn't listening or the buffer is empty
/*int UART_softDriver::readnb() { 
  
  if (!activeRX)
    return -1;
    
  if (receiveBufferRead == receiveBufferWrite)
    return -1;

  uint8_t inData = receiveBuffer[receiveBufferRead];  

  noRXInterrupts();
  receiveBufferRead = (receiveBufferRead + 1) % SS_MAX_RX_BUFF;
  rxInterrupts();
  
  return inData;
}
*/

// Blocking read to be compatible with HardwareSerial
// Blocks until byte is available in buffer
// Returns -1 if instance is not activeRX
int16_t UART_softDriver::read() { 
  
  if (!activeRX)
    return -1;
    
  // Wait if buffer is empty
  while (receiveBufferRead == receiveBufferWrite);

  uint8_t inData = receiveBuffer[receiveBufferRead];  

  noRXInterrupts();
  receiveBufferRead = (receiveBufferRead + 1) % SS_MAX_RX_BUFF;
  rxInterrupts();
  
  return inData;
}


// Flush the receive buffer
void UART_softDriver::flush() {
  
  noRXInterrupts();
  receiveBufferRead = receiveBufferWrite = 0;
  rxInterrupts();

}


// Return the next item in the receive buffer but leave in buffer
/*int UART_softDriver::peek() {

  if (!activeRX)
    return -1;

  // If buffer is empty return false
  if (receiveBufferRead == receiveBufferWrite)
    return -1;

  // Otherwise read the byte at head of buffer but don't delete
  return receiveBuffer[receiveBufferRead];

}
*/

/******************************************************************************
* TX Related Public Method(s)
******************************************************************************/
// Sets current instance enabled for sending
// If his instance was already activeRX does nothing and returns false 
bool UART_softDriver::talk() {

  // If transmit not active then re-init and set activeTX
  if (!activeTX) {

    // Reset transmit buffer and mark active
    transmitBufferRead = transmitBufferWrite = 0;
    activeTX = true;

    // Turn transmit interrupts on
    txInterrupts();
      
    return true;
  }
  return false;
}


// Stop Sending - Shuts down only TX - Use end() to stop both rx and tx
// or "stopListening" for rx
// Returns true if sending already enabled when called
// This instance will stop sending at end of current byte immediately 
bool UART_softDriver::stopTalking() {

  if (activeTX) {

    while (txBitCount < 8);
    activeTX = false;
    noTXInterrupts();
    return true;

  } else
    return false;

}


#endif // CONFIG_HAL_BOARD

 


