/*
 * SerialDriver.cpp --- AP_HAL_REVOMINI SoftSerial driver.
 *
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_REVOMINI
#include "UART_SoftDriver.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>

#include <gpio_hal.h>


using namespace REVOMINI;

bool SerialDriver::_initialized=false;
bool SerialDriver::_inverse=false;
bool SerialDriver::_blocking=true;

uint16_t                SerialDriver::bitPeriod;

/*
volatile uint16_t       SerialDriver::rxTimingCount;
volatile uint16_t       SerialDriver::txTimingCount;
volatile uint8_t        SerialDriver::bufferOverflow;
*/

#ifdef SS_DEBUG
    static volatile uint8_t          SerialDriver::bufferOverflow;
#endif

volatile int8_t         SerialDriver::rxBitCount;
volatile uint16_t       SerialDriver::receiveBufferWrite;
volatile uint16_t       SerialDriver::receiveBufferRead;
volatile uint8_t        SerialDriver::receiveBuffer[SSI_RX_BUFF_SIZE];
uint8_t                 SerialDriver::receiveByte;

volatile int8_t         SerialDriver::txBitCount;
volatile uint16_t       SerialDriver::transmitBufferWrite;
volatile uint16_t       SerialDriver::transmitBufferRead;
volatile uint8_t        SerialDriver::transmitBuffer[SSI_TX_BUFF_SIZE];

bool                    SerialDriver::txSkip=false;
bool                    SerialDriver::rxSkip=false;
bool                    SerialDriver::activeRX=false;
bool                    SerialDriver::activeTX=false;

const struct TIM_Channel *SerialDriver::channel;

void SerialDriver::begin(uint32_t baud) {
    REVOMINIGPIO::_write(TX_PIN, _inverse?LOW:HIGH);
    REVOMINIGPIO::_pinMode(RX_PIN, INPUT_PULLUP);
    REVOMINIGPIO::_pinMode(TX_PIN, OUTPUT);

    channel = &PWM_Channels[5]; // setup like in PWM capture - on this pin

    
    timer_pause(channel->timer);
    uint32_t prescaler;
    
    if (baud > 2400) {
        bitPeriod = (uint16_t)((uint32_t)(CYCLES_PER_MICROSECOND * 1000000) / baud);
        prescaler=1;
    } else {
        bitPeriod = (uint16_t)(((uint32_t)(CYCLES_PER_MICROSECOND * 1000000) / 16) / baud);
        prescaler=16;
    }

//    attachTimer8CaptureCallback( timer_handler); 


    timer_set_prescaler(channel->timer, prescaler-1);
    
    timer_set_reload(channel->timer, bitPeriod/2); // for TX needs
        
    transmitBufferRead = transmitBufferWrite = 0;
    txBitCount = 9;

    // Set rx State machine start state, attach the bit interrupt and mask it until start bit is received
    receiveBufferRead = receiveBufferWrite = 0;
    rxBitCount = 9;

    rxSetCapture();

    timer_attach_interrupt(channel->timer, TIMER_RX_INTERRUPT,     rxNextBit, 0 );
    timer_attach_interrupt(channel->timer, TIMER_UPDATE_INTERRUPT, txNextBit, 0 ); // also enables interrupt
    // so disable it
    txDisableInterrupts();
    
    // Load the timer values and start it
    timer_generate_update(channel->timer);
    timer_resume(channel->timer);
    
    _initialized = true;
}


void SerialDriver::rxSetCapture(){
    TIM_ICInitTypeDef TIM_ICInitStructure;

    // input capture ************************************************************/
    TIM_ICInitStructure.TIM_Channel     = channel->tim_channel;
    TIM_ICInitStructure.TIM_ICPolarity  = _inverse?TIM_ICPolarity_Rising:TIM_ICPolarity_Falling; // wait for start bit
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter    = 0x3;
    TIM_ICInit(channel->tim, &TIM_ICInitStructure);
}

void SerialDriver::rxSetCompare(){
    timer_set_mode(channel->timer, channel->channel_n, TIMER_OUTPUT_COMPARE); // for RX needs, capture mode by hands
}



void SerialDriver::begin(uint32_t baud, uint16_t rxS, uint16_t txS) {
    begin(baud);
}

void SerialDriver::end() {
    timer_pause(channel->timer);
    REVOMINIGPIO::_write(TX_PIN, 1);
    _initialized = false;

}

void SerialDriver::flush() {
    receiveBufferRead = receiveBufferWrite = 0;
}

void SerialDriver::set_blocking_writes(bool blocking) {
    _blocking=blocking;
}

bool SerialDriver::tx_pending() {
    if(!_initialized) return 0;
    
    return (transmitBufferWrite + SS_MAX_TX_BUFF - transmitBufferRead) % SS_MAX_TX_BUFF;
}


/* REVOMINI implementations of Stream virtual methods */
uint32_t SerialDriver::available() {
    int i;

    if(!_initialized) return 0;

    i = (receiveBufferWrite + SS_MAX_RX_BUFF - receiveBufferRead) % SS_MAX_RX_BUFF;

    return i;
}

uint32_t SerialDriver::txspace() {
    if(!_initialized) return 0;

    return SS_MAX_TX_BUFF - tx_pending();
}

int16_t SerialDriver::read() {
    if (!_initialized)
        return -1;
  
    // Wait if buffer is empty
    if(receiveBufferRead == receiveBufferWrite) return -1; // no data
  
    uint8_t inData = receiveBuffer[receiveBufferRead];
  
    receiveBufferRead = (receiveBufferRead + 1) % SS_MAX_RX_BUFF;

    return inData;
}

/* REVOMINI implementations of Print virtual methods */
size_t SerialDriver::write(uint8_t c) {

    if (REVOMINIScheduler::_in_timerprocess()) {
        // not allowed from timers
        return 0;
    }

    // Check if transmit timer interrupt enabled and if not unmask it
    // transmit timer interrupt will get masked by transmit ISR when buffer becomes empty
    if (!activeTX) {
        activeTX=true;
        
      // Save new data in buffer
      transmitBuffer[transmitBufferWrite] = c;
      transmitBufferWrite = (transmitBufferWrite == SS_MAX_TX_BUFF) ? 0 : transmitBufferWrite + 1;

      // Set state to 10 (send start bit) and re-enable transmit interrupt
      txBitCount = 10;

      txEnableInterrupts(); // enable

    } else {

      // Blocks if buffer full
      bool i;
      do { // wait for free space
        i = (((transmitBufferWrite + 1) % SS_MAX_TX_BUFF) == transmitBufferRead);
        if(i && ! _blocking) return 0;
      } while (i);

      // Save new data in buffer and bump the write pointer
      transmitBuffer[transmitBufferWrite] = c;

      transmitBufferWrite = (transmitBufferWrite == SS_MAX_TX_BUFF) ? 0 : transmitBufferWrite + 1;

    }

    return 1;
}

size_t SerialDriver::write(const uint8_t *buffer, size_t size)
{
    size_t n = 0;
    while (size--) {
        n += write(*buffer++);
    }
    return n;
}


#define bitRead(value, bit)            (((value) >> (bit)) & 0x01)

// Transmits next bit. Called by timer update interrupt
void SerialDriver::txNextBit(TIM_TypeDef *tim) { // ISR

    txSkip= !txSkip;
    
    if(txSkip) return;


  // State 0 through 7 - transmit bits
  if (txBitCount <= 7) {
    if (bitRead(transmitBuffer[transmitBufferRead], txBitCount) == (_inverse?0:1)) {
      REVOMINIGPIO::_write(TX_PIN,HIGH); 
    } else {
      REVOMINIGPIO::_write(TX_PIN,LOW);
    }

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
    REVOMINIGPIO::_write(TX_PIN, _inverse?LOW:HIGH); 

    transmitBufferRead = (transmitBufferRead == SS_MAX_TX_BUFF ) ? 0 : transmitBufferRead + 1;

    if (transmitBufferRead != transmitBufferWrite) { // we have data do transmit
        txBitCount = 10;
    } else {
      // Buffer empty so shutdown timer until "write" puts data in
      txDisableInterrupts();
      activeTX=false;
    }

  // Send  start bit for new byte
  } else if (txBitCount >= 10) {
    REVOMINIGPIO::_write(TX_PIN, _inverse?HIGH:LOW);

    txBitCount = 0;                    
  }
  
}



// Receive next bit. Called by timer ch2 interrupt
void SerialDriver::rxNextBit(TIM_TypeDef *tim) { // ISR

    if(!activeRX) { // capture start bit


        // Test if this is really the start bit and not a spurious edge
        if (rxBitCount == 9) {  

            uint16_t pos = timer_get_capture(channel->timer, channel->channel_n);

            rxSetCompare(); // turn to compare mode
            
            timer_set_compare(channel->timer, channel->channel_n, pos); // captured value
    
            // Set state/bit to first bit
            txSkip=false;
            activeRX=true;
        }
    } else { // compare match twice per bit;
        txSkip= !txSkip;

        if(!txSkip) return; // not the middle of bit
        
        if (rxBitCount == 9) {   // check start bit again
            if (REVOMINIGPIO::_read(RX_PIN) == _inverse?HIGH:LOW) { // start OK
                rxBitCount = 0;
            } else { // false start
                activeRX=false;
                rxSetCapture(); // turn back to capture mode
            }
        } else if (rxBitCount < 8) { // get bits
            //receiveBuffer[receiveBufferWrite] >>= 1;  
            receiveByte >>= 1;  
            
            
            if (REVOMINIGPIO::_read(RX_PIN) == _inverse?LOW:HIGH) 
              receiveByte |= 0x80;

#if DEBUG_DELAY
            REVOMINIGPIO::_write(DEBUG_PIN,1);
            REVOMINIGPIO::_write(DEBUG_PIN,0);
#endif
    
      
            rxBitCount++;  

        // State 8 - Save incoming byte and update buffer
        } else if (rxBitCount == 8) {

            // Finish out stop bit while we...  
            //  Calculate location in buffer for next incoming byte
            //  Test if buffer full
            //  If the buffer isn't full update the tail pointer to point to next location
            //  Else if it is now full set the buffer overflow flag 
            // FYI - With this logic we effectively only have an (SS_MAX_RX_BUFF - 1) buffer size
            
            if (REVOMINIGPIO::_read(RX_PIN) == _inverse?LOW:HIGH) // valid STOP
                receiveBuffer[receiveBufferWrite] = receiveByte;
            
             uint8_t next = (receiveBufferWrite + 1) % SS_MAX_RX_BUFF;
             if (next != receiveBufferRead) {
                receiveBufferWrite = next;
             } 
#ifdef SS_DEBUG
             else {
                bufferOverflow = true;

      
#if DEBUG_DELAY
              overFlowTail = receiveBufferWrite;
              overFlowHead = receiveBufferRead;

              REVOMINIGPIO::_write(DEBUG_PIN1, 1);
              REVOMINIGPIO::_write(DEBUG_PIN1, 0);
#endif
            }
#endif

            // Set for state 9 to receive next byte
            rxBitCount = 9;
            activeRX=false;
            rxSetCapture(); // turn back to capture mode
 
        }
    }

}

#endif // CONFIG_HAL_BOARD