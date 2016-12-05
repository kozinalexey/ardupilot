/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL_REVOMINI/AP_HAL_REVOMINI.h>
#include "Semaphores.h"
#include "Scheduler.h"

using namespace REVOMINI;

extern const AP_HAL::HAL& hal;

#ifdef SEM_PROF 
uint64_t Semaphore::sem_time=0;    
#endif

// Constructor
Semaphore::Semaphore() : _taken(false) {}


bool Semaphore::give() {
    bool result = false;
    if (_taken) {
        _taken = false;
        result=true;
    }
    return result;
}

bool Semaphore::take(uint32_t timeout_ms) {
    if (REVOMINIScheduler::_in_timerprocess()) {
        if(timeout_ms) {
            AP_HAL::panic("PANIC: Semaphore::take used from inside timer process");
            return false; /* Never reached - panic does not return */
        } else
            return _take_nonblocking();
    }
    return _take_from_mainloop(timeout_ms);
}

bool Semaphore::take_nonblocking() {
    if (REVOMINIScheduler::_in_timerprocess()) {
        return _take_nonblocking();
    } else {
        return _take_from_mainloop(0);
    }
}

bool Semaphore::_take_from_mainloop(uint32_t timeout_ms) {
    /* Try to take immediately */
    if (_take_nonblocking()) {
        return true;
    } else if (timeout_ms == 0) {
        /* Return immediately if timeout is 0 */
        return false;
    }

    uint16_t timeout_ticks = timeout_ms*100;
    do {
        /* Delay 10us until we can successfully take, or we timed out */
        REVOMINIScheduler::_delay_microseconds(10);
#ifdef SEM_PROF 
        sem_time+=10; // calculate semaphore wait time
#endif
        timeout_ticks--;
        if (_take_nonblocking()) {
            return true;
        }
    } while(timeout_ticks > 0);

    return false;
}

bool Semaphore::_take_nonblocking() {
    bool result = false;
    noInterrupts();
    if (!_taken) {
        _taken = true;
        result = true;
    }
    interrupts();
    return result;
}

