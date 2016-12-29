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


    uint32_t t=systick_micros(); 
    uint32_t to = t + timeout_ms*1000; // timeout time

    do {
        REVOMINIScheduler::yield(100); // 100uS max task time - this is more useful  // REVOMINIScheduler::_delay_microseconds(10);
        if (_take_nonblocking()) {
#ifdef SEM_PROF 
            sem_time += systick_micros()-t; // calculate semaphore wait time
#endif
            return true;
        }
    } while(systick_micros() < to);

#ifdef SEM_PROF 
    sem_time += systick_micros()-t; // calculate semaphore wait time
#endif

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

