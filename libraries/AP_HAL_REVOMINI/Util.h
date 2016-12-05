
#ifndef __AP_HAL_REVOMINI_UTIL_H__
#define __AP_HAL_REVOMINI_UTIL_H__

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_REVOMINI_Namespace.h"

class REVOMINI::REVOMINIUtil : public AP_HAL::Util {
public:
    bool run_debug_shell(AP_HAL::BetterStream *stream) { return false; }
    
    uint64_t get_system_clock_ms() const {
        return AP_HAL::millis();
    }

    uint32_t available_memory(void) override
    {
        return 128*1024;
    }
    
    // create a new semaphore
    Semaphore *new_semaphore(void)  override { return new REVOMINI::Semaphore; } 
};

#endif // __AP_HAL_REVOMINI_UTIL_H__
