
#ifndef __AP_HAL_REVOMINI_SEMAPHORES_H__
#define __AP_HAL_REVOMINI_SEMAPHORES_H__

#include <AP_HAL_REVOMINI/AP_HAL_REVOMINI.h>

#define SEM_PROF 

class REVOMINI::Semaphore : public AP_HAL::Semaphore {
public:
    Semaphore();
    bool give();
    bool take(uint32_t timeout_ms);
    bool take_nonblocking();

#ifdef SEM_PROF 
    static uint64_t sem_time;    
#endif

protected:
    bool _take_from_mainloop(uint32_t timeout_ms);
    bool _take_nonblocking();

    volatile bool _taken;


};

#endif // __AP_HAL_REVOMINI_SEMAPHORES_H__
