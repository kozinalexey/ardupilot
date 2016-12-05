
#ifndef __AP_HAL_REVOMINI_SCHEDULER_H__
#define __AP_HAL_REVOMINI_SCHEDULER_H__

#include <AP_HAL_REVOMINI/AP_HAL_REVOMINI.h>
#include <delay.h>
#include <systick.h>
#include <boards.h>
#include <timer.h>
#include <AP_HAL/AP_HAL.h>

#define REVOMINI_SCHEDULER_MAX_TIMER_PROCS 10

#define REVOMINI_SCHEDULER_MAX_SHEDULED_PROCS 32


typedef enum {
    SCHED_CANCEL=0,
    SCHED_OK=1,
    SCHED_RESCHEDULE=2,
} SchedState;


#define SHED_PROF

typedef struct RevoTimer {
    uint32_t period;
    uint32_t time;
    uint64_t proc;          //AP_HAL::Device::PeriodicCb proc and AP_HAL::MemberProc mp together
    REVOMINI::Semaphore *sem;
    uint8_t mode;
#ifdef SHED_PROF
    uint32_t micros;
    uint32_t count;
    uint64_t fulltime;
#endif
} revo_timer;

union Revo_cb { // кровь кишки ассемблер :) преобразование функторов в унифицированный вид
    AP_HAL::MemberProc mp;
    AP_HAL::Device::PeriodicCb pcb;
    uint64_t h; // treat as handle
    uint32_t w[2]; // words, to check
};

class REVOMINI::REVOMINIScheduler : public AP_HAL::Scheduler {
public:
    REVOMINIScheduler();
    void     init();
    void     delay(uint16_t ms) { _delay(ms); }
    void     delay_microseconds(uint16_t us) { _delay_microseconds(us); }
    
    inline uint32_t millis() {    return systick_uptime(); }
    inline uint32_t micros() {    return systick_micros(); }
    
    void     register_delay_callback(AP_HAL::Proc, uint16_t min_time_ms);
    void     register_timer_process(AP_HAL::MemberProc);
    void     register_io_process(AP_HAL::MemberProc);
    void     suspend_timer_procs();
    void     resume_timer_procs();

    inline bool in_timerprocess() {   return _in_timer_proc; }

    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us);

    void     system_initialized();

    void     reboot(bool hold_in_bootloader);

// drivers are not the best place for its own sheduler
    static AP_HAL::Device::PeriodicHandle register_timer_task(uint32_t period_us, AP_HAL::Device::PeriodicCb proc, REVOMINI::Semaphore *sem) {
        Revo_cb r = { .pcb=proc };
        return _register_timer_task(period_us, r.h, sem, 0);
    }

    static void _delay(uint16_t ms);
    static void _delay_microseconds(uint16_t us);
    static inline bool _in_timerprocess() {   return _in_timer_proc; }
    
    static bool           adjust_timer_task(AP_HAL::Device::PeriodicHandle h, uint32_t period_us);
    static bool           unregister_timer_task(AP_HAL::Device::PeriodicHandle h);
    void                  loop();      // to add ability to print out scheduler's stats in main thread

//    bool                  _run_1khz_procs();
    
private:

    static AP_HAL::Device::PeriodicHandle _register_timer_task(uint32_t period_us, uint64_t proc, REVOMINI::Semaphore *sem, uint8_t mode=0);

    static volatile bool _in_timer_proc;

    static AP_HAL::Proc _delay_cb;
    static uint16_t _min_delay_cb_ms;
    static bool _initialized;

    /* _timer_isr_event() and _run_timer_procs are static so they can be
     * called from an interrupt. */
    static void _timer_isr_event(TIM_TypeDef *tim);
    static void _run_timer_procs(bool called_from_isr);

    static AP_HAL::Proc _failsafe;

    static volatile bool _timer_suspended;
    static volatile bool _timer_event_missed;
//    static AP_HAL::MemberProc _timer_proc[REVOMINI_SCHEDULER_MAX_TIMER_PROCS];
//    static uint8_t _num_timer_procs;
    static uint32_t _scheduler_last_call;
    static uint32_t _armed_last_call;
    static uint16_t _scheduler_led;

    static revo_timer _timers[REVOMINI_SCHEDULER_MAX_SHEDULED_PROCS];
    static uint8_t    _num_timers;

    static void _run_timers(void);

    
#ifdef SHED_PROF
    static uint64_t shed_time;
    static uint64_t task_time;
    static bool flag_10s;
    
    static uint64_t delay_time;
    static uint64_t delay_int_time;
    
    bool _set_10s_flag();
#endif
};

#endif // __AP_HAL_REVOMINI_SCHEDULER_H__
