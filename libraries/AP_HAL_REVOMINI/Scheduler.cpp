
#include "Scheduler.h"
#include "Semaphores.h"
#include <delay.h>
#include <timer.h>
#include <systick.h>
#include <AP_Notify/AP_Notify.h>
#include "GPIO.h"

using namespace REVOMINI;

extern const AP_HAL::HAL& hal;

AP_HAL::Proc REVOMINIScheduler::_failsafe = NULL;
volatile bool REVOMINIScheduler::_timer_suspended = false;
volatile bool REVOMINIScheduler::_timer_event_missed = false;
volatile bool REVOMINIScheduler::_in_timer_proc = false;
//AP_HAL::MemberProc REVOMINIScheduler::_timer_proc[REVOMINI_SCHEDULER_MAX_TIMER_PROCS] IN_CCM;
//uint8_t REVOMINIScheduler::_num_timer_procs = 0;
uint32_t REVOMINIScheduler::_scheduler_last_call = 0;
uint32_t REVOMINIScheduler::_armed_last_call = 0;
uint16_t REVOMINIScheduler::_scheduler_led = 0;

revo_timer REVOMINIScheduler::_timers[REVOMINI_SCHEDULER_MAX_SHEDULED_PROCS] IN_CCM;
uint8_t    REVOMINIScheduler::_num_timers = 0;

AP_HAL::Proc REVOMINIScheduler::_delay_cb=NULL;
uint16_t REVOMINIScheduler::_min_delay_cb_ms=65535;
bool REVOMINIScheduler::_initialized=false;

#ifdef SHED_PROF
uint64_t REVOMINIScheduler::shed_time = 0;
bool     REVOMINIScheduler::flag_10s = false;
uint64_t REVOMINIScheduler::task_time = 0;
uint64_t REVOMINIScheduler::delay_time = 0;
uint64_t REVOMINIScheduler::delay_int_time = 0;
#endif

REVOMINIScheduler::REVOMINIScheduler()
{}


#define SHED_FREQ 8000 // in Hz

void REVOMINIScheduler::init()
{
    uint32_t period = (2000000UL / SHED_FREQ) - 1; 
    uint32_t prescaler =  (uint16_t) ((SystemCoreClock /2) / 2000000UL) - 1; //2MHz 0.5us ticks

    timer_pause(TIMER7);
    timer_set_prescaler(TIMER7,prescaler);
    timer_set_count(TIMER7,0);
    timer_set_reload(TIMER7,period);
    
//    memset(_timer_proc, 0, sizeof(_timer_proc) );
    memset(_timers,     0, sizeof(_timers) );
    
    timer_attach_interrupt(TIMER7, TIMER_UPDATE_INTERRUPT, _timer_isr_event, 7); // low priority
//    NVIC_SetPriority(TIM7_IRQn,5); priority in the above call
    timer_resume(TIMER7);

    //systick_attach_callback(_timer_isr_event); // 1kHz is too slow :(

    // run standard Ardupilot tasks on 1kHz 
//    register_timer_task(1000, FUNCTOR_BIND_MEMBER(&REVOMINIScheduler::_run_1khz_procs, bool), NULL); now in the same scheduler
    
#ifdef SHED_PROF
// set flag for stats output each 10 seconds
    register_timer_task(10000000, FUNCTOR_BIND_MEMBER(&REVOMINIScheduler::_set_10s_flag, bool), NULL);
#endif
}

void REVOMINIScheduler::_delay(uint16_t ms)
{
    uint32_t start = systick_micros();
    
    while (ms > 0) {
        while ((systick_micros() - start) >= 1000) {
            ms--;
            if (ms == 0) break;
            start += 1000;
        }
        if (_min_delay_cb_ms <= ms) {
            if (_delay_cb) {
                _delay_cb();
            }
        }
    }
}


void REVOMINIScheduler::_delay_microseconds(uint16_t us)
{
    stopwatch_delay_us((uint32_t)us); // it not a stopwatch anymore - @NG

#ifdef SHED_PROF
    if(_in_timer_proc)
        delay_int_time +=us;
    else
        delay_time     +=us;
#endif

}

void REVOMINIScheduler::register_delay_callback(AP_HAL::Proc proc, uint16_t min_time_ms)
{
    _delay_cb        = proc;
    _min_delay_cb_ms = min_time_ms;
}

void REVOMINIScheduler::register_timer_process(AP_HAL::MemberProc proc)
{

    Revo_cb r = { .mp=proc };

//    r.mp=proc;
    _register_timer_task(1000, r.h, NULL, 1);
}


void REVOMINIScheduler::register_io_process(AP_HAL::MemberProc proc)
{
    // IO processes not supported
}

void REVOMINIScheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us) {
    /* XXX Assert period_us == 1000 */
    _failsafe = failsafe;
}
void REVOMINIScheduler::suspend_timer_procs()
{
    _timer_suspended = true;
}


void REVOMINIScheduler::resume_timer_procs()
{
    _timer_suspended = false;
    if (_timer_event_missed == true) {
        _run_timer_procs(false);        // TODO here code executes on main thread, not in interrupt level!
        _timer_event_missed = false;
    }
}


void REVOMINIScheduler::_run_timer_procs(bool called_from_isr) {

    if (_in_timer_proc) {
        return;
    }
    _in_timer_proc = true;

    if (!_timer_suspended) {
        _run_timers(); 
    } else if (called_from_isr) {
        _timer_event_missed = true;
    }

    // and the failsafe, if one is setup
    if (_failsafe) {
        _failsafe();
    }

    _in_timer_proc = false;
}

void REVOMINIScheduler::_timer_isr_event(TIM_TypeDef *tim) {
    _run_timer_procs(true);
}

// PX4 writes as
// *(uint32_t *)0x40002850 = 0xb007b007;
#define BOOT_RTC_SIGNATURE	0xb007b007

static void
board_set_rtc_signature(uint32_t sig)
{
        // enable the backup registers.
        PWR->CR   |= PWR_CR_DBP;
        RCC->BDCR |= RCC_BDCR_RTCEN;

        RTC_WriteBackupRegister(0, sig);
 
        // disable the backup registers
//        RCC->BDCR &= RCC_BDCR_RTCEN;
        PWR->CR   &= ~PWR_CR_DBP;
}




void REVOMINIScheduler::system_initialized()
{
    if (_initialized) {
        AP_HAL::panic("PANIC: scheduler::system_initialized called more than once");
    }
    _initialized = true;
    
    board_set_rtc_signature(0); // clear bootloader flag after init done
}


void REVOMINIScheduler::reboot(bool hold_in_bootloader) {
    hal.console->println("GOING DOWN FOR A REBOOT\r\n");

    if(hold_in_bootloader)
        board_set_rtc_signature(BOOT_RTC_SIGNATURE);

    _delay(100);

    NVIC_SystemReset();

    return;
}

void REVOMINIScheduler::loop(){    // executes in main thread
#ifdef SHED_PROF
    if(flag_10s) {
        flag_10s=false;
        uint32_t t=millis();
        const int Kf=100;
        
        float eff= (task_time)/(float)(task_time+shed_time);
        
        static float shed_eff=0;
    
        if(shed_eff==0.0) shed_eff = eff;
        else              shed_eff = shed_eff*(1 - 1/Kf) + eff*(1/Kf);

        hal.console->printf("\nScheduler stats:\n  %% of full time: %5.2f  Efficiency %5.3f \n", (task_time/10.0)/t /* in percent*/ , shed_eff );
        hal.console->printf("delay times: in main %5.2f including in semaphore %5.2f  in timer %5.2f in isr %5.2f \nTask times:\n", (delay_time/10.0)/t, (Semaphore::sem_time/10.0)/t,  (delay_int_time/10.0)/t, (isr_time/10.0)/t );

        for(int i=0; i< _num_timers; i++) {
            if(_timers[i].proc){    // task not cancelled?
                hal.console->printf("task 0x%llX tim %8.1f int %5.3f%% tot %6.4f%% mean time %5.1f\n", _timers[i].proc, _timers[i].fulltime/1000.0, _timers[i].fulltime*100.0 / task_time, (_timers[i].fulltime / 10.0) / t, (float)_timers[i].fulltime/_timers[i].count  );
            }
        }
    }
    
#endif
}


#ifdef SHED_PROF
bool REVOMINIScheduler::_set_10s_flag(){
    flag_10s=true;
    return true;
}
#endif

/*
    common realization of all Device.PeriodicCallback;
[
*/
AP_HAL::Device::PeriodicHandle REVOMINIScheduler::_register_timer_task(uint32_t period_us, uint64_t proc, REVOMINI::Semaphore *sem, uint8_t mode){
    int i;
    
    for (i = 0; i < _num_timers; i++) {
        if ( _timers[i].proc == 0L /* free slot */ ) {
            goto store;        

        } else if (_timers[i].proc == proc /* the same */ ) {
            _timers[i].proc = 0L; // clear proc - temporary disable task
            goto store;
        }
    }

    if (_num_timers < REVOMINI_SCHEDULER_MAX_SHEDULED_PROCS) {
        /* this write to _timers[] can be outside the critical section
         * because that memory won't be used until _num_timers is
         * incremented or where proc is NULL. */
         
        i = _num_timers;
        _timers[i].proc = 0L; // clear proc - this entry will be skipped
        _num_timers++; // now nulled proc guards us
store:        
        _timers[i].period = period_us;
        _timers[i].time = period_us;
        _timers[i].sem = sem;
        _timers[i].mode = mode;
#ifdef SHED_PROF
        _timers[i].count = 0;
        _timers[i].micros = 0;
        _timers[i].fulltime = 0;
#endif
        noInterrupts();            // 64-bits should be 
        _timers[i].proc = proc;    //     last one, not interferes - guard is over
        interrupts();
        return (AP_HAL::Device::PeriodicHandle)&_timers[i];
    }

    return NULL;
}


bool REVOMINIScheduler::adjust_timer_task(AP_HAL::Device::PeriodicHandle h, uint32_t period_us)
{
    revo_timer *p = (revo_timer *)h;
    p->period = period_us;
    
    return true;
}
bool REVOMINIScheduler::unregister_timer_task(AP_HAL::Device::PeriodicHandle h)
{
    revo_timer *p = (revo_timer *)h;
    noInterrupts(); // 64-bits should be 
    p->proc=0L;
    interrupts();
    return true;
}

#define TIMER_PERIOD (1000000 / SHED_FREQ)  //250 // interrupts period in uS

void REVOMINIScheduler::_run_timers(){
#ifdef SHED_PROF
    uint32_t full_t = systick_micros();
    uint32_t job_t = 0;
#endif                

    for(int i = 0; i<_num_timers; i++){
        if(_timers[i].proc){    // task not cancelled?
            if(_timers[i].time < TIMER_PERIOD) { // time to run?
                if(_timers[i].sem && !_timers[i].sem->take_nonblocking()) { // semaphore active? take!
                    // can't get semaphore, just do nothing - will try next time
                    continue;
                }
#ifdef SHED_PROF
                uint32_t t = systick_micros();
#endif          
                bool ret=false;
                Revo_cb r = { .h=_timers[i].proc }; // don't touch it without hardware debugger!
                switch(_timers[i].mode){
                case 0:
                    ret = (r.pcb)();       // call task
                    break;
                case 1:
                    (r.mp)();              // call task
                    ret=1;
                    break;
                }
#ifdef SHED_PROF
                t = systick_micros() - t;               // work time
#endif                
                if(_timers[i].sem) _timers[i].sem->give(); //  semaphore active? give back ASAP!
#ifdef SHED_PROF
                _timers[i].micros    =  t;          // last time
                _timers[i].count     += 1;          // number of calls
                _timers[i].fulltime  += t;          // full time, mean time = full / count
                job_t += t;                  // time of all jobs
#endif                
                if(ret)
                    _timers[i].time = _timers[i].period;  // reshedule
                else
                    _timers[i].proc = 0L;               // cancel task
            } else
                _timers[i].time -= TIMER_PERIOD; //     just count time
        }
    }

#ifdef SHED_PROF
    full_t = systick_micros() - full_t;         // full time of scheduler
    uint32_t shed_t = full_t - job_t;   // net time

    task_time += job_t; // full time in tasks
    shed_time += shed_t;
#endif                

}

// ]

