
#include "Scheduler.h"
#include "Semaphores.h"
#include <delay.h>
#include <timer.h>
#include <systick.h>
#include <AP_Notify/AP_Notify.h>
#include "GPIO.h"
#include <AP_Math/AP_Math.h>


/*

stats for Copter

Scheduler stats:
  % of full time: 24.56  Efficiency 0.957 
delay times: in main 86.09 including in semaphore  0.00  in timer  4.89 in isr  0.00 
Task times:
task 0x808FFB1200074C4 tim      0.0 int 0.000% tot 0.0000% mean time   0.0
task 0x8090BB1200074F0 tim    293.6 int 2.321% tot 0.5701% mean time   8.3
task 0x808B07D200073E8 tim      1.8 int 0.014% tot 0.0035% mean time   0.9
task 0x804206720009438 tim   1711.7 int 13.532% tot 3.3244% mean time 442.5
task 0x804403720009900 tim   1845.0 int 14.586% tot 3.5834% mean time 632.1
task 0x804AEDD200099F0 tim   8797.0 int 69.547% tot 17.0855% mean time 265.9

*/

using namespace REVOMINI;

extern const AP_HAL::HAL& hal;

AP_HAL::Proc REVOMINIScheduler::_failsafe = NULL;
volatile bool REVOMINIScheduler::_timer_suspended = false;
volatile bool REVOMINIScheduler::_timer_event_missed = false;
volatile bool REVOMINIScheduler::_in_timer_proc = false;
//AP_HAL::MemberProc REVOMINIScheduler::_timer_proc[REVOMINI_SCHEDULER_MAX_TIMER_PROCS] IN_CCM;
//uint8_t REVOMINIScheduler::_num_timer_procs = 0;

revo_timer REVOMINIScheduler::_timers[REVOMINI_SCHEDULER_MAX_SHEDULED_PROCS] IN_CCM;
uint8_t    REVOMINIScheduler::_num_timers = 0;

AP_HAL::MemberProc REVOMINIScheduler::_io_process[REVOMINI_SCHEDULER_MAX_IO_PROCS] IN_CCM;
uint8_t            REVOMINIScheduler::_num_io_proc=0;

AP_HAL::Proc REVOMINIScheduler::_delay_cb=NULL;
uint16_t REVOMINIScheduler::_min_delay_cb_ms=0;

bool REVOMINIScheduler::_initialized=false;

// Main task and run queue
REVOMINIScheduler::task_t REVOMINIScheduler::s_main = {
  &REVOMINIScheduler::s_main,
  &REVOMINIScheduler::s_main,
  { 0 },
  NULL,
  0,    // id
#ifdef MTASK_PROF
    0, // task_ticks
    0, // task_start
    0, // task_time
    0, // task_delay
#endif
};

// Reference running task
REVOMINIScheduler::task_t* REVOMINIScheduler::s_running = &REVOMINIScheduler::s_main;

// Initial top stack for task allocation
size_t REVOMINIScheduler::s_top = MAIN_STACK_SIZE;

uint16_t REVOMINIScheduler::task_n=0;

#ifdef SHED_PROF
uint64_t REVOMINIScheduler::shed_time = 0;
bool     REVOMINIScheduler::flag_10s = false;
uint64_t REVOMINIScheduler::task_time = 0;
uint64_t REVOMINIScheduler::delay_time = 0;
uint64_t REVOMINIScheduler::delay_int_time = 0;
#endif


#ifdef MTASK_PROF
 uint64_t REVOMINIScheduler::yield_time=0;
 uint32_t REVOMINIScheduler::yield_count=0;
#endif

REVOMINIScheduler::REVOMINIScheduler()
{

#ifdef MTASK_PROF
    s_main.start=systick_micros();
#endif

}


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
    memset(_io_process,     0, sizeof(_io_process));
    
    timer_attach_interrupt(TIMER7, TIMER_UPDATE_INTERRUPT, _timer_isr_event, 7); // low priority
//    NVIC_SetPriority(TIM7_IRQn,5); priority in the above call
    timer_resume(TIMER7);

    // run standard Ardupilot tasks on 1kHz 
//    register_timer_task(1000, FUNCTOR_BIND_MEMBER(&REVOMINIScheduler::_run_1khz_procs, bool), NULL); now in the same scheduler
    
    
#ifdef SHED_PROF
// set flag for stats output each 10 seconds
    register_timer_task(10000000, FUNCTOR_BIND_MEMBER(&REVOMINIScheduler::_set_10s_flag, bool), NULL);
#endif

// for debug
//    register_io_process(FUNCTOR_BIND_MEMBER(&REVOMINIScheduler::stats_proc, void) );

}

void REVOMINIScheduler::_delay(uint16_t ms)
{
    uint32_t start = systick_micros();
#ifdef SHED_PROF
    uint32_t t=start;
#endif
    
    while (ms > 0) {
        if(!_in_timer_proc && !in_interrupt())  // not switch context in interrupts
            yield(ms*1000); // time in micros
            
        while ((systick_micros() - start) >= 1000) {
            ms--;
            if (ms == 0) break;
            start += 1000;
        }
        if (_min_delay_cb_ms <= ms) { // MAVlink callback uses 5ms
            if (_delay_cb) {
                _delay_cb();
            }
        }
    }

#ifdef SHED_PROF
    uint32_t us=systick_micros()-t;
    if(_in_timer_proc)
        delay_int_time +=us;
    else
        delay_time     +=us;
#endif
}

void REVOMINIScheduler::_delay_microseconds_boost(uint16_t us){
    _delay_microseconds(us);
}

void REVOMINIScheduler::_delay_microseconds(uint16_t us)
{
#ifdef SHED_PROF
    uint32_t t = systick_micros(); 
#endif

    uint32_t rtime = stopwatch_getticks(); // start ticks
    uint32_t dt    = us_ticks * us;  // delay time in ticks

    uint32_t ny = 10 * us_ticks; // 10 uS in ticks
    uint32_t tw;

    while ((tw = stopwatch_getticks() - rtime) < dt) { // tw - time waiting, in ticks
        if((dt - tw) > ny // No Yeld time - 10uS to end of wait 
           && !_in_timer_proc && !in_interrupt()) {  // not switch context in interrupts
            yield((dt - tw) / us_ticks); // in micros
        }
    }    

#ifdef SHED_PROF
    us=systick_micros()-t; // real time
    
    if(_in_timer_proc)
        delay_int_time +=us;
    else
        delay_time     +=us;
#endif

}

void REVOMINIScheduler::register_delay_callback(AP_HAL::Proc proc, uint16_t min_time_ms)
{
    static bool init_done=false;
    if(!init_done){     // small hack to load HAL parameters in needed time

        ((HAL_REVOMINI&) hal).lateInit();
        
        init_done=true;
    }

    _delay_cb        = proc;
    _min_delay_cb_ms = min_time_ms;
}

void REVOMINIScheduler::register_timer_process(AP_HAL::MemberProc proc)
{

    Revo_cb r = { .mp=proc };

//    r.mp=proc;
    _register_timer_task(1000, r.h, NULL, 1);
}

void do_io_process();

void REVOMINIScheduler::_do_io_process(){
    for (int i = 0; i < _num_io_proc; i++) {
        if (_io_process[i]) {
            _io_process[i]();
        }
        yield(); // one in a time
    }
}

void do_io_process(){
    REVOMINIScheduler::_do_io_process();
}

void REVOMINIScheduler::register_io_process(AP_HAL::MemberProc proc)
{
    if(_num_io_proc>=REVOMINI_SCHEDULER_MAX_IO_PROCS) return;


    if(_num_io_proc==0) { // the 1st
        //        setup  loop
        _io_process[_num_io_proc]=proc;
        start_task(NULL, do_io_process);
    } else {
        for (int i = 0; i < _num_io_proc; i++) {
            if (_io_process[i] == proc) {
                return;
            }
        }
        _io_process[_num_io_proc]=proc;    
    }

    _num_io_proc++;
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

     _print_stats();
}

void REVOMINIScheduler::stats_proc(void){
//    _print_stats(); only for debug

}

void REVOMINIScheduler::_print_stats(){
#ifdef SHED_PROF
    if(flag_10s) {
        flag_10s=false;
        uint32_t t=millis();
        const int Kf=100;
        
        float eff= (task_time)/(float)(task_time+shed_time);
        
        static float shed_eff=0;
    
        if(is_zero(shed_eff)) shed_eff = eff;
        else              shed_eff = shed_eff*(1 - 1/Kf) + eff*(1/Kf);

        hal.console->printf("\nScheduler stats:\n  %% of full time: %5.2f  Efficiency %5.3f \n", (task_time/10.0)/t /* in percent*/ , shed_eff );
        hal.console->printf("delay times: in main %5.2f including in semaphore %5.2f  in timer %5.2f in isr %5.2f \nTask times:\n", (delay_time/10.0)/t, (Semaphore::sem_time/10.0)/t,  (delay_int_time/10.0)/t, (isr_time/10.0)/t );

        yield();

        for(int i=0; i< _num_timers; i++) {
            if(_timers[i].proc){    // task not cancelled?
                hal.console->printf("task 0x%llX tim %8.1f int %5.3f%% tot %6.4f%% mean time %5.1f\n", _timers[i].proc, _timers[i].fulltime/1000.0, _timers[i].fulltime*100.0 / task_time, (_timers[i].fulltime / 10.0) / t, (float)_timers[i].fulltime/_timers[i].count  );
                yield();
            }
        }

#ifdef MTASK_PROF
    
        task_t* ptr = &s_main;

        hal.console->printf("task switch time %7.3f count %ld mean %6.3f \n", yield_time/(float)us_ticks, yield_count, yield_time /(float)us_ticks / (float)yield_count );
        
        yield();
        do {
            hal.console->printf("task %d times: full %lld max %ld \n",  ptr->id, ptr->time, ptr->delay );
            yield();
        
            ptr = ptr->next;
        } while(ptr != &s_main);

#endif


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
            noInterrupts();            // 64-bits should be 
            _timers[i].proc = 0L; // clear proc - temporary disable task
            interrupts();
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
        _timers[i].time_to_run = systick_micros() + period_us;
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
    uint32_t now = systick_micros();
    static uint32_t last_run = 0;


#ifdef SHED_PROF
    uint32_t full_t = now;
    uint32_t job_t = 0;
#endif                

    uint32_t dt = now - last_run; // time from last run

    for(int i = 0; i<_num_timers; i++){
        if(_timers[i].proc){    // task not cancelled?
            if(_timers[i].time_to_run < now) { // time to run?
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
                now = systick_micros();
#ifdef SHED_PROF
                t = now - t;               // work time
#endif                
                if(_timers[i].sem) _timers[i].sem->give(); //  semaphore active? give back ASAP!
#ifdef SHED_PROF
                if(_timers[i].micros < t)
                    _timers[i].micros    =  t;      // max time
                _timers[i].count     += 1;          // number of calls
                _timers[i].fulltime  += t;          // full time, mean time = full / count
                job_t += t;                  // time of all jobs
#endif                
                if(ret)
                    _timers[i].time_to_run += _timers[i].period;  // reschedule
                else
                    _timers[i].proc = 0L;               // cancel task
            }
        }
    }

    last_run = now;

#ifdef SHED_PROF
    full_t = systick_micros() - full_t;         // full time of scheduler
    uint32_t shed_t = full_t - job_t;   // net time

    task_time += job_t; // full time in tasks
    shed_time += shed_t;
#endif                

}

// ]


//[ -------- realization of cooperative multitasking --------

bool REVOMINIScheduler::adjust_stack(size_t stackSize)
{  // Set main task stack size
  s_top = stackSize;
  return true;
}

void REVOMINIScheduler::init_task(func_t t_setup, func_t t_loop, const uint8_t* stack)
{
  // Add task last in run queue (main task)
    task_t task;
    task.next = &s_main;
    task.prev = s_main.prev;
    s_main.prev->next = &task;
    s_main.prev = &task;
    task.stack = stack;
    task.id = ++task_n; // counter

#ifdef MTASK_PROF
    task.time=0;   // total time
    task.delay=0;  // max execution time
    task.start=systick_micros(); 
#endif

  // Create context for new task, caller will return
  if (setjmp(task.context)) {
    // we comes via longjmp
    if (t_setup != NULL) {
         t_setup();
         yield();
    }
    while (1) {
        t_loop();
        yield();        // in case that function not uses dalay();
    }
  }
  // caller returns
}


bool REVOMINIScheduler::start_task(func_t taskSetup, func_t taskLoop, size_t stackSize)
{
  // Check called from main task and valid task loop function
  if (!is_main_task() || (taskLoop == NULL)) return false;

  // Adjust stack size with size of task context
  stackSize += sizeof(task_t);

  // Allocate stack(s) and check if main stack top should be set
  size_t frame = RAMEND - (size_t) &frame;
  volatile uint8_t stack[s_top - frame]; // should be volatile else it will be optimized out
  if (s_main.stack == NULL) s_main.stack = (const uint8_t*)stack; // remember on first call stack of main task

  // Check that the task can be allocated
  if (s_top + stackSize > STACK_MAX) return false;

  // Adjust stack top for next task allocation
  s_top += stackSize;

  // Initiate task with given functions and stack top
  init_task(taskSetup, taskLoop, (const uint8_t*)(stack - stackSize));
  return true;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-label"
void REVOMINIScheduler::yield(uint16_t ttw) // time to work 
{
    if(task_n==0) return;

#ifdef MTASK_PROF
    uint32_t t =  systick_micros();
    uint32_t dt =  t - s_running->start; // time in task
    s_running->time+=dt;                           // calculate sum
    if(dt>s_running->delay) s_running->delay = dt; // and remember maximum
    
    uint64_t ticks = stopwatch_getticks();
#endif
    if (setjmp(s_running->context)) {
        // we come here via longjmp - context switch is over
#ifdef MTASK_PROF
        yield_time += stopwatch_getticks() - s_running->ticks; // time of longjmp
        yield_count++;                  // count each context switch
#endif
        return;
    }
    // begin of context switch
#ifdef MTASK_PROF
    yield_time += stopwatch_getticks()-ticks; // time of setjmp
#endif

next:
    // Next task in run queue will continue
    s_running = s_running->next;
//  if(!s_running->active) goto next; // a way to skip unneeded tasks
// and here we can check task max execution time and reject task if more than we have (if ID not 0!)


#ifdef MTASK_PROF
    s_running->start = systick_micros();
    s_running->ticks = stopwatch_getticks();
#endif
    longjmp(s_running->context, true);
    // never comes here
}
#pragma GCC diagnostic pop


size_t REVOMINIScheduler::task_stack(){
  unsigned char marker;
  return (&marker - s_running->stack);
}

