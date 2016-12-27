#ifndef _HAL_H_
#define _HAL_H_

#include <errno.h>

#include <stm32f4xx.h>

#include "hal_types.h"
#include "stm32.h"
#include "stopwatch.h"
#include "util.h"
#include "gpio_hal.h"
#include "delay.h"
#include "adc.h"


#define ISR_PROF


#define OK	1
#define ERROR	0

#define I2C_OK		0
#define I2C_NO_DEVICE	1
#define I2C_ERROR	2

/*
 * Where to put usercode, based on space reserved for bootloader.
 *
 * FIXME this has no business being here
 */

//#define USER_ADDR_ROM 0x08005000
//#define USER_ADDR_RAM 0x20000C00
//#define STACK_TOP     0x20000800

extern void clock_gettime(uint32_t a1, void *a2);

#ifndef IN_CCM
#define IN_CCM  __attribute__((section("ccm")))
#endif


#ifdef ISR_PROF
    extern uint64_t isr_time;
#endif

/*
union Revo_cb { // кровь кишки ассемблер :) преобразование функторов в унифицированный вид
    AP_HAL::MemberProc mp;
    AP_HAL::Device::PeriodicCb pcb;
    uint64_t h; // treat as handle
    uint32_t w[2]; // words, to check
};
*/

union Revo_handler { // кровь кишки ассемблер :) преобразование функторов в унифицированный вид
    voidFuncPtr vp;
    uint64_t h;    // treat as handle
    uint32_t w[2]; // words, to check
};



#endif

