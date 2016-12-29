/******************************************************************************
 * The MIT License
 *
based on:
 
 * Copyright (c) 2010 Perry Hung.
 *
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

/**
 * @brief Generic board initialization routines.
 *
 */

#include "boards.h"
#include "systick.h"
#include "gpio_hal.h"
#include "timer.h"
#include "adc.h"
#include <usb.h>

static void setupFlash(void);
static void setupClocks(void);
static void setupNVIC(void);
static void enableFPU(void);
static void setupCCM(void);

void setupADC(void);
void setupTimers(void);
void usb_init(void);


void usb_init(void){


    usb_attr_t usb_attr;
    usb_open();

    usb_default_attr(&usb_attr);
    usb_attr.preempt_prio = 1;
    usb_attr.sub_prio = 3;
    usb_attr.use_present_pin = 1;
    usb_attr.present_port = _GPIOC;
    usb_attr.present_pin = 5;

    usb_configure(&usb_attr);

}

inline void enableFPU(void){
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));	// set CP10 and CP11 Full Access
#endif
}


inline static void setupFlash(void) {

}

/*
 * Clock setup.  Note that some of this only takes effect if we're
 * running bare metal and the bootloader hasn't done it for us
 * already.
 *
 */
inline static void setupClocks() {
}


inline static void setupCCM(){
    extern unsigned _sccm,_eccm; // defined by link script

    RCC->AHB1ENR |= RCC_AHB1ENR_CCMDATARAMEN;
    asm volatile("dsb \n");

    volatile unsigned *dest = &_sccm;
    while (dest < &_eccm) {
        *dest++ = 0;
    }
}

inline static void setupNVIC() {
    /* 4 bit preemption,  0 bit subpriority */
    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4);
}


static void timerDefaultConfig(timer_dev*);

void setupTimers() {
    timer_foreach(timerDefaultConfig);
}


static void timerDefaultConfig(timer_dev *dev) {

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    timer_reset(dev);
    timer_pause(dev);

    dev->regs->CR1 = TIMER_CR1_ARPE;
    dev->regs->PSC = 1;
    dev->regs->SR = 0;
    dev->regs->DIER = 0;
    dev->regs->EGR = TIMER_EGR_UG;

    switch (dev->type) {
    case TIMER_ADVANCED:
        dev->regs->BDTR = TIMER_BDTR_MOE | TIMER_BDTR_LOCK_OFF;
        // fall-through
    case TIMER_GENERAL:
	/* set period to 490 Hz as default */   
	/* SystemCoreClock is set to 168 MHz for STM32F4xx devices */
	if (dev->regs == TIM1 || dev->regs == TIM8 || dev->regs == TIM9 || dev->regs == TIM10 || dev->regs == TIM11){
		/* Timer clock: 168 Mhz */
		/* 
		 * The objective is to generate PWM signal at 490 Hz 
		 * The lowest possible prescaler is 5
		 * Period = (SystemCoreClock / (490 * 6)) - 1 = 57141
		 */			
		TIM_TimeBaseStructure.TIM_Prescaler = 5;
		TIM_TimeBaseStructure.TIM_Period = 57141;
	} else {
		/* Timer clock: 84 Mhz */
		/* 
		 * The objective is to generate 2MHz 
		 */
	        uint32_t period = (2000000UL / 50) - 1; // 50 Hz
	        uint32_t prescaler =  (uint16_t) ((SystemCoreClock /2) / 2000000) - 1; //2MHz 0.5us ticks
	        TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
		TIM_TimeBaseStructure.TIM_Period = period;
	}

	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(dev->regs, &TIM_TimeBaseStructure);
	
	TIM_SelectOCxM(dev->regs, TIM_Channel_1, TIM_OCMode_PWM1);
	TIM_OC1PreloadConfig(dev->regs, TIM_OCPreload_Enable);

	TIM_SelectOCxM(dev->regs, TIM_Channel_2, TIM_OCMode_PWM1);
	TIM_OC2PreloadConfig(dev->regs, TIM_OCPreload_Enable);

	TIM_SelectOCxM(dev->regs, TIM_Channel_3, TIM_OCMode_PWM1);
	TIM_OC3PreloadConfig(dev->regs, TIM_OCPreload_Enable);

	TIM_SelectOCxM(dev->regs, TIM_Channel_4, TIM_OCMode_PWM1);
	TIM_OC4PreloadConfig(dev->regs, TIM_OCPreload_Enable);
	break;

    case TIMER_BASIC:
        break;
    }

    timer_resume(dev);
}

// 1st executing function

void inline init(void) {
    setupCCM();
    setupFlash();  // empty
    setupClocks(); // empty

    SystemInit();
    SystemCoreClockUpdate();

    enableFPU();
    setupNVIC();
    systick_init(SYSTICK_RELOAD_VAL);

    stopwatch_init(); // will use stopwatch_delay_us

    boardInit();
/*
     only CPU init here, all another moved to modules .init() functions
*/
}

void pre_init(){ // before any stack usage @NG

    init();
}
