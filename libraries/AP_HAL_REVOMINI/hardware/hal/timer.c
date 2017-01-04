/******************************************************************************
 * The MIT License

 based on:

 *
 * Copyright (c) 2011 LeafLabs, LLC.
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
 * @file   timer.c
 * @author Marti Bolivar <mbolivar@leaflabs.com>
 * @brief  New-style timer interface
 */

#include "timer.h"
#include <string.h>


/* Just like the corresponding DIER bits:
 * [0] = Update handler;
 * [1,2,3,4] = capture/compare 1,2,3,4 handlers, respectively;
 * [5] = COM;
 * [6] = TRG;
 * [7] = BRK. */
#define NR_ADV_HANDLERS                 8
/* Update, capture/compare 1,2,3,4; <junk>; trigger. */
#define NR_GEN_HANDLERS                 7
/* Update only. */
#define NR_BAS_HANDLERS                 1


TimerHandler tim1_handlers[NR_ADV_HANDLERS]= {0};
const timer_dev timer1 = {
    .regs         = TIM1,
    .clk	  = RCC_APB2Periph_TIM1,
    .handlers     = tim1_handlers,
    .af           = GPIO_AF_TIM1,
    .type         = TIMER_ADVANCED,
    .n_handlers   = NR_ADV_HANDLERS,
    .bus          = 1,
    .id           = 1,
};
/** Timer 1 device (advanced) */

TimerHandler tim2_handlers[NR_GEN_HANDLERS]={0};
const timer_dev timer2 = {
    .regs         = TIM2,
    .clk    	  = RCC_APB1Periph_TIM2,
    .handlers     = tim2_handlers,
    .af           = GPIO_AF_TIM2,
    .type         = TIMER_GENERAL,
    .n_handlers   = NR_GEN_HANDLERS,
    .bus          = 0,
    .id           = 2,
};
/** Timer 2 device (general-purpose) */

TimerHandler tim3_handlers[NR_GEN_HANDLERS]={0};
const timer_dev timer3 = {
    .regs         = TIM3,
    .clk	  = RCC_APB1Periph_TIM3,
    .handlers     = tim3_handlers,
    .af           = GPIO_AF_TIM3,
    .type         = TIMER_GENERAL,
    .n_handlers   = NR_GEN_HANDLERS,
    .bus          = 0,
    .id           = 3,
};
/** Timer 3 device (general-purpose) */

TimerHandler tim4_handlers[NR_GEN_HANDLERS]={0};
const timer_dev timer4 = {
    .regs         = TIM4,
    .clk       	  = RCC_APB1Periph_TIM4,
    .handlers     = tim4_handlers,
    .af           = GPIO_AF_TIM4,
    .type         = TIMER_GENERAL,
    .n_handlers   = NR_GEN_HANDLERS,
    .bus          = 0,
    .id           = 4,
};
/** Timer 4 device (general-purpose) */

TimerHandler tim5_handlers[NR_GEN_HANDLERS]={0};
const timer_dev timer5 = {
    .regs         = TIM5,
    .clk          = RCC_APB1Periph_TIM5,
    .handlers     = tim5_handlers,
    .af           = GPIO_AF_TIM5,
    .type         = TIMER_GENERAL,
    .n_handlers   = NR_GEN_HANDLERS,
    .bus          = 0,
    .id           = 5,
};
/** Timer 5 device (general-purpose) */

TimerHandler tim6_handlers[NR_BAS_HANDLERS]={0};
const timer_dev timer6 = {
    .regs         = TIM6,
    .clk          = RCC_APB1Periph_TIM6,
    .handlers     = tim6_handlers,
    .af           = 0,
    .type         = TIMER_BASIC,
    .n_handlers   = NR_BAS_HANDLERS,
    .bus          = 0,
    .id           = 6,
};
/** Timer 6 device (basic) */

TimerHandler tim7_handlers[NR_BAS_HANDLERS]={0};
const timer_dev timer7 = {
    .regs         = TIM7,
    .clk          = RCC_APB1Periph_TIM7,
    .handlers     = tim7_handlers,
    .af           = 0,
    .type         = TIMER_BASIC,
    .n_handlers   = NR_BAS_HANDLERS,
    .bus          = 0,
    .id           = 7,
};
/** Timer 7 device (basic) */

TimerHandler tim8_handlers[NR_ADV_HANDLERS]={0};
const timer_dev timer8 = {
    .regs         = TIM8,
    .clk          = RCC_APB2Periph_TIM8,
    .handlers     = tim8_handlers,
    .af           = GPIO_AF_TIM8,
    .type         = TIMER_ADVANCED,
    .n_handlers   = NR_ADV_HANDLERS,
    .bus          = 1,
    .id           = 8,
}; /** Timer 8 device (advanced) */

TimerHandler tim12_handlers[NR_GEN_HANDLERS]={0};
const timer_dev timer12 = {
    .regs         = TIM12,
    .clk          = RCC_APB1Periph_TIM12,
    .handlers     = tim12_handlers,
    .af           = GPIO_AF_TIM12,
    .type         = TIMER_GENERAL,
    .n_handlers   = NR_GEN_HANDLERS,
    .bus          = 0,
    .id           = 12,
}; /** Timer 12 device (general-purpose) */

const timer_dev * const TIMER1 = &timer1;
const timer_dev * const TIMER2 = &timer2;
const timer_dev * const TIMER3 = &timer3;
const timer_dev * const TIMER4 = &timer4;
const timer_dev * const TIMER5 = &timer5;
const timer_dev * const TIMER6 = &timer6;
const timer_dev * const TIMER7 = &timer7;
const timer_dev * const TIMER8 = &timer8;
const timer_dev * const TIMER12 = &timer12;


/*
 * Convenience routines
 */

static void disable_channel(const timer_dev *dev, uint8_t channel);
static void pwm_mode(const timer_dev *dev, uint8_t channel);
static void output_compare_mode(const timer_dev *dev, uint8_t channel);

static inline void enable_irq(const timer_dev *dev, uint8_t interrupt, uint8_t priority);


#pragma GCC push_options
#pragma GCC optimize ("O0")

/**
 * Initialize a timer (enable timer clock)
 * @param dev Timer to initialize
 */
void timer_init(const timer_dev *dev) {

    if(dev->bus)
    	RCC_APB2PeriphClockCmd(dev->clk, ENABLE);
    else
	RCC_APB1PeriphClockCmd(dev->clk, ENABLE);

}

/**
 * Initialize a timer, and reset its register map.
 * @param dev Timer to initialize
 */
void timer_reset(const timer_dev *dev) {
    memset(dev->handlers, 0, dev->n_handlers * sizeof(TimerHandler));

    if(dev->bus)
    	RCC_APB2PeriphClockCmd(dev->clk, ENABLE);
    else
	RCC_APB1PeriphClockCmd(dev->clk, ENABLE);

    TIM_DeInit(dev->regs);

}

/**
 * @brief Disable a timer.
 *
 * The timer will stop counting, all DMA requests and interrupts will
 * be disabled, and no state changes will be output.
 *
 * @param dev Timer to disable.
 */
void timer_disable(const timer_dev *dev) {
    dev->regs->CR1 = 0;
    dev->regs->DIER = 0;

    switch (dev->type) {
    case TIMER_ADVANCED:        /* fall-through */
    case TIMER_GENERAL:
        (dev->regs)->CCER = 0;
        break;
    case TIMER_BASIC:
        break;
    }
}

/**
 * Sets the mode of an individual timer channel.
 *
 * Note that not all timers can be configured in every mode.  For
 * example, basic timers cannot be configured to output compare mode.
 * Be sure to use a timer which is appropriate for the mode you want.
 *
 * @param dev Timer whose channel mode to set
 * @param channel Relevant channel
 * @param mode New timer mode for channel
 */
void timer_set_mode(const timer_dev *dev, uint8_t channel, timer_mode mode) {
    assert_param(channel > 0 && channel <= 4);

    /* TODO decide about the basic timers */
    assert_param(dev->type != TIMER_BASIC);
    if (dev->type == TIMER_BASIC)
        return;

    switch (mode) {
    case TIMER_DISABLED:
        disable_channel(dev, channel);
        break;
    case TIMER_PWM:
        pwm_mode(dev, channel);
        break;
    case TIMER_OUTPUT_COMPARE:
        output_compare_mode(dev, channel);
        break;
    }
}

/**
 * @brief Call a function on timer devices.
 * @param fn Function to call on each timer device.
 */
void timer_foreach(void (*fn)(const timer_dev*)) {
    //fn(TIMER1);
    fn(TIMER2);
    fn(TIMER3);
    fn(TIMER4);
    //fn(TIMER5);
    //fn(TIMER6);
    fn(TIMER7);
    //fn(TIMER8);
}

/**
 * @brief Attach a timer interrupt.
 * @param dev Timer device
 * @param interrupt Interrupt number to attach to; this may be any
 *                  timer_interrupt_id or timer_channel value appropriate
 *                  for the timer.
 * @param handler Handler to attach to the given interrupt.
 * @see timer_interrupt_id
 * @see timer_channel
 */
void timer_attach_interrupt(const timer_dev *dev, uint8_t interrupt, TimerHandler handler, uint8_t priority) {
    if(interrupt>=dev->n_handlers) return;

    dev->handlers[interrupt] = handler;
    timer_enable_irq(dev, interrupt);
    enable_irq(dev, interrupt, priority);
}

// attach all timer's interrupts to one handler - for PWM/PPM input
void timer_attach_all_interrupts(const timer_dev *dev,  TimerHandler handler) {
    uint16_t i;
    for(i=0; i < dev->n_handlers; i++) {
        dev->handlers[i] = handler;
    }
    // enabling by caller
}


/**
 * @brief Detach a timer interrupt.
 * @param dev Timer device
 * @param interrupt Interrupt number to detach; this may be any
 *                  timer_interrupt_id or timer_channel value appropriate
 *                  for the timer.
 * @see timer_interrupt_id
 * @see timer_channel
 */
void timer_detach_interrupt(const timer_dev *dev, uint8_t interrupt) {
    timer_disable_irq(dev, interrupt);
    if(interrupt>=dev->n_handlers) return;
    dev->handlers[interrupt] = NULL;
}

/*
 * IRQ handlers
 */

static inline void dispatch_adv_brk(const timer_dev *dev);
static inline void dispatch_adv_up(const timer_dev *dev);
static inline void dispatch_adv_trg_com(const timer_dev *dev);
static inline void dispatch_adv_cc(const timer_dev *dev);
static inline void dispatch_general(const timer_dev *dev);
static inline void dispatch_general_h(const timer_dev *dev);
static inline void dispatch_basic(const timer_dev *dev);

void TIM1_BRK_TIM9_IRQHandler(void);
void TIM1_UP_TIM10_IRQHandler(void);
void TIM1_TRG_COM_TIM11_IRQHandler(void);
void TIM1_CC_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
void TIM5_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);
void TIM7_IRQHandler(void);
void TIM8_BRK_TIM12_IRQHandler(void);
void TIM8_CC_IRQHandler(void);
void TIM8_UP_TIM13_IRQHandler(void);
void TIM8_TRG_COM_TIM14_IRQHandler(void);




//*
void TIM1_BRK_TIM9_IRQHandler(void)
{
    dispatch_adv_brk(TIMER1);
}

void TIM1_UP_TIM10_IRQHandler(void) {
    dispatch_adv_up(TIMER1);
}

void TIM1_TRG_COM_TIM11_IRQHandler(void) {
    dispatch_adv_trg_com(TIMER1);
}

void TIM1_CC_IRQHandler(void) {
    dispatch_adv_cc(TIMER1);
}
//*/
void TIM2_IRQHandler(void) {
    dispatch_general(TIMER2);
}

void TIM3_IRQHandler(void) {
    dispatch_general(TIMER3);
}

void TIM4_IRQHandler(void) {
    dispatch_general(TIMER4);
}
//
void TIM5_IRQHandler(void) {
    dispatch_general(TIMER5);
}

void TIM6_DAC_IRQHandler(void) {
    dispatch_basic(TIMER6);
}
//
void TIM7_IRQHandler(void) {
    dispatch_basic(TIMER7);
}

// used in PWM
void TIM8_BRK_TIM12_IRQHandler(void) { // used in PWM tim12
    dispatch_adv_brk(TIMER8);
    dispatch_general_h(TIMER12);
}

void TIM8_CC_IRQHandler(void) { // used in PWM tim8
    dispatch_adv_cc(TIMER8);
}
//

void TIM8_UP_TIM13_IRQHandler(void) { // not conflicts with PWM
    dispatch_adv_up(TIMER8);
}

void TIM8_TRG_COM_TIM14_IRQHandler(void) {
    dispatch_adv_trg_com(TIMER8);
}

  
/* Note: the following dispatch routines make use of the fact that
 * DIER interrupt enable bits and SR interrupt flags have common bit
 * positions.  Thus, ANDing DIER and SR lets us check if an interrupt
 * is enabled and if it has occurred simultaneously.
 */

/* A special-case dispatch routine for single-interrupt NVIC lines.
 * This function assumes that the interrupt corresponding to `iid' has
 * in fact occurred (i.e., it doesn't check DIER & SR). */
static INLINE  void dispatch_single_irq(const timer_dev *dev,
                                       timer_interrupt_id iid,
                                       uint32_t irq_mask) {

    
    uint32_t dsr = dev->regs->DIER & dev->regs->SR & irq_mask;
    if (dsr) {
        dev->regs->SR &= ~dsr;  // reset IRQ inspite of installed handler! @NG

        TimerHandler handler = (dev->handlers)[iid];
        if (handler) {
            handler(dev->regs);
        }
        
    }
}

/* For dispatch routines which service multiple interrupts. */
static INLINE void handle_irq(const timer_dev *dev, uint32_t dier_sr, uint32_t irq_mask, uint32_t iid) {
    if ((dier_sr) & (irq_mask)) {                                 
        TimerHandler handler = (dev->handlers)[iid];                
        if (handler) {                                          
            handler(dev->regs);                                          
        }
    }
}

static inline void dispatch_adv_brk(const timer_dev *dev) {
#ifdef ISR_PERF
    uint32_t t=systick_micros();
#endif
    dispatch_single_irq(dev, TIMER_BREAK_INTERRUPT, TIMER_SR_BIF);
#ifdef ISR_PERF
    isr_time += systick_micros() - t;
#endif
}

static inline void dispatch_adv_up(const timer_dev *dev) {
#ifdef ISR_PERF
    uint32_t t=systick_micros();
#endif
    dispatch_single_irq(dev, TIMER_UPDATE_INTERRUPT, TIMER_SR_UIF);
#ifdef ISR_PERF
    isr_time += systick_micros() - t;
#endif
}

static inline void dispatch_adv_trg_com(const timer_dev *dev) {
#ifdef ISR_PERF
    uint32_t t=systick_micros();
#endif

    uint32_t dsr = dev->regs->DIER & dev->regs->SR;
                            /* Logical OR of SR interrupt flags we end up
                             * handling.  We clear these.  User handlers
                             * must clear overcapture flags, to avoid
                             * wasting time in output mode. */

    dev->regs->SR &= ~dsr;     // handled ALL enabled interrupts! BEFORE ISR itself!

    handle_irq(dev, dsr, TIMER_SR_TIF,   TIMER_TRG_INTERRUPT);
    handle_irq(dev, dsr, TIMER_SR_COMIF, TIMER_COM_INTERRUPT);

#ifdef ISR_PERF
    isr_time += systick_micros() - t;
#endif
}

static inline void dispatch_adv_cc(const timer_dev *dev) {
#ifdef ISR_PERF
    uint32_t t=systick_micros();
#endif

    uint32_t dsr = dev->regs->DIER & dev->regs->SR;
    dev->regs->SR &= ~dsr;      // handled ALL enabled interrupts! BEFORE ISR itself!
    
    handle_irq(dev, dsr, TIMER_SR_CC4IF, TIMER_CC4_INTERRUPT);
    handle_irq(dev, dsr, TIMER_SR_CC3IF, TIMER_CC3_INTERRUPT);
    handle_irq(dev, dsr, TIMER_SR_CC2IF, TIMER_CC2_INTERRUPT);
    handle_irq(dev, dsr, TIMER_SR_CC1IF, TIMER_CC1_INTERRUPT);

#ifdef ISR_PERF
    isr_time += systick_micros() - t;
#endif
}

static inline void dispatch_general(const timer_dev *dev) {
#ifdef ISR_PERF
    uint32_t t=systick_micros();
#endif

    uint32_t dsr = dev->regs->DIER & dev->regs->SR;

    dev->regs->SR &= ~dsr; // handled ALL enabled interrupts! BEFORE ISR itself!

    handle_irq(dev, dsr, TIMER_SR_TIF,   TIMER_TRG_INTERRUPT);
    handle_irq(dev, dsr, TIMER_SR_CC4IF, TIMER_CC4_INTERRUPT);
    handle_irq(dev, dsr, TIMER_SR_CC3IF, TIMER_CC3_INTERRUPT);
    handle_irq(dev, dsr, TIMER_SR_CC2IF, TIMER_CC2_INTERRUPT);
    handle_irq(dev, dsr, TIMER_SR_CC1IF, TIMER_CC1_INTERRUPT);
    handle_irq(dev, dsr, TIMER_SR_UIF,   TIMER_UPDATE_INTERRUPT);

#ifdef ISR_PERF
    isr_time += systick_micros() - t;
#endif

}

static inline void dispatch_general_h(const timer_dev *dev) {
#ifdef ISR_PERF
    uint32_t t=systick_micros();
#endif

    uint32_t dsr = dev->regs->DIER & dev->regs->SR;
    dev->regs->SR &= ~dsr; // handled ALL enabled interrupts! BEFORE ISR itself!

    handle_irq(dev, dsr, TIMER_SR_TIF,   TIMER_TRG_INTERRUPT);
    handle_irq(dev, dsr, TIMER_SR_CC2IF, TIMER_CC2_INTERRUPT);
    handle_irq(dev, dsr, TIMER_SR_CC1IF, TIMER_CC1_INTERRUPT);
    handle_irq(dev, dsr, TIMER_SR_UIF,   TIMER_UPDATE_INTERRUPT);

#ifdef ISR_PERF
    isr_time += systick_micros() - t;
#endif

}


// don't count time of basic timer because it used only as timer scheduler's interrupt
static inline void dispatch_basic(const timer_dev *dev) {
    dispatch_single_irq(dev, TIMER_UPDATE_INTERRUPT, TIMER_SR_UIF);
}

/*
 * Utilities
 */

static void disable_channel(const timer_dev *dev, uint8_t channel) {
    timer_detach_interrupt(dev, channel);
    timer_cc_disable(dev, channel);
}

static void pwm_mode(const timer_dev *dev, uint8_t channel) {
    timer_disable_irq(dev, channel);
    timer_oc_set_mode(dev, channel, TIMER_OC_MODE_PWM_1, TIMER_OC_PE);

    timer_cc_enable(dev, channel);
}

static void output_compare_mode(const timer_dev *dev, uint8_t channel) {
    timer_oc_set_mode(dev, channel, TIMER_OC_MODE_ACTIVE_ON_MATCH, 0);
    timer_cc_enable(dev, channel);
}

static void enable_advanced_irq(const timer_dev *dev, timer_interrupt_id id, uint8_t priority);
static void enable_nonmuxed_irq(const timer_dev *dev, uint8_t priority);

static inline void enable_irq(const timer_dev *dev, timer_interrupt_id iid, uint8_t priority) {
    if (dev->type == TIMER_ADVANCED) {
        enable_advanced_irq(dev, iid, priority);
    } else {
        enable_nonmuxed_irq(dev, priority);
    }
}

static void enable_advanced_irq(const timer_dev *dev, timer_interrupt_id id, uint8_t priority) {
    uint8_t is_timer1 = (dev->id == 1);

    IRQn_Type irq;
    
    switch (id) {
    case TIMER_UPDATE_INTERRUPT:
        NVIC_EnableIRQ(is_timer1 ? TIM1_UP_TIM10_IRQn : TIM8_UP_TIM13_IRQn);
        break;
    case TIMER_CC1_INTERRUPT:
    case TIMER_CC2_INTERRUPT:
    case TIMER_CC3_INTERRUPT:
    case TIMER_CC4_INTERRUPT:
        irq = is_timer1 ? TIM1_CC_IRQn : TIM8_CC_IRQn;
        break;
    case TIMER_COM_INTERRUPT:
    case TIMER_TRG_INTERRUPT:
        irq = is_timer1 ? TIM1_TRG_COM_TIM11_IRQn : TIM8_TRG_COM_TIM14_IRQn;
        break;
    case TIMER_BREAK_INTERRUPT:
        irq = is_timer1 ? TIM1_BRK_TIM9_IRQn : TIM8_BRK_TIM12_IRQn;
        break;
    }
    NVIC_EnableIRQ(irq);
    NVIC_SetPriority(irq,priority);
}

static void enable_nonmuxed_irq(const timer_dev *dev, uint8_t priority) {
    IRQn_Type irq;

    switch(dev->id){
    case 2: irq=TIM2_IRQn; break;
    case 3: irq=TIM3_IRQn; break;
    case 4: irq=TIM4_IRQn; break;
    case 5: irq=TIM5_IRQn; break;
    case 6: irq=TIM6_DAC_IRQn;  break;
    case 7: irq=TIM7_IRQn; break;
    default: return;
    }

    NVIC_EnableIRQ(irq);
    NVIC_SetPriority(irq,priority);
}
