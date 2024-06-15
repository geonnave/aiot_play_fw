#include <string.h>
#include "periodictimer.h"

//=========================== define ==========================================

//=========================== variables =======================================

typedef struct {
    uint16_t        period_s;
    periodtimer_cbt periodtimer_cb;
    uint16_t        countdown_s;
} periodictimer_vars_t;

periodictimer_vars_t periodictimer_vars;

typedef struct {
    uint32_t        num_ISR_RTC0_IRQHandler;
    uint32_t        num_ISR_RTC0_IRQHandler_COMPARE0;
} periodictimer_dbg_t;

periodictimer_dbg_t periodictimer_dbg;

//=========================== prototypes ======================================

//=========================== public ==========================================

void periodictimer_init(uint16_t period_s, periodtimer_cbt periodtimer_cb) {

    // reset variables
    memset(&periodictimer_vars,0x00,sizeof(periodictimer_vars_t));

    // store params
    periodictimer_vars.period_s        = period_s;
    periodictimer_vars.periodtimer_cb  = periodtimer_cb;

    // configure/start the RTC
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // xxxx xxxx xxxx FEDC xxxx xxxx xxxx xxBA (C=compare 0)
    // 0000 0000 0000 0001 0000 0000 0000 0000 
    //    0    0    0    1    0    0    0    0 0x00010000
    NRF_RTC0->EVTENSET                 = 0x00010000;       // enable compare 0 event routing
    NRF_RTC0->INTENSET                 = 0x00010000;       // enable compare 0 interrupts

    // enable interrupts
    NVIC_SetPriority(RTC0_IRQn, 1);
    NVIC_ClearPendingIRQ(RTC0_IRQn);
    NVIC_EnableIRQ(RTC0_IRQn);

    // prepare the countdown
    periodictimer_vars.countdown_s     = periodictimer_vars.period_s;
    
    // have RTC tick every second
    NRF_RTC0->CC[0]                    = 32768;            // 32768 = 1 s
    NRF_RTC0->TASKS_START              = 0x00000001;       // start RTC0
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================

void RTC0_IRQHandler(void) {

    // debug
    periodictimer_dbg.num_ISR_RTC0_IRQHandler++;

    // handle compare[0]
    if (NRF_RTC0->EVENTS_COMPARE[0] == 0x00000001 ) {

        // clear flag
        NRF_RTC0->EVENTS_COMPARE[0]    = 0x00000000;

        // clear COUNTER
        NRF_RTC0->TASKS_CLEAR          = 0x00000001;

        // debug
        periodictimer_dbg.num_ISR_RTC0_IRQHandler_COMPARE0++;

        // handle
        periodictimer_vars.countdown_s--;
        if (periodictimer_vars.countdown_s==0) {
            periodictimer_vars.countdown_s = periodictimer_vars.period_s;
            periodictimer_vars.periodtimer_cb();
        }
    }
}