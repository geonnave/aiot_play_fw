#include <string.h>
#include "pwm.h"

//=========================== variables =======================================

typedef struct {
    uint16_t   pwm_params[4];
} pwm_vars_t;

pwm_vars_t pwm_vars;

//=========================== prototypes ======================================

//=========================== public ==========================================

void pwm_init(void) {
    
    // clear module variables
    memset(&pwm_vars,0x00,sizeof(pwm_vars_t));

    //=== GPIO

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... ...A A: DIR:     1=Output
    // .... .... .... .... .... .... .... ..B. B: INPUT:   1=Disconnect
    // .... .... .... .... .... .... .... CC.. C: PULL:    0=Disabled
    // .... .... .... .... .... .DDD .... .... D: DRIVE:   0=S0S1
    // .... .... .... ..EE .... .... .... .... E: SENSE:   0=Disabled
    // xxxx xxxx xxxx xx00 xxxx xxxx xxxx 0011 
    //    0    0    0    0    0    0    0    3 0x00000003
    NRF_P0->PIN_CNF[2]            = 0x00000003; // P0.02
    NRF_P0->PIN_CNF[3]            = 0x00000003; // P0.03

    //=== PWM

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... ...A AAAA A: PIN:      02 (P0.02)
    // .... .... .... .... .... .... ..B. .... B: PORT:     0  (P0.00)
    // C... .... .... .... .... .... .... .... C: CONNECT:  0=Connected
    // 0xxx xxxx xxxx xxxx xxxx xxxx xx00 0010 
    //    0    0    0    0    0    0    0    0 0x00000002
    NRF_PWM0->PSEL.OUT[0]         = 0x00000002; // P0.02
    NRF_PWM0->PSEL.OUT[1]         = 0x00000003; // P0.03

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... ...A A: ENABLE:   1=Enabled
    // 0xxx xxxx xxxx xxxx xxxx xxxx xxxx xxx1 
    //    0    0    0    0    0    0    0    1 0x00000001
    NRF_PWM0->ENABLE              = 0x00000001;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... ...A A: UPDOWN:   0=Up
    // 0xxx xxxx xxxx xxxx xxxx xxxx xxxx xxx0 
    //    0    0    0    0    0    0    0    0 0x00000000
    NRF_PWM0->MODE                = 0x00000000;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... .AAA A: PRESCALER: 0=DIV_1
    // 0xxx xxxx xxxx xxxx xxxx xxxx xxxx x000 
    //    0    0    0    0    0    0    0    0 0x00000000
    NRF_PWM0->PRESCALER           = 2;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... AAAA AAAA AAAA AAAA A: CNT:      0=Disabled
    // 0xxx xxxx xxxx xxxx 0000 0000 0000 0000 
    //    0    0    0    0    0    0    0    0 0x00000000
    NRF_PWM0->LOOP                = 0x00000000;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... ..AA A: LOAD:     2=Individual
    // .... .... .... .... .... ...B .... .... B: MODE:     0=RefreshCount
    // 0xxx xxxx xxxx xxxx xxxx xxx0 xxxx xx10 
    //    0    0    0    0    0    0    0    2 0x00000002
    NRF_PWM0->DECODER             = 0x00000002;
    NRF_PWM0->SEQ[0].PTR          = (uint32_t)(pwm_vars.pwm_params);
    NRF_PWM0->SEQ[0].CNT          = (sizeof(pwm_vars.pwm_params) / sizeof(uint16_t));

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... AAAA AAAA AAAA AAAA AAAA AAAA A: CNT:      0=Continuous
    // 0xxx xxxx 0000 0000 0000 0000 0000 0000 
    //    0    0    0    0    0    0    0    0 0x00000000
    NRF_PWM0->SEQ[0].REFRESH      = 0;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... AAAA AAAA AAAA AAAA AAAA AAAA A: CNT
    // 0xxx xxxx 0000 0000 0000 0000 0000 0000 
    //    0    0    0    0    0    0    0    0 0x00000000
    NRF_PWM0->SEQ[0].ENDDELAY     = 0;
}

void pwm_setperiod(uint16_t period) {
    
    // stop
    pwm_stop();

    // set
    NRF_PWM0->COUNTERTOP          = period & 0x7fff; // COUNTERTOP is a 24-bit register
    pwm_vars.pwm_params[0]        = 0x0000 | period/2; // POLARITY: 0x0000==RisingEdge
    pwm_vars.pwm_params[1]        = 0x8000 | period/2; // POLARITY: 0x8000==FallingEdge
    pwm_vars.pwm_params[2]        = 0;
    pwm_vars.pwm_params[3]        = 0;

    // start
    NRF_PWM0->EVENTS_SEQSTARTED[0]=0;
    NRF_PWM0->TASKS_SEQSTART[0]   = 0x00000001;
    while(NRF_PWM0->EVENTS_SEQSTARTED[0]==0);
}

void pwm_stop(void) {
    if (NRF_PWM0->EVENTS_SEQSTARTED[0]==1) {
        NRF_PWM0->EVENTS_STOPPED  = 0;
        NRF_PWM0->TASKS_STOP      = 0x00000001;
        while(NRF_PWM0->EVENTS_STOPPED==0);
        NRF_PWM0->EVENTS_SEQSTARTED[0] = 0;
    }
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================
