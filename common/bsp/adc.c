#include <string.h>
#include "gpio.h"

//=========================== variables =======================================

typedef struct {
    int16_t        lastAdcReading;
} adc_vars_t;

adc_vars_t adc_vars;

//=========================== prototypes ======================================

//=========================== public ==========================================

void adc_init(void) {

    
}

// P0.02
uint16_t adc_read_P002(void) {
    int32_t returnVal;

    //=== config

    NRF_SAADC->RESULT.PTR              = (uint32_t)&adc_vars.lastAdcReading;
    NRF_SAADC->RESULT.MAXCNT           = 1;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... .AAA VAL:    1=10bit (doesn't want to use higher?!?)
    // xxxx xxxx xxxx xxxx xxxx xxxx xxxx x001 
    //    0    0    0    0    0    0    0    1 0x00000001
    NRF_SAADC->RESOLUTION              = 0x00000001;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... ..AA RESP:   0=Bypass
    // .... .... .... .... .... .... ..BB .... RESN:   0=Bypass
    // .... .... .... .... .... .CCC .... .... GAIN:   0=Gain1_6
    // .... .... .... .... ...D .... .... .... REFSEL: 0=Internal, Internal reference (0.6 V)
    // .... .... .... .EEE .... .... .... .... TACQ:   5=40us
    // .... .... ...F .... .... .... .... .... MODE:   0=SE, Single-ended
    // .... ...G .... .... .... .... .... .... BURST:  0=Disabled
    // xxxx xxx0 xxx0 x101 xxx0 x000 xx00 xx00 
    //    0    0    0    5    0    0    0    0 0x00050000
    NRF_SAADC->CH[0].CONFIG            = 0x00050000;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... ...A AAAA PSELP: 1=AnalogInput0 (P0.02)
    // xxxx xxxx xxxx xxxx xxxx xxxx xxx0 0001 
    //    0    0    0    0    0    0    0    1 0x00000001
    NRF_SAADC->CH[0].PSELP             = 0x00000001;
    
    //=== prepare

    // enable (acquires analog input pins)
    NRF_SAADC->ENABLE                  = 0x00000001;

    // calibrate
    NRF_SAADC->EVENTS_CALIBRATEDONE    = 0;
    NRF_SAADC->TASKS_CALIBRATEOFFSET   = 1;
    while(NRF_SAADC->EVENTS_CALIBRATEDONE!=1) {}

    // start
    NRF_SAADC->EVENTS_STARTED          = 0;
    NRF_SAADC->TASKS_START             = 1;
    while(NRF_SAADC->EVENTS_STARTED!=1) {}

    //=== sample

    // sample
    NRF_SAADC->EVENTS_RESULTDONE       = 0;
    NRF_SAADC->TASKS_SAMPLE            = 1;
    while(NRF_SAADC->EVENTS_RESULTDONE!=1) {}

    //=== teardown

    // stop
    NRF_SAADC->EVENTS_STOPPED          = 0;
    NRF_SAADC->TASKS_STOP              = 1;
    while(NRF_SAADC->EVENTS_STOPPED!=1) {}

    // disable
    NRF_SAADC->ENABLE                  = 0x00000000;

    // convert
    // 0x0000 == 0mV, 0x03ff == 1023 == 3600mV
    // one lsb equals 3600/1023 = 3.519061584mV
    // approach:
    // - multiply by 3519 (max value is 1023*3519=3,599,937=0x0036EE41, which confortably fits in uint32_t)
    // - divide by 1,000
    // (max error <1mV)
    returnVal  = adc_vars.lastAdcReading;
    returnVal *=                    3519;
    returnVal /=                    1000;
    
    return (int16_t)returnVal;
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================
