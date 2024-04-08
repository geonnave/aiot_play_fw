#include <string.h>
#include <stdbool.h>
#include "board.h"
#include "periodictimer.h"
#include "ntw.h"
#include "music.h"
#include "leds.h"

//=========================== defines =========================================

#define TICKS_PER_SLOT       234            // 0.00725*32768
#define ASN1_POLLING_PERIOD  (32768>>0)     // 32768>>1 = 500 ms

//=========================== typedef =========================================

typedef enum {
    STEP_1_WAITING_ASN3,
    STEP_2_WAITING_ASN4_ROLLOVER,
} step_t;

//=========================== variables =======================================

typedef struct {
    step_t         step;
    uint16_t       moteId;
    uint8_t        asn[5];
} app_vars_t;

app_vars_t app_vars;

typedef struct {
    uint32_t       numcalls_ntw_getMoteId_cb;
    uint32_t       numcalls_ntw_getTime_cb;
    uint32_t       numcalls_ntw_receive_cb;
    uint32_t       num_STEP_1_WAITING_ASN3;
    uint32_t       num_STEP_2_WAITING_ASN4_ROLLOVER;
    uint32_t       num_rc_error;
} app_dbg_t;

app_dbg_t app_dbg;

//=========================== prototypes ======================================

void _ntw_joining_cb(void);
void _ntw_getMoteId_cb(dn_ipmt_getParameter_moteId_rpt* reply);
void _ntw_getTime_cb(dn_ipmt_getParameter_time_rpt* reply);
void _ntw_receive_cb(uint8_t* buf, uint8_t bufLen);

//=========================== main ============================================

int main(void) {

    // initialize variables
    memset(&app_vars,0x00,sizeof(app_vars));
    memset(&app_dbg, 0x00,sizeof(app_dbg));
    
    // bsp
    board_init();

    // ntw
    ntw_init(
        _ntw_joining_cb,     // ntw_joining_cb
        _ntw_getMoteId_cb,   // ntw_getMoteId_cb
        _ntw_getTime_cb,     // ntw_getTime_cb
        _ntw_receive_cb      // ntw_receive_cb
    );

    // music
    music_init();

    // leds
    leds_init();

    // RTC0
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

    // query ASN every 500ms
    app_vars.step                      = STEP_1_WAITING_ASN3;
    NRF_RTC0->CC[0]                    = ASN1_POLLING_PERIOD;
    NRF_RTC0->TASKS_START              = 0x00000001;

    // red led: no connection to mote yet
    leds_red_on();

    // main loop
    while(1) {

        // wait for event
        board_sleep();
    }
}

//=========================== private =========================================

void _ntw_joining_cb(void) {
    
    // red led: no connection to mote yet
    leds_red_off();

    // green led: joining
    leds_green_on();
}

void _ntw_getMoteId_cb(dn_ipmt_getParameter_moteId_rpt* reply) {

    // debug
    app_dbg.numcalls_ntw_getMoteId_cb++;

    // red led: no connection to mote yet
    leds_red_off();

    // green led: we have joined when getting a moteId
    leds_green_off();

    do {
        if (reply->RC!=DN_ERR_NONE) {
            app_dbg.num_rc_error++;
            break;
        }
        
        // store
        app_vars.moteId = reply->moteId;
    } while(0);
}

void _ntw_getTime_cb(dn_ipmt_getParameter_time_rpt* reply) {
    uint32_t num_asns_to_wait;
    uint32_t num_ticks_to_wait;
    uint8_t  trackIdx;

    // debug
    app_dbg.numcalls_ntw_getTime_cb++;

    do {
        if (reply->RC!=DN_ERR_NONE) {
            app_dbg.num_rc_error++;
            break;
        }
        if (reply->upTime==0) {
            break;
        }

        // green led: we have joined when getting valid getTime
        leds_green_off();

        // copy over to local copy for easier debug
        memcpy(app_vars.asn,reply->asn,sizeof(app_vars.asn));

        switch (app_vars.step) {
            case STEP_1_WAITING_ASN3:
                app_dbg.num_STEP_1_WAITING_ASN3++;
                if ( (app_vars.asn[3]&0x3f)==0) {
                    // step 2: I'm at the right ASN[3]
                    // wait for ASN[4] to roll over

                    num_asns_to_wait  = 0xff-app_vars.asn[4];
                    num_ticks_to_wait = num_asns_to_wait*TICKS_PER_SLOT;
                    app_vars.step     = STEP_2_WAITING_ASN4_ROLLOVER;
                    NRF_RTC0->CC[0]   = num_ticks_to_wait;
                }
                break;
            case STEP_2_WAITING_ASN4_ROLLOVER:
                app_dbg.num_STEP_2_WAITING_ASN4_ROLLOVER++;
                app_vars.step         = STEP_1_WAITING_ASN3;
                NRF_RTC0->CC[0]       = ASN1_POLLING_PERIOD;
                trackIdx              = app_vars.moteId-2; // the first mote has moteId 2, yet we want trackIdx 0 for it
                music_play(SONGTITLE_STAR_WARS,trackIdx);
                break;
        }

    } while(0);
}

void _ntw_receive_cb(uint8_t* buf, uint8_t bufLen) {
    
    // debug
    app_dbg.numcalls_ntw_receive_cb++;

    // handle
    if (buf[0]==0x00) {
        music_inhibit(true);
    } else {
        music_inhibit(false);
    }
}

//=========================== interrupt handlers ==============================

void RTC0_IRQHandler(void) {
    
    // handle compare[0]
    if (NRF_RTC0->EVENTS_COMPARE[0] == 0x00000001 ) {

        // clear flag
        NRF_RTC0->EVENTS_COMPARE[0]    = 0x00000000;

        // clear COUNTER
        NRF_RTC0->TASKS_CLEAR          = 0x00000001;

        // handle
        if (app_vars.moteId==0x000) {
            ntw_getMoteId();
        } else {
            ntw_getTime();
        }
    }
}
