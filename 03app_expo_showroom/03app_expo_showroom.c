#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "board.h"
#include "periodictimer.h"
#include "ntw.h"
#include "music.h"
#include "gpio.h"
#include "dn_ipmt.h"

//=========================== defines =========================================

#define TICKS_PER_SLOT       234            // 0.00725*32768
#define ASN1_POLLING_PERIOD  (32768>>0)     // 32768>>1 = 500 ms

//=========================== typedef =========================================

typedef enum {
    STEP_1_WAITING_ASN3,
    STEP_2_WAITING_ASN4_ROLLOVER,
} step_t;

#define PLAY_MODE_STOP         0
#define PLAY_MODE_STARWARS     1
#define PLAY_MODE_HARRY_POTTER 2

//=========================== variables =======================================

typedef struct {
    step_t         step;
    uint16_t       moteId;
    uint8_t        asn[5];
    uint8_t        music_play_mode;
    uint8_t        asn_trigger_music[5];
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

    gpio_P005_output_init();
    gpio_P020_output_init();
    gpio_P011_output_init();
    gpio_P020_output_high();
    // ntw
    ntw_init(
        _ntw_getMoteId_cb,   // ntw_getMoteId_cb
        _ntw_getTime_cb,     // ntw_getTime_cb
        _ntw_receive_cb      // ntw_receive_cb
    );

    app_vars.asn_trigger_music[0] = 0xff;
    app_vars.asn_trigger_music[1] = 0xff;
    app_vars.asn_trigger_music[2] = 0xff;
    app_vars.asn_trigger_music[3] = 0xff;
    app_vars.asn_trigger_music[4] = 0xff;

    // music
    music_init();


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

    // main loop
    while(1) {

        // wait for event
        board_sleep();
    }
}

//=========================== private =========================================

void _ntw_getMoteId_cb(dn_ipmt_getParameter_moteId_rpt* reply) {

    // debug
    app_dbg.numcalls_ntw_getMoteId_cb++;

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
    //printf("enter _ntw_getTime_cb %d\n", app_dbg.numcalls_ntw_getTime_cb);

    do {
        if (reply->RC!=DN_ERR_NONE) {
            app_dbg.num_rc_error++;
            break;
        }
        if (reply->upTime==0) {
            break;
        }

        // copy over to local copy for easier debug
        memcpy(app_vars.asn,reply->asn,sizeof(app_vars.asn));

        switch (app_vars.step) {
            case STEP_1_WAITING_ASN3:
                app_dbg.num_STEP_1_WAITING_ASN3++;

                uint32_t current_asn =   
                                       (app_vars.asn[1]<<24) 
                                       + (app_vars.asn[2]<<16) 
                                       + (app_vars.asn[3]<<8) 
                                       + (app_vars.asn[4]<<0);
                uint32_t asn_trigger_music = 
                                            (app_vars.asn_trigger_music[1]<<24) 
                                            + (app_vars.asn_trigger_music[2]<<16) 
                                            + (app_vars.asn_trigger_music[3]<<8) 
                                            + (app_vars.asn_trigger_music[4]<<0);

                //uint64_t current_asn =   (app_vars.asn[0]<<32) 
                //                       + (app_vars.asn[1]<<24) 
                //                       + (app_vars.asn[2]<<16) 
                //                       + (app_vars.asn[3]<<8) 
                //                       + (app_vars.asn[4]<<0);
                //uint64_t asn_trigger_music =  (app_vars.asn_trigger_music[0]<<32) 
                //                            + (app_vars.asn_trigger_music[1]<<24) 
                //                            + (app_vars.asn_trigger_music[2]<<16) 
                //                            + (app_vars.asn_trigger_music[3]<<8) 
                //                            + (app_vars.asn_trigger_music[4]<<0);
                if ( (asn_trigger_music - current_asn) < ASN1_POLLING_PERIOD/TICKS_PER_SLOT) {
                    num_asns_to_wait  = asn_trigger_music - current_asn;
                    num_ticks_to_wait = num_asns_to_wait*TICKS_PER_SLOT;
                    app_vars.step     = STEP_2_WAITING_ASN4_ROLLOVER;
                    NRF_RTC0->CC[0]   = num_ticks_to_wait;
                }
                //if ( (app_vars.asn[3]&0x3f)==0) {
                //    // step 2: I'm at the right ASN[3]
                //    // wait for ASN[4] to roll over

                //    num_asns_to_wait  = 0xff-app_vars.asn[4];
                //    num_ticks_to_wait = num_asns_to_wait*TICKS_PER_SLOT;
                //    app_vars.step     = STEP_2_WAITING_ASN4_ROLLOVER;
                //    NRF_RTC0->CC[0]   = num_ticks_to_wait;
                //}
                break;
            case STEP_2_WAITING_ASN4_ROLLOVER:
                app_dbg.num_STEP_2_WAITING_ASN4_ROLLOVER++;
                app_vars.step         = STEP_1_WAITING_ASN3;
                NRF_RTC0->CC[0]       = ASN1_POLLING_PERIOD;
                trackIdx              = app_vars.moteId-2; // the first mote has moteId 2, yet we want trackIdx 0 for it
                

                if (app_vars.music_play_mode == PLAY_MODE_STARWARS){
                     
                     music_play(SONGTITLE_STAR_WARS,trackIdx);
                     
                }
                else if (app_vars.music_play_mode == PLAY_MODE_HARRY_POTTER){
                    music_play(SONGTITLE_HARRY_POTTER,trackIdx);

                }
                app_vars.music_play_mode = PLAY_MODE_STOP;

                //if ((app_vars.asn[3]&0x40)==0) {
                //    music_play(SONGTITLE_STAR_WARS,trackIdx);
                //} else {
                //    music_play(SONGTITLE_HARRY_POTTER,trackIdx);
                //}
                break;
        }

    } while(0);
}



void _ntw_receive_cb(uint8_t* buf, uint8_t bufLen){
    
    if (app_vars.asn_trigger_music[3] != *(buf+4) && app_vars.asn_trigger_music[4] != *(buf+5)){ // update only once if recast
        app_vars.music_play_mode = *buf;
        app_vars.asn_trigger_music[0] = *(buf+1);
        app_vars.asn_trigger_music[1] = *(buf+2);
        app_vars.asn_trigger_music[2] = *(buf+3);
        app_vars.asn_trigger_music[3] = *(buf+4);
        app_vars.asn_trigger_music[4] = *(buf+5);
        printf("msg received asn[4] %d\n",*(buf+5));
        //dn_ipmt_cancelTx(); //ADDED hotfix, try to fix a bug where at a random moment dn_ipmt_vars.busyTx in true prevents call of _ntw_getTime_cb
                            // bug is still there but it seems like its less present, if a musician doesnt trigger it does at the next command.
                            //TODO fix bug ? add lock mechanism to remove that arbitrary command? why does it affect motes in different ways, some bugging more  freuently than others? music partition length difference for each mote?
                            // add message acknowledgment from mote to prevent that issue?
                            // si on arme une callback pour demander le timing rtc et qu'on recoit une commande à ce moment précis est-ce que c'est pas ca qui fait planter le système?
                            // avec par une commande obscure un busytx qui revient à false, notamment lorsqu'on reset la partie réseau du mote?
                            // pourquoi ca touche plus les motes 70-1a-06 et 70-53-ba olutot que 68-0a-ab? est-ce l'eui ou le moteid dans le réseau qui change qqch?
    }

    // debug
    app_dbg.numcalls_ntw_receive_cb++;
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
            //printf("rtcirq\n");
            ntw_getTime();
        }
    }
}
