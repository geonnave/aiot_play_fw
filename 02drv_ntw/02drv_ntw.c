#include <string.h>
#include <stdbool.h>
#include "board.h"
#include "periodictimer.h"
#include "ntw.h"

//=========================== defines =========================================

#define DATA_PERIOD_S 10

//=========================== typedef =========================================

//=========================== variables =======================================

typedef struct {
    uint8_t        txCounter;
} app_vars_t;

app_vars_t app_vars;

typedef struct {
    uint32_t       numReceive;
    uint32_t       numTransmit;
    uint32_t       numTransmit_success;
    uint32_t       numTransmit_fail;
} app_dbg_t;

app_dbg_t app_dbg;

//=========================== prototypes ======================================

void _periodtimer_cb(void);
void _ntw_receive_cb(uint8_t* buf, uint8_t bufLen);

//=========================== main ============================================

int main(void) {

    // initialize variables
    memset(&app_vars,0x00,sizeof(app_vars));
    memset(&app_dbg, 0x00,sizeof(app_dbg));
    
    // bsp
    board_init();

    // ntw
    ntw_init(NULL, NULL, _ntw_receive_cb);

    // initialize the periodic timer
    periodictimer_init(
        DATA_PERIOD_S,       // period_s
        _periodtimer_cb      // periodtimer_cb
    );

    // main loop
    while(1) {

        // wait for event
        board_sleep();
    }
}

//=========================== private =========================================

void _periodtimer_cb(void) {
    uint8_t txBuf[4];
    bool    success;

    // increment
    app_vars.txCounter++;

    // fill txBuf
    txBuf[0] = app_vars.txCounter;
    txBuf[1] = 0x01;
    txBuf[2] = 0x02;
    txBuf[3] = 0x03;

    // send txBuf
    success = ntw_transmit(txBuf,sizeof(txBuf));

    // debug
    app_dbg.numTransmit++;
    if (success==true) {
        app_dbg.numTransmit_success++;
    } else {
        app_dbg.numTransmit_fail++;
    }
}

void _ntw_receive_cb(uint8_t* buf, uint8_t bufLen) {
    
    // debug
    app_dbg.numReceive++;
}
