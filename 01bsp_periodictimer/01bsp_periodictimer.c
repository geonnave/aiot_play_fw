#include <string.h>
#include "board.h"
#include "periodictimer.h"

//=========================== defines =========================================

//=========================== typedef =========================================

//=========================== variables =======================================

typedef struct {
    uint8_t counter;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void _periodtimer_cb(void);

//=========================== main ============================================

int main(void) {

    // clear variables
    memset(&app_vars,0x00,sizeof(app_vars_t));

    // bsp
    board_init();

    // initialize the periodic timer
    periodictimer_init(
        2,                   // period_s
        _periodtimer_cb      // periodtimer_cb
    );

    // main loop
    while(1) {
        
        // sleep while waiting for an event
        board_sleep();
    }
}

//=========================== private =========================================

void _periodtimer_cb(void) {

    // increment
    app_vars.counter++;
}