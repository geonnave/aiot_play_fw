#include <string.h>
#include "board.h"
#include "us.h"
#include "periodictimer.h"

//=========================== defines =========================================

//=========================== typedef =========================================

//=========================== variables =======================================

typedef struct {
    int16_t  us_val;
    uint8_t  us_num_reads;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void _periodtimer_cb(void);

//=========================== main ============================================

int main(void) {
   
    // reset variables
    memset(&app_vars,0x00,sizeof(app_vars_t));

    // bsp
    board_init();

    // US init
    us_init();

    // initialize the periodic timer
    periodictimer_init(
        1,                   // period_s
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

    // read
    app_vars.us_val = us_measure();

    // increment
    app_vars.us_num_reads++;
}
