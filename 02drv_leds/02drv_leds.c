#include <string.h>
#include <stdbool.h>
#include "board.h"
#include "periodictimer.h"
#include "leds.h"

//=========================== defines =========================================

#define BLINK_PERIOD_S 2

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

void _periodtimer_cb(void);

//=========================== main ============================================

int main(void) {
    
    // bsp
    board_init();

    // leds
    leds_init();

    // initialize the periodic timer
    periodictimer_init(
        BLINK_PERIOD_S,      // period_s
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
    leds_0_on();
    leds_1_on();
    leds_2_on();
}
