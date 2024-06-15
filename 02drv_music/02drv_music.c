#include <string.h>
#include <stdbool.h>
#include "board.h"
#include "music.h"
#include "periodictimer.h"

//=========================== defines =========================================

#define PLAY_PERIOD_S 60

//=========================== typedef =========================================

//=========================== variables =======================================

uint8_t track;

//=========================== prototypes ======================================

void _periodtimer_cb(void);

//=========================== main ============================================

int main(void) {
    
    // bsp
    board_init();

    // music
    music_init();

    // initialize the periodic timer
    music_play(SONGTITLE_HARRY_POTTER,track++);
    periodictimer_init(
        PLAY_PERIOD_S,       // period_s
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
     music_play(SONGTITLE_HARRY_POTTER,track++);
}