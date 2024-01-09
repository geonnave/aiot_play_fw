#include <string.h>
#include <stdbool.h>
#include "board.h"
#include "pwm.h"

//=========================== defines =========================================

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== main ============================================

int main(void) {
    
    // bsp
    board_init();

    // pwm
    pwm_init();
    pwm_setperiod(NOTE_SI_3);
      
    // main loop
    while(1) {

        // wait for event
        board_sleep();
    }
}

//=========================== private =========================================
