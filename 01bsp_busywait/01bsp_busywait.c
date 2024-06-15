#include <string.h>
#include "board.h"
#include "busywait.h"

//=========================== defines =========================================

//=========================== typedef =========================================

//=========================== variables =======================================

typedef struct {
    uint8_t counter;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

//=========================== main ============================================

int main(void) {

    // clear variables
    memset(&app_vars,0x00,sizeof(app_vars_t));

    // bsp
    board_init();

    // main loop
    while(1) {
        
        // increment
        app_vars.counter++;
        
        // wait
        busywait_approx_500ms();
    }
}

//=========================== private =========================================
