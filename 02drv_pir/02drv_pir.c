#include <string.h>
#include <stdbool.h>
#include "board.h"
#include "pir.h"

//=========================== defines =========================================

//=========================== typedef =========================================

//=========================== variables =======================================

typedef struct {
    pir_state_t pir_state;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void _pir_cb(pir_state_t pir_state);

//=========================== main ============================================

int main(void) {
    
    // bsp
    board_init();

    // pir
    pir_init(_pir_cb);
      
    // main loop
    while(1) {

        // wait for event
        board_sleep();
    }
}

//=========================== private =========================================

void _pir_cb(pir_state_t pir_state) {
    app_vars.pir_state = pir_state;
}
