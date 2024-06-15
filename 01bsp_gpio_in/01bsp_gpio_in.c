#include <string.h>
#include "board.h"
#include "gpio.h"
#include "busywait.h"

//=========================== defines =========================================

//=========================== typedef =========================================

//=========================== variables =======================================

typedef struct {
    uint8_t num_events;
    uint8_t pin_state;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void pin_toggle_cb(uint8_t pin_state);

//=========================== main ============================================

int main(void) {
   
    // reset variables
    memset(&app_vars,0x00,sizeof(app_vars_t));

    // bsp
    board_init();

    // P0.02 as input
    gpio_P002_input_init(pin_toggle_cb);

    // main loop
    while(1) {
        
        // sleep
        board_sleep();
    }
}

//=========================== private =========================================

void pin_toggle_cb(uint8_t pin_state) {
    app_vars.num_events++;
    app_vars.pin_state = pin_state;
}