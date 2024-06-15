#include <string.h>
#include "pir.h"
#include "gpio.h"

//=========================== variables =======================================

typedef struct {
    pir_cbt pir_cb;
} pir_vars_t;

pir_vars_t pir_vars;

//=========================== prototypes ======================================

void _gpio_cb(uint8_t pin_state);

//=========================== public ==========================================

void pir_init(pir_cbt pir_cb) {
    
    // store params
    pir_vars.pir_cb = pir_cb;
    
    // gpio
    gpio_init();
    gpio_P005_input_init(_gpio_cb);
}

//=========================== private =========================================

void _gpio_cb(uint8_t pin_state) {
    pir_state_t pir_state;
    
    if (pin_state==0x01) {
        pir_state = PIR_STATE_OCCUPIED;
    } else {
        pir_state = PIR_STATE_FREE;
    }

    pir_vars.pir_cb(pir_state);
}

//=========================== interrupt handlers ==============================
