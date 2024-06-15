#include <string.h>
#include "gpio.h"

//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== public ==========================================

void leds_init(void) {
    
    gpio_P002_output_init(); // leds_0
    gpio_P003_output_init(); // leds_1
    gpio_P004_output_init(); // leds_2
}

void leds_0_on(void) {
    gpio_P002_output_high();
}

void leds_0_off(void) {
    gpio_P002_output_low();
}

void leds_1_on(void) {
    gpio_P003_output_high();
}

void leds_1_off(void) {
    gpio_P003_output_low();
}

void leds_2_on(void) {
    gpio_P004_output_high();
}

void leds_2_off(void) {
    gpio_P004_output_low();
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================
