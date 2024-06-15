#include <string.h>
#include "leds.h"
#include "gpio.h"

//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== public ==========================================

void leds_init(void) {
    
    gpio_P028_output_init(); // leds_white
    leds_white_off();
    gpio_P004_output_init(); // leds_green
    leds_green_off();
    gpio_P011_output_init(); // leds_blue
    leds_blue_off();
    gpio_P005_output_init(); // leds_red
    leds_red_off();
}

void leds_white_on(void) {
    gpio_P028_output_low();
}

void leds_white_off(void) {
    gpio_P028_output_high();
}

void leds_green_on(void) {
    gpio_P004_output_low();
}

void leds_green_off(void) {
    gpio_P004_output_high();
}

void leds_blue_on(void) {
    gpio_P005_output_low();
}

void leds_blue_off(void) {
    gpio_P005_output_high();
}

void leds_red_on(void) {
    gpio_P011_output_low();
}

void leds_red_off(void) {
    gpio_P011_output_high();
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================