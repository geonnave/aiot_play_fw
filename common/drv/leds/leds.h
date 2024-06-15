#ifndef __LEDS_H
#define __LEDS_H

#include "gpio.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

void leds_init(void);
void leds_white_on(void);
void leds_white_off(void);
void leds_green_on(void);
void leds_green_off(void);
void leds_blue_on(void);
void leds_blue_off(void);
void leds_red_on(void);
void leds_red_off(void);

#endif