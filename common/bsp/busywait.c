#include "busywait.h"

//=========================== define ==========================================

#define DELAY_1s    0x003fffff
#define DELAY_500ms (DELAY_1s>>1)
#define DELAY_250ms (DELAY_1s>>2)
#define DELAY_125ms (DELAY_1s>>3)

//=========================== variables =======================================

//=========================== prototypes ======================================

static void _busywait(uint32_t delay);

//=========================== public ==========================================

void busywait_approx_1s(void)     { _busywait(DELAY_1s);    }
void busywait_approx_500ms(void)  { _busywait(DELAY_500ms); }
void busywait_approx_250ms(void)  { _busywait(DELAY_250ms); }
void busywait_approx_125ms(void)  { _busywait(DELAY_125ms); }

//=========================== private =========================================

static void _busywait(uint32_t delay) {
   volatile uint32_t counter;

   for (counter=0; counter<delay; counter++) {
       __NOP();
   }
}

//=========================== interrupt handlers ==============================
