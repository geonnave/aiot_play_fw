#ifndef __PERIODICTIMER_H
#define __PERIODICTIMER_H

#include "nrf52833.h"

//=========================== define ==========================================

//=========================== typedef =========================================

typedef void (*periodtimer_cbt)(void);

//=========================== variables =======================================

//=========================== prototypes ======================================

void periodictimer_init(uint16_t period_s, periodtimer_cbt periodtimer_cb);

/**
\}
\}
*/

#endif