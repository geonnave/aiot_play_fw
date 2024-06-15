#ifndef __PIR_H
#define __PIR_H

#include "gpio.h"

//=========================== define ==========================================

typedef enum {
    PIR_STATE_OCCUPIED,
    PIR_STATE_FREE
} pir_state_t;

//=========================== typedef =========================================

typedef void (*pir_cbt)(pir_state_t);

//=========================== variables =======================================

//=========================== prototypes ======================================

void pir_init(pir_cbt pir_cb);

#endif
