#include <string.h>
#include "gpio.h"

//=========================== variables =======================================

typedef struct {
    uint8_t        pin_in;
    pin_toggle_cbt pin_toggle_cb;
} gpio_vars_t;

gpio_vars_t gpio_vars;

//=========================== prototypes ======================================

static void _gpio_output_init(uint8_t pin);
static void _gpio_output_high(uint8_t pin);
static void _gpio_output_low(uint8_t pin);
static void _gpio_input_init(uint8_t pin, pin_toggle_cbt pin_toggle_cb);

//=========================== public ==========================================

void gpio_init(void) {
    
    // reset variables
    memset(&gpio_vars,0x00,sizeof(gpio_vars_t));
}

// P0.02
void gpio_P002_output_init(void)                           { _gpio_output_init(02);                }
void gpio_P002_output_high(void)                           { _gpio_output_high(02);                }
void gpio_P002_output_low(void)                            { _gpio_output_low(02);                 }
void gpio_P002_input_init(pin_toggle_cbt pin_toggle_cb)    { _gpio_input_init(02,pin_toggle_cb);   }
// P0.03
void gpio_P003_output_init(void)                           { _gpio_output_init(03);                }
void gpio_P003_output_high(void)                           { _gpio_output_high(03);                }
void gpio_P003_output_low(void)                            { _gpio_output_low(03);                 }
void gpio_P003_input_init(pin_toggle_cbt pin_toggle_cb)    { _gpio_input_init(03,pin_toggle_cb);   }
// P0.04
void gpio_P004_output_init(void)                           { _gpio_output_init(04);                }
void gpio_P004_output_high(void)                           { _gpio_output_high(04);                }
void gpio_P004_output_low(void)                            { _gpio_output_low(04);                 }
void gpio_P004_input_init(pin_toggle_cbt pin_toggle_cb)    { _gpio_input_init(04,pin_toggle_cb);   }
// P0.05
void gpio_P005_output_init(void)                           { _gpio_output_init(05);                }
void gpio_P005_output_high(void)                           { _gpio_output_high(05);                }
void gpio_P005_output_low(void)                            { _gpio_output_low(05);                 }
void gpio_P005_input_init(pin_toggle_cbt pin_toggle_cb)    { _gpio_input_init(05,pin_toggle_cb);   }
// P0.11
void gpio_P011_output_init(void)                           { _gpio_output_init(11);                }
void gpio_P011_output_high(void)                           { _gpio_output_high(11);                }
void gpio_P011_output_low(void)                            { _gpio_output_low(11);                 }
void gpio_P011_input_init(pin_toggle_cbt pin_toggle_cb)    { _gpio_input_init(11,pin_toggle_cb);   }
// P0.15
void gpio_P015_output_init(void)                           { _gpio_output_init(15);                }
void gpio_P015_output_high(void)                           { _gpio_output_high(15);                }
void gpio_P015_output_low(void)                            { _gpio_output_low(15);                 }
void gpio_P015_input_init(pin_toggle_cbt pin_toggle_cb)    { _gpio_input_init(15,pin_toggle_cb);   }
// P0.17
void gpio_P017_output_init(void)                           { _gpio_output_init(17);                }
void gpio_P017_output_high(void)                           { _gpio_output_high(17);                }
void gpio_P017_output_low(void)                            { _gpio_output_low(17);                 }
void gpio_P017_input_init(pin_toggle_cbt pin_toggle_cb)    { _gpio_input_init(17,pin_toggle_cb);   }
// P0.18
void gpio_P018_output_init(void)                           { _gpio_output_init(18);                }
void gpio_P018_output_high(void)                           { _gpio_output_high(18);                }
void gpio_P018_output_low(void)                            { _gpio_output_low(18);                 }
void gpio_P018_input_init(pin_toggle_cbt pin_toggle_cb)    { _gpio_input_init(18,pin_toggle_cb);   }
// P0.19 
void gpio_P019_output_init(void)                           { _gpio_output_init(19);                }
void gpio_P019_output_high(void)                           { _gpio_output_high(19);                }
void gpio_P019_output_low(void)                            { _gpio_output_low(19);                 }
void gpio_P019_input_init(pin_toggle_cbt pin_toggle_cb)    { _gpio_input_init(19,pin_toggle_cb);   }
// P0.20
void gpio_P020_output_init(void)                           { _gpio_output_init(20);                }
void gpio_P020_output_high(void)                           { _gpio_output_high(20);                }
void gpio_P020_output_low(void)                            { _gpio_output_low(20);                 }
void gpio_P020_input_init(pin_toggle_cbt pin_toggle_cb)    { _gpio_input_init(20,pin_toggle_cb);   }
// P0.28
void gpio_P028_output_init(void)                           { _gpio_output_init(28);                }
void gpio_P028_output_high(void)                           { _gpio_output_high(28);                }
void gpio_P028_output_low(void)                            { _gpio_output_low(28);                 }
void gpio_P028_input_init(pin_toggle_cbt pin_toggle_cb)    { _gpio_input_init(28,pin_toggle_cb);   }
// P0.29
void gpio_P029_output_init(void)                           { _gpio_output_init(29);                }
void gpio_P029_output_high(void)                           { _gpio_output_high(29);                }
void gpio_P029_output_low(void)                            { _gpio_output_low(29);                 }
void gpio_P029_input_init(pin_toggle_cbt pin_toggle_cb)    { _gpio_input_init(29,pin_toggle_cb);   }
// P0.30
void gpio_P030_output_init(void)                           { _gpio_output_init(30);                }
void gpio_P030_output_high(void)                           { _gpio_output_high(30);                }
void gpio_P030_output_low(void)                            { _gpio_output_low(30);                 }
void gpio_P030_input_init(pin_toggle_cbt pin_toggle_cb)    { _gpio_input_init(30,pin_toggle_cb);   }

//=========================== private =========================================

void _gpio_output_init(uint8_t pin) {
    NRF_P0->PIN_CNF[pin]          = 0x00000003;
}

void _gpio_output_high(uint8_t pin) {
    NRF_P0->OUTSET                = (0x00000001 << pin);
}

void _gpio_output_low(uint8_t pin) {
    NRF_P0->OUTCLR                = (0x00000001 << pin);
}

void _gpio_input_init(uint8_t pin, pin_toggle_cbt pin_toggle_cb) {

    // store params
    gpio_vars.pin_in         = pin;
    gpio_vars.pin_toggle_cb  = pin_toggle_cb;
    
    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... ...A A: DIR:   0=Input
    // .... .... .... .... .... .... .... ..B. B: INPUT: 0=Connect
    // .... .... .... .... .... .... .... CC.. C: PULL:  0=Disabled
    // .... .... .... .... .... .DDD .... .... D: DRIVE: 0=S0S1
    // .... .... .... ..EE .... .... .... .... E: SENSE: 2=High
    // xxxx xxxx xxxx xx10 xxxx xxxx xxxx 0000 
    //    0    0    0    2    0    0    0    0 0x00020000
    NRF_P0->PIN_CNF[pin]     = 0x00020000;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... ..AA A: MODE:     01=Event
    // .... .... .... .... ...B BBBB .... .... B: PSEL:      pin
    // .... .... .... .... ..C. .... .... .... C: PORT:      0=P0
    // .... .... .... ..DD .... .... .... .... D: POLARITY: 11=Toggle
    // .... .... ...E .... .... .... .... .... E: OUTINIT:  no effect in event mode
    // xxxx xxxx xxx0 xx11 xx0? ???? xxxx xx01 
    //    0    0    0    3    0    0    0    1 0x00030001
    NRF_GPIOTE->CONFIG[0]    = 0x00030001 | (pin << 8);

    // enable interrupts
    NRF_GPIOTE->INTENSET     = 0x00000001;
    NVIC_SetPriority(GPIOTE_IRQn,1);
    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    NVIC_EnableIRQ(GPIOTE_IRQn);
}

//=========================== interrupt handlers ==============================

void GPIOTE_IRQHandler(void) {
    uint8_t pin_state;

    if (NRF_GPIOTE->EVENTS_IN[0] == 0x00000001 ) {
        // pin toggled

        // clear
        NRF_GPIOTE->EVENTS_IN[0] = 0x00000000;

        // read pin state
        pin_state = (NRF_P0->IN & (1<<gpio_vars.pin_in))? 1:0;

        // handle
        if (gpio_vars.pin_toggle_cb!=0x00000000) {
            gpio_vars.pin_toggle_cb(pin_state);
        }
    }
}
