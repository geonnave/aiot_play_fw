#include "nrf52840.h"
#include "dn_uart.h"

//=========================== variables =======================================

typedef struct {
   dn_uart_rxByte_cbt   ipmt_uart_rxByte_cb;
} dn_uart_vars_t;

dn_uart_vars_t dn_uart_vars;

typedef struct {
    uint8_t             rx_history_idx;
    uint8_t             rx_history[256];
    uint32_t            num_UART_TXed_bytes;
    uint32_t            num_ISR_UARTE0_UART0_IRQHandler;
    uint32_t            num_ISR_UARTE0_UART0_IRQHandler_RXDRDY;
} dn_uart_dbg_t;

dn_uart_dbg_t dn_uart_dbg;

//=========================== prototypes ======================================

//=========================== public ==========================================

void dn_uart_init(dn_uart_rxByte_cbt rxByte_cb){
   
    // remember callback function
    dn_uart_vars.ipmt_uart_rxByte_cb = rxByte_cb;
   
    // initialize the UART
    // do after HFCLOCK started

    //=== UART0 to mote
    
    // <==  TX: P0.09
    //  ==> RX: P0.10
    
    // configure
    NRF_UART0->ENABLE                  = 0x00000004; // 0x00000004==enable
    NRF_UART0->PSEL.TXD                = 0x00000009; // 0x00000009==P0.09
    NRF_UART0->PSEL.RXD                = 0x0000000a; // 0x0000000a==P0.02
    NRF_UART0->CONFIG                  = 0x00000000; // 0x00000000==no flow control, no parity bits, 1 stop bit
    NRF_UART0->BAUDRATE                = 0x01D7E000; // 0x01D7E000==115200 baud (actual rate: 115942)
    NRF_UART0->TASKS_STARTTX           = 0x00000001; // 0x00000001==start TX state machine; write to TXD to send
    NRF_UART0->TASKS_STARTRX           = 0x00000001; // 0x00000001==start RX state machine; read received byte from RXD register
    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... ...0 A: CTS
    // .... .... .... .... .... .... .... ..0. B: NCTS
    // .... .... .... .... .... .... .... .1.. C: RXDRDY
    // .... .... .... .... .... .... 0... .... D: TXDRDY
    // .... .... .... .... .... ..0. .... .... E: ERROR
    // .... .... .... ..0. .... .... .... .... F: RXTO
    // xxxx xxxx xxxx xx0x xxxx xx0x 0xxx x100 
    //    0    0    0    0    0    0    0    4 0x00000004
    NRF_UART0->INTENSET                = 0x00000004;
    
    // enable interrupts
    NVIC_SetPriority(UARTE0_UART0_IRQn, 1);
    NVIC_ClearPendingIRQ(UARTE0_UART0_IRQn);
    NVIC_EnableIRQ(UARTE0_UART0_IRQn);
}

void dn_uart_txByte(uint8_t byte){
    // blocking
   
    NRF_UART0->EVENTS_TXDRDY = 0x00000000;
    NRF_UART0-> TXD          = byte;
    while(NRF_UART0->EVENTS_TXDRDY == 0x00000000);

    // debug
    dn_uart_dbg.num_UART_TXed_bytes++;
}

void dn_uart_txFlush(){
   // nothing to do since nRF UART driver is byte-oriented
}

//=========================== private =========================================

//=========================== helpers =========================================

//=========================== interrupt handlers ==============================

void UARTE0_UART0_IRQHandler(void) {
    uint8_t rxByte;

    // debug
    dn_uart_dbg.num_ISR_UARTE0_UART0_IRQHandler++;

    if (NRF_UART0->EVENTS_RXDRDY == 0x00000001) {
        // byte received from computer

        // clear
        NRF_UART0->EVENTS_RXDRDY = 0x00000000;

        // read
        rxByte = NRF_UART0->RXD;

        // debug
        dn_uart_dbg.rx_history[dn_uart_dbg.rx_history_idx++] = rxByte;

        // read
        dn_uart_vars.ipmt_uart_rxByte_cb(rxByte);

        // debug
        dn_uart_dbg.num_ISR_UARTE0_UART0_IRQHandler_RXDRDY++;
    }
}
