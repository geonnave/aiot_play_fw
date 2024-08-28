#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "board.h"
#include "ntw.h"
#include "lakers.h"
#include "busywait.h"

//=========================== defines =========================================

#define TIMEOUT_S 10

//=========================== typedef =========================================

//=========================== variables =======================================

static const uint8_t CRED_I[] = {0xA2, 0x02, 0x77, 0x34, 0x32, 0x2D, 0x35, 0x30, 0x2D, 0x33, 0x31, 0x2D, 0x46, 0x46, 0x2D, 0x45, 0x46, 0x2D, 0x33, 0x37, 0x2D, 0x33, 0x32, 0x2D, 0x33, 0x39, 0x08, 0xA1, 0x01, 0xA5, 0x01, 0x02, 0x02, 0x41, 0x2B, 0x20, 0x01, 0x21, 0x58, 0x20, 0xAC, 0x75, 0xE9, 0xEC, 0xE3, 0xE5, 0x0B, 0xFC, 0x8E, 0xD6, 0x03, 0x99, 0x88, 0x95, 0x22, 0x40, 0x5C, 0x47, 0xBF, 0x16, 0xDF, 0x96, 0x66, 0x0A, 0x41, 0x29, 0x8C, 0xB4, 0x30, 0x7F, 0x7E, 0xB6, 0x22, 0x58, 0x20, 0x6E, 0x5D, 0xE6, 0x11, 0x38, 0x8A, 0x4B, 0x8A, 0x82, 0x11, 0x33, 0x4A, 0xC7, 0xD3, 0x7E, 0xCB, 0x52, 0xA3, 0x87, 0xD2, 0x57, 0xE6, 0xDB, 0x3C, 0x2A, 0x93, 0xDF, 0x21, 0xFF, 0x3A, 0xFF, 0xC8};
static const uint8_t CRED_R[] = {0xA2, 0x02, 0x60, 0x08, 0xA1, 0x01, 0xA5, 0x01, 0x02, 0x02, 0x41, 0x0A, 0x20, 0x01, 0x21, 0x58, 0x20, 0xBB, 0xC3, 0x49, 0x60, 0x52, 0x6E, 0xA4, 0xD3, 0x2E, 0x94, 0x0C, 0xAD, 0x2A, 0x23, 0x41, 0x48, 0xDD, 0xC2, 0x17, 0x91, 0xA1, 0x2A, 0xFB, 0xCB, 0xAC, 0x93, 0x62, 0x20, 0x46, 0xDD, 0x44, 0xF0, 0x22, 0x58, 0x20, 0x45, 0x19, 0xE2, 0x57, 0x23, 0x6B, 0x2A, 0x0C, 0xE2, 0x02, 0x3F, 0x09, 0x31, 0xF1, 0xF3, 0x86, 0xCA, 0x7A, 0xFD, 0xA6, 0x4F, 0xCD, 0xE0, 0x10, 0x8C, 0x22, 0x4C, 0x51, 0xEA, 0xBF, 0x60, 0x72};
static const BytesP256ElemLen I = {0xfb, 0x13, 0xad, 0xeb, 0x65, 0x18, 0xce, 0xe5, 0xf8, 0x84, 0x17, 0x66, 0x08, 0x41, 0x14, 0x2e, 0x83, 0x0a, 0x81, 0xfe, 0x33, 0x43, 0x80, 0xa9, 0x53, 0x40, 0x6a, 0x13, 0x05, 0xe8, 0x70, 0x6b};

typedef struct {
    bool               handshake_performed;
    bool               msg_rcvd;
    EdhocMessageBuffer message;
    uint8_t            txCounter;
} app_vars_t;

app_vars_t app_vars;

typedef struct {
    uint32_t       numReceive;
    uint32_t       numTransmit;
    uint32_t       numTransmit_success;
    uint32_t       numTransmit_fail;
} app_dbg_t;

app_dbg_t app_dbg;

//=========================== prototypes ======================================

extern void mbedtls_memory_buffer_alloc_init(uint8_t *buf, size_t len);

void _periodtimer_cb(void);
void _ntw_receive_cb(uint8_t* buf, uint8_t bufLen);
EdhocMessageBuffer* _send_edhoc_message(EdhocMessageBuffer *message, bool message_1, int timeout_s, uint8_t c_r);

//=========================== main ============================================

int main(void) {


    // EDHOC credentials
    CredentialC cred_i = {0};
    CredentialC cred_r = {0};
    IdCred id_cred_r = {0};
    CredentialC fetched_cred_r = {0};
    EADItemC dummy_ead = {0};

    int res = 0;
    EdhocInitiator initiator;
    EdhocMessageBuffer message_1;
    EdhocMessageBuffer *message_2;
    EdhocMessageBuffer message_3;
    uint8_t c_r;

    uint8_t prk_out[SHA256_DIGEST_LEN];


    // memory buffer for mbedtls, required by crypto-psa-baremetal backend
    uint8_t buffer[4096 * 2] = {0};

    // initialize variables
    memset(&app_vars,0x00,sizeof(app_vars));
    memset(&app_dbg, 0x00,sizeof(app_dbg));
    
    // bsp
    board_init();

    // initialize the network uC
    ntw_init(NULL, NULL, NULL, _ntw_receive_cb);

    // initialize memory buffer for PSA crypto backend
    mbedtls_memory_buffer_alloc_init(buffer, 4096 * 2);

    credential_new(&cred_i, CRED_I, 107);
    credential_new(&cred_r, CRED_R, 84);

    initiator_new(&initiator);
    printf("acting as edhoc initiator.");

    // Send message_1
    res = initiator_prepare_message_1(&initiator, NULL, NULL, &message_1);
    if (res != 0) {
        return 1;
    }
   
    message_2 = _send_edhoc_message(&message_1, true, 10, 0xf5);
    if (message_2) {
        res = initiator_parse_message_2(&initiator, message_2, &c_r, &id_cred_r, &dummy_ead);
    } else {
        // Error while sending
        return 1;
    }

    // Verify message_2
    if (!res) {
        res = credential_check_or_fetch(&cred_r, &id_cred_r, &fetched_cred_r);
    } else {
      return 1;
    }

    // verify cred
    if (!res) {
        res = initiator_verify_message_2(&initiator, &I, &cred_i, &fetched_cred_r);
    } else {
      return 1;
    }

    if(!res) {
      res = initiator_prepare_message_3(&initiator, ByReference, NULL, &message_3, &prk_out);
    } else {
      return 1;
    }

    if (!res) {
      _send_edhoc_message(&message_3, false, 10, c_r);
    } else {
      return 1;
    }


    // main loop
    while(1) {
        // wait for event
        board_sleep();
    }
}

//=========================== private =========================================

EdhocMessageBuffer* _send_edhoc_message(EdhocMessageBuffer *message, bool message_1, int timeout_s, uint8_t c_r) {
   int counter = 0;
   bool ntw_success = false;
   bool timeout_occured = false;
   uint8_t payload[MAX_MESSAGE_SIZE_LEN] = {0};

   payload[0] = c_r;
   memcpy(&payload[1], message->content, message->len);
   while (!ntw_success) {
        ntw_success = ntw_transmit(payload, message->len + 1);
    }

   if (message_1) {
      while (!app_vars.msg_rcvd) {
          busywait_approx_125ms();
          counter++;
          if (counter >= timeout_s * 8) { // magic number 8 to convert from 125 ms ticks to 1s ticks
            timeout_occured = true;
            break;
          }
      }
   }

   return timeout_occured || !message_1 ? NULL : &app_vars.message;
}

void _ntw_receive_cb(uint8_t* buf, uint8_t bufLen) {
    app_vars.msg_rcvd = true;
    if (bufLen <= MAX_MESSAGE_SIZE_LEN) {
      memcpy(app_vars.message.content, buf, bufLen);
      app_vars.message.len = bufLen;
    } else {
      app_vars.message.len = 0;
    }
    // debug
    app_dbg.numReceive++;
}
