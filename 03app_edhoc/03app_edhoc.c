#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "board.h"
#include "ntw.h"
#include "lakers.h"
#include "gpio.h"
#include "busywait.h"
#include "nrf52833_bitfields.h"

//=========================== defines =========================================

#define TIMEOUT_S 10

//=========================== typedef =========================================

//=========================== variables =======================================

static const uint8_t CRED_I[] = {0xA2, 0x02, 0x77, 0x34, 0x32, 0x2D, 0x35, 0x30, 0x2D, 0x33, 0x31, 0x2D, 0x46, 0x46, 0x2D, 0x45, 0x46, 0x2D, 0x33, 0x37, 0x2D, 0x33, 0x32, 0x2D, 0x33, 0x39, 0x08, 0xA1, 0x01, 0xA5, 0x01, 0x02, 0x02, 0x41, 0x2B, 0x20, 0x01, 0x21, 0x58, 0x20, 0xAC, 0x75, 0xE9, 0xEC, 0xE3, 0xE5, 0x0B, 0xFC, 0x8E, 0xD6, 0x03, 0x99, 0x88, 0x95, 0x22, 0x40, 0x5C, 0x47, 0xBF, 0x16, 0xDF, 0x96, 0x66, 0x0A, 0x41, 0x29, 0x8C, 0xB4, 0x30, 0x7F, 0x7E, 0xB6, 0x22, 0x58, 0x20, 0x6E, 0x5D, 0xE6, 0x11, 0x38, 0x8A, 0x4B, 0x8A, 0x82, 0x11, 0x33, 0x4A, 0xC7, 0xD3, 0x7E, 0xCB, 0x52, 0xA3, 0x87, 0xD2, 0x57, 0xE6, 0xDB, 0x3C, 0x2A, 0x93, 0xDF, 0x21, 0xFF, 0x3A, 0xFF, 0xC8};
//static const uint8_t CRED_R[] = {0xA2, 0x02, 0x60, 0x08, 0xA1, 0x01, 0xA5, 0x01, 0x02, 0x02, 0x41, 0x0A, 0x20, 0x01, 0x21, 0x58, 0x20, 0xBB, 0xC3, 0x49, 0x60, 0x52, 0x6E, 0xA4, 0xD3, 0x2E, 0x94, 0x0C, 0xAD, 0x2A, 0x23, 0x41, 0x48, 0xDD, 0xC2, 0x17, 0x91, 0xA1, 0x2A, 0xFB, 0xCB, 0xAC, 0x93, 0x62, 0x20, 0x46, 0xDD, 0x44, 0xF0, 0x22, 0x58, 0x20, 0x45, 0x19, 0xE2, 0x57, 0x23, 0x6B, 0x2A, 0x0C, 0xE2, 0x02, 0x3F, 0x09, 0x31, 0xF1, 0xF3, 0x86, 0xCA, 0x7A, 0xFD, 0xA6, 0x4F, 0xCD, 0xE0, 0x10, 0x8C, 0x22, 0x4C, 0x51, 0xEA, 0xBF, 0x60, 0x72};
static const BytesP256ElemLen I = {0xfb, 0x13, 0xad, 0xeb, 0x65, 0x18, 0xce, 0xe5, 0xf8, 0x84, 0x17, 0x66, 0x08, 0x41, 0x14, 0x2e, 0x83, 0x0a, 0x81, 0xfe, 0x33, 0x43, 0x80, 0xa9, 0x53, 0x40, 0x6a, 0x13, 0x05, 0xe8, 0x70, 0x6b};

// for EAD authz
static const uint8_t ID_U[4] = {0xa1, 0x04, 0x41, 0x01};
static const size_t ID_U_LEN = sizeof(ID_U) / sizeof(ID_U[0]);
static const BytesP256ElemLen G_W = {0xFF, 0xA4, 0xF1, 0x02, 0x13, 0x40, 0x29, 0xB3, 0xB1, 0x56, 0x89, 0x0B, 0x88, 0xC9, 0xD9, 0x61, 0x95, 0x01, 0x19, 0x65, 0x74, 0x17, 0x4D, 0xCB, 0x68, 0xA0, 0x7D, 0xB0, 0x58, 0x8E, 0x4D, 0x41};
static const uint8_t LOC_W[] = "http://localhost:18000";
static const uint8_t LOC_W_LEN = (sizeof(LOC_W) / sizeof(LOC_W[0])) - 1; // -1 to discard the \0 at the end
static const uint8_t SS = 2;

typedef struct {
    bool               handshake_performed;
    bool               msg_rcvd;
    EdhocMessageBuffer message;
    uint8_t            txCounter;
    uint16_t           *mote_id;
    bool               joined;
    bool               time_rcvd;
    uint32_t           utcSecs_offset;
    uint32_t           utcUsecs_offset;
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
void _ntw_joining_cb(void);
void _ntw_getMoteId_cb(dn_ipmt_getParameter_moteId_rpt* reply);
void _ntw_time_cb(dn_ipmt_getParameter_time_rpt* reply);
void _ntw_receive_cb(uint8_t* buf, uint8_t bufLen);
EdhocMessageBuffer* _send_edhoc_message(EdhocMessageBuffer *message, bool message_1, int timeout_s, uint8_t c_r);
EdhocMessageBuffer* _send_eap_message(uint8_t payload[MAX_MESSAGE_SIZE_LEN], uint8_t payload_len);

//========================= evaluation ========================================
uint32_t _timestamp(void) {
    NRF_TIMER0->TASKS_CAPTURE[0] = 1;
    return NRF_TIMER0->CC[0];
}
void encode_time(uint32_t total_microseconds, uint8_t* buffer) {
    // Split into seconds and microseconds
    uint32_t seconds = total_microseconds / 1000000;    // Get the seconds component
    uint32_t microseconds = total_microseconds % 1000000; // Get the remaining microseconds

    // account for the offset
    microseconds += app_vars.utcUsecs_offset;
    if (microseconds >= 1000000) {
        // handle the carry
        uint32_t carry_secs = microseconds / 1000000;
        seconds += carry_secs;
        microseconds = microseconds % 1000000;
    }
    seconds += app_vars.utcSecs_offset;

    // prefix header indicating time
    buffer[0] = buffer[1] = buffer[2] = buffer[3] = 0xbb;

    // Encode seconds into the buffer (4 bytes for seconds)
    buffer[4] = (seconds >> 24) & 0xFF; // MSB
    buffer[5] = (seconds >> 16) & 0xFF;
    buffer[6] = (seconds >> 8) & 0xFF;
    buffer[7] = seconds & 0xFF;         // LSB

    // Encode microseconds into the buffer (4 bytes for microseconds)
    buffer[8] = (microseconds >> 24) & 0xFF; // MSB
    buffer[9] = (microseconds >> 16) & 0xFF;
    buffer[10] = (microseconds >> 8) & 0xFF;
    buffer[11] = microseconds & 0xFF;         // LSB
}

//=========================== main ============================================

int main(void) {
    // memory buffer for mbedtls, required by crypto-psa-baremetal backend
    uint8_t buffer[4096 * 2] = {0};

    // bsp
    board_init();

    // gpios (not needed when measuring overall duration using RTC and network time)
    gpio_P002_output_init(); // gpio 1 for cpu
    gpio_P003_output_init(); // gpio 2 for cpu
    gpio_P011_output_init(); // gpio 1 for radio
    gpio_P015_output_init(); // gpio 2 for radio
    gpio_P020_output_init(); // gpio for syncing saleae / otii
    gpio_P030_output_init(); // gpio for outer syncing (maybe unneeded?)

    // Configure timer used for timestamping events (should not be used when measuring power consumption)
    NRF_TIMER0->TASKS_CLEAR = 1;
    NRF_TIMER0->PRESCALER   = 4; // adjusts the timer frequency, 2^4 = 16
    NRF_TIMER0->BITMODE     = (TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos); // 32-bit timer
    NRF_TIMER0->INTENSET    = (1 << TIMER_INTENSET_COMPARE0_Pos); // Enable interrupt on compare event
    NVIC_EnableIRQ(TIMER0_IRQn);
    NRF_TIMER0->TASKS_START = 1; // Start the timer

    // initialize the network uC
    ntw_init(_ntw_joining_cb, _ntw_getMoteId_cb, _ntw_time_cb, _ntw_receive_cb);

    // initialize memory buffer for PSA crypto backend
    mbedtls_memory_buffer_alloc_init(buffer, 4096 * 2);

    // initialize variables
    memset(&app_vars,0x00,sizeof(app_vars));
    memset(&app_dbg, 0x00,sizeof(app_dbg));

    // EDHOC credentials
    CredentialC cred_i = {0};
    //CredentialC cred_r = {0};
    IdCred id_cred_r = {0};
    CredentialC fetched_cred_r = {0};

    int res = 0;
    EdhocInitiator initiator = {0};
    EdhocMessageBuffer message_1 = {0};
    EdhocMessageBuffer message_2 = {0};
    EdhocMessageBuffer message_3 = {0};
    EADItemC dummy_ead = {0};
    uint8_t c_r;

    uint8_t prk_out[SHA256_DIGEST_LEN];

    // used for eap-edhoc
    bool eap_res = true;
    EdhocMessageBuffer *signal_joined_response = NULL;
    EdhocMessageBuffer *eap3_start;
    EdhocMessageBuffer *eap5_message_2;
    EdhocMessageBuffer *eap7_message_4;
    EdhocMessageBuffer *eap9_success;


    // only to signal alive again
    uint8_t signal_joined_dummy[MAX_MESSAGE_SIZE_LEN] = { 0xaa, 0xaa, 0xaa, 0xaa };
    uint8_t signal_joined_dummy_len = 4;
    // only to sync time
    uint8_t time_sync_eap_t1[MAX_MESSAGE_SIZE_LEN] = { 0 };
    uint8_t time_sync_eap_t1_len = 12;

    // EAP-Request/Identity: 5 bytes
    uint8_t eap_packet_1[MAX_MESSAGE_SIZE_LEN] = { 0x01, 0x01, 0x00, 0x05, 0x01 };
    uint8_t eap_packet_1_len = 5;
    // EAP-Response/Identity: 16 bytes
    uint8_t eap_packet_2[MAX_MESSAGE_SIZE_LEN] = { 0x02, 0x01, 0x00, 0x10, 0x01, 0x65, 0x78, 0x61, 0x6d, 0x70, 0x6c, 0x65, 0x2e, 0x63, 0x6f, 0x6d };
    uint8_t eap_packet_2_len = 16;
    // EAP-Request/EAP-EDHOC Start: 6 bytes
    uint8_t eap_packet_3[MAX_MESSAGE_SIZE_LEN] = { 0x01, 0x01, 0x00, 0x06, 0x7f, 0x10 };
    uint8_t eap_packet_3_len = 6;
    // EAP-Response/EAP-EDHOC message_1: 7 bytes (header only)
    uint8_t eap_packet_4_header[MAX_MESSAGE_SIZE_LEN] = { 0x02, 0x01, 0x00, 0x2c, 0x7f, 0x01, 0x26 };
    uint8_t eap_packet_4_header_len = 7;
    // EAP-Request/EAP-EDHOC message_2: 7 bytes (header only)
    uint8_t eap_packet_5_header[MAX_MESSAGE_SIZE_LEN] = { 0x01, 0x01, 0x00, 0x88, 0x7f, 0x01, 0x82 };
    uint8_t eap_packet_5_header_len = 7;
    // EAP-Response/EAP-EDHOC message_3: 7 bytes (header only)
    uint8_t eap_packet_6_header[MAX_MESSAGE_SIZE_LEN] = { 0x02, 0x01, 0x00, 0x1a, 0x7f, 0x01, 0x14 };
    uint8_t eap_packet_6_header_len = 7;
    // EAP-Request/EAP-EDHOC message_4: 7 bytes (header only)
    uint8_t eap_packet_7_header[MAX_MESSAGE_SIZE_LEN] = { 0x01, 0x01, 0x00, 0x16, 0x7f, 0x01, 0x10 };
    uint8_t eap_packet_7_header_len = 7;
    // EAP-Response/EAP-EDHOC: 6 bytes
    uint8_t eap_packet_8[MAX_MESSAGE_SIZE_LEN] = { 0x02, 0x01, 0x00, 0x06, 0x7f, 0x00 };
    uint8_t eap_packet_8_len = 6;
    // EAP-Success: 4 bytes
    uint8_t eap_packet_9[MAX_MESSAGE_SIZE_LEN] = { 0x03, 0x01, 0x00, 0x04 };
    uint8_t eap_packet_9_len = 4;


    credential_new(&cred_i, CRED_I, 107);
    //credential_new(&cred_r, CRED_R, 84);

    puts("\nacting as edhoc initiator.");


    printf("waiting for oper...");
    while (!ntw_getMoteId()) {
        for (int i = 0; i < 8; i++) { // wait for 1s
            busywait_approx_125ms();
        }
        printf(".");
    };
    puts(" joined!!");

    printf("waiting for time...");
    while (!app_vars.time_rcvd) {
        ntw_getTime();
        for (int i = 0; i < 8; i++) { // wait for 1s
            busywait_approx_125ms();
        }
        printf(".");
    };
    puts(" timed!!");


    // only to signal that the mote has joined
    printf("Signaling mote has joined");
    while (!signal_joined_response) {
        printf(".");
        signal_joined_response = _send_eap_message(signal_joined_dummy, signal_joined_dummy_len);
    }
    puts(" ok");


    gpio_P020_output_high();

    int counter = 0;
    while (1) {
        if (app_vars.msg_rcvd) {
            if (0 == memcmp(app_vars.message.content, eap_packet_1, eap_packet_1_len)) {
                break;
            }
        }
        busywait_approx_125ms();
        counter++;
        if (counter >= 30 * 8) { // magic number 8 to convert from 125 ms ticks to 1s ticks
          return 1;
        }
    }

    uint32_t t0 = _timestamp();

    eap3_start = _send_eap_message(eap_packet_2, eap_packet_2_len);
    if (!eap3_start) return 1;

    gpio_P002_output_high();
    initiator_new(&initiator);
    gpio_P002_output_low();

    // Send message_1
    if (!res) {
        gpio_P003_output_high();
        res = initiator_prepare_message_1(&initiator, NULL, NULL, &message_1);
        gpio_P003_output_low();
    } else {
      return 1;
    }
    if (res != 0) {
      return 1;
    }
    // NOTE prepend eap stuff
    //message_2 = _send_edhoc_message(&message_1, true, 10, 0xf5);
    eap_packet_4_header[eap_packet_4_header_len] = 0xf5;
    memcpy(&eap_packet_4_header[eap_packet_4_header_len + 1], message_1.content, message_1.len);
    eap5_message_2 = _send_eap_message(eap_packet_4_header, eap_packet_4_header_len + 1 + message_1.len);

    // Parse message_2 + credential_check_or_fetch
    if (eap5_message_2) {
        gpio_P002_output_high();
        message_2.len = eap5_message_2->len-eap_packet_5_header_len;
        memcpy(message_2.content, &eap5_message_2->content[eap_packet_5_header_len], message_2.len);
        res = initiator_parse_message_2(&initiator, &message_2, &c_r, &id_cred_r, &dummy_ead);
    } else {
        // Error while sending
        return 1;
    }

    // credential_check_or_fetch
    if (!res) {
        res = credential_check_or_fetch(NULL, &id_cred_r, &fetched_cred_r);
        gpio_P002_output_low();
    } else {
      return 1;
    }

    // verify message_2
    if (!res) {
        gpio_P003_output_high();
        res = initiator_verify_message_2(&initiator, &I, &cred_i, &fetched_cred_r);
        gpio_P003_output_low();
    } else {
      return 1;
    }

    // prepare message_3
    if(!res) {
      gpio_P002_output_high();
      res = initiator_prepare_message_3(&initiator, ByReference, NULL, &message_3, &prk_out);
      gpio_P002_output_low();
    } else {
      return 1;
    }

    // NOTE prepend eap stuff
    if (!res) {
      //_send_edhoc_message(&message_3, false, 10, c_r);
      eap_packet_6_header[eap_packet_6_header_len] = c_r;
      memcpy(&eap_packet_6_header[eap_packet_6_header_len + 1], message_3.content, message_3.len);
      eap7_message_4 = _send_eap_message(eap_packet_6_header, eap_packet_6_header_len + 1 + message_3.len);
    } else {
      return 1;
    }

    if (eap7_message_4) {
        eap9_success = _send_eap_message(eap_packet_8, eap_packet_8_len);
    } else {
        // Error while sending
        return 1;
    }

    if (!eap9_success) return 1;
    uint32_t t1 = _timestamp();

    printf("Time diff: %zu\n", t1-t0); // give more or less an idea of the overall duration

    // only to sync time
    encode_time(t1, time_sync_eap_t1);
    _send_eap_message(time_sync_eap_t1, time_sync_eap_t1_len);

    gpio_P020_output_low();

    printf("done! will re-do! ");

    // wait for 20 seconds to give chance for manager to reset, then reset the mote
    for (int i = 0; i < 20 * 8; i++) {
        printf(".");
        busywait_approx_125ms();
    }
    printf("\n");
    busywait_approx_125ms();

    NVIC_SystemReset();

    // main loop
    while(1) {
        // wait for event
        board_sleep();
    }
}

//=========================== private =========================================

void _ntw_joining_cb(void) {
    //puts("joined!");
}

void _ntw_getMoteId_cb(dn_ipmt_getParameter_moteId_rpt* reply) {
    //puts("got mote id");
}

void _ntw_time_cb(dn_ipmt_getParameter_time_rpt* reply) {
    printf(" received time ");
    if (reply->utcUsecs) {
        uint32_t ts = _timestamp(); // Get the current timestamp in microseconds
        // Split ts into seconds and microseconds components
        uint32_t ts_secs = ts / 1000000;
        uint32_t ts_usecs = ts % 1000000;

        // Detect if carry would be needed and return early
        if (reply->utcUsecs < ts_usecs) {
            printf(" ignore-carry ");
            return;  // Early return if carry is needed
        }
        app_vars.time_rcvd = true;

        // Calculate the difference in microseconds (no carry needed)
        app_vars.utcUsecs_offset = reply->utcUsecs - ts_usecs;

        // Calculate the seconds difference
        uint32_t secs = 0;
        for (int i = 4; i < 8; i++) {
            secs = (secs << 8) | reply->utcSecs[i];
        }
        app_vars.utcSecs_offset = secs - ts_secs;

        // Print debug information
        puts("");
        printf("ts:     secs: %u, usecs: %u\n", ts_secs, ts_usecs);
        printf("reply:  secs: %u, usecs: %u\n", secs, reply->utcUsecs);
        printf("offset: secs: %u, usecs: %u\n", app_vars.utcSecs_offset, app_vars.utcUsecs_offset);
    }
}


EdhocMessageBuffer* _send_edhoc_message(EdhocMessageBuffer *message, bool message_1, int timeout_s, uint8_t c_r) {
   int counter = 0;
   bool ntw_success = false;
   bool timeout_occured = false;
   uint8_t payload[MAX_MESSAGE_SIZE_LEN] = {0};

   payload[0] = c_r;
   memcpy(&payload[1], message->content, message->len);
   gpio_P011_output_high();
   while (!ntw_success) {
        ntw_success = ntw_transmit(payload, message->len + 1);
   }
   gpio_P011_output_low();

   if (message_1) {
      gpio_P015_output_high();
      while (!app_vars.msg_rcvd) {
          busywait_approx_125ms();
          counter++;
          if (counter >= timeout_s * 8) { // magic number 8 to convert from 125 ms ticks to 1s ticks
            timeout_occured = true;
            break;
          }
      }
      gpio_P015_output_low();
   }

   return timeout_occured || !message_1 ? NULL : &app_vars.message;
}

EdhocMessageBuffer* _send_eap_message(uint8_t payload[MAX_MESSAGE_SIZE_LEN], uint8_t payload_len) {
  int timeout_s = 10;
  int counter = 0;
  bool ntw_success = false;
  bool timeout_occured = false;

  app_vars.msg_rcvd = false;
  memset(&app_vars.message.content, 0x00, sizeof(app_vars.message.content));
  app_vars.message.len = 0;

  gpio_P011_output_high();
  while (!ntw_success) {
      ntw_success = ntw_transmit(payload, payload_len);
  }
  gpio_P011_output_low();

  gpio_P015_output_high();
  while (!app_vars.msg_rcvd) {
      busywait_approx_125ms();
      counter++;
      if (counter >= timeout_s * 8) { // magic number 8 to convert from 125 ms ticks to 1s ticks
        timeout_occured = true;
        break;
      }
  }
  gpio_P015_output_low();

  return timeout_occured ? NULL : &app_vars.message;
}

void _ntw_receive_cb(uint8_t* buf, uint8_t bufLen) {
    static bool reassembly_in_progress = false;
    static uint8_t reassembled_len = 0;

    uint8_t header = buf[0]; // First byte is the header
    uint8_t* payload = &buf[1]; // The actual message payload
    uint8_t payload_len = bufLen - 1; // Adjusted length for the payload

    // No fragmentation
    if (header == 0x00) {
        if (payload_len <= MAX_MESSAGE_SIZE_LEN) {
            memcpy(app_vars.message.content, payload, payload_len);
            app_vars.message.len = payload_len;
        } else {
            reassembly_in_progress = false;
            reassembled_len = 0;
        }
    }
    // Start of the message
    else if (header == 0x01) {
        reassembly_in_progress = true;
        reassembled_len = 0;
        if (payload_len <= MAX_MESSAGE_SIZE_LEN) {
            memcpy(app_vars.message.content, payload, payload_len);
            reassembled_len = payload_len;
        } else {
            reassembly_in_progress = false;
            reassembled_len = 0;
        }
    }
    // Middle of the message
    else if (header == 0x02 && reassembly_in_progress) {
        if ((reassembled_len + payload_len) <= MAX_MESSAGE_SIZE_LEN) {
            memcpy(&app_vars.message.content[reassembled_len], payload, payload_len);
            reassembled_len += payload_len;
        } else {
            reassembly_in_progress = false;
            reassembled_len = 0;
        }
    }
    // End of the message
    else if (header == 0x03 && reassembly_in_progress) {
        if ((reassembled_len + payload_len) <= MAX_MESSAGE_SIZE_LEN) {
            memcpy(&app_vars.message.content[reassembled_len], payload, payload_len);
            reassembled_len += payload_len;
            app_vars.message.len = reassembled_len;
            reassembly_in_progress = false;
        } else {
            reassembly_in_progress = false;
            reassembled_len = 0;
        }
    }
    // Invalid or unexpected fragment
    else {
        reassembly_in_progress = false;
        reassembled_len = 0;
        app_vars.message.len = 0;
    }

    // If the message is fully reassembled
    if (!reassembly_in_progress && app_vars.message.len > 0) {
        app_vars.msg_rcvd = true;
        app_dbg.numReceive++;
    } else {
        app_vars.msg_rcvd = false;
    }
}
