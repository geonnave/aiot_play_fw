/*
 * ================================================================================================
 *  WARNING: This file is automatically generated by cbindgen. Manual edits are likely to be lost.
 * ================================================================================================
 */

#ifndef LAKERS_SHARED_H
#define LAKERS_SHARED_H

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#if !defined(QUADRUPLE_SIZES)
/**
 * Configured upscaling applied to fixed-size buffers
 *
 * Do not rely on this: It is only pub because cbindgen needs it.
 */
#define SCALE_FACTOR 1
#endif

#if defined(QUADRUPLE_SIZES)
#define SCALE_FACTOR 4
#endif

#define MAX_MESSAGE_SIZE_LEN (SCALE_FACTOR * (128 + 64))

#define ID_CRED_LEN 4

#define SUITES_LEN 9

#define SUPPORTED_SUITES_LEN 1

#define EDHOC_METHOD 3

#define P256_ELEM_LEN 32

#define SHA256_DIGEST_LEN 32

#define AES_CCM_KEY_LEN 16

#define AES_CCM_IV_LEN 13

#define AES_CCM_TAG_LEN 8

#define MAC_LENGTH 8

#define MAC_LENGTH_2 MAC_LENGTH

#define MAC_LENGTH_3 MAC_LENGTH_2

#define ENCODED_VOUCHER_LEN (1 + MAC_LENGTH)

#define MAX_KDF_CONTEXT_LEN (SCALE_FACTOR * 256)

#define MAX_KDF_LABEL_LEN 15

#define MAX_BUFFER_LEN ((SCALE_FACTOR * 256) + 64)

#define CBOR_BYTE_STRING 88

#define CBOR_TEXT_STRING 120

#define CBOR_UINT_1BYTE 24

#define CBOR_NEG_INT_1BYTE_START 32

#define CBOR_NEG_INT_1BYTE_END 55

#define CBOR_UINT_1BYTE_START 0

#define CBOR_UINT_1BYTE_END 23

#define CBOR_MAJOR_TEXT_STRING 96

#define CBOR_MAJOR_BYTE_STRING 64

#define CBOR_MAJOR_BYTE_STRING_MAX 87

#define CBOR_MAJOR_ARRAY 128

#define CBOR_MAJOR_ARRAY_MAX 151

#define CBOR_MAJOR_MAP 160

#define MAX_INFO_LEN ((((((2 + SHA256_DIGEST_LEN) + 1) + MAX_KDF_LABEL_LEN) + 1) + MAX_KDF_CONTEXT_LEN) + 1)

#define KCSS_LABEL 14

#define KID_LABEL 4

#define ENC_STRUCTURE_LEN ((8 + 5) + SHA256_DIGEST_LEN)

#define MAX_EAD_SIZE_LEN (SCALE_FACTOR * 64)

#define MAX_SUITES_LEN 9

typedef enum CredentialTransfer {
  ByReference,
  ByValue,
} CredentialTransfer;

typedef enum CredentialType {
  CCS,
  CCS_PSK,
} CredentialType;

/**
 * An owned u8 vector of a limited length
 *
 * It is used to represent the various messages in encrypted and in decrypted form, as well as
 * other data items. Its maximum length is [MAX_MESSAGE_SIZE_LEN].
 */
typedef struct EdhocMessageBuffer {
  uint8_t content[MAX_MESSAGE_SIZE_LEN];
  uintptr_t len;
} EdhocMessageBuffer;

typedef uint8_t BytesMac[MAC_LENGTH];

typedef uint8_t BytesMac2[MAC_LENGTH_2];

/**
 * A fixed-size (but parameterized) buffer for EDHOC messages.
 *
 * Trying to have an API as similar as possible to `heapless::Vec`,
 * so that in the future it can be hot-swappable by the application.
 */
typedef struct EdhocBuffer_16 {
  uint8_t content[16];
  uintptr_t len;
} EdhocBuffer_16;

typedef struct EdhocBuffer_16 BufferKid;

/**
 * A fixed-size (but parameterized) buffer for EDHOC messages.
 *
 * Trying to have an API as similar as possible to `heapless::Vec`,
 * so that in the future it can be hot-swappable by the application.
 */
typedef struct EdhocBuffer_192 {
  uint8_t content[192];
  uintptr_t len;
} EdhocBuffer_192;

typedef struct EdhocBuffer_192 BufferCred;

typedef struct EdhocBuffer_192 BufferIdCred;

typedef uint8_t BytesKeyAES128[16];

typedef uint8_t BytesKeyEC2[32];

typedef enum CredentialKey_Tag {
  Symmetric,
  EC2Compact,
} CredentialKey_Tag;

typedef struct CredentialKey {
  CredentialKey_Tag tag;
  union {
    struct {
      BytesKeyAES128 symmetric;
    };
    struct {
      BytesKeyEC2 ec2_compact;
    };
  };
} CredentialKey;

/**
 * A value of ID_CRED_x: a credential identifier
 *
 * Possible values include key IDs, credentials by value and others.
 */
typedef struct IdCred {
  /**
   * The value is always stored in the ID_CRED_x form as a serialized one-element dictionary;
   * while this technically wastes two bytes, it has the convenient property of having the full
   * value available as a slice.
   */
  BufferIdCred bytes;
} IdCred;

/**
 * A fixed-size (but parameterized) buffer for EDHOC messages.
 *
 * Trying to have an API as similar as possible to `heapless::Vec`,
 * so that in the future it can be hot-swappable by the application.
 */
typedef struct EdhocBuffer_MAX_SUITES_LEN {
  uint8_t content[MAX_SUITES_LEN];
  uintptr_t len;
} EdhocBuffer_MAX_SUITES_LEN;

typedef uint8_t BytesP256ElemLen[P256_ELEM_LEN];

typedef struct InitiatorStart {
  struct EdhocBuffer_MAX_SUITES_LEN suites_i;
  uint8_t method;
  BytesP256ElemLen x;
  BytesP256ElemLen g_x;
} InitiatorStart;

typedef uint8_t BytesHashLen[SHA256_DIGEST_LEN];

typedef struct WaitM2 {
  BytesP256ElemLen x;
  BytesHashLen h_message_1;
} WaitM2;

typedef struct Completed {
  BytesHashLen prk_out;
  BytesHashLen prk_exporter;
} Completed;

typedef struct ProcessedM2 {
  BytesHashLen prk_3e2m;
  BytesHashLen prk_4e3m;
  BytesHashLen th_3;
} ProcessedM2;

#endif /* LAKERS_SHARED_H */
