#ifndef HW_XBOX360_CRYPTO_H
#define HW_XBOX360_CRYPTO_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define XEKEY_1BL_KEY        0x00
#define XEKEY_1BL_CF_KEY     0x01
#define XEKEY_1BL_SD_KEY     0x02
#define XEKEY_KEY_VAULT_KET  0x03
#define XEKEY_ODD_KEY        0x04
#define XEKEY_EVEN_KEY       0x05
#define XEKEY_CPU_KEY        0x06
#define XEKEY_CONSOLE_KEY    0x07
#define XEKEY_CONSOLE_ID     0x08
#define XEKEY_ROAMABLE_ODD_KEY 0x09
#define XEKEY_ROAMABLE_EVEN_KEY 0x0A
#define XEKEY_DVD_KEY        0x0B
#define XEKEY_MFG_DVD_KEY    0x0C
#define XEKEY_MFG_CHALLANGE_RESP 0x0D
#define XEKEY_KEY_VAULT_PRIV 0x0E
#define XEKEY_KEY_VAULT_PUB  0x0F
#define XEKEY_SYSTEM_ROOT_KEY 0x10

#define XECRYPT_SHA_DIGEST_SIZE   20
#define XECRYPT_SHA_CTX_SIZE      96
#define XECRYPT_MD5_DIGEST_SIZE   16
#define XECRYPT_MD5_CTX_SIZE      88
#define XECRYPT_RC4_KEY_SIZE      256
#define XECRYPT_DES3_KEY_SIZE     24
#define XECRYPT_DES3_BLOCK_SIZE   8
#define XECRYPT_AES_BLOCK_SIZE    16
#define XECRYPT_AES_ROUNDS_128    10
#define XECRYPT_AES_ROUNDS_192    12
#define XECRYPT_AES_ROUNDS_256    14

typedef struct XECRYPT_SHA_STATE {
  uint32_t count[2];
  uint32_t state[5];
  uint8_t buffer[64];
}

typedef struct XECRYPT_MD5_STATE {
    uint32_t state[4];
    uint32_t count[2];
    uint8_t buffer[64];
} XECRYPT_MD5_STATE;

typedef struct XECRYPT_RC4_STATE {
    uint8_t S[256];
    uint8_t i, j;
} XECRYPT_RC4_STATE;

typedef struct XECRYPT_AES_STATE {
    uint32_t Nr;     // Rounds
    uint32_t ek[60]; // Expanded key
    uint32_t dk[60]; // Decryption key
} XECRYPT_AES_STATE;

typedef struct XECRYPT_DES3_STATE {
    uint32_t ek[3][32];
    uint32_t dk[3][32];
} XECRYPT_DES3_STATE;

/* ========= SHA-1 ========= (9) */
void xeCryptShaInit(XECRYPT_SHA_STATE *state);
void xeCryptShaUpdate(XECRYPT_SHA_STATE *state, const uint8_t *data, size_t len);
void xeCryptShaFinal(XECRYPT_SHA_STATE *state, uint8_t *digest);
void xeCryptSha(const uint8_t *data, size_t len, uint8_t digest[20]);

/* ========= MD5 ========= */
void xeCryptMd5Init(XECRYPT_MD5_STATE *state);
void xeCryptMd5Update(XECRYPT_MD5_STATE *state, const uint8_t *data, size_t len);
void xeCryptMd5Final(XECRYPT_MD5_STATE *state, uint8_t digest[16]);
void xeCryptMd5(const uint8_t *data, size_t len, uint8_t digest[16]);

/* ========= RC4 ========= */
void xeCryptRc4Key(XECRYPT_RC4_STATE *state, const uint8_t *key, size_t key_len);
void xeCryptRc4Ecb(XECRYPT_RC4_STATE *state, uint8_t *data, size_t len);
void xeCryptRc4(const uint8_t *key, size_t key_len, uint8_t *data, size_t len);

/* ======== AES ========= */
void xeCryptAesKey(XECRYPT_AES_STATE *state, const uint8_t *key, size_t key_len);
void xeCryptAesEncryptBlock(XECRYPT_AES_STATE *state, const uint8_t *in, uint8_t *out);
void xeCryptAesDecryptBlock(XECRYPT_AES_STATE *state, const uint8_t *in, uint8_t *out);
void xeCryptAesCbcEncrypt(XECRYPT_AES_STATE *state, uint8_t *iv, uint8_t *data, size_t len);
void xeCryptAesCbcDecrypt(XECRYPT_AES_STATE *state, uint8_t *iv, uint8_t *data, size_t len);

/* ========= DES3 ========= */
void xeCryptDes3Key(XECRYPT_DES3_STATE *state, const uint8_t *key);
void xeCryptDes3EcbEncrypt(XECRYPT_DES3_STATE *state, const uint8_t *in, uint8_t *out);
void xeCryptDes3EcbDecrypt(XECRYPT_DES3_STATE *state, const uint8_t *in, uint8_t *out);
void xeCryptDes3CbcEncrypt(XECRYPT_DES3_STATE *state, uint8_t *iv, uint8_t *data, size_t len);
void xeCryptDes3CbcDecrypt(XECRYPT_DES3_STATE *state, uint8_t *iv, uint8_t *data, size_t len);

/* ========= XBOX SPECIFIC ========= */
void xeCryptHmacSha(const uint8_t *key, size_t key_len, const uint8_t *data1, size_t data1_len, const uint8_t *data2, size_t data2_len, uint8_t digest[20]);
void xeCryptRotSumSha(const uint8_t *data1, size_t data1_len, const uint8_t *data2, size_t data2_len, uint8_t digest[20]);
void xeCryptBnQw_SwapDwQwLeBe(uint64_t *qw, uint32_t size);
void xeCryptBnQwNeRsaPubCrypt(const uint64_t *src, uint64_t *dst, const uint8_t *modulus, uint32_t modulus_size, uint64_t exponent);

bool xeCryptRsaVerify(const uint8_t *hash, const uint8_t *signature, const uint8_t *modulus, uint32_t modulus_size, uint64_t exponent);
bool xeCryptVerify1BLSignature(const uint8_t *data, size_t size, const uint8_t *signature);
bool xeCryptVerifyCBSignature(const uint8_t *data, size_t size, const uint8_t *signature);

/* ======== KEY VAULT ======== */
typedef struct XECRYPT_KEY_VAULT {
    uint8_t cpu_key[16];
    uint8_t console_key[16];
    uint8_t console_id[5];
    uint8_t key_vault_key[16];
    uint8_t odd_key[16];
    uint8_t even_key[16];
    uint8_t dvd_key[16];
    uint8_t system_root_key[256];
} XECRYPT_KEY_VAULT;

bool xeKeysInit(XECRYPT_KEY_VAULT *vault, const uint8_t *cpu_key);
bool xeKeysGetKey(XECRYPT_KEY_VAULT *vault, uint32_t key_id,
                 uint8_t *output, uint32_t *output_size);
bool xeKeysObtainFuseKey(uint8_t *output, uint32_t key_type);

/* ========== UTILITIES ========= */
void xeCryptRandom(uint8_t *buffer, size_t size);
uint32_t xeCryptCrc32(const uint8_t *data, size_t size, uint32_t seed);

uint64_t xeCryptBytesToQwBe(const uint8_t *bytes);
void xeCryptQwToBytesBe(uint64_t qw, uint8_t *bytes);

#endif
