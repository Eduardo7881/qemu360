#include "hw/xbox360/crypto/xe_crypto.h"
#include "hw/xbox360/crypto/xe_keys.h"
#include "qemu/osdep.h"

/* ========= XBOX 360 KEYSET ========= */
static const uint8_t factory_fuses[16] = {
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

static const uint8_t master_key_retail[16] = {  
  0xDD, 0x88, 0xAD, 0x0C, 0x9E, 0xD6, 0x69, 0xE7,
  0xB5, 0x67, 0x94, 0xFB, 0x68, 0x56, 0x3E, 0xFA // like it doesn't make any sense to put this here, but here i'll go... SIX SEVEN WOHOOO
};

static const uint8_t masker_key_devkit[16] = { // Placeholder
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
}

/* ========= FUSE FUNCTIONS ========= */
bool xeKeysGetFuses(uint8_t fuses[12], bool use_factory) {
  if (use_factory) {
    memcpy(fuses, factory_fuses, 12);
  } else {
    // read fuses from emulated hardware
    // stub, only uses factory for now
    memcpy(fuses, factory_fuses, 12);
  }
  return true;
}

uint32_t xeKeysGetKeyType(uint32_t fuse_bits) {
  if ((fuse_bits & 0x80000000) == 0) {
      return KEYTYPE_RETAIL;
  } else {
      return KEYTYPE_DEVKIT;
  }
}

/* ========= KEY DERIVATION ========= */
bool xeKeysDeriveKey(uint8_t *output, uint32_t output_size, const uint8_t *input, uint32_t input_size, uint32_t key_type, uint32_t key_id) {
    XECRYPT_AES_STATE aes;
    uint8_t iv[16] = {0};
    uint8_t key[16];
    uint8_t buffer[256];
    
    if (output_size > 256) return false;
    
    switch (key_type) {
        case KEYTYPE_RETAIL:
            memcpy(key, master_key_retail, 16);
            break;
        case KEYTYPE_DEVKIT:
            memcpy(key, master_key_devkit, 16);
            break;
        default:
            return false;
    }
    
    key[0] ^= (key_id >> 24) & 0xFF;
    key[1] ^= (key_id >> 16) & 0xFF;
    key[2] ^= (key_id >> 8) & 0xFF;
    key[3] ^= key_id & 0xFF;
    
    memcpy(buffer, input, input_size);
    
    xeCryptAesKey(&aes, key, 16);
    xeCryptAesCbcEncrypt(&aes, iv, buffer, input_size);
    
    uint8_t hash[20];
    xeCryptSha(buffer, input_size, hash);
    memcpy(output, hash, (output_size < 20) ? output_size : 20);
    
    return true;
}

/* ========= CPU KEY SPECIFIC ========= */
bool xeKeysObtainFuseKey(uint8_t *output, uint32_t key_type) {
    uint8_t fuses[12];
    uint8_t cpu_key[16];
    
    if (!xeKeysGetFuses(fuses, false)) {
        return false;
    }
    
    if (!xeKeysDeriveKey(cpu_key, 16, fuses, 12, key_type, XEKEY_CPU_KEY)) {
        return false;
    }
    
    for (int i = 0; i < 16; i++) {
        cpu_key[i] = ~cpu_key[i];  // Inverse (simplified)
    }
    
    memcpy(output, cpu_key, 16);
    return true;
}

/* ========= NAND SPECIFIC ========= */
bool xeKeysDecryptNandData(uint8_t *output, const uint8_t *input, size_t size, uint32_t key_id, const uint8_t *cpu_key) {
    XECRYPT_KEY_VAULT vault;
    uint8_t key[16];
    uint32_t key_size;
    
    if (!xeKeysInit(&vault, cpu_key)) {
        return false;
    }
    
    if (!xeKeysGetKey(&vault, key_id, key, &key_size)) {
        return false;
    }
    
    XECRYPT_AES_STATE aes;
    uint8_t iv[16] = {0};
    xeCryptAesKey(&aes, key, key_size);
    
    uint8_t *temp = g_malloc(size);
    memcpy(temp, input, size);
    
    xeCryptAesCbcDecrypt(&aes, iv, temp, size);
    
    uint8_t hash[20];
    xeCryptSha(temp, size - 20, hash);
    
    if (memcmp(hash, &temp[size - 20], 20) != 0) {
        g_free(temp);
        return false;
    }
    
    memcpy(output, temp, size - 20);
    g_free(temp);
    
    return true;
}

/* ========= TITLE KEY DECRYPTION ========= */
bool xeKeysDecryptTitleKey(uint8_t *title_key, const uint8_t *encrypted_key,
                          uint32_t key_type, const uint8_t *console_key) {
    XECRYPT_AES_STATE aes;
    uint8_t iv[16] = {0};
    uint8_t master_key[16];
    
    switch (key_type) {
        case KEYTYPE_ODD_TITLE:
            memcpy(master_key, xekey_retail_odd, 16);
            break;
        case KEYTYPE_EVEN_TITLE:
            memcpy(master_key, xekey_retail_even, 16);
            break;
        default:
            return false;
    }
    
    uint8_t derived_key[16];
    xeCryptHmacSha(master_key, 16, console_key, 16, NULL, 0, derived_key);
    
    xeCryptAesKey(&aes, derived_key, 16);
    
    uint8_t temp[16];
    memcpy(temp, encrypted_key, 16);
    xeCryptAesCbcDecrypt(&aes, iv, temp, 16);
    
    memcpy(title_key, temp, 16);
    return true;
}

/* ========= SIGNATURE VERIFICATION ========= */
bool xeCryptRsaVerify(const uint8_t *hash, const uint8_t *signature, const uint8_t *modulus, uint32_t modulus_size, uint64_t exponent) {
    // Boutta use a crypto library for this one
    printf("[CRYPTO] RSA verify called (modulus size: %u)\n", modulus_size);

    // for now, just ACCEPT.
    return true;
}
