#include "hw/xbox360/crypto/xe_crypto.h"
#include "qemu/osdep.h"
#include "qemu/bswap.h"

// AES Tables (S-box, Rcon, etc.)
static const uint8_t aes_sbox[256] = {
    0x63, 0x7C, 0x77, 0x7B, 0xF2, 0x6B, 0x6F, 0xC5,
    0x30, 0x01, 0x67, 0x2B, 0xFE, 0xD7, 0xAB, 0x76,
    // ... Full AES Table
};

static const uint8_t aes_inv_sbox[256] = {
    0x52, 0x09, 0x6A, 0xD5, 0x30, 0x36, 0xA5, 0x38,
    0xBF, 0x40, 0xA3, 0x9E, 0x81, 0xF3, 0xD7, 0xFB,
    // ... TODO: Inverse Table
};

static const uint32_t sha1_k[4] = {
    0x5A827999, 0x6ED9EBA1, 0x8F1BBCDC, 0xCA62C1D6
};

/* ==================== SHA-1 ==================== */

static uint32_t sha1_rol(uint32_t value, uint32_t bits) {
    return (value << bits) | (value >> (32 - bits));
}

static void sha1_transform(XECRYPT_SHA_STATE *ctx, const uint8_t data[64]) {
    uint32_t a, b, c, d, e, i, j, t, m[80];
    
    for (i = j = 0; i < 16; ++i, j += 4) {
        m[i] = (data[j] << 24) | (data[j+1] << 16) | 
               (data[j+2] << 8) | data[j+3];
    }
    
    for (; i < 80; ++i) {
        m[i] = sha1_rol(m[i-3] ^ m[i-8] ^ m[i-14] ^ m[i-16], 1);
    }
    
    a = ctx->state[0];
    b = ctx->state[1];
    c = ctx->state[2];
    d = ctx->state[3];
    e = ctx->state[4];
    
    for (i = 0; i < 80; ++i) {
        if (i < 20) {
            t = sha1_rol(a, 5) + ((b & c) ^ (~b & d)) + e + sha1_k[0] + m[i];
        } else if (i < 40) {
            t = sha1_rol(a, 5) + (b ^ c ^ d) + e + sha1_k[1] + m[i];
        } else if (i < 60) {
            t = sha1_rol(a, 5) + ((b & c) ^ (b & d) ^ (c & d)) + e + sha1_k[2] + m[i];
        } else {
            t = sha1_rol(a, 5) + (b ^ c ^ d) + e + sha1_k[3] + m[i];
        }
        
        e = d;
        d = c;
        c = sha1_rol(b, 30);
        b = a;
        a = t;
    }
    
    ctx->state[0] += a;
    ctx->state[1] += b;
    ctx->state[2] += c;
    ctx->state[3] += d;
    ctx->state[4] += e;
}

void xeCryptShaInit(XECRYPT_SHA_STATE *state) {
    state->count[0] = state->count[1] = 0;
    state->state[0] = 0x67452301;
    state->state[1] = 0xEFCDAB89;
    state->state[2] = 0x98BADCFE;
    state->state[3] = 0x10325476;
    state->state[4] = 0xC3D2E1F0;
}

void xeCryptShaUpdate(XECRYPT_SHA_STATE *state, const uint8_t *data, size_t len) {
    uint32_t i, index, part_len;
    
    index = (state->count[0] >> 3) & 0x3F;
    
    if ((state->count[0] += (len << 3)) < (len << 3)) {
        state->count[1]++;
    }
    state->count[1] += (len >> 29);
    
    part_len = 64 - index;
    
    if (len >= part_len) {
        memcpy(&state->buffer[index], data, part_len);
        sha1_transform(state, state->buffer);
        
        for (i = part_len; i + 63 < len; i += 64) {
            sha1_transform(state, &data[i]);
        }
        
        index = 0;
    } else {
        i = 0;
    }
    
    memcpy(&state->buffer[index], &data[i], len - i);
}

void xeCryptShaFinal(XECRYPT_SHA_STATE *state, uint8_t digest[20]) {
    uint8_t bits[8];
    uint32_t index, pad_len;
    uint32_t i;
    
    for (i = 0; i < 8; ++i) {
        bits[i] = (state->count[((i >= 4) ? 1 : 0)] >> ((3 - (i & 3)) * 8)) & 0xFF;
    }
    
    index = (state->count[0] >> 3) & 0x3F;
    pad_len = (index < 56) ? (56 - index) : (120 - index);
    uint8_t padding[64] = {0};
    padding[0] = 0x80;
    xeCryptShaUpdate(state, padding, pad_len);
    xeCryptShaUpdate(state, bits, 8);
    
    for (i = 0; i < 5; ++i) {
        digest[i*4+0] = (state->state[i] >> 24) & 0xFF;
        digest[i*4+1] = (state->state[i] >> 16) & 0xFF;
        digest[i*4+2] = (state->state[i] >> 8) & 0xFF;
        digest[i*4+3] = state->state[i] & 0xFF;
    }
}

void xeCryptSha(const uint8_t *data, size_t len, uint8_t digest[20]) {
    XECRYPT_SHA_STATE ctx;
    xeCryptShaInit(&ctx);
    xeCryptShaUpdate(&ctx, data, len);
    xeCryptShaFinal(&ctx, digest);
}

/* ==================== RC4 ==================== */
void xeCryptRc4Key(XECRYPT_RC4_STATE *state, const uint8_t *key, size_t key_len) {
    uint32_t i, j = 0;
    uint8_t t;
    
    for (i = 0; i < 256; i++) {
        state->S[i] = i;
    }
    
    for (i = 0; i < 256; i++) {
        j = (j + state->S[i] + key[i % key_len]) & 0xFF;
        t = state->S[i];
        state->S[i] = state->S[j];
        state->S[j] = t;
    }
    
    state->i = state->j = 0;
}

static uint8_t rc4_next_byte(XECRYPT_RC4_STATE *state) {
    uint8_t t;
    
    state->i = (state->i + 1) & 0xFF;
    state->j = (state->j + state->S[state->i]) & 0xFF;
    
    t = state->S[state->i];
    state->S[state->i] = state->S[state->j];
    state->S[state->j] = t;
    
    return state->S[(state->S[state->i] + state->S[state->j]) & 0xFF];
}

void xeCryptRc4Ecb(XECRYPT_RC4_STATE *state, uint8_t *data, size_t len) {
    size_t i;
    for (i = 0; i < len; i++) {
        data[i] ^= rc4_next_byte(state);
    }
}

void xeCryptRc4(const uint8_t *key, size_t key_len, uint8_t *data, size_t len) {
    XECRYPT_RC4_STATE state;
    xeCryptRc4Key(&state, key, key_len);
    xeCryptRc4Ecb(&state, data, len);
}

/* ==================== AES ==================== */
static uint32_t aes_sub_word(uint32_t word) {
    return (aes_sbox[word >> 24] << 24) |
           (aes_sbox[(word >> 16) & 0xFF] << 16) |
           (aes_sbox[(word >> 8) & 0xFF] << 8) |
           aes_sbox[word & 0xFF];
}

static uint32_t aes_rot_word(uint32_t word) {
    return (word << 8) | (word >> 24);
}

void xeCryptAesKey(XECRYPT_AES_STATE *state, const uint8_t *key, size_t key_len) {
    uint32_t *ek = state->ek;
    uint32_t Nk, Nr, i, j;
    uint32_t temp;
    static const uint32_t rcon[10] = {
        0x01000000, 0x02000000, 0x04000000, 0x08000000,
        0x10000000, 0x20000000, 0x40000000, 0x80000000,
        0x1B000000, 0x36000000
    };
    
    if (key_len == 16) {      // AES-128
        Nk = 4;
        Nr = 10;
    } else if (key_len == 24) { // AES-192
        Nk = 6;
        Nr = 12;
    } else if (key_len == 32) { // AES-256
        Nk = 8;
        Nr = 14;
    } else {
        state->Nr = 0;
        return;
    }
    
    state->Nr = Nr;
    
    for (i = 0; i < Nk; i++) {
        ek[i] = (key[4*i] << 24) | (key[4*i+1] << 16) |
                (key[4*i+2] << 8) | key[4*i+3];
    }
    
    for (i = Nk; i < 4 * (Nr + 1); i++) {
        temp = ek[i-1];
        if (i % Nk == 0) {
            temp = aes_sub_word(aes_rot_word(temp)) ^ rcon[i/Nk - 1];
        } else if (Nk > 6 && i % Nk == 4) {
            temp = aes_sub_word(temp);
        }
        ek[i] = ek[i-Nk] ^ temp;
    }
    
    for (i = 0; i < 4 * (Nr + 1); i++) {
        state->dk[i] = ek[i];
    }
}

static void aes_add_round_key(uint32_t state[4], const uint32_t *w, int round) {
    int i;
    for (i = 0; i < 4; i++) {
        state[i] ^= w[round*4 + i];
    }
}

static void aes_sub_bytes(uint32_t state[4]) {
    int i, j;
    uint8_t *s = (uint8_t *)state;
    
    for (i = 0; i < 16; i++) {
        s[i] = aes_sbox[s[i]];
    }
}

static void aes_shift_rows(uint32_t state[4]) {
    uint8_t *s = (uint8_t *)state;
    uint8_t t;
    
    // Row 1
    t = s[1]; s[1] = s[5]; s[5] = s[9]; s[9] = s[13]; s[13] = t;
    
    // Row 2
    t = s[2]; s[2] = s[10]; s[10] = t;
    t = s[6]; s[6] = s[14]; s[14] = t;
    
    // Row 3
    t = s[15]; s[15] = s[11]; s[11] = s[7]; s[7] = s[3]; s[3] = t;
}

static uint8_t aes_gf_mult(uint8_t a, uint8_t b) {
    uint8_t p = 0;
    uint8_t hi_bit;
    
    for (int i = 0; i < 8; i++) {
        if (b & 1) {
            p ^= a;
        }
        hi_bit = a & 0x80;
        a <<= 1;
        if (hi_bit) {
            a ^= 0x1B; // x^8 + x^4 + x^3 + x + 1
        }
        b >>= 1;
    }
    
    return p;
}

static void aes_mix_columns(uint32_t state[4]) {
    uint8_t *s = (uint8_t *)state;
    uint8_t a[4], b[4];
    int i, j;
    
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            a[j] = s[i*4 + j];
        }
        
        b[0] = aes_gf_mult(a[0], 2) ^ aes_gf_mult(a[1], 3) ^ a[2] ^ a[3];
        b[1] = a[0] ^ aes_gf_mult(a[1], 2) ^ aes_gf_mult(a[2], 3) ^ a[3];
        b[2] = a[0] ^ a[1] ^ aes_gf_mult(a[2], 2) ^ aes_gf_mult(a[3], 3);
        b[3] = aes_gf_mult(a[0], 3) ^ a[1] ^ a[2] ^ aes_gf_mult(a[3], 2);
        
        for (j = 0; j < 4; j++) {
            s[i*4 + j] = b[j];
        }
    }
}

void xeCryptAesEncryptBlock(XECRYPT_AES_STATE *state, const uint8_t *in, uint8_t *out) {
    uint32_t s[4];
    int i, round;
    
    for (i = 0; i < 4; i++) {
        s[i] = (in[4*i] << 24) | (in[4*i+1] << 16) |
               (in[4*i+2] << 8) | in[4*i+3];
    }
    
    aes_add_round_key(s, state->ek, 0);
    
    for (round = 1; round < state->Nr; round++) {
        aes_sub_bytes(s);
        aes_shift_rows(s);
        aes_mix_columns(s);
        aes_add_round_key(s, state->ek, round);
    }
    
    aes_sub_bytes(s);
    aes_shift_rows(s);
    aes_add_round_key(s, state->ek, state->Nr);
    
    for (i = 0; i < 4; i++) {
        out[4*i] = s[i] >> 24;
        out[4*i+1] = s[i] >> 16;
        out[4*i+2] = s[i] >> 8;
        out[4*i+3] = s[i];
    }
}

void xeCryptAesDecryptBlock(XECRYPT_AES_STATE *state, const uint8_t *in, uint8_t *out) {
    // Simillar but with inverse rounds
    xeCryptAesEncryptBlock(state, in, out); // Placeholder
}

void xeCryptAesCbcEncrypt(XECRYPT_AES_STATE *state, uint8_t *iv, uint8_t *data, size_t len) {
    size_t i, j;
    uint8_t block[16];
    
    for (i = 0; i < len; i += 16) {
        for (j = 0; j < 16; j++) {
            data[i + j] ^= iv[j];
        }
        
        // Encrypt
        xeCryptAesEncryptBlock(state, &data[i], block);
        
        memcpy(&data[i], block, 16);
        memcpy(iv, block, 16);
    }
}

void xeCryptAesCbcDecrypt(XECRYPT_AES_STATE *state, uint8_t *iv, uint8_t *data, size_t len) {
    size_t i, j;
    uint8_t block[16], temp[16];
    
    for (i = 0; i < len; i += 16) {
        memcpy(temp, &data[i], 16);
        
        // Decrypt
        xeCryptAesDecryptBlock(state, &data[i], block);
        
        for (j = 0; j < 16; j++) {
            data[i + j] = block[j] ^ iv[j];
        }
        
        memcpy(iv, temp, 16);
    }
}

/* ==================== KEY VAULT ==================== */
static const uint8_t xekey_retail_odd[16] = {
    0xDD, 0x88, 0xAD, 0x0C, 0x9E, 0xD6, 0x69, 0xE7,
    0xB5, 0x67, 0x94, 0xFB, 0x68, 0x56, 0x3E, 0xFA
};

static const uint8_t xekey_retail_even[16] = {
    0xE1, 0xBC, 0x15, 0x9C, 0x73, 0xB1, 0xEA, 0xE9,
    0xAB, 0x9C, 0x28, 0x2E, 0x8F, 0x58, 0xFA, 0x17
};

bool xeKeysInit(XECRYPT_KEY_VAULT *vault, const uint8_t *cpu_key) {
    if (!vault || !cpu_key) return false;
    memcpy(vault->cpu_key, cpu_key, 16);
    
    uint8_t salt[16] = {0};
    uint8_t hash[20];
    xeCryptSha(cpu_key, 16, hash);
    memcpy(vault->console_key, hash, 16);
    memcpy(vault->console_id, hash, 5);
  
    memcpy(vault->odd_key, xekey_retail_odd, 16);
    memcpy(vault->even_key, xekey_retail_even, 16);
    
    // DVD key
    uint8_t dvd_salt[16] = {0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    xeCryptHmacSha(cpu_key, 16, dvd_salt, 16, NULL, 0, hash);
    memcpy(vault->dvd_key, hash, 16);
    
    return true;
}

bool xeKeysGetKey(XECRYPT_KEY_VAULT *vault, uint32_t key_id, uint8_t *output, uint32_t *output_size) {
    if (!vault || !output) return false;
    
    switch (key_id) {
        case XEKEY_CPU_KEY:
            memcpy(output, vault->cpu_key, 16);
            if (output_size) *output_size = 16;
            break;
            
        case XEKEY_CONSOLE_KEY:
            memcpy(output, vault->console_key, 16);
            if (output_size) *output_size = 16;
            break;
            
        case XEKEY_ODD_KEY:
            memcpy(output, vault->odd_key, 16);
            if (output_size) *output_size = 16;
            break;
            
        case XEKEY_EVEN_KEY:
            memcpy(output, vault->even_key, 16);
            if (output_size) *output_size = 16;
            break;
            
        case XEKEY_DVD_KEY:
            memcpy(output, vault->dvd_key, 16);
            if (output_size) *output_size = 16;
            break;
            
        default:
            return false;
    }
    
    return true;
}

/* ==================== BOOTING ==================== */
bool xeCryptVerify1BLSignature(const uint8_t *data, size_t size, const uint8_t *signature) {
    static const uint8_t pubkey_modulus[256] = {
        // ... (TODO: Full Public Modulus from Xbox 360)
    };
    
    uint8_t hash[20];
    xeCryptSha(data, size - 0x100, hash);
    
    return xeCryptRsaVerify(hash, signature, pubkey_modulus, 256, 0x10001);
}

/* ==================== UTILITIES ==================== */
void xeCryptHmacSha(const uint8_t *key, size_t key_len, const uint8_t *data1, size_t data1_len, const uint8_t *data2, size_t data2_len, uint8_t digest[20]) {
    uint8_t k_ipad[64], k_opad[64];
    XECRYPT_SHA_STATE ctx;
    uint8_t inner_hash[20];
    size_t i;
    
    memset(k_ipad, 0x36, 64);
    memset(k_opad, 0x5C, 64);
    
    if (key_len > 64) {
        xeCryptSha(key, key_len, k_ipad);
        memcpy(k_opad, k_ipad, 20);
        for (i = 20; i < 64; i++) {
            k_ipad[i] = 0x36;
            k_opad[i] = 0x5C;
        }
    } else {
        for (i = 0; i < key_len; i++) {
            k_ipad[i] ^= key[i];
            k_opad[i] ^= key[i];
        }
    }
    
    // Inner hash
    xeCryptShaInit(&ctx);
    xeCryptShaUpdate(&ctx, k_ipad, 64);
    if (data1_len > 0) xeCryptShaUpdate(&ctx, data1, data1_len);
    if (data2_len > 0) xeCryptShaUpdate(&ctx, data2, data2_len);
    xeCryptShaFinal(&ctx, inner_hash);
    
    // Outer hash
    xeCryptShaInit(&ctx);
    xeCryptShaUpdate(&ctx, k_opad, 64);
    xeCryptShaUpdate(&ctx, inner_hash, 20);
    xeCryptShaFinal(&ctx, digest);
}

uint64_t xeCryptBytesToQwBe(const uint8_t *bytes) {
    return ((uint64_t)bytes[0] << 56) | ((uint64_t)bytes[1] << 48) |
           ((uint64_t)bytes[2] << 40) | ((uint64_t)bytes[3] << 32) |
           ((uint64_t)bytes[4] << 24) | ((uint64_t)bytes[5] << 16) |
           ((uint64_t)bytes[6] << 8) | bytes[7];
}

void xeCryptQwToBytesBe(uint64_t qw, uint8_t *bytes) {
    bytes[0] = qw >> 56;
    bytes[1] = qw >> 48;
    bytes[2] = qw >> 40;
    bytes[3] = qw >> 32;
    bytes[4] = qw >> 24;
    bytes[5] = qw >> 16;
    bytes[6] = qw >> 8;
    bytes[7] = qw;
}
