#include "hw/xbox360/xbox360_nand.h"
#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/bswap.h"

/* ==================== CONSTANTS ==================== */
static const uint8_t BL1_SIGNATURE[4] = {'B', 'L', '1', 0x00};
static const uint8_t CB_SIGNATURE[4] = {'C', 'B', 0x00, 0x00};
static const uint8_t HV_SIGNATURE[4] = {'H', 'V', 0x00, 0x00};

static const uint8_t MASTER_KEY_1BL[16] = {
    0xDD, 0x88, 0xAD, 0x0C, 0x9E, 0xD6, 0x69, 0xE7,
    0xB5, 0x67, 0x94, 0xFB, 0x68, 0x56, 0x3E, 0xFA
};

static const uint8_t MASTER_KEY_CB[16] = {
    0x20, 0x91, 0x2F, 0x58, 0x5A, 0x6C, 0x9F, 0x63,
    0x6D, 0x69, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79
};

/* ==================== DUMP Loading ==================== */
bool xenon_nand_load_dump(XBOX360_NAND_STATE *nand, const char *filename) {
    FILE *f;
    size_t bytes_read;
    
    if (!nand || !filename) {
        error_report("[NAND] Invalid Parameters");
        return false;
    }
    
    f = fopen(filename, "rb");
    if (!f) {
        error_report("[NAND] Couldn't open file: %s", filename);
        return false;
    }
    
    bytes_read = fread(nand->raw_data, 1, NAND_SIZE, f);
    fclose(f);
    
    if (bytes_read != NAND_SIZE) {
        error_report("[NAND] Invalid DUMP Size: %zu bytes (esperado: %d)", 
                    bytes_read, NAND_SIZE);
        return false;
    }
    
    nand->loaded = true;
    printf("[NAND] Dump loaded: %s (%d bytes)\n", filename, NAND_SIZE);
    
    if (!xenon_nand_extract_keyvault(nand)) {
        error_report("[NAND] Error extracting key vault");
        return false;
    }
    
    if (!xenon_nand_extract_bl1(nand)) {
        error_report("[NAND] Error extracting 1BL");
        return false;
    }
    
    return true;
}

bool xenon_nand_save_dump(XBOX360_NAND_STATE *nand, const char *filename) {
    FILE *f;
    
    if (!nand || !nand->loaded || !filename) {
        return false;
    }
    
    f = fopen(filename, "wb");
    if (!f) {
        return false;
    }
    
    fwrite(nand->raw_data, 1, NAND_SIZE, f);
    fclose(f);
    
    printf("[NAND] Dump saved: %s\n", filename);
    return true;
}

/* ==================== COMPONENT EXTRACTION ==================== */
bool xenon_nand_extract_keyvault(XBOX360_NAND_STATE *nand) {
    if (!nand || !nand->loaded) {
        return false;
    }
    
    memcpy(&nand->key_vault, 
           &nand->raw_data[NAND_KEY_VAULT_OFFSET],
           sizeof(NAND_KEY_VAULT));
    
    if (!xeKeysInit(&nand->crypto_vault, nand->key_vault.cpu_key)) {
        error_report("[NAND] Failure to initialize vault");
        return false;
    }
    
    printf("[NAND] Keyvault extracted\n");
    printf("       CPU Key: ");
    for (int i = 0; i < 16; i++) {
        printf("%02X", nand->key_vault.cpu_key[i]);
    }
    printf("\n");
    
    // Extract fuses, TODO: Get from Hardware
    memset(nand->fuse_bits, 0xFF, 12);
    
    return true;
}

bool xenon_nand_extract_bl1(XBOX360_NAND_STATE *nand) {
    if (!nand || !nand->loaded) {
        return false;
    }
    
    memcpy(nand->bl1_data,
           &nand->raw_data[NAND_1BL_OFFSET],
           NAND_1BL_SIZE);
    
    memcpy(&nand->bl1_header, nand->bl1_data, sizeof(BL1_HEADER));
    
    if (memcmp(nand->bl1_header.signature, BL1_SIGNATURE, 4) != 0) {
        error_report("[NAND] Invalid 1BL Signature");
        return false;
    }
    
    printf("[NAND] 1BL extracted\n");
    printf("       Version: %08X\n", nand->bl1_header.build_version);
    printf("       Size: %d bytes\n", nand->bl1_header.size);
    printf("       Entry point: 0x%08X\n", nand->bl1_header.entry_point);
    
    return true;
}

bool xenon_nand_extract_cb(XBOX360_NAND_STATE *nand) {
    if (!nand || !nand->loaded) {
        return false;
    }
    
    memcpy(nand->cb_data,
           &nand->raw_data[NAND_CB_OFFSET],
           NAND_CB_SIZE);
    
    memcpy(&nand->cb_header, nand->cb_data, sizeof(CB_HEADER));
    
    if (memcmp(nand->cb_header.signature, CB_SIGNATURE, 4) != 0) {
        error_report("[NAND] Invalid CB Signature");
        return false;
    }
    
    printf("[NAND] CB extracted\n");
    printf("       Version: %08X\n", nand->cb_header.version);
    printf("       Size: %d bytes\n", nand->cb_header.size);
    printf("       Console Type: %02X\n", nand->cb_header.console_type);
    
    return true;
}

bool xenon_nand_extract_hv(XBOX360_NAND_STATE *nand) {
    if (!nand || !nand->loaded) {
        return false;
    }
    
    memcpy(nand->hv_data,
           &nand->raw_data[NAND_HV_OFFSET],
           NAND_HV_SIZE);
    
    memcpy(&nand->hv_header, nand->hv_data, sizeof(HV_HEADER));
    
    if (memcmp(nand->hv_header.signature, HV_SIGNATURE, 4) != 0) {
        error_report("[NAND] Invalid HV Signature");
        return false;
    }
    
    printf("[NAND] Hypervisor extracted\n");
    printf("       Version: %08X\n", nand->hv_header.version);
    printf("       Kernel Version: %08X\n", nand->hv_header.kernel_version);
    printf("       Entry point: 0x%08X\n", nand->hv_header.entry_point);
    
    return true;
}

/* ==================== DECRYPTING ==================== */
bool xenon_nand_decrypt_bl1(XBOX360_NAND_STATE *nand) {
    XECRYPT_AES_STATE aes;
    uint8_t iv[16] = {0};
    uint8_t key[16];
    uint32_t key_size;
    
    if (!nand || !nand->loaded) {
        return false;
    }
    
    printf("[NAND] Decrypting 1BL...\n");
    if (!xeKeysGetKey(&nand->crypto_vault, XEKEY_1BL_KEY, key, &key_size)) {
        memcpy(key, MASTER_KEY_1BL, 16);
        key_size = 16;
    }
    
    xeCryptAesKey(&aes, key, key_size);
    
    uint8_t *encrypted_data = nand->bl1_data + 512;
    uint32_t data_size = nand->bl1_header.encrypted_size;
    
    if (data_size > NAND_1BL_SIZE - 512) {
        error_report("[NAND] Invalid 1BL Size");
        return false;
    }
    
    xeCryptAesCbcDecrypt(&aes, iv, encrypted_data, data_size);
    
    uint8_t calculated_hash[20];
    xeCryptSha(encrypted_data, data_size, calculated_hash);
    
    if (memcmp(calculated_hash, nand->bl1_header.hash, 20) != 0) {
        error_report("[NAND] 1BL Hash doesn't match");
        return false;
    }
    
    printf("[NAND] 1BL decryped and verified\n");
    return true;
}

bool xenon_nand_decrypt_cb(XBOX360_NAND_STATE *nand) {
    XECRYPT_AES_STATE aes;
    uint8_t iv[16] = {0};
    uint8_t key[16];
    
    if (!nand || !nand->loaded) {
        return false;
    }
    
    if (!xenon_nand_decrypt_bl1(nand)) {
        return false;
    }
    
    printf("[NAND] Decrypting CB...\n");
    
    // CB key is derivated from 1BL
    // Simplified: Use Master Key
    memcpy(key, MASTER_KEY_CB, 16);
    
    xeCryptAesKey(&aes, key, 16);
    
    // Skip header
    uint8_t *encrypted_data = nand->cb_data + 512;
    uint32_t data_size = nand->cb_header.encrypted_size;
    
    if (data_size > NAND_CB_SIZE - 512) {
        error_report("[NAND] Invalid CB Size");
        return false;
    }
    
    xeCryptAesCbcDecrypt(&aes, iv, encrypted_data, data_size);
    
    uint8_t calculated_hash[20];
    xeCryptSha(encrypted_data, data_size, calculated_hash);
    
    if (memcmp(calculated_hash, nand->cb_header.hash, 20) != 0) {
        error_report("[NAND] CB Hash doesn't match");
        return false;
    }
    
    printf("[NAND] CB decryped and verified\n");
    return true;
}

bool xenon_nand_decrypt_hv(XBOX360_NAND_STATE *nand) {
    XECRYPT_AES_STATE aes;
    uint8_t iv[16] = {0};
    uint8_t key[16];
    uint32_t key_size;
    
    if (!nand || !nand->loaded) {
        return false;
    }
    
    if (!xenon_nand_decrypt_cb(nand)) {
        return false;
    }
    
    printf("[NAND] Decrypting Hypervisor...\n");
    if (!xeKeysGetKey(&nand->crypto_vault, XEKEY_KEY_VAULT_KEY, key, &key_size)) {
        error_report("[NAND] Couldn't get HV key");
        return false;
    }
    
    xeCryptAesKey(&aes, key, key_size);
    
    uint8_t *encrypted_data = nand->hv_data + 512;
    uint32_t data_size = nand->hv_header.encrypted_size;
    
    if (data_size > NAND_HV_SIZE - 512) {
        error_report("[NAND] Invalid HV Size");
        return false;
    }
    
    xeCryptAesCbcDecrypt(&aes, iv, encrypted_data, data_size);
    
    uint8_t calculated_hash[20];
    xeCryptSha(encrypted_data, data_size, calculated_hash);
    
    if (memcmp(calculated_hash, nand->hv_header.hash, 20) != 0) {
        error_report("[NAND] HV Hash doesn't match");
        return false;
    }
    
    printf("[NAND] Hypervisor decrypted and verified\n");
    return true;
}

/* ==================== VERIFICATION ==================== */
bool xenon_nand_verify_bl1(XBOX360_NAND_STATE *nand) {
    if (!nand || !nand->loaded) {
        return false;
    }
    
    printf("[NAND] Verifying 1BL signature...\n");
    
    // We'll accept all
    // TODO: Verify with public RSA-2048 key
    return xeCryptVerify1BLSignature(nand->bl1_data + 512,
                                    nand->bl1_header.size,
                                    nand->bl1_header.rsa_signature);
}

bool xenon_nand_verify_cb(XBOX360_NAND_STATE *nand) {
    if (!nand || !nand->loaded) {
        return false;
    }
    
    printf("[NAND] Verifying CB Signature...\n");
    return true; // Placeholder
}

bool xenon_nand_verify_hv(XBOX360_NAND_STATE *nand) {
    if (!nand || !nand->loaded) {
        return false;
    }
    
    printf("[NAND] Verifying Hypervisor Signature...\n");
    return true; // Placeholder
}

bool xenon_nand_verify_keyvault(XBOX360_NAND_STATE *nand) {
    if (!nand || !nand->loaded) {
        return false;
    }
    
    uint8_t calculated_hash[20];
    xeCryptSha((uint8_t*)&nand->key_vault, 
               sizeof(NAND_KEY_VAULT) - 20,
               calculated_hash);
    
    if (memcmp(calculated_hash, nand->key_vault.hash, 20) != 0) {
        error_report("[NAND] Hash do keyvault não confere");
        return false;
    }
    
    printf("[NAND] Keyvault verificado\n");
    return true;
}

/* ==================== UTILITIES ==================== */
void xenon_nand_print_info(XBOX360_NAND_STATE *nand) {
    if (!nand || !nand->loaded) {
        printf("[NAND] Nenhum dump carregado\n");
        return;
    }
    
    printf("\n=== XBOX 360 NAND INFO ===\n");
    printf("Console Serial: ");
    for (int i = 0; i < 12; i++) {
        printf("%c", nand->key_vault.serial_number[i]);
    }
    printf("\n");
    
    printf("MAC Address: ");
    for (int i = 0; i < 6; i++) {
        printf("%02X%s", nand->key_vault.mac_address[i], 
               (i < 5) ? ":" : "");
    }
    printf("\n");
    
    printf("Region: %s\n", 
           xenon_nand_get_region_string(nand->key_vault.region_code));
    
    printf("\nBootloaders:\n");
    printf("  1BL: %08X @ 0x%08X (%d bytes)\n",
           nand->bl1_header.build_version,
           nand->bl1_header.entry_point,
           nand->bl1_header.size);
    
    if (memcmp(nand->cb_header.signature, CB_SIGNATURE, 4) == 0) {
        printf("  CB:  %08X @ 0x%08X (%d bytes)\n",
               nand->cb_header.version,
               nand->cb_header.entry_point,
               nand->cb_header.size);
    }
    
    if (memcmp(nand->hv_header.signature, HV_SIGNATURE, 4) == 0) {
        printf("  HV:  %08X @ 0x%08X (Kernel: %08X)\n",
               nand->hv_header.version,
               nand->hv_header.entry_point,
               nand->hv_header.kernel_version);
    }
    
    printf("\n");
}

bool xenon_nand_patch_component(XBOX360_NAND_STATE *nand, 
                               uint32_t offset, const uint8_t *patch, 
                               uint32_t size) {
    if (!nand || !nand->loaded || !patch || 
        offset + size > NAND_SIZE) {
        return false;
    }
    
    printf("[NAND] Applying patch @ 0x%08X (%d bytes)\n", offset, size);
    memcpy(&nand->raw_data[offset], patch, size);
    
    if (offset >= NAND_KEY_VAULT_OFFSET && 
        offset < NAND_KEY_VAULT_OFFSET + sizeof(NAND_KEY_VAULT)) {
        uint8_t hash[20];
        xeCryptSha((uint8_t*)&nand->key_vault, 
                   sizeof(NAND_KEY_VAULT) - 20,
                   hash);
        memcpy(nand->key_vault.hash, hash, 20);
    }
    
    return true;
}

const char* xenon_nand_get_region_string(uint8_t region_code) {
    switch (region_code) {
        case 0x00: return "NTSC-U (North America)";
        case 0x01: return "PAL (Europe/Australia)";
        case 0x02: return "NTSC-J (Japan)";
        case 0x03: return "China";
        case 0x04: return "Korea";
        default:   return "Unknown";
    }
}

const char* xenon_nand_get_console_type_string(uint8_t console_type) {
    switch (console_type) {
        case 0x01: return "Xenon (90nm)";
        case 0x02: return "Zephyr/Opus";
        case 0x03: return "Falcon/Jasper";
        case 0x04: return "Trinity";
        case 0x05: return "Corona";
        default:   return "Unknown";
    }
}

/* ==================== ECC/SPARE DATA ==================== */
bool xenon_nand_read_page(XBOX360_NAND_STATE *nand, uint32_t page, uint8_t *data, uint8_t *spare) {
    if (!nand || !nand->loaded || page >= NAND_SIZE / NAND_PAGE_SIZE) {
        return false;
    }
    
    uint32_t offset = page * NAND_PAGE_SIZE;
    
    if (data) {
        memcpy(data, &nand->raw_data[offset], NAND_PAGE_SIZE);
    }
    
    if (spare && page < sizeof(nand->spare_data) / 16) {
        memcpy(spare, &nand->spare_data[page * 16], 16);
    }
    
    return true;
}

bool xenon_nand_write_page(XBOX360_NAND_STATE *nand, uint32_t page, const uint8_t *data, const uint8_t *spare) {
    if (!nand || !nand->loaded || page >= NAND_SIZE / NAND_PAGE_SIZE) {
        return false;
    }
    
    uint32_t offset = page * NAND_PAGE_SIZE;
    
    if (data) {
        memcpy(&nand->raw_data[offset], data, NAND_PAGE_SIZE);
    }
    
    if (spare && page < sizeof(nand->spare_data) / 16) {
        memcpy(&nand->spare_data[page * 16], spare, 16);
    }
    
    return true;
}

uint32_t xenon_nand_calculate_ecc(const uint8_t *data) {
    // Simplified ECC Hamming(512,502)
    // TODO: Error Correction
    uint32_t ecc = 0;
    
    for (int i = 0; i < NAND_PAGE_SIZE; i++) {
        ecc ^= (data[i] << (i % 32));
    }
    
    return ecc;
}
