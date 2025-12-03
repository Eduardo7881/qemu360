#include "hw/xbox360/xbox360_nand.h"
#include "qemu/osdep.h"
#include "qemu/error-report.h"

/* ==================== NAND DUMP LOADING ==================== */

bool xenon_nand_load_dump(XBOX360_NAND_STATE *nand, const char *filename) {
    FILE *f;
    size_t bytes_read;
    
    if (!nand || !filename) {
        return false;
    }
    
    f = fopen(filename, "rb");
    if (!f) {
        return false;
    }
    
    bytes_read = fread(nand->raw_data, 1, NAND_SIZE, f);
    fclose(f);
    
    if (bytes_read != NAND_SIZE) {
        return false;
    }
    
    nand->loaded = true;
    
    xenon_nand_extract_keyvault(nand);
    xenon_nand_extract_bl1(nand);
    xenon_nand_extract_cb(nand);
    xenon_nand_extract_hv(nand);
    
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
        return false;
    }
    
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
    
    return true;
}

/* ==================== DECRYPTION ==================== */

bool xenon_nand_decrypt_bl1(XBOX360_NAND_STATE *nand) {
    XECRYPT_AES_STATE aes;
    uint8_t iv[16] = {0};
    uint8_t key[16];
    uint32_t key_size;
    
    if (!nand || !nand->loaded) {
        return false;
    }
    
    if (!xeKeysGetKey(&nand->crypto_vault, XEKEY_1BL_KEY, key, &key_size)) {
        const uint8_t master_key[16] = {
            0xDD, 0x88, 0xAD, 0x0C, 0x9E, 0xD6, 0x69, 0xE7,
            0xB5, 0x67, 0x94, 0xFB, 0x68, 0x56, 0x3E, 0xFA
        };
        memcpy(key, master_key, 16);
        key_size = 16;
    }
    
    xeCryptAesKey(&aes, key, key_size);
    
    uint8_t *encrypted_data = nand->bl1_data + 512;
    uint32_t data_size = nand->bl1_header.encrypted_size;
    
    if (data_size > NAND_1BL_SIZE - 512) {
        return false;
    }
    
    xeCryptAesCbcDecrypt(&aes, iv, encrypted_data, data_size);
    
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
    
    const uint8_t master_key[16] = {
        0x20, 0x91, 0x2F, 0x58, 0x5A, 0x6C, 0x9F, 0x63,
        0x6D, 0x69, 0x79, 0x79, 0x79, 0x79, 0x79, 0x79
    };
    memcpy(key, master_key, 16);
    
    xeCryptAesKey(&aes, key, 16);
    
    uint8_t *encrypted_data = nand->cb_data + 512;
    uint32_t data_size = nand->cb_header.encrypted_size;
    
    if (data_size > NAND_CB_SIZE - 512) {
        return false;
    }
    
    xeCryptAesCbcDecrypt(&aes, iv, encrypted_data, data_size);
    
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
    
    if (!xeKeysGetKey(&nand->crypto_vault, XEKEY_KEY_VAULT_KEY, key, &key_size)) {
        return false;
    }
    
    xeCryptAesKey(&aes, key, key_size);
    
    uint8_t *encrypted_data = nand->hv_data + 512;
    uint32_t data_size = nand->hv_header.encrypted_size;
    
    if (data_size > NAND_HV_SIZE - 512) {
        return false;
    }
    
    xeCryptAesCbcDecrypt(&aes, iv, encrypted_data, data_size);
    
    return true;
}

/* ==================== VERIFICATION ==================== */

bool xenon_nand_verify_bl1(XBOX360_NAND_STATE *nand) {
    if (!nand || !nand->loaded) {
        return false;
    }
    
    return true;
}

bool xenon_nand_verify_cb(XBOX360_NAND_STATE *nand) {
    if (!nand || !nand->loaded) {
        return false;
    }
    
    return true;
}

bool xenon_nand_verify_hv(XBOX360_NAND_STATE *nand) {
    if (!nand || !nand->loaded) {
        return false;
    }
    
    return true;
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
        return false;
    }
    
    return true;
}

/* ==================== UTILITIES ==================== */

void xenon_nand_print_info(XBOX360_NAND_STATE *nand) {
    if (!nand || !nand->loaded) {
        return;
    }
    
    printf("NAND Info:\n");
    printf("  Console Serial: ");
    for (int i = 0; i < 12; i++) {
        printf("%c", nand->key_vault.serial_number[i]);
    }
    printf("\n");
    
    printf("  Region: %s\n", xenon_nand_get_region_string(nand->key_vault.region_code));
    
    if (nand->bl1_header.build_version) {
        printf("  1BL: %08X @ 0x%08X (%d bytes)\n",
               nand->bl1_header.build_version,
               nand->bl1_header.entry_point,
               nand->bl1_header.size);
    }
    
    if (nand->cb_header.version) {
        printf("  CB:  %08X @ 0x%08X (%d bytes)\n",
               nand->cb_header.version,
               nand->cb_header.entry_point,
               nand->cb_header.size);
    }
    
    if (nand->hv_header.version) {
        printf("  HV:  %08X @ 0x%08X\n",
               nand->hv_header.version,
               nand->hv_header.entry_point);
    }
}

const char* xenon_nand_get_region_string(uint8_t region_code) {
    switch (region_code) {
        case 0x00: return "NTSC-U";
        case 0x01: return "PAL";
        case 0x02: return "NTSC-J";
        case 0x03: return "China";
        case 0x04: return "Korea";
        default:   return "Unknown";
    }
}

const char* xenon_nand_get_console_type_string(uint8_t console_type) {
    switch (console_type) {
        case 0x01: return "Xenon";
        case 0x02: return "Zephyr/Opus";
        case 0x03: return "Falcon/Jasper";
        case 0x04: return "Trinity";
        case 0x05: return "Corona";
        default:   return "Unknown";
    }
}

/* ==================== NAND OPERATIONS ==================== */

bool xenon_nand_read_page(XBOX360_NAND_STATE *nand, uint32_t page, 
                         uint8_t *data, uint8_t *spare) {
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

bool xenon_nand_write_page(XBOX360_NAND_STATE *nand, uint32_t page, 
                          const uint8_t *data, const uint8_t *spare) {
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
