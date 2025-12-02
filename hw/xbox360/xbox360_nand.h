#ifndef HW_XBOX360_NAND_H
#define HW_XBOX360_NAND_H

#include "hw/xbox360/crypto/xe_crypto.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== NAND LAYOUT ==================== */

#define NAND_SIZE                0x400000      // 4MB for Trinity
#define NAND_PAGE_SIZE           0x200         // 512 bytes
#define NAND_PAGES_PER_BLOCK     0x40          // 64 pages/block
#define NAND_BLOCK_SIZE          (NAND_PAGE_SIZE * NAND_PAGES_PER_BLOCK) // 32KB

#define NAND_1BL_OFFSET          0x0           // 1st Bootloader
#define NAND_1BL_SIZE            0x8000        // 32KB
#define NAND_SMC_CONFIG_OFFSET   0x8000        // SMC Configuration
#define NAND_KEY_VAULT_OFFSET    0x8400        // Key Vault
#define NAND_KEY_VAULT_SIZE      0x400         // 1KB
#define NAND_SFC_OFFSET          0x8800        // SFC (Serial Flash Config)
#define NAND_CB_OFFSET           0x10000       // Console Bootloader
#define NAND_CB_SIZE             0x30000       // 192KB
#define NAND_CF_OFFSET           0x40000       // Config Data
#define NAND_HV_OFFSET           0x80000       // Hypervisor
#define NAND_HV_SIZE             0x20000       // 128KB
#define NAND_DASH_OFFSET         0xA0000       // Dashboard
#define NAND_DASH_SIZE           0x100000      // 1MB

/* ==================== BOOTLOADER STRUCTURES ==================== */
typedef struct BL1_HEADER {
  uint8_t signature[4];     // "BL1\x00"
  uint32_t build_version;   // e.g., 2.0.17559.0
  uint32_t entry_point;     // 0x800000
  uint32_t size;            // BL1 size
  uint32_t encrypted_size;  // Encrypted BL1 Size
  uint8_t hash[20];         // Content SHA-1
  uint8_t rsa_signature[256]; // Signature RSA-2048
  uint8_t reserved[192];    // Reserved
} BL1_HEADER;

typedef struct CB_HEADER {
  uint8_t signature[4];    // "CB\x00\x00"
  uint32_t version;
  uint32_t entry_point;    // 0x80002000
  uint32_t size;
  uint32_t encrypted_size;
  uint8_t hash[20];
  uint8_t rsa_signature[256];
  uint32_t load_address;  // 0x80002000
  uint32_t stack_pointer; // 0x8000F000
  uint8_t console_type;   // 0x02 = Trinity
  uint8_t reserved[31];
} CB_HEADER;

typedef struct HV_HEADER {
  uint8_t signature[4];    // "CB\x00\x00"
  uint32_t version;
  uint32_t entry_point;    // 0x80002000
  uint32_t size;
  uint32_t encrypted_size;
  uint8_t hash[20];
  uint8_t rsa_signature[256];
  uint32_t kernel_version; // e.g., 2.0.17559.0
  uint32_t capabilities; // Hypervisor capabilities
  uint8_t reserved[216];
} HV_HEADER;

/* ==================== NAND STRUCTURES ==================== */
typedef struct NAND_KEY_VAULT {
    uint8_t cpu_key[16];           // Unique CPU Key
    uint8_t console_key[16];       // Console Key
    uint8_t console_id[5];         // Console ID (5 bytes)
    uint8_t odd_key[16];           // Odd Key
    uint8_t even_key[16];          // Even Key
    uint8_t dvd_key[16];           // DVD Key
    uint8_t region_code;           // 0x00 = NTSC-U, 0x01 = PAL, etc.
    uint8_t dash_version[4];       // Dashboard Ver
    uint8_t parental_code[8];      // Parental Code
    uint8_t serial_number[12];     // Serial
    uint8_t mac_address[6];        // MAC
    uint8_t reserved[146];         // Reserved
    uint8_t hash[20];              // Keyvault SHA-1
} NAND_KEY_VAULT;

typedef struct XBOX360_NAND_STATE {
    uint8_t raw_data[NAND_SIZE];           // NAND DUMP
    bool loaded;                           // NAND loaded?
    
    uint8_t bl1_data[NAND_1BL_SIZE];
    uint8_t cb_data[NAND_CB_SIZE];
    uint8_t hv_data[NAND_HV_SIZE];
    uint8_t dash_data[NAND_DASH_SIZE];
    
    BL1_HEADER bl1_header;
    CB_HEADER cb_header;
    HV_HEADER hv_header;
    NAND_KEY_VAULT key_vault;
    
    XECRYPT_KEY_VAULT crypto_vault;
    uint8_t fuse_bits[12];
    uint8_t smc_config[256];
    
    uint8_t spare_data[NAND_SIZE / NAND_PAGE_SIZE * 16];
} XBOX360_NAND_STATE;

/* ==================== PUBLIC FUNCTIONS ==================== */
bool xenon_nand_load_dump(XBOX360_NAND_STATE *nand, const char *filename);
bool xenon_nand_save_dump(XBOX360_NAND_STATE *nand, const char *filename);

// Extraction
bool xenon_nand_extract_bl1(XBOX360_NAND_STATE *nand);
bool xenon_nand_extract_cb(XBOX360_NAND_STATE *nand);
bool xenon_nand_extract_hv(XBOX360_NAND_STATE *nand);
bool xenon_nand_extract_keyvault(XBOX360_NAND_STATE *nand);

// Decrypt
bool xenon_nand_decrypt_bl1(XBOX360_NAND_STATE *nand);
bool xenon_nand_decrypt_cb(XBOX360_NAND_STATE *nand);
bool xenon_nand_decrypt_hv(XBOX360_NAND_STATE *nand);

// Verifications
bool xenon_nand_verify_bl1(XBOX360_NAND_STATE *nand);
bool xenon_nand_verify_cb(XBOX360_NAND_STATE *nand);
bool xenon_nand_verify_hv(XBOX360_NAND_STATE *nand);
bool xenon_nand_verify_keyvault(XBOX360_NAND_STATE *nand);

// Utils
void xenon_nand_print_info(XBOX360_NAND_STATE *nand);
bool xenon_nand_patch_component(XBOX360_NAND_STATE *nand, uint32_t offset, const uint8_t *patch,  uint32_t size);
const char* xenon_nand_get_region_string(uint8_t region_code);
const char* xenon_nand_get_console_type_string(uint8_t console_type);

// Low Level Functions (EEC/Spare)
bool xenon_nand_read_page(XBOX360_NAND_STATE *nand, uint32_t page, uint8_t *data, uint8_t *spare);
bool xenon_nand_write_page(XBOX360_NAND_STATE *nand, uint32_t page, const uint8_t *data, const uint8_t *spare);
uint32_t xenon_nand_calculate_ecc(const uint8_t *data);

#endif
