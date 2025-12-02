#include "hw/xbox360/xbox360.h"
#include "hw/xbox360/xbox360_nand.h"
#include "hw/xbox360/crypto/xe_crypto.h"
#include "qemu/osdep.h"
#include "hw/loader.h"

/* ==================== BOOT STATES ==================== */
typedef enum {
    BOOT_STATE_RESET = 0,
    BOOT_STATE_LOAD_NAND,
    BOOT_STATE_EXTRACT_COMPONENTS,
    BOOT_STATE_DECRYPT_1BL,
    BOOT_STATE_VERIFY_1BL,
    BOOT_STATE_EXECUTE_1BL,
    BOOT_STATE_DECRYPT_CB,
    BOOT_STATE_VERIFY_CB,
    BOOT_STATE_EXECUTE_CB,
    BOOT_STATE_DECRYPT_HV,
    BOOT_STATE_VERIFY_HV,
    BOOT_STATE_EXECUTE_HV,
    BOOT_STATE_HV_INIT,
    BOOT_STATE_LOAD_DASHBOARD,
    BOOT_STATE_RUNNING,
    BOOT_STATE_ERROR
} BOOT_STATE;

/* ==================== BOOT CONTEXT ==================== */
typedef struct BOOT_CONTEXT {
    BOOT_STATE state;
    BOOT_STATE next_state;
    uint32_t retry_count;
    uint64_t state_enter_time;
    
    uint8_t *bl1_code;
    uint8_t *cb_code;
    uint8_t *hv_code;
    
    uint32_t bl1_size;
    uint32_t cb_size;
    uint32_t hv_size;
    
    uint32_t bl1_entry;
    uint32_t cb_entry;
    uint32_t hv_entry;
    
    uint32_t error_code;
    char error_message[256];
} BOOT_CONTEXT;

/* ==================== BOOT FUNCTIONS ==================== */
static bool xenon_boot_load_nand(XenonState *s) {
    if (!s->nand_loaded) {
        printf("[BOOT] Loading NAND Dump...\n");
        
        if (xenon_nand_load_dump(&s->nand_state, "nanddump.bin")) {
            s->nand_loaded = true;
            xenon_nand_print_info(&s->nand_state);
            return true;
        }
        
        const char *alternatives[] = {
            "nand.bin",
            "flash.bin",
            "xbox360.bin",
            NULL
        };
        
        for (int i = 0; alternatives[i]; i++) {
            printf("[BOOT] Trying %s...\n", alternatives[i]);
            if (xenon_nand_load_dump(&s->nand_state, alternatives[i])) {
                s->nand_loaded = true;
                xenon_nand_print_info(&s->nand_state);
                return true;
            }
        }
        
        printf("[BOOT] No NAND dump found. Make sure it's named 'nand.bin', 'flash.bin' or 'xbox360.bin'.\n");
        return false;
    }
    
    return true;
}

static bool xenon_boot_extract_components(XenonState *s) {
    printf("[BOOT] Extracting components...\n");
    
    if (!xenon_nand_extract_bl1(&s->nand_state)) {
        printf("[BOOT] Error extracting 1BL\n");
        return false;
    }
    
    if (!xenon_nand_extract_cb(&s->nand_state)) {
        printf("[BOOT] Error extracting CB\n");
        return false;
    }
    
    if (!xenon_nand_extract_hv(&s->nand_state)) {
        printf("[BOOT] Error extracting HV\n");
        return false;
    }
    
    if (!xenon_nand_verify_keyvault(&s->nand_state)) {
        printf("[BOOT] Warning: Invalid Keyvault (continuing...)\n");
        // Continue with invalid keyvault for testing
    }
    
    return true;
}

static bool xenon_boot_decrypt_chain(XenonState *s) {
    printf("[BOOT] Decrypting chain of trust...\n");
    
    if (!xenon_nand_decrypt_bl1(&s->nand_state)) {
        printf("[BOOT] Error decrypting 1BL 1BL\n");
        return false;
    }
    
    if (!xenon_nand_verify_bl1(&s->nand_state)) {
        printf("[BOOT] Warning: Invalid 1BL Signature (continuing...)\n");
    }
    
    if (!xenon_nand_decrypt_cb(&s->nand_state)) {
        printf("[BOOT] Error decrypting CB\n");
        return false;
    }
    
    if (!xenon_nand_verify_cb(&s->nand_state)) {
        printf("[BOOT] Warning: Invalid CB Signature (continuing...)\n");
    }
    
    if (!xenon_nand_decrypt_hv(&s->nand_state)) {
        printf("[BOOT] Error decrypting HV\n");
        return false;
    }
    
    if (!xenon_nand_verify_hv(&s->nand_state)) {
        printf("[BOOT] Warning: Invalid HV Signature (continuing...)\n");
    }
    
    return true;
}

static void xenon_boot_load_into_memory(XenonState *s) {
    printf("[BOOT] Loading Components...\n");
    
    cpu_physical_memory_write(0x80000000, 
                             s->nand_state.bl1_data + 512,
                             s->nand_state.bl1_header.size);
    
    printf("  1BL @ 0x80000000 (%d bytes)\n", s->nand_state.bl1_header.size);
    
    cpu_physical_memory_write(0x80002000,
                             s->nand_state.cb_data + 512,
                             s->nand_state.cb_header.size);
    
    printf("  CB  @ 0x80002000 (%d bytes)\n", s->nand_state.cb_header.size);
    
    cpu_physical_memory_write(0x80000200,
                             s->nand_state.hv_data + 512,
                             s->nand_state.hv_header.size);
    
    printf("  HV  @ 0x80000200 (%d bytes)\n", s->nand_state.hv_header.size);
    
    PowerPCCPU *cpu = s->cpu[0];
    
    cpu->env.nip = s->nand_state.bl1_header.entry_point; // 0x80000000
    cpu->env.gpr[1] = 0x8000F000;
    
    printf("[BOOT] CPU configurada: NIP=0x%08X, SP=0x%08X\n",
           cpu->env.nip, cpu->env.gpr[1]);
}

static bool xenon_boot_patch_for_emulation(XenonState *s) {
    printf("[BOOT] Applying patches for emulation...\n");
    
    // Patch 1: Remove SMC verification in 1BL
    uint8_t *bl1_code = s->nand_state.bl1_data + 512;
    uint32_t bl1_size = s->nand_state.bl1_header.size;
    
    for (uint32_t i = 0; i < bl1_size - 4; i++) {
        // Procurar por "lis rX, 0x8000" seguido por "ori rX, rX, 0x0200"
        if (bl1_code[i] == 0x3C && bl1_code[i+2] == 0x60 && 
            bl1_code[i+3] == 0x80 && bl1_code[i+4] == 0x00) {
            if (i + 8 < bl1_size && bl1_code[i+5] == 0x60 && 
                bl1_code[i+7] == 0x02 && bl1_code[i+8] == 0x00) {
                printf("  Patch SMC access @ 0x%08X\n", 0x80000000 + i);
                
                // Overwrite with NOPs (0x60000000)
                bl1_code[i] = 0x60; bl1_code[i+1] = 0x00;
                bl1_code[i+2] = 0x00; bl1_code[i+3] = 0x00;
                bl1_code[i+4] = 0x60; bl1_code[i+5] = 0x00;
                bl1_code[i+6] = 0x00; bl1_code[i+7] = 0x00;
                bl1_code[i+8] = 0x60; bl1_code[i+9] = 0x00;
                bl1_code[i+10] = 0x00; bl1_code[i+11] = 0x00;
            }
        }
    }
    
    // Patch 2: Remove temperature verification
    for (uint32_t i = 0; i < bl1_size - 8; i++) {
        // cmpwi rX, 0x50 (0x2CXX0050)
        if (bl1_code[i] == 0x2C && bl1_code[i+3] == 0x00 && 
            bl1_code[i+4] == 0x50) {
            printf("  Patch temperature check @ 0x%08X\n", 0x80000000 + i);
            
            // Change to "li rX, 0x20" (0x38XX0020) - safe temperature
            bl1_code[i] = 0x38; // li
            bl1_code[i+4] = 0x20; // 0x20 = 32°C
        }
    }
    
    uint8_t *hv_code = s->nand_state.hv_data + 512;
    uint32_t hv_size = s->nand_state.hv_header.size;

    // Patch 3 (temporary): Remove GPU Calls
    for (uint32_t i = 0; i < hv_size - 4; i++) {
        // Search for "bl 0xXXXXXX" (calls to init GPU)
        if (hv_code[i] == 0x48 && hv_code[i+1] == 0x00) {
            uint32_t target = (hv_code[i+2] << 16) | (hv_code[i+3] << 8) | hv_code[i+4];
            if (target >= 0xC0000000 && target < 0xD0000000) {
                printf("  Patch GPU init @ 0x%08X\n", 0x80000200 + i);
                
                hv_code[i] = 0x60; hv_code[i+1] = 0x00;
                hv_code[i+2] = 0x00; hv_code[i+3] = 0x00;
            }
        }
    }
    
    printf("[BOOT] Patches Applyed \n");
    return true;
}

/* ==================== MAIN BOOT SEQUENCE ==================== */
bool xenon_boot_sequence(XenonState *s) {
    static BOOT_CONTEXT ctx = {0};
    
    if (ctx.state == BOOT_STATE_RESET) {
        printf("\n========================================\n");
        printf("XBOX 360 BOOT SEQUENCE STARTING\n");
        printf("========================================\n");
        ctx.state = BOOT_STATE_LOAD_NAND;
    }
    
    switch (ctx.state) {
        case BOOT_STATE_LOAD_NAND:
            if (!xenon_boot_load_nand(s)) {
                printf("[BOOT] ERROR: Failed to load NAND\n");
                ctx.state = BOOT_STATE_ERROR;
                return false;
            }
            ctx.state = BOOT_STATE_EXTRACT_COMPONENTS;
            break;
            
        case BOOT_STATE_EXTRACT_COMPONENTS:
            if (!xenon_boot_extract_components(s)) {
                printf("[BOOT] ERROR: Failed to extract components\n");
                ctx.state = BOOT_STATE_ERROR;
                return false;
            }
            ctx.state = BOOT_STATE_DECRYPT_1BL;
            break;
            
        case BOOT_STATE_DECRYPT_1BL:
            if (!xenon_boot_decrypt_chain(s)) {
                printf("[BOOT] ERROR: Failed to decrypt boot chain\n");
                ctx.state = BOOT_STATE_ERROR;
                return false;
            }
            ctx.state = BOOT_STATE_VERIFY_1BL;
            break;
            
        case BOOT_STATE_VERIFY_1BL:
            ctx.state = BOOT_STATE_PATCH_FOR_EMULATION;
            break;
            
        case BOOT_STATE_PATCH_FOR_EMULATION:
            if (!xenon_boot_patch_for_emulation(s)) {
                printf("[BOOT] Warning: Failed to apply patches\n");
            }
            ctx.state = BOOT_STATE_LOAD_INTO_MEMORY;
            break;
            
        case BOOT_STATE_LOAD_INTO_MEMORY:
            xenon_boot_load_into_memory(s);
            ctx.state = BOOT_STATE_EXECUTE_1BL;
            break;
            
        case BOOT_STATE_EXECUTE_1BL:
            printf("\n========================================\n");
            printf("EXECUTING 1BL\n");
            printf("========================================\n");
            
            PowerPCCPU *cpu = s->cpu[0];
            cpu->env.nip = 0x80000000;
            cpu->env.gpr[1] = 0x8000F000; // Stack pointer
            
            // Configure SDR1 (MMU) - disabled for now
            cpu->env.spr[SPR_SDR1] = 0;
            
            printf("Starting 1BL at 0x%08X\n", cpu->env.nip);
            ctx.state = BOOT_STATE_RUNNING;
            break;
            
        case BOOT_STATE_RUNNING:
            // Boot completed, system running
            return true;
            
        case BOOT_STATE_ERROR:
            printf("[BOOT] ERROR: Boot failed at state %d\n", ctx.state);
            return false;
            
        default:
            printf("[BOOT] Unknown state: %d\n", ctx.state);
            ctx.state = BOOT_STATE_ERROR;
            return false;
    }
    
    return true;
}

/* ==================== MACHINE INTEGRATION ==================== */
void xenon_machine_boot(XenonState *s) {
    printf("\n=== XBOX 360 BOOT ===\n");
    
    if (!s->nand_state.loaded) {
        memset(&s->nand_state, 0, sizeof(XBOX360_NAND_STATE));
    }
    
    if (!xenon_boot_sequence(s)) {
        printf("[BOOT] Boot sequence failed\n");
        
        // Fallback: load very barebones basic ROM
        printf("[BOOT] Falling back to test ROM\n");
        uint8_t basic_rom[] = {
            0x48, 0x00, 0x00, 0x08, // b 0x80000008
            0x60, 0x00, 0x00, 0x00, // nop
            0x38, 0x60, 0x00, 0x42, // li r3, 0x42
            0x4E, 0x80, 0x00, 0x20  // blr
        };
        
        cpu_physical_memory_write(0x80000000, basic_rom, sizeof(basic_rom));
        s->cpu[0]->env.nip = 0x80000000;
    }
}

/* ==================== DEBUG/DIAGNOSTICS ==================== */
void xenon_boot_debug_dump(XenonState *s) {
    printf("\n=== BOOT DEBUG INFO ===\n");
    
    if (s->nand_state.loaded) {
        printf("NAND loaded: Yes\n");
        printf("CPU Key: ");
        for (int i = 0; i < 16; i++) {
            printf("%02X", s->nand_state.key_vault.cpu_key[i]);
        }
        printf("\n");
        
        printf("1BL size: %d bytes\n", s->nand_state.bl1_header.size);
        printf("CB size:  %d bytes\n", s->nand_state.cb_header.size);
        printf("HV size:  %d bytes\n", s->nand_state.hv_header.size);
    } else {
        printf("NAND loaded: No\n");
    }
    
    printf("CPU State:\n");
    printf("  NIP: 0x%08X\n", s->cpu[0]->env.nip);
    printf("  MSR: 0x%08X\n", s->cpu[0]->env.msr);
    printf("  GPR1: 0x%08X\n", s->cpu[0]->env.gpr[1]);
}
