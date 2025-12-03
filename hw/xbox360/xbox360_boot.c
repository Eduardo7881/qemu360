#include "hw/xbox360/xbox360.h"
#include "hw/xbox360/xbox360_nand.h"
#include "hw/xbox360/crypto/xe_crypto.h"
#include "qemu/osdep.h"
#include "hw/loader.h"

/* ==================== BOOT SEQUENCE ==================== */

bool xenon_boot_load_nand(XenonState *s) {
    if (!s->nand_loaded) {
        if (xenon_nand_load_dump(&s->nand_state, "nand.bin")) {
            s->nand_loaded = true;
            return true;
        }
        
        const char *alternatives[] = {
            "nanddump.bin",
            "flash.bin",
            "xbox360.bin",
            NULL
        };
        
        for (int i = 0; alternatives[i]; i++) {
            if (xenon_nand_load_dump(&s->nand_state, alternatives[i])) {
                s->nand_loaded = true;
                return true;
            }
        }
        
        return false;
    }
    
    return true;
}

bool xenon_boot_extract_components(XenonState *s) {
    if (!xenon_nand_extract_bl1(&s->nand_state)) {
        return false;
    }
    
    if (!xenon_nand_extract_cb(&s->nand_state)) {
        return false;
    }
    
    if (!xenon_nand_extract_hv(&s->nand_state)) {
        return false;
    }
    
    if (!xenon_nand_verify_keyvault(&s->nand_state)) {
        return false;
    }
    
    return true;
}

bool xenon_boot_decrypt_chain(XenonState *s) {
    if (!xenon_nand_decrypt_bl1(&s->nand_state)) {
        return false;
    }
    
    if (!xenon_nand_verify_bl1(&s->nand_state)) {
        return false;
    }
    
    if (!xenon_nand_decrypt_cb(&s->nand_state)) {
        return false;
    }
    
    if (!xenon_nand_verify_cb(&s->nand_state)) {
        return false;
    }
    
    if (!xenon_nand_decrypt_hv(&s->nand_state)) {
        return false;
    }
    
    if (!xenon_nand_verify_hv(&s->nand_state)) {
        return false;
    }
    
    return true;
}

void xenon_boot_load_into_memory(XenonState *s) {
    cpu_physical_memory_write(0x80000000, 
                             s->nand_state.bl1_data + 512,
                             s->nand_state.bl1_header.size);
    
    cpu_physical_memory_write(0x80002000,
                             s->nand_state.cb_data + 512,
                             s->nand_state.cb_header.size);
    
    cpu_physical_memory_write(0x80000200,
                             s->nand_state.hv_data + 512,
                             s->nand_state.hv_header.size);
    
    PowerPCCPU *cpu = s->cpu[0];
    cpu->env.nip = s->nand_state.bl1_header.entry_point;
    cpu->env.gpr[1] = 0x8000F000;
    cpu->env.spr[SPR_SDR1] = 0;
}

bool xenon_boot_patch_for_emulation(XenonState *s) {
    uint8_t *bl1_code = s->nand_state.bl1_data + 512;
    uint32_t bl1_size = s->nand_state.bl1_header.size;
    
    for (uint32_t i = 0; i < bl1_size - 4; i++) {
        if (bl1_code[i] == 0x3C && bl1_code[i+2] == 0x60 && 
            bl1_code[i+3] == 0x80 && bl1_code[i+4] == 0x00) {
            if (i + 8 < bl1_size && bl1_code[i+5] == 0x60 && 
                bl1_code[i+7] == 0x02 && bl1_code[i+8] == 0x00) {
                bl1_code[i] = 0x60; bl1_code[i+1] = 0x00;
                bl1_code[i+2] = 0x00; bl1_code[i+3] = 0x00;
                bl1_code[i+4] = 0x60; bl1_code[i+5] = 0x00;
                bl1_code[i+6] = 0x00; bl1_code[i+7] = 0x00;
                bl1_code[i+8] = 0x60; bl1_code[i+9] = 0x00;
                bl1_code[i+10] = 0x00; bl1_code[i+11] = 0x00;
            }
        }
    }
    
    for (uint32_t i = 0; i < bl1_size - 8; i++) {
        if (bl1_code[i] == 0x2C && bl1_code[i+3] == 0x00 && 
            bl1_code[i+4] == 0x50) {
            bl1_code[i] = 0x38;
            bl1_code[i+4] = 0x20;
        }
    }
    
    uint8_t *hv_code = s->nand_state.hv_data + 512;
    uint32_t hv_size = s->nand_state.hv_header.size;
    
    for (uint32_t i = 0; i < hv_size - 4; i++) {
        if (hv_code[i] == 0x48 && hv_code[i+1] == 0x00) {
            uint32_t target = (hv_code[i+2] << 16) | (hv_code[i+3] << 8) | hv_code[i+4];
            if (target >= 0xC0000000 && target < 0xD0000000) {
                hv_code[i] = 0x60; hv_code[i+1] = 0x00;
                hv_code[i+2] = 0x00; hv_code[i+3] = 0x00;
            }
        }
    }
    
    return true;
}

/* ==================== MAIN BOOT SEQUENCE ==================== */

bool xenon_boot_sequence(XenonState *s) {
    if (!xenon_boot_load_nand(s)) {
        return false;
    }
    
    if (!xenon_boot_extract_components(s)) {
        return false;
    }
    
    if (!xenon_boot_decrypt_chain(s)) {
        return false;
    }
    
    if (!xenon_boot_patch_for_emulation(s)) {
        return false;
    }
    
    xenon_boot_load_into_memory(s);
    
    PowerPCCPU *cpu = s->cpu[0];
    cpu->env.nip = 0x80000000;
    cpu->env.gpr[1] = 0x8000F000;
    
    return true;
}

void xenon_machine_boot(XenonState *s) {
    memset(&s->nand_state, 0, sizeof(XBOX360_NAND_STATE));
    
    if (!xenon_boot_sequence(s)) {
        uint8_t basic_rom[] = {
            0x48, 0x00, 0x00, 0x08,
            0x60, 0x00, 0x00, 0x00,
            0x38, 0x60, 0x00, 0x42,
            0x4E, 0x80, 0x00, 0x20
        };
        
        cpu_physical_memory_write(0x80000000, basic_rom, sizeof(basic_rom));
        s->cpu[0]->env.nip = 0x80000000;
    }
}

/* ==================== DEBUG ==================== */

void xenon_boot_debug_dump(XenonState *s) {
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
