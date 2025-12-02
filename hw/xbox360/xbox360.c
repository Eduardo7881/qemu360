#include "qemu/osdep.h"
#include "hw/xbox360/xbox360.h"
#include "hw/xbox360/xbox360_smc.h"
#include "hw/xbox360/xbox360_gpu.h"
#include "hw/xbox360/xbox360_kernel.h"
#include "hw/xbox360/xbox360_syscall.h"
#include "hw/xbox360/xbox360_nand.h"
#include "hw/xbox360/xbox360_boot.h"
#include "hw/pci/pci.h"
#include "hw/loader.h"
#include "hw/qdev-properties.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "elf.h"
#include "cpu.h"
#include "sysemu/reset.h"
#include "sysemu/sysemu.h"
#include "hw/ppc/ppc.h"
#include "hw/irq.h"

// ==================== SYSTEM CONFIGURATION ====================
#define XBOX360_SMC_BASE        0x80000200
#define XBOX360_GPU_BASE        0xC0000000
#define XBOX360_CPU_BASE        0x80000100
#define SMC_BASE_ADDRESS        0x80000200
#define HV_BASE_ADDRESS         0x80000200
#define BOOT_ROM_ADDRESS        0x80000000
#define RAM_BASE_ADDRESS        0x00000000
#define RAM_SIZE                (512 * 1024 * 1024)  // 512MB
#define BOOT_ROM_SIZE           (16 * 1024 * 1024)   // 16MB

typedef enum {
    BOOT_STATE_RESET = 0,
    BOOT_STATE_1BL = 1,
    BOOT_STATE_CB = 2,
    BOOT_STATE_HV = 3,
    BOOT_STATE_DASHBOARD = 4,
    BOOT_STATE_GAME = 5
} BootState;

// ==================== FUNCTIONS ====================
static void xenon_hardware_reset(void *opaque) {
    XenonState *s = opaque;
    
    for (int i = 0; i < 3; i++) {
        if (s->cpu[i]) {
            PowerPCCPU *cpu = s->cpu[i];
            cpu_reset(CPU(cpu));
            
            cpu->env.spr[SPR_PVR] = 0x710000;      // Processor Version Register
            cpu->env.spr[SPR_HID0] = 0x80000000;   // Hardware Implementation Register 0
            cpu->env.spr[SPR_HID1] = 0x00000000;
            
            if (i == 0) {
                cpu->env.nip = BOOT_ROM_ADDRESS;    // Bootloader address
                cpu->env.gpr[1] = 0x8000F000;       // Stack pointer
                cpu->env.gpr[3] = 0x00000176;       // TOC - Table of Contents
                cpu->env.gpr[4] = 0x80000100;       // System info
                cpu->env.msr = 0x00009000;          // Machine State Register
                
                printf("[CPU0] Inicializado: NIP=0x%08X, SP=0x%08X, MSR=0x%08X\n",
                       cpu->env.nip, cpu->env.gpr[1], cpu->env.msr);
            } else {
                cpu->env.nip = 0xFFFFFFFC;          // Wait state
                cpu->env.msr = 0x00000000;
            }
        }
    }
    
    if (s->smc) {
        xbox360_smc_set_power_state(s->smc, POWER_STATE_ON);
        xbox360_smc_set_boot_reason(s->smc, BOOT_REASON_POWER_ON);
        
        xbox360_smc_set_temperature(s->smc, 450, 500);  // 45°C CPU, 50°C GPU
        xbox360_smc_set_fan_speed(s->smc, 30);          // 30% fan speed
    }
    
    if (s->gpu) {
        device_reset(DEVICE(s->gpu));
    }
    
    memset(&s->nand_state, 0, sizeof(s->nand_state));
    if (xenon_nand_load_dump(&s->nand_state, "nand.bin")) {
        printf("[NAND] Dump loaded: %d bytes\n", NAND_SIZE);
        
        xenon_nand_extract_bl1(&s->nand_state);
        xenon_nand_extract_cb(&s->nand_state);
        xenon_nand_extract_hv(&s->nand_state);
        xenon_nand_extract_keyvault(&s->nand_state);
        
        printf("[NAND] Components extracted\n");
    } else {
        printf("[NAND] WARNING: Using Virtual NAND\n");
        create_virtual_nand(&s->nand_state);
    }
    
    s->boot_state = BOOT_STATE_RESET;
}

// Handler de interrupções do GPU
static void xenon_gpu_interrupt_handler(void *opaque, uint32_t interrupts) {
    XenonState *s = opaque;
    
    if (interrupts & GPU_INTR_VBLANK) {
        // TODO: IRQ routing to CPU
        static int frame_count = 0;
        frame_count++;
        
        if ((frame_count % 60) == 0) {
            printf("[GPU] VBLANK: Frame %d\n", frame_count);
        }
    }
    
    if (interrupts & GPU_INTR_3D) {
        printf("[GPU] 3D operation completed\n");
    }
    
    if (interrupts & GPU_INTR_2D) {
        printf("[GPU] 2D blit operation completed\n");
    }
    
    if (interrupts & GPU_INTR_CP) {
        printf("[GPU] Command processor completed\n");
    }
    
    if (interrupts & GPU_INTR_ERROR) {
        printf("[GPU] ERROR: Interrupts=0x%08X\n", interrupts);
        if (s->gpu) {
            device_reset(DEVICE(s->gpu));
        }
    }
}

// Handler de controle de energia do SMC
static void xenon_power_management_handler(void *opaque, POWER_STATE state) {
    XenonState *s = opaque;
    
    switch (state) {
        case POWER_STATE_OFF:
            qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
            break;
            
        case POWER_STATE_STANDBY:
            s->boot_state = BOOT_STATE_RESET;
            break;
            
        case POWER_STATE_ON:
            xenon_hardware_reset(s);
            
            if (xenon_boot_sequence(s)) {
                s->boot_state = BOOT_STATE_DASHBOARD;
            } else {
                printf("[BOOT] ERROR: Error on initialization\n");
                qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
            }
            break;
            
        case POWER_STATE_REBOOT:
            printf("REBOOT - Restarting\n");
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
            break;
            
        default:
            printf("UNKNOWN (%d)\n", state);
            break;
    }
}

// ==================== MACHINE INIT ====================
const char* asciiart =
"\x1b[32m"
" $$$$$$\\  $$$$$$$$\\ $$\\      $$\\ $$\\   $$\\  $$$$$$\\   $$$$$$\\   $$$$$$\\ \n"
"$$  __$$\\ $$  _____|$$$\\    $$$ |$$ |  $$ |$$ ___$$\\ $$  __$$\\ $$$ __$$\\    \n"
"$$ /  $$ |$$ |      $$$$\\  $$$$ |$$ |  $$ |\\_/   $$ |$$ /  \\__|$$$$\\ $$ |    \n"
"$$ |  $$ |$$$$$\\    $$\\$$\\$$ $$ |$$ |  $$ |  $$$$$ / $$$$$$$\\  $$\\$$\\$$ |   \n"
"$$ |  $$ |$$  __|   $$ \\$$$  $$ |$$ |  $$ |  \\___$$\\ $$  __$$\\ $$ \\$$$$ |   \n"
"$$ $$\\$$ |$$ |      $$ |\\$  /$$ |$$ |  $$ |$$\\   $$ |$$ /  $$ |$$ |\\$$$ |    \n"
"\\$$$$$$ / $$$$$$$$\ $$ | \\_/ $$ |\\$$$$$$  |\\$$$$$$  | $$$$$$  |\\$$$$$$  /    \n"
" \\___$$$\ \\________|\\__|     \\__| \\______/  \\______/  \\______/  \\______/ \n"
"     \\___|                                                                      \n"
"\1b[0m";

static void xenon_machine_init(MachineState *machine) {
    XenonState *s = XBOX360_MACHINE(machine);
    Error *err = NULL;

    printf("%s", asciiart);
    printf("                    Microsoft Xbox 360                       \n");
    printf("                    Emulator v1.0.0                         \n");
    printf("\n");
    
    for (int i = 0; i < 3; i++) {
        s->cpu[i] = POWERPC_CPU(cpu_create(machine->cpu_type));
        if (!s->cpu[i]) {
            error_report("Fail to create CPU %d", i);
            exit(1);
        }
        
        PowerPCCPU *cpu = s->cpu[i];
        cpu->env.spr[SPR_PVR] = 0x710000;
        cpu->env.spr[SPR_SDR1] = 0;
        
        printf("[CPU%d] PowerPC 750CL @ 3.2GHz inicializado\n", i);
    }
    
    memory_region_init_ram(&s->ram, OBJECT(s), "xbox360.ram", 
                          RAM_SIZE, &error_fatal);
    memory_region_add_subregion(get_system_memory(), 
                               RAM_BASE_ADDRESS, &s->ram);
    printf("[RAM] 512MB @ 0x00000000\n");
    
    memory_region_init_ram(&s->boot_rom, OBJECT(s), "xbox360.bootrom",
                          BOOT_ROM_SIZE, &error_fatal);
    memory_region_add_subregion(get_system_memory(),
                               BOOT_ROM_ADDRESS, &s->boot_rom);
    printf("[ROM] 16MB @ 0x80000000\n");
    
    memory_region_init_io(&s->mmio, OBJECT(s), &xbox360_mmio_ops, s,
                         "xbox360.mmio", 0x10000000);
    memory_region_add_subregion(get_system_memory(),
                               0xC0000000, &s->mmio);
    printf("[MMIO] 256MB @ 0xC0000000\n");

    s->smc = xbox360_smc_create(get_system_memory(), 
                               SMC_BASE_ADDRESS,
                               &s->nand_state);
    xbox360_smc_set_power_callback(s->smc, xenon_power_management_handler, s);
    xbox360_smc_set_reset_callback(s->smc, xenon_hardware_reset, s);
    printf("[SMC] System Management Controller @ 0x%08X\n", SMC_BASE_ADDRESS);
    
    s->pci_bus = pci_new_bus(DEVICE(s), "pci", NULL, NULL, 0, TYPE_PCI_BUS);
    s->gpu = xbox360_gpu_create(s->pci_bus, get_system_memory(), NULL);
    xbox360_gpu_set_interrupt_callback(s->gpu, xenon_gpu_interrupt_handler, s);
    printf("[GPU] Xenos R500 @ PCI Bus\n");
    
    s->kernel_state = g_new0(XBOX360_KERNEL_STATE, 1);
    xbox360_kernel_init(s->kernel_state, s);
    xbox360_install_syscall_handler(s);
    printf("[KERNEL] Xbox 360 Kernel v2.0.17559.0\n");
    
    qemu_register_reset(xenon_hardware_reset, s);
    xenon_machine_boot(s);
}

// ==================== MMIO ====================
static uint64_t xbox360_mmio_read(void *opaque, hwaddr addr, unsigned size) {
    XenonState *s = opaque;
    uint32_t value = 0;
    
    if (addr >= 0xC0000000 && addr < 0xC0001000) {
        value = xbox360_gpu_vram_read(s->gpu, addr - 0xC0000000, size);
    } else if (addr >= 0x80000000 && addr < 0x80010000) {
        // TODO: Implement HV access
        value = 0xDEADBEEF;
    } else if (addr >= 0x20000000 && addr < 0x21000000) {
        uint32_t offset = addr - 0x20000000;
        if (offset < VRAM_SIZE) {
            value = xbox360_gpu_vram_read(s->gpu, offset, size);
        }
    }
    
    return value;
}

static void xbox360_mmio_write(void *opaque, hwaddr addr, uint64_t value, unsigned size) {
    XenonState *s = opaque;
    
    if (addr >= 0xC0000000 && addr < 0xC0001000) {
        xbox360_gpu_vram_write(s->gpu, addr - 0xC0000000, value, size);
    } else if (addr >= 0x80000000 && addr < 0x80010000) {
        // TODO: Implement HV Access
        printf("[MMIO] HV Write @ 0x%08"HWADDR_PRIx": 0x%08"PRIx64"\n", addr, value);
    } else if (addr >= 0x20000000 && addr < 0x21000000) {
        uint32_t offset = addr - 0x20000000;
        if (offset < VRAM_SIZE) {
            xbox360_gpu_vram_write(s->gpu, offset, value, size);
        }
    }
}

static const MemoryRegionOps xbox360_mmio_ops = {
    .read = xbox360_mmio_read,
    .write = xbox360_mmio_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

// ==================== MACHINE ====================
static void xenon_machine_class_init(ObjectClass *oc, void *data) {
    MachineClass *mc = MACHINE_CLASS(oc);
    
    mc->desc = "Microsoft Xbox 360 (Xenon)";
    mc->init = xenon_machine_init;
    mc->max_cpus = 3;
    mc->min_cpus = 1;
    mc->default_cpus = 1;
    mc->default_ram_size = RAM_SIZE;
    mc->default_ram_id = "xbox360.ram";
    mc->default_cpu_type = POWERPC_CPU_TYPE_NAME("7400");
}

static const TypeInfo xenon_machine_type = {
    .name = TYPE_XBOX360_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(XenonState),
    .class_init = xenon_machine_class_init,
};

static void xenon_machine_register_types(void) {
    type_register_static(&xenon_machine_type);
}

type_init(xenon_machine_register_types);
