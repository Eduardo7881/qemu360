#include "qemu/osdep.h"
#include "hw/xbox360/xbox360.h"
#include "hw/xbox360/xbox360_smc.h"
#include "hw/xbox360/xbox360_gpu.h"
#include "hw/xbox360/xbox360_kernel.h"
#include "hw/xbox360/xbox360_syscall.h"
#include "hw/pci/pci.h"
#include "hw/loader.h"
#include "hw/qdev-properties.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "elf.h"

#define XBOX360_SMC_BASE        0x80000200
#define XBOX360_GPU_BASE        0xC0000000
#define XBOX360_CPU_BASE        0x80000100

#define BOOT_STAGE_RESET        0 /* resetting CPU */
#define BOOT_STAGE_1BL          1 /* 1BL Stage */
#define BOOT_STATE_CB           2 /* CB  Stage */
#define BOOT_STATE_HV           3 /* Hypervisor Stage */
#define BOOT_STAGE_DASHBOARD    4 /* On dashboard */

static void xenon_cpu_reset(void *opaque)
{
  XenonState *s = opaque;

  for (int = 0, i < 3; i++) {
    if (s->cpu[i]) {
      cpu_reset(CPU(s->cpu[i]));
    }
  }

  s->boot_stage = BOOT_STAGE_RESET;
  printf("[XBOX360] CPU Reset\n");
}

static uint64_t xenon_syscall(void *opaque, uint64_t addr, unsigned size)
{
  uint32_t syscall_id = addr & 0xFFF;
  printf("[XBOX360] Syscall %03X called from HV\n", syscall_id);

  // to prevent crashes, return success.
  // this is still a STUB (not complete)
  switch(syscall_id) {
    case 0x001: // GetVersion
      return 0x00000200; // V2.0
    case 0x002: // GetFuseLine
      return 0xFFFFFFFF; // All fuses OK
    case 0x003: // GetTemperature
      return 45; // 45ºC
    default:
      return 0; // success
  }
}

static void xenon_create_gpu(XenonState *s) {
    printf("[XBOX360] Creating GPU...\n");
    s->pci_bus = pci_new_bus(DEVICE(s), "pci", NULL, NULL, 0, TYPE_PCI_BUS);
    
    s->gpu = xbox360_gpu_create(s->pci_bus, get_system_memory(), NULL);
    xbox360_gpu_set_interrupt_callback(s->gpu, xenon_gpu_interrupt_callback, s);
    
    printf("[XBOX360] GPU initialized\n");
}

static void xenon_gpu_interrupt_callback(void *opaque, uint32_t interrupts) {
    XenonState *s = opaque;
    printf("[XBOX360] GPU Interrupt: 0x%08X\n", interrupts);
    
    // TODO: Map GPU Interrupt to Interrupt Controller
    // I'm only logging here... HELP
}

static void xenon_init_boot_rom(XenonState *s)
{
  /* Creates a Minimal Boot ROM */
  /* This is still a STUB */

  uint8_t *rom_data = memory_region_get_ram_ptr(&s->boot_rom);

  /* Instruction: "b 0x100" (branch to 0x100) */
  rom_data[0] = 0x48;
  rom_data[1] = 0x00;
  rom_data[2] = 0x01;
  rom_data[3] = 0x00;

  /* In 0x100: "li r3, 0x42; blr (return 0x42) */
  rom_data[0x100] = 0x38;
  rom_data[0x101] = 0x60;
  rom_data[0x102] = 0x00;
  rom_data[0x103] = 0x42;
  rom_data[0x104] = 0x4E;
  rom_data[0x105] = 0x80;
  rom_data[0x106] = 0x00;
  rom_data[0x107] = 0x20;

  printf("[XBOX360] Boot ROM Initialized\n");
}

static void xenon_init_cpus(XenonState *s)
{
  const char *cpu_type = "powerpc-embedded";

  for (int i = 0; i < 3; i++) {
    s->cpu[i] = POWERPC_CPU(cpu_create(cpu_type));
    if (!s->cpu[i]) { error_report("Failed to create CPU %d", i); exit(1); }

    s->cpu[i]->env.spr[SPR_PVR] = 0x710000;
    s->cpu[i]->env.nip = 0x80000000;
  }

  /* Only Core 0 starts active */
  cpu_reset(CPU(s->cpu[0]));
}

static void xenon_create_smc(XenonState *s) {
    printf("[XBOX360] Creating SMC...\n");
    
    s->smc = xbox360_smc_create(get_system_memory(), 
                                SMC_BASE_ADDRESS,
                                &s->nand_state);
    
    xbox360_smc_set_power_callback(s->smc, xenon_smc_power_callback, s);
    xbox360_smc_set_reset_callback(s->smc, xenon_smc_reset_callback, s);
    
    xbox360_smc_set_boot_reason(s->smc, BOOT_REASON_RGH);
    
    printf("[XBOX360] SMC initialized at 0x%08X\n", SMC_BASE_ADDRESS);
}

static void xenon_smc_power_callback(void *opaque, POWER_STATE state) {
    XenonState *s = opaque;
    
    printf("[XBOX360] SMC Power Callback: state=%d\n", state);
    
    switch (state) {
        case POWER_STATE_OFF:
            // Shutdown
            // qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
            break;
            
        case POWER_STATE_REBOOT:
            // Restart
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
            break;
            
        default:
            break;
    }
}

static void xenon_smc_reset_callback(void *opaque) {
    XenonState *s = opaque;
    
    printf("[XBOX360] SMC Reset Callback\n");
    
    for (int i = 0; i < 3; i++) {
        if (s->cpu[i]) {
            cpu_reset(CPU(s->cpu[i]));
        }
    }
    
    s->boot_stage = BOOT_STAGE_RESET;
}

static void xenon_init_memory(XenonState *s)
{
  /* Main RAM: 512MB @ 0x00000000 */
  memory_region_init_ram(&s->ram, NULL, "xbox360.ram", 512 * 1024 * 1024, &error_fatal);
  memory_region_add_subregion(get_system_memory(), 0x00000000, &s->ram);

  /* Boot ROM: 16MB @ 0x80000000 (where 1BL expects) */
  memory_region_init_ram(&s->boot_rom, NULL, "xbox360.bootrom", 16 * 1024 * 1024, &error_fatal);
  memory_region_add_subregion(get_system_memory(), 0x80000000, &s->boot_rom);

  /* MMIO Region @ 0xC0000000 */
  memory_region_init_io(&s->mmio, NULL, &xenon_syscall_stub, s, "xbox360.mmio", 0x10000000);
  memory_region_add_subregion(get_system_memory(), 0xC0000000, &s->mmio);

  printf("[XBOX360] Memory initialized\n");
}

static void xenon_machine_init(MachineState *machine)
{
  XenonState *s = XBOX360_MACHINE(machine);
    
  printf("\n========================================\n");
  printf("XBOX 360 (Board: Trinity)\n");
  printf("========================================\n");
    
  xenon_init_cpus(s);
  xenon_init_memory(s);
  xenon_create_smc(s);
  xenon_create_gpu(s);
  xenon_init_boot_rom(s);

  qemu_register_reset(xenon_cpu_reset, s);
    
  printf("[XBOX360] Machine initialized successfully\n");
  printf("Boot Stage: %d (RESET)\n", s->boot_stage);
}

static void xenon_machine_class_init(ObjectClass *oc, void *data)
{
  MachineClass *mc = MACHINE_CLASS(oc);

  mc->desc = "Microsoft Xbox 360";
  mc->init = xenon_machine_init;
  mc->max_cpus = 3;
  mc->min_cpus = 1;
  mc->default_cpus = 1;
  mc->default_ram_size = 512 * 1024 * 1024;
  mc->default_ram_id = "xbox360.ram";
}

static const TypeInfo xenon_machine_type = {
  .name = TYPE_XBOX360_MACHINE,
  .parent = TYPE_MACHINE,
  .instance_size = sizeof(XenonState);
  .class_init = xenon_machine_class_init,
};

static void xenon_machine_register_types(void)
{
  type_register_static(&xenon_machine_type);
}

type_init(xenon_machine_register_types);
