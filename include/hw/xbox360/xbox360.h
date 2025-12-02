#ifndef HW_XBOX360_H
#define HW_XBOX360_H

#include "hw/boards.h"
#include "hw/ppc/ppc.h"
#include "hw/xbox360/xbox360_smc.h"
#include "hw/xbox360/xbox360_gpu.h"
#include "hw/pci/pci.h"
#Include "qom/object.h"

#define TYPE_XBOX360_MACHINE MACHINE_TYPE_NAME("xbox360")

typedef struct XenonState XenonState;
DECLARE_INSTANCE_CHECKER(XenonState, XBOX360_MACHINE, TYPE_XBOX360_MACHINE)

struct XenonState {
  /*< private >*/
  MachineState parent_obj;

  /*< public >*/
  PowerPCCPU *cpu[3]; // 3 Core Xenon
  DeviceState *gic;   // Interrupt Controller
  MemoryRegion ram;   // 512MB RAM
  MemoryRegion mmio;  // Memory-mapped I/O
  MemoryRegion boot_rom; // 16MB boot ROM

  /* SMC */
  Xbox360SMCState *smc;

  /* GPU */
  Xbox360GPUState *gpu;
  PCIBus *pci_bus;

  uint8_t nand_data[0x400000]; // 4MB NAND dump
  uint8_t cpu_key[16];         // Console's CPU KEY.
  uint32_t fuse_bits[16];      // Console Fuses
  uint32_t smc_version;        // SMC Version

  uint64_t hv_phys_addr;       // Hypervisor physical address;
  uint32_t boot_state;         // Current boot stage
};

void xenon_load_bootloader(XenonState *s);                                // Function to load the Bootloader from DUMP.
void xenon_path_hypervisor(XenonState *s, uint8_t *hv_data, size_t size); // Function that patches the Hypervisor to run in a VM.

// Credits: Xenia
void xenon_aes_decrypt(const uint8_t *key, const uint8_t *iv, uint8_t *data, size_t size);
uint8_t xenon_cpu_keystream(uint32_t seed);

#endif
