#ifndef HW_XBOX360_MMU_H
#define HW_XBOX360_MMU_H

#include "qom/object.h"
#include "hw/ppc/ppc.h"
#include <stdint.h>

/* MMU register offsets */
#define MMU_REG_MMUCR      0x00
#define MMU_REG_PID        0x01
#define MMU_REG_LPCR       0x02
#define MMU_REG_HID0       0x03
#define MMU_REG_L1CSR0     0x10
#define MMU_REG_L1CSR1     0x11
#define MMU_REG_L2CSR0     0x12
#define MMU_REG_L2CSR1     0x13

/* Xenon MMU context */
typedef struct XenonMMUState XenonMMUState;

/* Public functions */
void xenon_mmu_init(XenonMMUState *mmu, PowerPCCPU *cpu);
void xenon_mmu_reset(XenonMMUState *mmu);
int xenon_mmu_handle_mmu_fault(PowerPCCPU *cpu, vaddr eaddr, int rw, int mmu_idx);

uint32_t xenon_mmu_read_reg(XenonMMUState *mmu, uint32_t reg);
void xenon_mmu_write_reg(XenonMMUState *mmu, uint32_t reg, uint32_t value);

void xenon_cache_invalidate(XenonMMUState *mmu, target_ulong addr, size_t size);
void xenon_cache_flush(XenonMMUState *mmu);

void xenon_mmu_dump_state(XenonMMUState *mmu);

#endif
