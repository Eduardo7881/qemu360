#include "qemu/osdep.h"
#include "cpu.h"
#include "hw/ppc/xbox360_cpu.h"

/*
WARNING: This file is 99% stub and will not even boot into the dashboard
as it is only for testing for now.
*/

#define XENON_HID0      0x3F0
#define XENON_TB_ADDR   0x3F8
#define XENON_CIR       0x3FC

static uint64_t xenon_read_spr(CPUPPCState *env, int spr)
{
  switch(spr) {
    case XENON_HID0:
      return 0x80000000; // CPU revision and more
    case XENON_TB_ADDR:
      return env->spr[SPR_TBL] & 0xFFFFFFFF;
    case XENON_CIR:
      return 0x71000000; // Xenon chip ID
    default:
      /* Fallback to default PowerPC */
      return cpu_ppc_load_spr(env, spr);
  }
}

static void xenon_write_spr(CPUPPCState *env, int spr, uint64_t value)
{
  switch(spr) {
    case XENON_HID0:
    case XENON_TB_ADDR:
    case XENON_CIR:
      printf("[XENON] Write to read-only SPR %03X: %015lX\n", spr, value);
    break;
    default:
      cpu_ppc_store_spr(env, spr, value);
      break;
  }
}

void xenon_cpu_init(PowerPCCPU *cpu)
{
  CPUPPCState *env = &cpu->env;

  /* Overwrittes SPR handlers */
  env->spr_cb[SPR_GENERAL].read = xenon_read_spr;
  env->spr_cb[SPR_GENERAL].write = xenon_write_spr;

  /* Xenon Config */
  env->mmu_model = POWERPC_MMU_6xx;
  env->excp_model = POWERPC_EXCP_60x;
  env->flags = POWERPC_FLAG_HID0_EMCP | POWERPC_FLAG_HID0_SEID | POWERPC_FLAG_BE;

  env->nr_threads = 2;
  env->pvr = 0x710000;
}
