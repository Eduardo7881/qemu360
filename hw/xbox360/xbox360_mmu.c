#include "qemu/osdep.h"
#include "hw/xbox360/xbox360_mmu.h"
#include "hw/xbox360/xbox360.h"
#include "cpu.h"
#include "exec/exec-all.h"

/* ==================== XENON MMU CONFIGURATION ==================== */
#define XENON_TLB_SIZE           128
#define XENON_TLB_WAYS           2
#define XENON_PAGE_SIZE          4096
#define XENON_SEGMENT_SIZE       0x10000000  /* 256MB segments */

/* Xenon-specific MMU bits */
#define MMUCR_TID_SHIFT          16
#define MMUCR_TID_MASK           0x000F0000
#define MMUCR_GS                 (1 << 11)  /* Guest state */
#define MMUCR_PP                 (1 << 10)  /* Page protection */

/* TLB entry structure */
typedef struct XenonTLBEntry {
    target_ulong epid;           /* Effective Page ID */
    target_ulong rpn;            /* Real Page Number */
    uint32_t v;                  /* Valid */
    uint32_t ts;                 /* Translation space */
    uint32_t sr;                 /* Storage attribute */
    uint32_t size;               /* Page size: 0=4KB, 1=64KB, 2=1MB, 3=16MB */
    uint32_t tid;                /* Thread ID */
    uint32_t pp;                 /* Page protection */
    uint32_t ci;                 /* Cache inhibited */
    uint32_t wimg;               /* Memory attributes */
} XenonTLBEntry;

/* MMU state */
typedef struct XenonMMUState {
    XenonTLBEntry tlb[XENON_TLB_SIZE];
    uint32_t mmucr;              /* MMU Control Register */
    uint32_t pid;                /* Process ID */
    uint32_t lpcr;               /* Logical Partition Control */
    uint32_t hid0;               /* Hardware Implementation Register 0 */
    uint32_t l1csr0;             /* L1 Cache Control/Status */
    uint32_t l1csr1;
    uint32_t l2csr0;
    uint32_t l2csr1;
    
    /* Xenon-specific */
    uint32_t msr_gs;             /* Guest state MSR */
    uint32_t lpcr_gtse;          /* Guest translation enable */
    uint32_t hid0_xenon;         /* Xenon-specific HID0 bits */
} XenonMMUState;

/* ==================== TLB MANAGEMENT ==================== */

static uint32_t tlb_hash(target_ulong epid, uint32_t tid, uint32_t ts) {
    return ((epid >> 12) ^ tid ^ ts) & (XENON_TLB_SIZE - 1);
}

static XenonTLBEntry *tlb_lookup(XenonMMUState *mmu, target_ulong epid, 
                                uint32_t tid, uint32_t ts, int *index) {
    uint32_t hash = tlb_hash(epid, tid, ts);
    
    for (int way = 0; way < XENON_TLB_WAYS; way++) {
        int idx = (hash + way) % XENON_TLB_SIZE;
        XenonTLBEntry *entry = &mmu->tlb[idx];
        
        if (entry->v && entry->epid == epid && 
            entry->tid == tid && entry->ts == ts) {
            if (index) *index = idx;
            return entry;
        }
    }
    
    return NULL;
}

static void tlb_insert(XenonMMUState *mmu, target_ulong epid, target_ulong rpn,
                      uint32_t tid, uint32_t ts, uint32_t size,
                      uint32_t pp, uint32_t ci, uint32_t wimg) {
    uint32_t hash = tlb_hash(epid, tid, ts);
    int idx = hash;  /* Simple LRU - always replace at hash */
    
    XenonTLBEntry *entry = &mmu->tlb[idx];
    entry->epid = epid;
    entry->rpn = rpn;
    entry->tid = tid;
    entry->ts = ts;
    entry->size = size;
    entry->pp = pp;
    entry->ci = ci;
    entry->wimg = wimg;
    entry->v = 1;
    entry->sr = 0;  /* Supervisor read */
}

static void tlb_flush_all(XenonMMUState *mmu) {
    for (int i = 0; i < XENON_TLB_SIZE; i++) {
        mmu->tlb[i].v = 0;
    }
}

static void tlb_flush_tid(XenonMMUState *mmu, uint32_t tid) {
    for (int i = 0; i < XENON_TLB_SIZE; i++) {
        if (mmu->tlb[i].tid == tid) {
            mmu->tlb[i].v = 0;
        }
    }
}

/* ==================== ADDRESS TRANSLATION ==================== */

static int xenon_translate(XenonMMUState *mmu, target_ulong eaddr,
                          hwaddr *raddr, int rw, int mmu_idx) {
    uint32_t ts = (mmu->mmucr >> 12) & 1;  /* Translation space */
    uint32_t tid = (mmu->mmucr >> MMUCR_TID_SHIFT) & 0xF;
    
    /* Effective page ID (VPN) */
    target_ulong epid = eaddr >> 12;
    
    /* Lookup in TLB */
    XenonTLBEntry *entry = tlb_lookup(mmu, epid, tid, ts, NULL);
    if (entry) {
        /* Check protection */
        if (rw == MMU_DATA_STORE && (entry->pp & 2) == 0) {
            return -2;  /* Write protection fault */
        }
        if (rw == MMU_DATA_LOAD && (entry->pp & 1) == 0) {
            return -2;  /* Read protection fault */
        }
        
        /* Calculate physical address */
        target_ulong page_offset = eaddr & (XENON_PAGE_SIZE - 1);
        *raddr = (entry->rpn << 12) | page_offset;
        
        /* Apply memory attributes */
        if (entry->ci) {
            *raddr |= 0x80000000;  /* Cache inhibited */
        }
        
        return 0;
    }
    
    /* TLB miss - Xenon uses BAT (Block Address Translation) */
    /* Check BAT registers */
    uint32_t bat_upper[4] = {0};
    uint32_t bat_lower[4] = {0};
    
    for (int i = 0; i < 4; i++) {
        if (eaddr >= (bat_upper[i] & ~0x1FFFF) && 
            eaddr <= (bat_upper[i] | 0x1FFFF)) {
            if (bat_lower[i] & 0x40) {  /* BAT valid */
                target_ulong phys_base = bat_lower[i] & ~0x1FFFF;
                *raddr = phys_base + (eaddr & 0x1FFFF);
                
                /* Insert into TLB for future use */
                tlb_insert(mmu, epid, phys_base >> 12, tid, ts,
                          0,  /* 4KB page */
                          (bat_lower[i] >> 2) & 3,  /* PP */
                          (bat_lower[i] >> 5) & 1,  /* CI */
                          (bat_lower[i] >> 6) & 0xF); /* WIMG */
                
                return 0;
            }
        }
    }
    
    /* Page fault */
    return -1;
}

/* ==================== XENON-SPECIFIC MMU FUNCTIONS ==================== */

void xenon_mmu_init(XenonMMUState *mmu, PowerPCCPU *cpu) {
    memset(mmu, 0, sizeof(XenonMMUState));
    
    /* Xenon-specific MMU configuration */
    mmu->mmucr = 0x00000000;
    mmu->pid = 0;
    mmu->lpcr = 0x00000000;
    mmu->hid0 = 0x80000000;  /* EMCP = 1 (Enable Machine Check) */
    
    /* Xenon cache configuration */
    mmu->l1csr0 = 0x00000000;  /* L1 I-cache disabled */
    mmu->l1csr1 = 0x00000000;  /* L1 D-cache disabled */
    mmu->l2csr0 = 0x80000000;  /* L2 cache enabled */
    mmu->l2csr1 = 0x00000000;
    
    /* Guest state support */
    mmu->msr_gs = 0;
    mmu->lpcr_gtse = 0;
    mmu->hid0_xenon = 0x00008000;  /* Xenon-specific bit */
    
    /* Initialize TLB */
    tlb_flush_all(mmu);
    
    /* Set up Xenon memory regions in BAT registers */
    /* This maps the physical memory layout */
    cpu->env.spr[SPR_DBAT0U] = 0x80001FFF;  /* 0x80000000-0x801FFFFF */
    cpu->env.spr[SPR_DBAT0L] = 0x80000002;  /* WIMG=0010, PP=RW */
    cpu->env.spr[SPR_IBAT0U] = 0x80001FFF;
    cpu->env.spr[SPR_IBAT0L] = 0x80000002;
    
    cpu->env.spr[SPR_DBAT1U] = 0xC0001FFF;  /* 0xC0000000-0xC01FFFFF */
    cpu->env.spr[SPR_DBAT1L] = 0xC0000002;  /* MMIO region */
    cpu->env.spr[SPR_IBAT1U] = 0xC0001FFF;
    cpu->env.spr[SPR_IBAT1L] = 0xC0000000;  /* No execute */
    
    cpu->env.spr[SPR_DBAT2U] = 0x00001FFF;  /* 0x00000000-0x001FFFFF */
    cpu->env.spr[SPR_DBAT2L] = 0x00000002;  /* RAM */
    cpu->env.spr[SPR_IBAT2U] = 0x00001FFF;
    cpu->env.spr[SPR_IBAT2L] = 0x00000002;
}

void xenon_mmu_reset(XenonMMUState *mmu) {
    tlb_flush_all(mmu);
    mmu->mmucr = 0x00000000;
    mmu->pid = 0;
}

int xenon_mmu_handle_mmu_fault(PowerPCCPU *cpu, vaddr eaddr,
                              int rw, int mmu_idx) {
    XenonState *s = XBOX360_MACHINE(qdev_get_machine());
    XenonMMUState *mmu = &s->mmu_state[cpu->cpu_index];
    
    hwaddr raddr;
    int ret = xenon_translate(mmu, eaddr, &raddr, rw, mmu_idx);
    
    if (ret == 0) {
        /* Translation successful */
        cpu->env.nip = raddr;
        return 0;
    } else if (ret == -1) {
        /* Page fault */
        cpu->env.spr[SPR_DSISR] = (rw == MMU_DATA_STORE) ? 0x02000000 : 0x40000000;
        cpu->env.spr[SPR_DAR] = eaddr;
        cpu->env.error_code = 0x80000000;
        return 1;
    } else if (ret == -2) {
        /* Protection fault */
        cpu->env.spr[SPR_DSISR] = (rw == MMU_DATA_STORE) ? 0x08000000 : 0x10000000;
        cpu->env.spr[SPR_DAR] = eaddr;
        cpu->env.error_code = 0x40000000;
        return 1;
    }
    
    return 1;
}

/* ==================== CACHE MANAGEMENT ==================== */

void xenon_cache_invalidate(XenonMMUState *mmu, target_ulong addr, size_t size) {
    /* Xenon has 32KB L1, 1MB L2 cache */
    /* Simplified invalidation */
    if (mmu->l2csr0 & 0x80000000) {
        /* L2 cache enabled - invalidate range */
        // TODO: Implement proper cache invalidation
    }
}

void xenon_cache_flush(XenonMMUState *mmu) {
    /* Flush all caches */
    if (mmu->l1csr0 & 0x80000000) {
        mmu->l1csr0 |= 0x00000002;  /* L1 I-cache invalidate */
    }
    if (mmu->l1csr1 & 0x80000000) {
        mmu->l1csr1 |= 0x00000002;  /* L1 D-cache flush */
    }
    if (mmu->l2csr0 & 0x80000000) {
        mmu->l2csr0 |= 0x00000002;  /* L2 cache flush */
    }
}

/* ==================== REGISTER ACCESS ==================== */

uint32_t xenon_mmu_read_reg(XenonMMUState *mmu, uint32_t reg) {
    switch (reg) {
        case MMU_REG_MMUCR:
            return mmu->mmucr;
        case MMU_REG_PID:
            return mmu->pid;
        case MMU_REG_LPCR:
            return mmu->lpcr;
        case MMU_REG_HID0:
            return mmu->hid0;
        case MMU_REG_L1CSR0:
            return mmu->l1csr0;
        case MMU_REG_L1CSR1:
            return mmu->l1csr1;
        case MMU_REG_L2CSR0:
            return mmu->l2csr0;
        case MMU_REG_L2CSR1:
            return mmu->l2csr1;
        default:
            return 0;
    }
}

void xenon_mmu_write_reg(XenonMMUState *mmu, uint32_t reg, uint32_t value) {
    switch (reg) {
        case MMU_REG_MMUCR:
            mmu->mmucr = value;
            if (value & 0x00000200) {  /* TLBSX */
                tlb_flush_all(mmu);
            }
            break;
        case MMU_REG_PID:
            mmu->pid = value & 0x000000FF;
            break;
        case MMU_REG_LPCR:
            mmu->lpcr = value;
            mmu->lpcr_gtse = (value >> 2) & 1;
            break;
        case MMU_REG_HID0:
            mmu->hid0 = value;
            break;
        case MMU_REG_L1CSR0:
            mmu->l1csr0 = value;
            break;
        case MMU_REG_L1CSR1:
            mmu->l1csr1 = value;
            break;
        case MMU_REG_L2CSR0:
            mmu->l2csr0 = value;
            break;
        case MMU_REG_L2CSR1:
            mmu->l2csr1 = value;
            break;
    }
}

/* ==================== DEBUG FUNCTIONS ==================== */

void xenon_mmu_dump_state(XenonMMUState *mmu) {
    printf("Xenon MMU State:\n");
    printf("  MMUCR: 0x%08X\n", mmu->mmucr);
    printf("  PID:   0x%08X\n", mmu->pid);
    printf("  LPCR:  0x%08X\n", mmu->lpcr);
    printf("  HID0:  0x%08X\n", mmu->hid0);
    printf("  L1CSR0: 0x%08X\n", mmu->l1csr0);
    printf("  L1CSR1: 0x%08X\n", mmu->l1csr1);
    printf("  L2CSR0: 0x%08X\n", mmu->l2csr0);
    printf("  L2CSR1: 0x%08X\n", mmu->l2csr1);
    
    printf("  TLB entries:\n");
    for (int i = 0; i < XENON_TLB_SIZE; i++) {
        if (mmu->tlb[i].v) {
            printf("    [%3d] EPID: 0x%08lX -> RPN: 0x%08lX TID:%d TS:%d\n",
                   i, mmu->tlb[i].epid, mmu->tlb[i].rpn,
                   mmu->tlb[i].tid, mmu->tlb[i].ts);
        }
    }
}
