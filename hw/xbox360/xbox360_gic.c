#include "qemu/osdep.h"
#include "hw/xbox360/xbox360_gic.h"
#include "hw/xbox360/xbox360.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "migration/vmstate.h"

/* ==================== XENON GIC CONFIGURATION ==================== */
#define GIC_BASE_ADDRESS          0x80000000
#define GIC_REGISTER_SIZE         0x1000

#define GIC_MAX_IRQS              256
#define GIC_NUM_CPUS              3
#define GIC_NUM_PPIS              32
#define GIC_NUM_SPIS              224

/* GIC Distributor registers */
#define GICD_CTLR                 0x0000      /* Distributor Control */
#define GICD_TYPER                0x0004      /* Controller Type */
#define GICD_IIDR                 0x0008      /* Implementer ID */
#define GICD_IGROUPR(n)           (0x0080 + 4*(n))  /* Interrupt Group */
#define GICD_ISENABLER(n)         (0x0100 + 4*(n))  /* Interrupt Set Enable */
#define GICD_ICENABLER(n)         (0x0180 + 4*(n))  /* Interrupt Clear Enable */
#define GICD_ISPENDR(n)           (0x0200 + 4*(n))  /* Set Pending */
#define GICD_ICPENDR(n)           (0x0280 + 4*(n))  /* Clear Pending */
#define GICD_ISACTIVER(n)         (0x0300 + 4*(n))  /* Set Active */
#define GICD_ICACTIVER(n)         (0x0380 + 4*(n))  /* Clear Active */
#define GICD_IPRIORITYR(n)        (0x0400 + 4*(n))  /* Priority */
#define GICD_ITARGETSR(n)         (0x0800 + 4*(n))  /* CPU Targets */
#define GICD_ICFGR(n)             (0x0C00 + 4*(n))  /* Configuration */
#define GICD_SGIR                 0x0F00      /* Software Generated Interrupt */
#define GICD_CPENDSGIR(n)         (0x0F10 + 4*(n))  /* SGI Clear Pending */
#define GICD_SPENDSGIR(n)         (0x0F20 + 4*(n))  /* SGI Set Pending */

/* GIC CPU Interface registers */
#define GICC_CTLR                 0x1000      /* CPU Interface Control */
#define GICC_PMR                  0x1004      /* Priority Mask */
#define GICC_BPR                  0x1008      /* Binary Point */
#define GICC_IAR                  0x100C      /* Interrupt Acknowledge */
#define GICC_EOIR                 0x1010      /* End Of Interrupt */
#define GICC_RPR                  0x1014      /* Running Priority */
#define GICC_HPPIR                0x1018      /* Highest Pending Interrupt */
#define GICC_ABPR                 0x101C      /* Aliased Binary Point */
#define GICC_AIAR                 0x1020      /* Aliased IAR */
#define GICC_AEOIR                0x1024      /* Aliased EOIR */
#define GICC_AHPPIR               0x1028      /* Aliased HPPIR */
#define GICC_STATUS               0x102C      /* Status */
#define GICC_IIDR                 0x10FC      /* CPU Interface ID */

/* Xenon-specific IRQ assignments */
#define XENON_IRQ_GPU_VBLANK      32
#define XENON_IRQ_GPU_3D          33
#define XENON_IRQ_GPU_2D          34
#define XENON_IRQ_GPU_CP          35
#define XENON_IRQ_SMC             36
#define XENON_IRQ_AUDIO           40
#define XENON_IRQ_USB             44
#define XENON_IRQ_NETWORK         48
#define XENON_IRQ_DVD             52
#define XENON_IRQ_HDD             56
#define XENON_IRQ_TIMER0          64
#define XENON_IRQ_TIMER1          65
#define XENON_IRQ_TIMER2          66

/* ==================== GIC STATE ==================== */

typedef struct XenonGICState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    
    /* Distributor state */
    uint32_t gicd_ctlr;
    uint32_t gicd_typer;
    uint32_t gicd_iidr;
    uint32_t gicd_isenabler[8];      /* 256 interrupts / 32 = 8 */
    uint32_t gicd_icenabler[8];
    uint32_t gicd_ispendr[8];
    uint32_t gicd_icpendr[8];
    uint32_t gicd_isactiver[8];
    uint32_t gicd_icactiver[8];
    uint32_t gicd_ipriorityr[64];    /* 256 interrupts */
    uint32_t gicd_itargetsr[64];
    uint32_t gicd_icfgr[16];         /* 256 interrupts / 16 = 16 */
    uint32_t gicd_sgir;
    uint32_t gicd_cpendsgir[4];
    uint32_t gicd_spendsgir[4];
    
    /* Per-CPU interface state */
    struct {
        uint32_t gicc_ctlr;
        uint32_t gicc_pmr;
        uint32_t gicc_bpr;
        uint32_t gicc_iar;
        uint32_t gicc_eoir;
        uint32_t gicc_rpr;
        uint32_t gicc_hppir;
        uint32_t gicc_abpr;
        uint32_t gicc_aiar;
        uint32_t gicc_aeoir;
        uint32_t gicc_ahppir;
        uint32_t gicc_status;
        uint32_t gicc_iidr;
        
        uint8_t running_irq;         /* Currently running interrupt */
        uint8_t priority_mask;       /* Current priority mask */
    } cpu[GIC_NUM_CPUS];
    
    /* IRQ state */
    uint8_t irq_level[GIC_MAX_IRQS];
    uint8_t irq_pending[GIC_MAX_IRQS];
    uint8_t irq_active[GIC_MAX_IRQS];
    uint8_t irq_priority[GIC_MAX_IRQS];
    uint8_t irq_target[GIC_MAX_IRQS];
    uint8_t irq_config[GIC_MAX_IRQS]; /* 0=level, 1=edge */
    
    /* Parent bus for IRQ lines */
    qemu_irq parent_irq[GIC_NUM_CPUS];
    
    /* Xenon-specific */
    uint32_t xenon_features;
    uint32_t debug_ctrl;
} XenonGICState;

/* ==================== REGISTER ACCESS ==================== */

static uint64_t gic_read(void *opaque, hwaddr offset, unsigned size) {
    XenonGICState *s = opaque;
    uint32_t value = 0;
    int cpu, irq;
    
    /* Distributor registers */
    if (offset < 0x1000) {
        switch (offset) {
            case GICD_CTLR:
                value = s->gicd_ctlr;
                break;
            case GICD_TYPER:
                value = (GIC_NUM_IRQS << 0) |
                        (GIC_NUM_CPUS << 5) |
                        (0 << 11) |  /* Security extensions not supported */
                        (1 << 10);   /* Supports 1-of-N model */
                break;
            case GICD_IIDR:
                value = 0x0200143B;  /* ARM GIC v2, Xenon variant */
                break;
                
            default:
                /* Handle array registers */
                if (offset >= 0x0080 && offset < 0x0100) {
                    /* IGROUPR */
                    irq = (offset - 0x0080) * 8;
                    if (irq < GIC_MAX_IRQS) {
                        int idx = irq / 32;
                        value = s->gicd_isenabler[idx];
                    }
                } else if (offset >= 0x0100 && offset < 0x0180) {
                    /* ISENABLER */
                    irq = (offset - 0x0100) * 8;
                    if (irq < GIC_MAX_IRQS) {
                        int idx = irq / 32;
                        value = s->gicd_isenabler[idx];
                    }
                } else if (offset >= 0x0400 && offset < 0x0800) {
                    /* IPRIORITYR */
                    irq = (offset - 0x0400);
                    if (irq < GIC_MAX_IRQS) {
                        value = s->gicd_ipriorityr[irq / 4] >> ((irq % 4) * 8);
                    }
                } else if (offset >= 0x0800 && offset < 0x0C00) {
                    /* ITARGETSR */
                    irq = (offset - 0x0800);
                    if (irq < GIC_MAX_IRQS) {
                        value = s->gicd_itargetsr[irq / 4] >> ((irq % 4) * 8);
                    }
                }
                break;
        }
    }
    /* CPU Interface registers */
    else if (offset >= 0x1000 && offset < 0x2000) {
        cpu = (offset - 0x1000) / 0x100;
        if (cpu >= GIC_NUM_CPUS) {
            return 0;
        }
        
        offset = offset % 0x100;
        
        switch (offset) {
            case GICC_CTLR:
                value = s->cpu[cpu].gicc_ctlr;
                break;
            case GICC_PMR:
                value = s->cpu[cpu].gicc_pmr;
                break;
            case GICC_BPR:
                value = s->cpu[cpu].gicc_bpr;
                break;
            case GICC_IAR:
                value = gic_get_current_irq(s, cpu);
                break;
            case GICC_EOIR:
                value = 0;  /* Write-only */
                break;
            case GICC_RPR:
                value = s->cpu[cpu].gicc_rpr;
                break;
            case GICC_HPPIR:
                value = gic_get_highest_pending_irq(s, cpu);
                break;
            default:
                break;
        }
    }
    
    return value;
}

static void gic_write(void *opaque, hwaddr offset, 
                     uint64_t value, unsigned size) {
    XenonGICState *s = opaque;
    int cpu, irq;
    
    /* Distributor registers */
    if (offset < 0x1000) {
        switch (offset) {
            case GICD_CTLR:
                s->gicd_ctlr = value & 1;  /* Only enable bit supported */
                if (value & 1) {
                    gic_update(s);
                }
                break;
            case GICD_SGIR:
                /* Software Generated Interrupt */
                cpu = (value >> 16) & 0x3;
                irq = value & 0xF;
                if (cpu == 0 || cpu == 3) {
                    /* All CPUs or self */
                    for (int i = 0; i < GIC_NUM_CPUS; i++) {
                        gic_set_irq(s, irq, 1, i);
                    }
                } else {
                    gic_set_irq(s, irq, 1, cpu - 1);
                }
                break;
                
            default:
                /* Handle array registers */
                if (offset >= 0x0100 && offset < 0x0180) {
                    /* ISENABLER */
                    irq = (offset - 0x0100) * 8;
                    if (irq < GIC_MAX_IRQS) {
                        int idx = irq / 32;
                        s->gicd_isenabler[idx] |= value;
                        for (int i = 0; i < 32; i++) {
                            if (value & (1 << i)) {
                                int irq_num = irq + i;
                                if (irq_num < GIC_MAX_IRQS) {
                                    s->irq_level[irq_num] = 0;
                                }
                            }
                        }
                        gic_update(s);
                    }
                } else if (offset >= 0x0180 && offset < 0x0200) {
                    /* ICENABLER */
                    irq = (offset - 0x0180) * 8;
                    if (irq < GIC_MAX_IRQS) {
                        int idx = irq / 32;
                        s->gicd_icenabler[idx] |= value;
                        s->gicd_isenabler[idx] &= ~value;
                        gic_update(s);
                    }
                } else if (offset >= 0x0400 && offset < 0x0800) {
                    /* IPRIORITYR */
                    irq = (offset - 0x0400);
                    if (irq < GIC_MAX_IRQS) {
                        int idx = irq / 4;
                        int shift = (irq % 4) * 8;
                        s->gicd_ipriorityr[idx] &= ~(0xFF << shift);
                        s->gicd_ipriorityr[idx] |= (value & 0xFF) << shift;
                        s->irq_priority[irq] = value & 0xFF;
                    }
                } else if (offset >= 0x0800 && offset < 0x0C00) {
                    /* ITARGETSR */
                    irq = (offset - 0x0800);
                    if (irq < GIC_MAX_IRQS) {
                        int idx = irq / 4;
                        int shift = (irq % 4) * 8;
                        s->gicd_itargetsr[idx] &= ~(0xFF << shift);
                        s->gicd_itargetsr[idx] |= (value & 0xFF) << shift;
                        s->irq_target[irq] = value & 0xFF;
                    }
                }
                break;
        }
    }
    /* CPU Interface registers */
    else if (offset >= 0x1000 && offset < 0x2000) {
        cpu = (offset - 0x1000) / 0x100;
        if (cpu >= GIC_NUM_CPUS) {
            return;
        }
        
        offset = offset % 0x100;
        
        switch (offset) {
            case GICC_CTLR:
                s->cpu[cpu].gicc_ctlr = value & 1;
                gic_update(s);
                break;
            case GICC_PMR:
                s->cpu[cpu].gicc_pmr = value & 0xFF;
                s->cpu[cpu].priority_mask = value & 0xFF;
                gic_update(s);
                break;
            case GICC_EOIR:
                gic_complete_irq(s, cpu, value & 0x3FF);
                break;
            case GICC_BPR:
                s->cpu[cpu].gicc_bpr = value & 0x7;
                break;
        }
    }
}

static const MemoryRegionOps gic_ops = {
    .read = gic_read,
    .write = gic_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

/* ==================== IRQ MANAGEMENT ==================== */

static uint32_t gic_get_current_irq(XenonGICState *s, int cpu) {
    uint8_t running = s->cpu[cpu].running_irq;
    if (running != 255) {
        return running;
    }
    
    /* Find highest priority pending interrupt */
    uint32_t highest_irq = 1023;  /* Spurious interrupt */
    uint8_t highest_prio = 0xFF;
    
    for (int irq = 0; irq < GIC_MAX_IRQS; irq++) {
        if (s->irq_pending[irq] && s->irq_priority[irq] < highest_prio) {
            /* Check if targeting this CPU */
            if (s->irq_target[irq] & (1 << cpu)) {
                highest_prio = s->irq_priority[irq];
                highest_irq = irq;
            }
        }
    }
    
    if (highest_irq !=
