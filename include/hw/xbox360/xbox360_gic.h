#ifndef HW_XBOX360_GIC_H
#define HW_XBOX360_GIC_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_XENON_GIC "xenon.gic"
OBJECT_DECLARE_SIMPLE_TYPE(XenonGICState, XENON_GIC)

struct XenonGICState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq parent_irq[3];  /* One for each CPU */
};

/* Public functions */
XenonGICState *xenon_gic_create(MemoryRegion *parent, hwaddr base);
void xenon_gic_assert_irq(XenonGICState *s, int irq, int cpu);
void xenon_gic_deassert_irq(XenonGICState *s, int irq, int cpu);
void xenon_gic_send_sgi(XenonGICState *s, int sgi_num, int target_cpu, int target_list);
void xenon_gic_dump_state(XenonGICState *s);

/* Xenon-specific IRQ numbers */
enum XenonIRQs {
    IRQ_GPU_VBLANK = 32,
    IRQ_GPU_3D = 33,
    IRQ_GPU_2D = 34,
    IRQ_GPU_CP = 35,
    IRQ_SMC = 36,
    IRQ_AUDIO = 40,
    IRQ_DVD = 52,
    IRQ_HDD = 56,
    IRQ_TIMER0 = 64,
    IRQ_TIMER1 = 65,
    IRQ_TIMER2 = 66,
};

#endif
