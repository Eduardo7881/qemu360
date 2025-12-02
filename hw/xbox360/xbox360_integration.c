#include "hw/xbox360/xbox360.h"
#include "hw/xbox360/xbox360_mmu.h"
#include "hw/xbox360/xbox360_gic.h"
#include "qemu/osdep.h"
#include "cpu.h"

void xenon_mmu_init_all(XenonState *s) {
    for (int i = 0; i < 3; i++) {
        if (s->cpu[i]) {
            xenon_mmu_init(&s->mmu_state[i], s->cpu[i]);
            printf("[MMU] CPU%d MMU initialized\n", i);
        }
    }
}

void xenon_connect_interrupts(XenonState *s) {
    /* Connect GIC to CPUs */
    for (int i = 0; i < 3; i++) {
        if (s->cpu[i] && s->gic) {
            /* Connect CPU IRQ line to GIC */
            qemu_irq *cpu_irq = qemu_allocate_irqs(
                xenon_gic_set_irq, s->gic, 1);
            
            /* TODO - connection depends on QEMU's PowerPC model */
            printf("[IRQ] CPU%d connected to GIC\n", i);
        }
    }
    
    /* Connect GPU interrupts to GIC */
    if (s->gpu && s->gic) {
        /* Set up GPU interrupt callback */
        xbox360_gpu_set_interrupt_callback(s->gpu, 
            xenon_gpu_interrupt_handler, s);
    }
}

/* GPU interrupt handler with GIC routing */
static void xenon_gpu_interrupt_handler(void *opaque, uint32_t interrupts) {
    XenonState *s = opaque;
    
    if (interrupts & GPU_INTR_VBLANK) {
        xenon_gic_assert_irq(s->gic, IRQ_GPU_VBLANK, 0);
    }
    if (interrupts & GPU_INTR_3D) {
        xenon_gic_assert_irq(s->gic, IRQ_GPU_3D, 0);
    }
    if (interrupts & GPU_INTR_2D) {
        xenon_gic_assert_irq(s->gic, IRQ_GPU_2D, 0);
    }
    if (interrupts & GPU_INTR_CP) {
        xenon_gic_assert_irq(s->gic, IRQ_GPU_CP, 0);
    }
}
