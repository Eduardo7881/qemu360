#include "hw/xbox360/xbox360.h"
#include "hw/xbox360/xbox360_mmu.h"
#include "hw/xbox360/xbox360_gic.h"
#include "qemu/osdep.h"
#include "cpu.h"

void xenon_pcie_integration_init(XenonState *s) {
    s->pcie = xenon_pcie_create(get_system_memory(), 0x80002000);
    printf("[PCIE] Initialized at 0x80002000\n");
    
    s->dma = xenon_dma_create(get_system_memory(), 0x80004000);
    printf("[DMA]  Initialized at 0x80004000\n");
    
    if (s->gpu && s->pcie) {
        xenon_pcie_connect_gpu(s->pcie, s->gpu);
        
        xenon_dma_setup_linked_list(s->dma, DMA_CHANNEL_GPU,
                                   0x80000000, 256);  /* Example LL at boot ROM */
    }
    
    /* Connect DMA interrupts to GIC */
    if (s->dma && s->gic) {
        qemu_irq dma_irq = xenon_dma_get_irq(s->dma);
        /* Connect to GIC - TODO Depends on IRQ Routing */
    }
    
    if (s->gpu && s->dma) {
        /* Channel 0 for GPU command DMA */
        xenon_dma_start_transfer(s->dma, DMA_CHANNEL_GPU,
                                0x80000000, 0xC0000000, 0x10000);
    }
}

void xenon_mmu_init_all(XenonState *s) {
    for (int i = 0; i < 3; i++) {
        if (s->cpu[i]) {
            xenon_mmu_init(&s->mmu_state[i], s->cpu[i]);
            printf("[MMU] CPU%d MMU initialized\n", i);
        }
    }
}

void xenon_connect_interrupts(XenonState *s) {
    for (int i = 0; i < 3; i++) {
        if (s->cpu[i] && s->gic) {
            qemu_irq *cpu_irq = qemu_allocate_irqs(
                xenon_gic_set_irq, s->gic, 1);
            
            /* TODO - connection depends on QEMU's PowerPC model */
            printf("[IRQ] CPU%d connected to GIC\n", i);
        }
    }
    
    if (s->gpu && s->gic) {
        xbox360_gpu_set_interrupt_callback(s->gpu, 
            xenon_gpu_interrupt_handler, s);
    }
}

/* GPU interrupt handler with GIC routing */
static void xenon_gpu_interrupt_handler(void *opaque, uint32_t interrupts) {
    XenonState *s = opaque;

    if (interrupts & GPU_INTR_VBLANK) {
        xenon_gic_assert_irq(s->gic, IRQ_GPU_VBLANK, 0);
        
        if (s->dma) {
            xenon_dma_start_transfer(s->dma, DMA_CHANNEL_GPU,
                                     s->gpu->display.base_addr,
                                     0x20000000,  /* VRAM display area */
                                     s->gpu->display.width * s->gpu->display.height * 4);
        }
    }
    
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
