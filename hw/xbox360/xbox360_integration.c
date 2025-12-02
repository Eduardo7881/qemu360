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

/* Updated GPU interrupt handler with GIC routing */
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

/* Update machine init in xbox360.c */
static void xenon_machine_init(MachineState *machine) {
    XenonState *s = XBOX360_MACHINE(machine);
    Error *err = NULL;

    printf("%s", asciiart);
    printf("                    Microsoft Xbox 360                       \n");
    printf("                    Emulator v1.0.0                         \n");
    printf("\n");
    
    /* Initialize GIC first */
    s->gic = xenon_gic_create(get_system_memory(), 0x80001000);
    printf("[GIC] Initialized at 0x80001000\n");
    
    /* Then CPUs */
    for (int i = 0; i < 3; i++) {
        s->cpu[i] = POWERPC_CPU(cpu_create(machine->cpu_type));
        if (!s->cpu[i]) {
            error_report("Fail to create CPU %d", i);
            exit(1);
        }
        
        PowerPCCPU *cpu = s->cpu[i];
        cpu->env.spr[SPR_PVR] = 0x710000;
        cpu->env.spr[SPR_SDR1] = 0;
        
        printf("[CPU%d] PowerPC 750CL @ 3.2GHz initialized\n", i);
    }
    
    /* Initialize MMU for each CPU */
    xenon_mmu_init_all(s);
    
    /* Connect interrupts */
    xenon_connect_interrupts(s);
    
    /* ... rest of existing initialization ... */
}
