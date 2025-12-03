#ifndef HW_XBOX360_TIMER_H
#define HW_XBOX360_TIMER_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_XENON_TIMER "xenon.timer"
OBJECT_DECLARE_SIMPLE_TYPE(XenonTimerState, XENON_TIMER)

struct XenonTimerState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq timer_irqs[8];
};

/* Public functions */
XenonTimerState *xenon_timer_create(MemoryRegion *parent, hwaddr base);
void xenon_timer_start(XenonTimerState *s, int timer);
void xenon_timer_stop(XenonTimerState *s, int timer);
uint64_t xenon_timer_get_global_counter(XenonTimerState *s);
void xenon_timer_set_frequency(XenonTimerState *s, int timer, uint64_t freq_hz);
void xenon_timer_dump_state(XenonTimerState *s);
void xenon_timer_connect_irq(XenonTimerState *s, XenonGICState *gic);

/* Timer Assignments */
enum XenonTimers {
    TIMER_SYSTEM = 0,
    TIMER_GPU = 1,
    TIMER_AUDIO = 2,
    TIMER_NETWORK = 3,
    TIMER_USB = 4,
    TIMER_DVD = 5,
    TIMER_HDD = 6,
    TIMER_PROFILE = 7,
};

#endif
