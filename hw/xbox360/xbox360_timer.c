#include "qemu/osdep.h"
#include "hw/xbox360/xbox360_timer.h"
#include "hw/xbox360/xbox360.h"
#include "hw/irq.h"
#include "qemu/timer.h"
#include "migration/vmstate.h"

/* ==================== XENON TIMER CONFIGURATION ==================== */
#define TIMER_BASE_ADDRESS        0x80006000
#define TIMER_REGISTER_SIZE       0x1000

#define TIMER_COUNT               8
#define TIMER_MAX_VALUE           0xFFFFFFFF

/* Timer Registers */
#define TIMER_REG_CONTROL         0x0000
#define TIMER_REG_STATUS          0x0004
#define TIMER_REG_CONFIG          0x0008
#define TIMER_REG_GLOBAL_COUNTER  0x000C
#define TIMER_REG_IRQ_STATUS      0x0010
#define TIMER_REG_IRQ_MASK        0x0014
#define TIMER_REG_IRQ_CLEAR       0x0018

/* Per-Timer Registers */
#define TIMER_BASE(n)             (0x0020 + (n) * 0x20)
#define TIMER_COUNT_REG(n)        (TIMER_BASE(n) + 0x00)
#define TIMER_LOAD_REG(n)         (TIMER_BASE(n) + 0x04)
#define TIMER_MATCH_REG(n)        (TIMER_BASE(n) + 0x08)
#define TIMER_CONTROL_REG(n)      (TIMER_BASE(n) + 0x0C)
#define TIMER_PRESCALE_REG(n)     (TIMER_BASE(n) + 0x10)

/* Timer Control Bits */
#define TIMER_CTRL_ENABLE         (1 << 0)
#define TIMER_CTRL_ONESHOT        (1 << 1)
#define TIMER_CTRL_INTERRUPT      (1 << 2)
#define TIMER_CTRL_PRESCALE       (1 << 3)
#define TIMER_CTRL_MATCH_EN       (1 << 4)
#define TIMER_CTRL_LOAD_EN        (1 << 5)
#define TIMER_CTRL_RESET          (1 << 6)
#define TIMER_CTRL_WRAP           (1 << 7)

/* Timer Status Bits */
#define TIMER_STATUS_RUNNING      (1 << 0)
#define TIMER_STATUS_MATCH        (1 << 1)
#define TIMER_STATUS_OVERFLOW     (1 << 2)
#define TIMER_STATUS_UNDERFLOW    (1 << 3)

/* Xenon Timer Assignments */
#define TIMER_SYSTEM              0  /* System tick */
#define TIMER_GPU                 1  /* GPU synchronization */
#define TIMER_AUDIO               2  /* Audio timing */
#define TIMER_NETWORK             3  /* Network timing */
#define TIMER_USB                 4  /* USB timing */
#define TIMER_DVD                 5  /* DVD timing */
#define TIMER_HDD                 6  /* HDD timing */
#define TIMER_PROFILE             7  /* Profiling */

/* ==================== TIMER STATE ==================== */

typedef struct XenonTimer {
    uint32_t count;
    uint32_t load;
    uint32_t match;
    uint32_t control;
    uint32_t prescale;
    uint32_t status;
    
    uint32_t prescale_counter;
    bool enabled;
    bool oneshot;
    bool match_enabled;
    bool load_enabled;
    bool wrap;
    
    QEMUTimer *qtimer;
    uint64_t last_update;
    uint64_t period_ns;
} XenonTimer;

typedef struct XenonTimerState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    
    /* Controller registers */
    uint32_t control;
    uint32_t status;
    uint32_t config;
    uint64_t global_counter;
    uint32_t irq_status;
    uint32_t irq_mask;
    
    /* Timers */
    XenonTimer timers[TIMER_COUNT];
    
    /* Interrupts */
    qemu_irq irq;
    qemu_irq timer_irqs[TIMER_COUNT];
    
    /* Reference clock */
    uint64_t reference_freq;  /* Hz */
    uint64_t last_global_update;
    
    /* Statistics */
    uint64_t interrupts_generated;
    uint64_t total_ticks;
} XenonTimerState;

/* ==================== REGISTER ACCESS ==================== */

static uint64_t timer_read(void *opaque, hwaddr offset, unsigned size) {
    XenonTimerState *s = opaque;
    uint64_t value = 0;
    int timer_idx;
    
    switch (offset) {
        case TIMER_REG_CONTROL:
            value = s->control;
            break;
        case TIMER_REG_STATUS:
            value = s->status;
            break;
        case TIMER_REG_CONFIG:
            value = s->config;
            break;
        case TIMER_REG_GLOBAL_COUNTER:
            /* Update global counter */
            {
                uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
                uint64_t elapsed = now - s->last_global_update;
                s->global_counter += elapsed * s->reference_freq / NANOSECONDS_PER_SECOND;
                s->last_global_update = now;
                value = s->global_counter & 0xFFFFFFFF;
            }
            break;
        case TIMER_REG_IRQ_STATUS:
            value = s->irq_status;
            break;
        case TIMER_REG_IRQ_MASK:
            value = s->irq_mask;
            break;
        default:
            /* Timer-specific registers */
            for (int i = 0; i < TIMER_COUNT; i++) {
                if (offset >= TIMER_BASE(i) && offset < TIMER_BASE(i) + 0x20) {
                    timer_idx = i;
                    offset -= TIMER_BASE(i);
                    
                    XenonTimer *t = &s->timers[timer_idx];
                    switch (offset) {
                        case 0x00:  /* Count */
                            value = t->count;
                            break;
                        case 0x04:  /* Load */
                            value = t->load;
                            break;
                        case 0x08:  /* Match */
                            value = t->match;
                            break;
                        case 0x0C:  /* Control */
                            value = t->control;
                            break;
                        case 0x10:  /* Prescale */
                            value = t->prescale;
                            break;
                    }
                    break;
                }
            }
            break;
    }
    
    return value;
}

static void timer_write(void *opaque, hwaddr offset, 
                       uint64_t value, unsigned size) {
    XenonTimerState *s = opaque;
    int timer_idx;
    
    switch (offset) {
        case TIMER_REG_CONTROL:
            s->control = value;
            if (value & 1) {
                /* Enable all timers */
                for (int i = 0; i < TIMER_COUNT; i++) {
                    if (s->timers[i].control & TIMER_CTRL_ENABLE) {
                        timer_start(&s->timers[i]);
                    }
                }
            } else {
                /* Disable all timers */
                for (int i = 0; i < TIMER_COUNT; i++) {
                    timer_stop(&s->timers[i]);
                }
            }
            break;
        case TIMER_REG_CONFIG:
            s->config = value;
            break;
        case TIMER_REG_GLOBAL_COUNTER:
            s->global_counter = value;
            s->last_global_update = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            break;
        case TIMER_REG_IRQ_MASK:
            s->irq_mask = value;
            break;
        case TIMER_REG_IRQ_CLEAR:
            s->irq_status &= ~value;
            if (!(s->irq_status & s->irq_mask)) {
                qemu_set_irq(s->irq, 0);
            }
            break;
        default:
            /* Timer-specific registers */
            for (int i = 0; i < TIMER_COUNT; i++) {
                if (offset >= TIMER_BASE(i) && offset < TIMER_BASE(i) + 0x20) {
                    timer_idx = i;
                    offset -= TIMER_BASE(i);
                    
                    XenonTimer *t = &s->timers[timer_idx];
                    switch (offset) {
                        case 0x00:  /* Count */
                            t->count = value;
                            break;
                        case 0x04:  /* Load */
                            t->load = value;
                            if (t->load_enabled) {
                                t->count = t->load;
                            }
                            break;
                        case 0x08:  /* Match */
                            t->match = value;
                            break;
                        case 0x0C:  /* Control */
                            t->control = value;
                            t->enabled = (value & TIMER_CTRL_ENABLE) != 0;
                            t->oneshot = (value & TIMER_CTRL_ONESHOT) != 0;
                            t->match_enabled = (value & TIMER_CTRL_MATCH_EN) != 0;
                            t->load_enabled = (value & TIMER_CTRL_LOAD_EN) != 0;
                            t->wrap = (value & TIMER_CTRL_WRAP) != 0;
                            
                            if (value & TIMER_CTRL_RESET) {
                                timer_reset(t);
                            }
                            if (t->enabled && (s->control & 1)) {
                                timer_start(t);
                            } else {
                                timer_stop(t);
                            }
                            break;
                        case 0x10:  /* Prescale */
                            t->prescale = value & 0xFF;
                            break;
                    }
                    break;
                }
            }
            break;
    }
}

static const MemoryRegionOps timer_ops = {
    .read = timer_read,
    .write = timer_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 8 },
    .impl = { .min_access_size = 4, .max_access_size = 8 },
};

/* ==================== TIMER FUNCTIONS ==================== */

static void timer_callback(void *opaque) {
    XenonTimer *t = opaque;
    XenonTimerState *s = container_of(t, XenonTimerState, timers[t - s->timers]);
    int timer_idx = t - s->timers;
    
    if (!t->enabled) {
        return;
    }
    
    /* Update count */
    if (t->prescale > 0) {
        t->prescale_counter++;
        if (t->prescale_counter < t->prescale) {
            /* Not time to update count yet */
            timer_schedule_next(t);
            return;
        }
        t->prescale_counter = 0;
    }
    
    if (t->wrap) {
        /* Wrap-around mode */
        if (t->count == 0) {
            t->count = t->load;
            t->status |= TIMER_STATUS_OVERFLOW;
            
            if (t->control & TIMER_CTRL_INTERRUPT) {
                s->irq_status |= (1 << timer_idx);
                s->interrupts_generated++;
                qemu_set_irq(s->timer_irqs[timer_idx], 1);
                qemu_set_irq(s->timer_irqs[timer_idx], 0);
                
                if (s->irq_status & s->irq_mask) {
                    qemu_set_irq(s->irq, 1);
                }
            }
        } else {
            t->count--;
        }
    } else {
        /* Count-up mode */
        if (t->count == TIMER_MAX_VALUE) {
            t->status |= TIMER_STATUS_OVERFLOW;
            
            if (t->control & TIMER_CTRL_INTERRUPT) {
                s->irq_status |= (1 << timer_idx);
                s->interrupts_generated++;
                qemu_set_irq(s->timer_irqs[timer_idx], 1);
                qemu_set_irq(s->timer_irqs[timer_idx], 0);
                
                if (s->irq_status & s->irq_mask) {
                    qemu_set_irq(s->irq, 1);
                }
            }
            
            if (t->load_enabled) {
                t->count = t->load;
            } else {
                t->count = 0;
            }
        } else {
            t->count++;
        }
    }
    
    /* Check match */
    if (t->match_enabled && t->count == t->match) {
        t->status |= TIMER_STATUS_MATCH;
        
        if (t->control & TIMER_CTRL_INTERRUPT) {
            s->irq_status |= (1 << timer_idx);
            s->interrupts_generated++;
            qemu_set_irq(s->timer_irqs[timer_idx], 1);
            qemu_set_irq(s->timer_irqs[timer_idx], 0);
            
            if (s->irq_status & s->irq_mask) {
                qemu_set_irq(s->irq, 1);
            }
        }
    }
    
    s->total_ticks++;
    
    /* Schedule next tick if not oneshot */
    if (!t->oneshot) {
        timer_schedule_next(t);
    } else {
        t->enabled = false;
        t->status &= ~TIMER_STATUS_RUNNING;
    }
}

static void timer_start(XenonTimer *t) {
    if (t->enabled && !timer_pending(t->qtimer)) {
        t->status |= TIMER_STATUS_RUNNING;
        t->last_update = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        
        if (t->period_ns == 0) {
            /* Calculate period from frequency */
            uint64_t freq = 1000000000;  /* 1GHz reference clock */
            if (t->prescale > 0) {
                freq /= (t->prescale + 1);
            }
            t->period_ns = NANOSECONDS_PER_SECOND / freq;
        }
        
        timer_mod_ns(t->qtimer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + t->period_ns);
    }
}

static void timer_stop(XenonTimer *t) {
    t->status &= ~TIMER_STATUS_RUNNING;
    timer_del(t->qtimer);
}

static void timer_reset(XenonTimer *t) {
    t->count = t->load;
    t->prescale_counter = 0;
    t->status = 0;
    timer_del(t->qtimer);
}

static void timer_schedule_next(XenonTimer *t) {
    if (!t->enabled) {
        return;
    }
    
    uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t next = now + t->period_ns;
    
    /* Adjust for drift */
    uint64_t expected = t->last_update + t->period_ns;
    if (now > expected + t->period_ns / 10) {
        /* We're behind, catch up */
        next = now;
    }
    
    timer_mod_ns(t->qtimer, next);
    t->last_update = now;
}

/* ==================== DEVICE INITIALIZATION ==================== */

static void xenon_timer_realize(DeviceState *dev, Error **errp) {
    XenonTimerState *s = XENON_TIMER(dev);
    
    /* Initialize controller */
    s->control = 0x00000001;  /* Enabled */
    s->status = 0;
    s->config = 0;
    s->global_counter = 0;
    s->irq_status = 0;
    s->irq_mask = 0xFFFFFFFF;
    
    s->reference_freq = 1000000000;  /* 1GHz */
    s->last_global_update = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    
    /* Initialize timers */
    for (int i = 0; i < TIMER_COUNT; i++) {
        XenonTimer *t = &s->timers[i];
        
        t->count = 0;
        t->load = 0;
        t->match = 0;
        t->control = 0;
        t->prescale = 0;
        t->status = 0;
        
        t->prescale_counter = 0;
        t->enabled = false;
        t->oneshot = false;
        t->match_enabled = false;
        t->load_enabled = false;
        t->wrap = false;
        
        t->last_update = 0;
        t->period_ns = 0;
        
        /* Create QEMU timer */
        t->qtimer = timer_new_ns(QEMU_CLOCK_VIRTUAL, timer_callback, t);
        
        /* Set up timer-specific configurations */
        switch (i) {
            case TIMER_SYSTEM:
                t->load = 0xFFFFFFFF;
                t->control = TIMER_CTRL_ENABLE | TIMER_CTRL_WRAP;
                t->wrap = true;
                break;
            case TIMER_GPU:
                t->load = 16666666;  /* 60Hz */
                t->control = TIMER_CTRL_ENABLE | TIMER_CTRL_INTERRUPT;
                break;
            case TIMER_AUDIO:
                t->load = 20833;  /* 48kHz */
                t->control = TIMER_CTRL_ENABLE | TIMER_CTRL_INTERRUPT;
                break;
        }
    }
    
    /* Start enabled timers */
    for (int i = 0; i < TIMER_COUNT; i++) {
        if (s->timers[i].enabled) {
            timer_start(&s->timers[i]);
        }
    }
    
    /* Initialize statistics */
    s->interrupts_generated = 0;
    s->total_ticks = 0;
    
    /* Initialize memory region */
    memory_region_init_io(&s->iomem, OBJECT(s), &timer_ops, s,
                         "xenon.timer", TIMER_REGISTER_SIZE);
    
    printf("[TIMER] Xenon Timer Controller initialized\n");
    printf("[TIMER] %d timers, %ld Hz reference\n", TIMER_COUNT, s->reference_freq);
}

static void xenon_timer_reset(DeviceState *dev) {
    XenonTimerState *s = XENON_TIMER(dev);
    
    s->control = 0x00000001;
    s->irq_status = 0;
    s->global_counter = 0;
    s->last_global_update = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    
    /* Reset all timers */
    for (int i = 0; i < TIMER_COUNT; i++) {
        timer_reset(&s->timers[i]);
    }
    
    /* Restart enabled timers */
    for (int i = 0; i < TIMER_COUNT; i++) {
        if (s->timers[i].enabled) {
            timer_start(&s->timers[i]);
        }
    }
    
    qemu_set_irq(s->irq, 0);
    for (int i = 0; i < TIMER_COUNT; i++) {
        qemu_set_irq(s->timer_irqs[i], 0);
    }
}

/* ==================== QEMU DEVICE ==================== */

static void xenon_timer_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    
    dc->realize = xenon_timer_realize;
    dc->reset = xenon_timer_reset;
    dc->desc = "Xenon Timer Controller";
}

static const TypeInfo xenon_timer_type_info = {
    .name = TYPE_XENON_TIMER,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XenonTimerState),
    .class_init = xenon_timer_class_init,
};

static void xenon_timer_register_types(void) {
    type_register_static(&xenon_timer_type_info);
}

type_init(xenon_timer_register_types);

/* ==================== PUBLIC FUNCTIONS ==================== */

XenonTimerState *xenon_timer_create(MemoryRegion *parent, hwaddr base) {
    DeviceState *dev;
    XenonTimerState *s;
    
    dev = qdev_new(TYPE_XENON_TIMER);
    s = XENON_TIMER(dev);
    
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    
    /* Connect interrupts */
    s->irq = qemu_allocate_irq(xenon_timer_irq_handler, s, 0);
    for (int i = 0; i < TIMER_COUNT; i++) {
        s->timer_irqs[i] = qemu_allocate_irq(xenon_timer_individual_irq_handler, 
                                            s, i);
    }
    
    return s;
}

void xenon_timer_start(XenonTimerState *s, int timer) {
    if (timer >= 0 && timer < TIMER_COUNT) {
        XenonTimer *t = &s->timers[timer];
        t->enabled = true;
        t->control |= TIMER_CTRL_ENABLE;
        timer_start(t);
    }
}

void xenon_timer_stop(XenonTimerState *s, int timer) {
    if (timer >= 0 && timer < TIMER_COUNT) {
        XenonTimer *t = &s->timers[timer];
        t->enabled = false;
        t->control &= ~TIMER_CTRL_ENABLE;
        timer_stop(t);
    }
}

uint64_t xenon_timer_get_global_counter(XenonTimerState *s) {
    uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t elapsed = now - s->last_global_update;
    s->global_counter += elapsed * s->reference_freq / NANOSECONDS_PER_SECOND;
    s->last_global_update = now;
    return s->global_counter;
}

void xenon_timer_set_frequency(XenonTimerState *s, int timer, uint64_t freq_hz) {
    if (timer >= 0 && timer < TIMER_COUNT) {
        XenonTimer *t = &s->timers[timer];
        t->period_ns = NANOSECONDS_PER_SECOND / freq_hz;
        if (t->prescale > 0) {
            t->period_ns *= (t->prescale + 1);
        }
    }
}

void xenon_timer_dump_state(XenonTimerState *s) {
    printf("Xenon Timer Controller State:\n");
    printf("  Control: 0x%08X, Status: 0x%08X, Config: 0x%08X\n",
           s->control, s->status, s->config);
    printf("  Global Counter: %" PRIu64 "\n", s->global_counter);
    printf("  IRQ Status: 0x%08X, Mask: 0x%08X\n", s->irq_status, s->irq_mask);
    printf("  Statistics: %" PRIu64 " interrupts, %" PRIu64 " total ticks\n",
           s->interrupts_generated, s->total_ticks);
    
    printf("  Timers:\n");
    for (int i = 0; i < TIMER_COUNT; i++) {
        XenonTimer *t = &s->timers[i];
        if (t->enabled || t->load != 0) {
            printf("    Timer %d: %s, Count=%u, Load=%u, Match=%u\n",
                   i, t->enabled ? "Running" : "Stopped",
                   t->count, t->load, t->match);
            printf("      Control: 0x%08X, Status: 0x%08X, Prescale=%u\n",
                   t->control, t->status, t->prescale);
        }
    }
}
