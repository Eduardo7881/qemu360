#include "qemu/osdep.h"
#include "hw/xbox360/xbox360_dma.h"
#include "hw/xbox360/xbox360.h"
#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "migration/vmstate.h"

/* ==================== XENON DMA CONFIGURATION ==================== */
#define DMA_BASE_ADDRESS          0x80004000
#define DMA_REGISTER_SIZE         0x1000

#define DMA_MAX_CHANNELS          8
#define DMA_MAX_TRANSFER_SIZE     0x1000000  /* 16MB */
#define DMA_ALIGNMENT             32

/* DMA Channel Registers */
#define DMA_CH_BASE               0x0000
#define DMA_CH_SIZE               0x0040
#define DMA_CH_CTRL(n)           (DMA_CH_BASE + (n) * DMA_CH_SIZE + 0x00)
#define DMA_CH_STATUS(n)         (DMA_CH_BASE + (n) * DMA_CH_SIZE + 0x04)
#define DMA_CH_SRC_ADDR(n)       (DMA_CH_BASE + (n) * DMA_CH_SIZE + 0x08)
#define DMA_CH_DST_ADDR(n)       (DMA_CH_BASE + (n) * DMA_CH_SIZE + 0x0C)
#define DMA_CH_TRANSFER_SIZE(n)  (DMA_CH_BASE + (n) * DMA_CH_SIZE + 0x10)
#define DMA_CH_LL_ADDR(n)        (DMA_CH_BASE + (n) * DMA_CH_SIZE + 0x14)
#define DMA_CH_LL_COUNT(n)       (DMA_CH_BASE + (n) * DMA_CH_SIZE + 0x18)
#define DMA_CH_CONFIG(n)         (DMA_CH_BASE + (n) * DMA_CH_SIZE + 0x1C)

/* DMA Controller Registers */
#define DMA_CTRL_STATUS           0x0200
#define DMA_CTRL_CONFIG           0x0204
#define DMA_CTRL_IRQ_STATUS       0x0208
#define DMA_CTRL_IRQ_MASK         0x020C
#define DMA_CTRL_IRQ_CLEAR        0x0210
#define DMA_CTRL_VERSION          0x0214
#define DMA_CTRL_DEBUG            0x0218

/* DMA Control Bits */
#define DMA_CTRL_ENABLE           (1 << 0)
#define DMA_CTRL_START            (1 << 1)
#define DMA_CTRL_PAUSE            (1 << 2)
#define DMA_CTRL_RESET            (1 << 3)
#define DMA_CTRL_LL_ENABLE        (1 << 4)  /* Linked List mode */
#define DMA_CTRL_CIRCULAR         (1 << 5)  /* Circular buffer */
#define DMA_CTRL_INTERRUPT        (1 << 6)  /* Generate interrupt */
#define DMA_CTRL_DIR_MEM_TO_MEM   (0 << 8)
#define DMA_CTRL_DIR_MEM_TO_IO    (1 << 8)
#define DMA_CTRL_DIR_IO_TO_MEM    (2 << 8)
#define DMA_CTRL_DIR_IO_TO_IO     (3 << 8)

/* DMA Status Bits */
#define DMA_STATUS_BUSY           (1 << 0)
#define DMA_STATUS_COMPLETE       (1 << 1)
#define DMA_STATUS_ERROR          (1 << 2)
#define DMA_STATUS_LL_ACTIVE      (1 << 3)  /* Linked list active */
#define DMA_STATUS_FIFO_FULL      (1 << 4)
#define DMA_STATUS_FIFO_EMPTY     (1 << 5)

/* Xenon DMA Channels */
#define DMA_CHANNEL_GPU           0  /* GPU command buffer */
#define DMA_CHANNEL_AUDIO         1  /* Audio buffers */
#define DMA_CHANNEL_USB           2  /* USB transfers */
#define DMA_CHANNEL_NETWORK       3  /* Network packets */
#define DMA_CHANNEL_HDD           4  /* Disk I/O */
#define DMA_CHANNEL_DVD           5  /* DVD reads */
#define DMA_CHANNEL_MEMCOPY       6  /* Memory copy */
#define DMA_CHANNEL_SCRATCH       7  /* General purpose */

/* ==================== DMA LINKED LIST STRUCTURES ==================== */

typedef struct DMALLEntry {
    uint32_t src_addr;
    uint32_t dst_addr;
    uint32_t transfer_size;
    uint32_t control;
    uint32_t next_addr;  /* Next entry in linked list */
    uint32_t reserved[3];
} DMALLEntry;

/* ==================== DMA CHANNEL STATE ==================== */

typedef struct DMAChannelState {
    uint32_t control;
    uint32_t status;
    uint32_t src_addr;
    uint32_t dst_addr;
    uint32_t transfer_size;
    uint32_t ll_addr;      /* Linked list base address */
    uint32_t ll_count;     /* Linked list entry count */
    uint32_t config;
    
    uint32_t current_src;
    uint32_t current_dst;
    uint32_t remaining;
    uint32_t ll_current;
    uint32_t ll_remaining;
    
    bool active;
    bool ll_mode;
    bool circular;
    
    QEMUTimer *timer;
    uint64_t bytes_transferred;
} DMAChannelState;

/* ==================== DMA CONTROLLER STATE ==================== */

typedef struct XenonDMAState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    
    /* Controller registers */
    uint32_t ctrl_status;
    uint32_t ctrl_config;
    uint32_t irq_status;
    uint32_t irq_mask;
    uint32_t version;
    uint32_t debug;
    
    /* Channels */
    DMAChannelState channels[DMA_MAX_CHANNELS];
    
    /* Interrupts */
    qemu_irq irq;
    
    /* Statistics */
    uint64_t total_transfers;
    uint64_t total_bytes;
    
    /* Xenon-specific */
    uint32_t xenon_features;
    uint32_t fifo_size;
    uint8_t fifo[4096];  /* 4KB FIFO */
    uint32_t fifo_read_ptr;
    uint32_t fifo_write_ptr;
} XenonDMAState;

/* ==================== REGISTER ACCESS ==================== */

static uint64_t dma_read(void *opaque, hwaddr offset, unsigned size) {
    XenonDMAState *s = opaque;
    uint32_t value = 0;
    int ch;
    
    /* Channel registers */
    if (offset >= DMA_CH_BASE && offset < DMA_CH_BASE + DMA_MAX_CHANNELS * DMA_CH_SIZE) {
        ch = (offset - DMA_CH_BASE) / DMA_CH_SIZE;
        offset = (offset - DMA_CH_BASE) % DMA_CH_SIZE;
        
        if (ch < DMA_MAX_CHANNELS) {
            DMAChannelState *c = &s->channels[ch];
            switch (offset) {
                case 0x00: value = c->control; break;
                case 0x04: value = c->status; break;
                case 0x08: value = c->src_addr; break;
                case 0x0C: value = c->dst_addr; break;
                case 0x10: value = c->transfer_size; break;
                case 0x14: value = c->ll_addr; break;
                case 0x18: value = c->ll_count; break;
                case 0x1C: value = c->config; break;
            }
        }
        return value;
    }
    
    /* Controller registers */
    switch (offset) {
        case DMA_CTRL_STATUS:
            value = s->ctrl_status;
            for (int i = 0; i < DMA_MAX_CHANNELS; i++) {
                if (s->channels[i].active) {
                    value |= (1 << (16 + i));  /* Channel busy */
                }
            }
            break;
        case DMA_CTRL_CONFIG:
            value = s->ctrl_config;
            break;
        case DMA_CTRL_IRQ_STATUS:
            value = s->irq_status;
            break;
        case DMA_CTRL_IRQ_MASK:
            value = s->irq_mask;
            break;
        case DMA_CTRL_VERSION:
            value = 0x00010000;  /* Version 1.0 */
            break;
        case DMA_CTRL_DEBUG:
            value = s->debug;
            break;
        default:
            break;
    }
    
    return value;
}

static void dma_write(void *opaque, hwaddr offset, 
                     uint64_t value, unsigned size) {
    XenonDMAState *s = opaque;
    int ch;
    
    /* Channel registers */
    if (offset >= DMA_CH_BASE && offset < DMA_CH_BASE + DMA_MAX_CHANNELS * DMA_CH_SIZE) {
        ch = (offset - DMA_CH_BASE) / DMA_CH_SIZE;
        offset = (offset - DMA_CH_BASE) % DMA_CH_SIZE;
        
        if (ch < DMA_MAX_CHANNELS) {
            DMAChannelState *c = &s->channels[ch];
            switch (offset) {
                case 0x00:  /* Control */
                    c->control = value;
                    c->ll_mode = (value & DMA_CTRL_LL_ENABLE) != 0;
                    c->circular = (value & DMA_CTRL_CIRCULAR) != 0;
                    
                    if (value & DMA_CTRL_START) {
                        dma_channel_start(s, ch);
                    }
                    if (value & DMA_CTRL_PAUSE) {
                        dma_channel_pause(s, ch);
                    }
                    if (value & DMA_CTRL_RESET) {
                        dma_channel_reset(s, ch);
                    }
                    break;
                case 0x04:  /* Status (read-only) */
                    /* Writing clears some status bits */
                    if (value & DMA_STATUS_COMPLETE) {
                        c->status &= ~DMA_STATUS_COMPLETE;
                    }
                    if (value & DMA_STATUS_ERROR) {
                        c->status &= ~DMA_STATUS_ERROR;
                    }
                    break;
                case 0x08:  /* Source Address */
                    c->src_addr = value;
                    c->current_src = value;
                    break;
                case 0x0C:  /* Destination Address */
                    c->dst_addr = value;
                    c->current_dst = value;
                    break;
                case 0x10:  /* Transfer Size */
                    c->transfer_size = value & (DMA_MAX_TRANSFER_SIZE - 1);
                    c->remaining = c->transfer_size;
                    break;
                case 0x14:  /* Linked List Address */
                    c->ll_addr = value;
                    c->ll_current = value;
                    break;
                case 0x18:  /* Linked List Count */
                    c->ll_count = value;
                    c->ll_remaining = value;
                    break;
                case 0x1C:  /* Configuration */
                    c->config = value;
                    break;
            }
        }
        return;
    }
    
    /* Controller registers */
    switch (offset) {
        case DMA_CTRL_CONFIG:
            s->ctrl_config = value;
            if (value & 1) {
                dma_controller_reset(s);
            }
            break;
        case DMA_CTRL_IRQ_MASK:
            s->irq_mask = value;
            break;
        case DMA_CTRL_IRQ_CLEAR:
            s->irq_status &= ~value;
            if (!(s->irq_status & s->irq_mask)) {
                qemu_set_irq(s->irq, 0);
            }
            break;
        case DMA_CTRL_DEBUG:
            s->debug = value;
            break;
    }
}

static const MemoryRegionOps dma_ops = {
    .read = dma_read,
    .write = dma_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

/* ==================== DMA TRANSFER FUNCTIONS ==================== */

static void dma_transfer_complete(void *opaque, int channel) {
    XenonDMAState *s = opaque;
    DMAChannelState *c = &s->channels[channel];
    
    c->active = false;
    c->status |= DMA_STATUS_COMPLETE;
    c->status &= ~DMA_STATUS_BUSY;
    
    /* Update statistics */
    s->total_transfers++;
    s->total_bytes += c->bytes_transferred;
    
    /* Generate interrupt if enabled */
    if (c->control & DMA_CTRL_INTERRUPT) {
        s->irq_status |= (1 << channel);
        if (s->irq_status & s->irq_mask) {
            qemu_set_irq(s->irq, 1);
        }
    }
    
    /* Handle circular buffer mode */
    if (c->circular && !c->ll_mode) {
        c->current_src = c->src_addr;
        c->current_dst = c->dst_addr;
        c->remaining = c->transfer_size;
        dma_channel_start(s, channel);
    }
}

static void dma_perform_transfer(DMAChannelState *c) {
    uint32_t chunk_size = MIN(c->remaining, 4096);  /* Transfer in chunks */
    
    if (chunk_size == 0) {
        return;
    }
    
    /* Determine transfer direction */
    uint32_t dir = (c->control >> 8) & 3;
    
    switch (dir) {
        case DMA_CTRL_DIR_MEM_TO_MEM:
            cpu_physical_memory_read(c->current_src, s->fifo, chunk_size);
            cpu_physical_memory_write(c->current_dst, s->fifo, chunk_size);
            break;
        case DMA_CTRL_DIR_MEM_TO_IO:
            cpu_physical_memory_read(c->current_src, s->fifo, chunk_size);
            /* TODO: Write to IO device */
            break;
        case DMA_CTRL_DIR_IO_TO_MEM:
            /* TODO: Read from IO device */
            cpu_physical_memory_write(c->current_dst, s->fifo, chunk_size);
            break;
        case DMA_CTRL_DIR_IO_TO_IO:
            /* TODO: IO to IO transfer */
            break;
    }
    
    c->current_src += chunk_size;
    c->current_dst += chunk_size;
    c->remaining -= chunk_size;
    c->bytes_transferred += chunk_size;
    
    if (c->remaining == 0) {
        if (c->ll_mode && c->ll_remaining > 0) {
            dma_load_next_ll_entry(c);
        } else {
            c->status |= DMA_STATUS_COMPLETE;
        }
    }
}

static void dma_load_next_ll_entry(DMAChannelState *c) {
    DMALLEntry entry;
    
    if (c->ll_remaining == 0) {
        return;
    }
    
    /* Read linked list entry */
    cpu_physical_memory_read(c->ll_current, (uint8_t*)&entry, sizeof(DMALLEntry));
    
    c->src_addr = entry.src_addr;
    c->dst_addr = entry.dst_addr;
    c->transfer_size = entry.transfer_size;
    c->control = entry.control;
    
    c->current_src = c->src_addr;
    c->current_dst = c->dst_addr;
    c->remaining = c->transfer_size;
    
    c->ll_current = entry.next_addr;
    c->ll_remaining--;
    
    if (c->ll_remaining == 0 && (c->control & DMA_CTRL_CIRCULAR)) {
        /* Wrap to beginning */
        c->ll_current = c->ll_addr;
        c->ll_remaining = c->ll_count;
    }
}

static void dma_channel_start(XenonDMAState *s, int channel) {
    DMAChannelState *c = &s->channels[channel];
    
    if (c->active || c->transfer_size == 0) {
        return;
    }
    
    c->active = true;
    c->status |= DMA_STATUS_BUSY;
    c->status &= ~(DMA_STATUS_COMPLETE | DMA_STATUS_ERROR);
    c->bytes_transferred = 0;
    
    if (c->ll_mode) {
        if (c->ll_count == 0) {
            c->status |= DMA_STATUS_ERROR;
            c->active = false;
            return;
        }
        dma_load_next_ll_entry(c);
        c->status |= DMA_STATUS_LL_ACTIVE;
    }
    
    /* Start transfer timer */
    int64_t delay_ns = (c->transfer_size / 1024) * 10;  /* Simplified timing */
    timer_mod_ns(c->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + delay_ns);
}

static void dma_channel_pause(XenonDMAState *s, int channel) {
    DMAChannelState *c = &s->channels[channel];
    
    if (c->active) {
        timer_del(c->timer);
        c->active = false;
        c->status &= ~DMA_STATUS_BUSY;
    }
}

static void dma_channel_reset(XenonDMAState *s, int channel) {
    DMAChannelState *c = &s->channels[channel];
    
    timer_del(c->timer);
    
    c->control = 0;
    c->status = 0;
    c->src_addr = 0;
    c->dst_addr = 0;
    c->transfer_size = 0;
    c->ll_addr = 0;
    c->ll_count = 0;
    c->config = 0;
    
    c->current_src = 0;
    c->current_dst = 0;
    c->remaining = 0;
    c->ll_current = 0;
    c->ll_remaining = 0;
    
    c->active = false;
    c->ll_mode = false;
    c->circular = false;
    c->bytes_transferred = 0;
}

/* ==================== DEVICE INITIALIZATION ==================== */

static void xenon_dma_realize(DeviceState *dev, Error **errp) {
    XenonDMAState *s = XENON_DMA(dev);
    
    /* Initialize controller */
    s->ctrl_status = 0;
    s->ctrl_config = 0x00000001;  /* Enabled */
    s->irq_status = 0;
    s->irq_mask = 0xFFFFFFFF;
    s->version = 0x00010000;
    s->debug = 0;
    
    /* Initialize channels */
    for (int i = 0; i < DMA_MAX_CHANNELS; i++) {
        DMAChannelState *c = &s->channels[i];
        
        c->control = 0;
        c->status = 0;
        c->src_addr = 0;
        c->dst_addr = 0;
        c->transfer_size = 0;
        c->ll_addr = 0;
        c->ll_count = 0;
        c->config = 0;
        
        c->current_src = 0;
        c->current_dst = 0;
        c->remaining = 0;
        c->ll_current = 0;
        c->ll_remaining = 0;
        
        c->active = false;
        c->ll_mode = false;
        c->circular = false;
        c->bytes_transferred = 0;
        
        /* Create timer for this channel */
        c->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, dma_channel_timer_cb, s);
        timer_set_cb(c->timer, dma_channel_timer_cb, INT2VOIDPTR(i));
    }
    
    /* Initialize FIFO */
    s->fifo_size = sizeof(s->fifo);
    s->fifo_read_ptr = 0;
    s->fifo_write_ptr = 0;
    
    /* Initialize statistics */
    s->total_transfers = 0;
    s->total_bytes = 0;
    
    /* Initialize memory region */
    memory_region_init_io(&s->iomem, OBJECT(s), &dma_ops, s,
                         "xenon.dma", DMA_REGISTER_SIZE);
    
    printf("[DMA] Xenon DMA Controller initialized: %d channels\n", DMA_MAX_CHANNELS);
}

static void xenon_dma_reset(DeviceState *dev) {
    XenonDMAState *s = XENON_DMA(dev);
    
    /* Reset controller */
    s->ctrl_status = 0;
    s->irq_status = 0;
    
    /* Reset all channels */
    for (int i = 0; i < DMA_MAX_CHANNELS; i++) {
        dma_channel_reset(s, i);
    }
    
    /* Reset FIFO */
    s->fifo_read_ptr = 0;
    s->fifo_write_ptr = 0;
    
    qemu_set_irq(s->irq, 0);
}

static void dma_channel_timer_cb(void *opaque) {
    uintptr_t channel = (uintptr_t)opaque;
    XenonDMAState *s = container_of(opaque, XenonDMAState, channels[channel]);
    
    DMAChannelState *c = &s->channels[channel];
    
    if (!c->active) {
        return;
    }
    
    /* Perform a chunk of transfer */
    dma_perform_transfer(c);
    
    if (c->status & DMA_STATUS_COMPLETE) {
        dma_transfer_complete(s, channel);
    } else {
        /* Schedule next chunk */
        int64_t delay_ns = 1000;  /* 1 microsecond per chunk */
        timer_mod_ns(c->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + delay_ns);
    }
}

static void dma_controller_reset(XenonDMAState *s) {
    for (int i = 0; i < DMA_MAX_CHANNELS; i++) {
        dma_channel_reset(s, i);
    }
    s->irq_status = 0;
    qemu_set_irq(s->irq, 0);
}

/* ==================== QEMU DEVICE ==================== */

static void xenon_dma_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    
    dc->realize = xenon_dma_realize;
    dc->reset = xenon_dma_reset;
    dc->desc = "Xenon DMA Controller";
}

static const TypeInfo xenon_dma_type_info = {
    .name = TYPE_XENON_DMA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XenonDMAState),
    .class_init = xenon_dma_class_init,
};

static void xenon_dma_register_types(void) {
    type_register_static(&xenon_dma_type_info);
}

type_init(xenon_dma_register_types);

/* ==================== PUBLIC FUNCTIONS ==================== */

XenonDMAState *xenon_dma_create(MemoryRegion *parent, hwaddr base) {
    DeviceState *dev;
    XenonDMAState *s;
    
    dev = qdev_new(TYPE_XENON_DMA);
    s = XENON_DMA(dev);
    
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    
    /* Connect interrupt */
    s->irq = qemu_allocate_irq(xenon_dma_irq_handler, s, 0);
    
    return s;
}

void xenon_dma_start_transfer(XenonDMAState *s, int channel,
                             uint32_t src, uint32_t dst, uint32_t size) {
    if (channel < 0 || channel >= DMA_MAX_CHANNELS) {
        return;
    }
    
    DMAChannelState *c = &s->channels[channel];
    
    c->src_addr = src;
    c->dst_addr = dst;
    c->transfer_size = size;
    
    c->current_src = src;
    c->current_dst = dst;
    c->remaining = size;
    
    dma_channel_start(s, channel);
}

void xenon_dma_setup_linked_list(XenonDMAState *s, int channel,
                                uint32_t ll_addr, uint32_t count) {
    if (channel < 0 || channel >= DMA_MAX_CHANNELS) {
        return;
    }
    
    DMAChannelState *c = &s->channels[channel];
    
    c->ll_addr = ll_addr;
    c->ll_count = count;
    c->ll_mode = true;
    
    c->ll_current = ll_addr;
    c->ll_remaining = count;
}

void xenon_dma_dump_state(XenonDMAState *s) {
    printf("Xenon DMA Controller State:\n");
    printf("  Status: 0x%08X, Config: 0x%08X\n", s->ctrl_status, s->ctrl_config);
    printf("  IRQ Status: 0x%08X, Mask: 0x%08X\n", s->irq_status, s->irq_mask);
    printf("  Total transfers: %" PRIu64 ", Total bytes: %" PRIu64 "\n",
           s->total_transfers, s->total_bytes);
    
    printf("  Channels:\n");
    for (int i = 0; i < DMA_MAX_CHANNELS; i++) {
        DMAChannelState *c = &s->channels[i];
        if (c->active || c->transfer_size > 0) {
            printf("    Channel %d: %s, Src: 0x%08X, Dst: 0x%08X, Size: %d\n",
                   i, c->active ? "Active" : "Idle",
                   c->src_addr, c->dst_addr, c->transfer_size);
            if (c->ll_mode) {
                printf("      LL: Addr=0x%08X, Count=%d, Remaining=%d\n",
                       c->ll_addr, c->ll_count, c->ll_remaining);
            }
        }
    }
}
