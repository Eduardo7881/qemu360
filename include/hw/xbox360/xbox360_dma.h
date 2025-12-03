#ifndef HW_XBOX360_DMA_H
#define HW_XBOX360_DMA_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_XENON_DMA "xenon.dma"
OBJECT_DECLARE_SIMPLE_TYPE(XenonDMAState, XENON_DMA)

struct XenonDMAState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
};

/* Public functions */
XenonDMAState *xenon_dma_create(MemoryRegion *parent, hwaddr base);
void xenon_dma_start_transfer(XenonDMAState *s, int channel,
                             uint32_t src, uint32_t dst, uint32_t size);
void xenon_dma_setup_linked_list(XenonDMAState *s, int channel,
                                uint32_t ll_addr, uint32_t count);
void xenon_dma_dump_state(XenonDMAState *s);

/* DMA Channels */
enum XenonDMAChannels {
    DMA_CHANNEL_GPU = 0,
    DMA_CHANNEL_AUDIO = 1,
    DMA_CHANNEL_USB = 2,
    DMA_CHANNEL_NETWORK = 3,
    DMA_CHANNEL_HDD = 4,
    DMA_CHANNEL_DVD = 5,
    DMA_CHANNEL_MEMCOPY = 6,
    DMA_CHANNEL_SCRATCH = 7,
};

#endif
