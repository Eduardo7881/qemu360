#ifndef HW_XBOX360_XMA_H
#define HW_XBOX360_XMA_H

#include "hw/sysbus.h"
#include "qom/object.h"
#include "audio/audio.h"

#define TYPE_XENON_XMA "xenon.xma"
OBJECT_DECLARE_SIMPLE_TYPE(XenonXMAState, XENON_XMA)

struct XenonXMAState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    QEMUSoundCard card;
    qemu_irq irq;
};

/* Public functions */
XenonXMAState *xenon_xma_create(MemoryRegion *parent, hwaddr base);
void xenon_xma_play_buffer(XenonXMAState *s, int channel,
                          uint32_t buffer_addr, uint32_t buffer_size);
void xenon_xma_stop_channel(XenonXMAState *s, int channel);
void xenon_xma_dump_state(XenonXMAState *s);

/* XMA Channels */
enum XMAChannels {
    XMA_CHANNEL_MAIN = 0,
    XMA_CHANNEL_EFFECTS = 1,
    XMA_CHANNEL_VOICE = 2,
    XMA_CHANNEL_MUSIC = 3,
    XMA_CHANNEL_UI = 4,
};

#endif
