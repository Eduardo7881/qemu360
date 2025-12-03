#ifndef HW_XBOX360_NETWORK_H
#define HW_XBOX360_NETWORK_H

#include "hw/sysbus.h"
#include "qom/object.h"
#include "net/net.h"

#define TYPE_XENON_NETWORK "xenon.network"
OBJECT_DECLARE_SIMPLE_TYPE(XenonNetworkState, XENON_NETWORK)

struct XenonNetworkState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    NICState *nic;
    NICConf conf;
    qemu_irq irq;
};

/* Public functions */
XenonNetworkState *xenon_network_create(MemoryRegion *parent, hwaddr base);
void xenon_network_dump_state(XenonNetworkState *s);
void xenon_network_connect_irq(XenonNetworkState *s, XenonGICState *gic, int irq_num);

#endif
