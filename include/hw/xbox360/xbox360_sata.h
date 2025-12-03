#ifndef HW_XBOX360_SATA_H
#define HW_XBOX360_SATA_H

#include "hw/sysbus.h"
#include "qom/object.h"
#include "sysemu/block-backend.h"

#define TYPE_XENON_SATA "xenon.sata"
OBJECT_DECLARE_SIMPLE_TYPE(XenonSATAState, XENON_SATA)

struct XenonSATAState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
};

/* Public functions */
XenonSATAState *xenon_sata_create(MemoryRegion *parent, hwaddr base);
void xenon_sata_attach_drive(XenonSATAState *s, int port, BlockBackend *blk,
                            const char *serial, const char *model);
void xenon_sata_dump_state(XenonSATAState *s);
void xenon_sata_connect_irq(XenonSATAState *s, XenonGICState *gic, int hdd_irq, int dvd_irq);
BlockBackend *xenon_sata_get_backend(XenonSATAState *s, int port);

/* SATA Ports */
enum SATAPorts {
    SATA_PORT_HDD = 0,
    SATA_PORT_DVD = 1,
};

#endif
