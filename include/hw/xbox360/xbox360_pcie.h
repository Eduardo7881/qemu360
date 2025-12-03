#ifndef HW_XBOX360_PCIE_H
#define HW_XBOX360_PCIE_H

#include "hw/sysbus.h"
#include "qom/object.h"
#include "hw/pci/pci.h"

#define TYPE_XENON_PCIE "xenon.pcie"
OBJECT_DECLARE_SIMPLE_TYPE(XenonPCIeState, XENON_PCIE)

struct XenonPCIeState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    PCIBus bus;
    qemu_irq irq;
    qemu_irq msi_irq;
};

/* Public functions */
XenonPCIeState *xenon_pcie_create(MemoryRegion *parent, hwaddr base);
void xenon_pcie_connect_gpu(XenonPCIeState *s, Xbox360GPUState *gpu);
void xenon_pcie_send_msi(XenonPCIeState *s, uint32_t vector);
void xenon_pcie_dump_state(XenonPCIeState *s);

/* PCIe Ports */
enum XenonPCIePorts {
    PCIE_PORT_GPU = 0,
    PCIE_PORT_EHCI = 1,
    PCIE_PORT_OHCI = 2,
    PCIE_PORT_SATA = 3,
    PCIE_PORT_SMBUS = 4,
    PCIE_PORT_AUDIO = 5,
    PCIE_PORT_ETHERNET = 6,
    PCIE_PORT_BRIDGE = 7,
};

#endif
