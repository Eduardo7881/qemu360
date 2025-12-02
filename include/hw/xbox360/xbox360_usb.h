#ifndef HW_XBOX360_USB_H
#define HW_XBOX360_USB_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_XENON_USB "xenon.usb"
OBJECT_DECLARE_SIMPLE_TYPE(XenonUSBState, XENON_USB)

struct XenonUSBState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
};

/* Public functions */
XenonUSBState *xenon_usb_create(MemoryRegion *parent);
void xenon_usb_attach_device(XenonUSBState *s, int port, uint8_t speed);
void xenon_usb_detach_device(XenonUSBState *s, int port);
void xenon_usb_dump_state(XenonUSBState *s);

/* USB Ports */
enum USBPorts {
    USB_PORT_CONTROLLER1 = 0,
    USB_PORT_CONTROLLER2 = 1,
    USB_PORT_MEMORY_UNIT1 = 2,
    USB_PORT_MEMORY_UNIT2 = 3,
};

/* USB Speeds */
enum USBSpeeds {
    USB_SPEED_LOW = 0,
    USB_SPEED_FULL = 1,
    USB_SPEED_HIGH = 2,
};

#endif
