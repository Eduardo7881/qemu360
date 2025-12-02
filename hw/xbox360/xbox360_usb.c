#include "qemu/osdep.h"
#include "hw/xbox360/xbox360_usb.h"
#include "hw/xbox360/xbox360.h"
#include "hw/usb/hcd-ehci.h"
#include "hw/usb/hcd-ohci.h"
#include "hw/pci/pci.h"
#include "hw/irq.h"
#include "qemu/timer.h"
#include "migration/vmstate.h"

/* ==================== XENON USB CONFIGURATION ==================== */
#define USB_EHCI_BASE             0x80008000
#define USB_OHCI_BASE             0x80009000
#define USB_REGISTER_SIZE         0x1000

#define USB_MAX_PORTS             4
#define USB_MAX_DEVICES           127

/* USB Controller Registers */
#define USB_REG_CONTROL           0x0000
#define USB_REG_STATUS            0x0004
#define USB_REG_INTERRUPT         0x0008
#define USB_REG_FRAME_NUMBER      0x000C
#define USB_REG_FRAME_LIST        0x0010
#define USB_REG_ASYNC_LIST        0x0014
#define USB_REG_CONFIG_FLAG       0x0018
#define USB_REG_PORT_STATUS       0x0020

/* USB Port Status Bits */
#define USB_PORT_CONNECT          (1 << 0)
#define USB_PORT_ENABLE           (1 << 1)
#define USB_PORT_SUSPEND          (1 << 2)
#define USB_PORT_RESET            (1 << 3)
#define USB_PORT_POWER            (1 << 8)
#define USB_PORT_LOW_SPEED        (1 << 9)

/* USB Speeds */
#define USB_SPEED_LOW             0
#define USB_SPEED_FULL            1
#define USB_SPEED_HIGH            2

/* Xenon USB Port Assignments */
#define USB_PORT_CONTROLLER1      0  /* Top port */
#define USB_PORT_CONTROLLER2      1  /* Bottom port */
#define USB_PORT_MEMORY_UNIT1     2  /* Left MU */
#define USB_PORT_MEMORY_UNIT2     3  /* Right MU */

/* ==================== USB DEVICE STRUCTURES ==================== */

typedef struct USBDeviceState {
    USBDevice *udev;
    uint8_t address;
    uint8_t speed;
    bool connected;
    bool configured;
    uint32_t last_packet;
} USBDeviceState;

typedef struct USBPortState {
    uint32_t status;
    uint32_t control;
    USBDeviceState *device;
    bool powered;
    bool overcurrent;
} USBPortState;

/* ==================== USB CONTROLLER STATE ==================== */

typedef struct XenonUSBState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    
    /* EHCI (High-speed) controller */
    struct {
        uint32_t usbcmd;
        uint32_t usbsts;
        uint32_t usbintr;
        uint32_t frindex;
        uint32_t ctrldssegment;
        uint32_t periodiclistbase;
        uint32_t asynclistaddr;
        uint32_t configflag;
        
        USBPortState ports[USB_MAX_PORTS];
        USBDeviceState *devices[USB_MAX_DEVICES];
        int device_count;
        
        QEMUTimer *frame_timer;
        uint32_t frame_number;
        bool running;
    } ehci;
    
    /* OHCI (Full/low-speed) controller */
    struct {
        uint32_t hccontrol;
        uint32_t hcstatus;
        uint32_t hcinterrupt;
        uint32_t hcinterruptenable;
        uint32_t hchcca;
        uint32_t hcperiodcurrent;
        uint32_t hccontrolheaded;
        uint32_t hccontrolcurrent;
        uint32_t hcbulkheaded;
        uint32_t hcbulkcurrent;
        uint32_t hcdonehead;
        
        USBPortState ports[USB_MAX_PORTS];
        USBDeviceState *devices[USB_MAX_DEVICES];
        int device_count;
        
        QEMUTimer *frame_timer;
        uint32_t frame_number;
        bool running;
    } ohci;
    
    /* Common */
    uint32_t control;
    uint32_t status;
    uint32_t interrupt;
    
    /* Interrupts */
    qemu_irq irq;
    
    /* Statistics */
    uint64_t packets_sent;
    uint64_t packets_received;
    uint64_t bytes_sent;
    uint64_t bytes_received;
    
    /* Xenon-specific */
    uint32_t xenon_features;
    uint32_t debug;
} XenonUSBState;

/* ==================== REGISTER ACCESS ==================== */

static uint64_t usb_read(void *opaque, hwaddr offset, unsigned size) {
    XenonUSBState *s = opaque;
    uint32_t value = 0;
    
    /* EHCI registers */
    if (offset < 0x800) {
        switch (offset) {
            case 0x000: value = s->ehci.usbcmd; break;
            case 0x004: value = s->ehci.usbsts; break;
            case 0x008: value = s->ehci.usbintr; break;
            case 0x00C: value = s->ehci.frindex; break;
            case 0x010: value = s->ehci.ctrldssegment; break;
            case 0x014: value = s->ehci.periodiclistbase; break;
            case 0x018: value = s->ehci.asynclistaddr; break;
            case 0x01C: value = s->ehci.configflag; break;
            
            /* Port status registers */
            case 0x020 ... 0x02C:
                {
                    int port = (offset - 0x020) / 4;
                    if (port < USB_MAX_PORTS) {
                        value = s->ehci.ports[port].status;
                    }
                }
                break;
        }
    }
    /* OHCI registers */
    else if (offset >= 0x800 && offset < 0x1000) {
        offset -= 0x800;
        
        switch (offset) {
            case 0x000: value = s->ohci.hccontrol; break;
            case 0x004: value = s->ohci.hcstatus; break;
            case 0x008: value = s->ohci.hcinterrupt; break;
            case 0x00C: value = s->ohci.hcinterruptenable; break;
            case 0x010: value = s->ohci.hchcca; break;
            case 0x014: value = s->ohci.hcperiodcurrent; break;
            case 0x018: value = s->ohci.hccontrolheaded; break;
            case 0x01C: value = s->ohci.hccontrolcurrent; break;
            case 0x020: value = s->ohci.hcbulkheaded; break;
            case 0x024: value = s->ohci.hcbulkcurrent; break;
            case 0x028: value = s->ohci.hcdonehead; break;
            
            /* Port status registers */
            case 0x030 ... 0x03C:
                {
                    int port = (offset - 0x030) / 4;
                    if (port < USB_MAX_PORTS) {
                        value = s->ohci.ports[port].status;
                    }
                }
                break;
        }
    }
    
    return value;
}

static void usb_write(void *opaque, hwaddr offset, 
                     uint64_t value, unsigned size) {
    XenonUSBState *s = opaque;
    
    /* EHCI registers */
    if (offset < 0x800) {
        switch (offset) {
            case 0x000:  /* USBCMD */
                s->ehci.usbcmd = value;
                if (value & 1) {
                    usb_ehci_start(&s->ehci);
                } else {
                    usb_ehci_stop(&s->ehci);
                }
                if (value & (1 << 2)) {
                    usb_ehci_reset(&s->ehci);
                }
                break;
            case 0x004:  /* USBSTS */
                /* Write 1 to clear bits */
                s->ehci.usbsts &= ~value;
                break;
            case 0x008:  /* USBINTR */
                s->ehci.usbintr = value;
                break;
            case 0x00C:  /* FRINDEX */
                s->ehci.frindex = value & 0x3FFF;
                break;
            case 0x010:  /* CTRLDSSEGMENT */
                s->ehci.ctrldssegment = value;
                break;
            case 0x014:  /* PERIODICLISTBASE */
                s->ehci.periodiclistbase = value & 0xFFFFF000;
                break;
            case 0x018:  /* ASYNCLISTADDR */
                s->ehci.asynclistaddr = value & 0xFFFFFFF0;
                break;
            case 0x01C:  /* CONFIGFLAG */
                s->ehci.configflag = value;
                break;
                
            /* Port control registers */
            case 0x020 ... 0x02C:
                {
                    int port = (offset - 0x020) / 4;
                    if (port < USB_MAX_PORTS) {
                        USBPortState *p = &s->ehci.ports[port];
                        
                        if (value & USB_PORT_RESET) {
                            usb_port_reset(p);
                        }
                        if (value & USB_PORT_POWER) {
                            p->powered = true;
                            p->status |= USB_PORT_POWER;
                        } else {
                            p->powered = false;
                            p->status &= ~USB_PORT_POWER;
                        }
                        
                        p->control = value;
                    }
                }
                break;
        }
    }
    /* OHCI registers */
    else if (offset >= 0x800 && offset < 0x1000) {
        offset -= 0x800;
        
        switch (offset) {
            case 0x000:  /* HcControl */
                s->ohci.hccontrol = value;
                if (value & 1) {
                    usb_ohci_start(&s->ohci);
                } else {
                    usb_ohci_stop(&s->ohci);
                }
                if (value & (1 << 2)) {
                    usb_ohci_reset(&s->ohci);
                }
                break;
            case 0x004:  /* HcStatus */
                /* Write 1 to clear bits */
                s->ohci.hcstatus &= ~value;
                break;
            case 0x008:  /* HcInterrupt */
                s->ohci.hcinterrupt = value;
                break;
            case 0x00C:  /* HcInterruptEnable */
                s->ohci.hcinterruptenable = value;
                break;
            case 0x010:  /* HcHCCA */
                s->ohci.hchcca = value & 0xFFFFFF00;
                break;
            case 0x014:  /* HcPeriodCurrent */
                s->ohci.hcperiodcurrent = value & 0xFFFFFFF0;
                break;
            case 0x018:  /* HcControlHeadEd */
                s->ohci.hccontrolheaded = value & 0xFFFFFFF0;
                break;
            case 0x01C:  /* HcControlCurrent */
                s->ohci.hccontrolcurrent = value & 0xFFFFFFF0;
                break;
            case 0x020:  /* HcBulkHeadEd */
                s->ohci.hcbulkheaded = value & 0xFFFFFFF0;
                break;
            case 0x024:  /* HcBulkCurrent */
                s->ohci.hcbulkcurrent = value & 0xFFFFFFF0;
                break;
            case 0x028:  /* HcDoneHead */
                s->ohci.hcdonehead = value & 0xFFFFFFF0;
                break;
                
            /* Port control registers */
            case 0x030 ... 0x03C:
                {
                    int port = (offset - 0x030) / 4;
                    if (port < USB_MAX_PORTS) {
                        USBPortState *p = &s->ohci.ports[port];
                        
                        if (value & USB_PORT_RESET) {
                            usb_port_reset(p);
                        }
                        if (value & USB_PORT_POWER) {
                            p->powered = true;
                            p->status |= USB_PORT_POWER;
                        } else {
                            p->powered = false;
                            p->status &= ~USB_PORT_POWER;
                        }
                        
                        p->control = value;
                    }
                }
                break;
        }
    }
}

static const MemoryRegionOps usb_ops = {
    .read = usb_read,
    .write = usb_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

/* ==================== USB PORT MANAGEMENT ==================== */

static void usb_port_reset(USBPortState *port) {
    if (port->device) {
        /* Reset connected device */
        port->device->configured = false;
        port->device->address = 0;
    }
    
    port->status &= ~(USB_PORT_ENABLE | USB_PORT_SUSPEND);
    port->status |= USB_PORT_RESET;
    
    /* Clear reset after 10ms */
    // timer_mod_ns(port->reset_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 10 * 1000000);
}

static void usb_port_connect(USBPortState *port, USBDeviceState *dev, uint8_t speed) {
    if (port->device) {
        /* Port already occupied */
        return;
    }
    
    port->device = dev;
    port->status |= USB_PORT_CONNECT;
    
    if (speed == USB_SPEED_LOW) {
        port->status |= USB_PORT_LOW_SPEED;
    } else {
        port->status &= ~USB_PORT_LOW_SPEED;
    }
    
    dev->connected = true;
    dev->speed = speed;
    
    printf("[USB] Device connected to port: speed=%s\n",
           speed == USB_SPEED_HIGH ? "High" :
           speed == USB_SPEED_FULL ? "Full" : "Low");
}

static void usb_port_disconnect(USBPortState *port) {
    if (!port->device) {
        return;
    }
    
    port->device->connected = false;
    port->device = NULL;
    
    port->status &= ~(USB_PORT_CONNECT | USB_PORT_ENABLE | 
                     USB_PORT_SUSPEND | USB_PORT_LOW_SPEED);
    
    printf("[USB] Device disconnected from port\n");
}

/* ==================== USB DEVICE MANAGEMENT ==================== */

static USBDeviceState *usb_create_device(uint8_t speed) {
    USBDeviceState *dev = g_new0(USBDeviceState, 1);
    
    dev->address = 0;
    dev->speed = speed;
    dev->connected = false;
    dev->configured = false;
    dev->last_packet = 0;
    
    /* Create QEMU USB device */
    // dev->udev = usbdevice_create("usb-mouse");  /* Example */
    
    return dev;
}

static void usb_destroy_device(USBDeviceState *dev) {
    if (dev->udev) {
        // usb_device_destroy(dev->udev);
    }
    g_free(dev);
}

static int usb_assign_address(XenonUSBState *s, USBDeviceState *dev) {
    for (int i = 1; i < USB_MAX_DEVICES; i++) {
        if (!s->ehci.devices[i] && !s->ohci.devices[i]) {
            dev->address = i;
            
            /* Add to appropriate controller */
            if (dev->speed == USB_SPEED_HIGH) {
                s->ehci.devices[i] = dev;
                s->ehci.device_count++;
            } else {
                s->ohci.devices[i] = dev;
                s->ohci.device_count++;
            }
            
            return i;
        }
    }
    
    return 0;  /* No address available */
}

/* ==================== EHCI CONTROLLER FUNCTIONS ==================== */

static void usb_ehci_start(struct ehci *ehci) {
    if (ehci->running) {
        return;
    }
    
    ehci->running = true;
    ehci->frame_number = 0;
    
    /* Start frame timer (1ms frames) */
    timer_mod(ehci->frame_timer, 
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000);
    
    printf("[USB-EHCI] Controller started\n");
}

static void usb_ehci_stop(struct ehci *ehci) {
    if (!ehci->running) {
        return;
    }
    
    ehci->running = false;
    timer_del(ehci->frame_timer);
    
    printf("[USB-EHCI] Controller stopped\n");
}

static void usb_ehci_reset(struct ehci *ehci) {
    ehci->usbcmd = 0;
    ehci->usbsts = 0;
    ehci->usbintr = 0;
    ehci->frindex = 0;
    ehci->configflag = 0;
    
    for (int i = 0; i < USB_MAX_PORTS; i++) {
        ehci->ports[i].status = 0;
        ehci->ports[i].control = 0;
        ehci->ports[i].powered = false;
        ehci->ports[i].overcurrent = false;
    }
    
    timer_del(ehci->frame_timer);
    ehci->running = false;
    
    printf("[USB-EHCI] Controller reset\n");
}

static void usb_ehci_frame_timer(void *opaque) {
    struct ehci *ehci = opaque;
    
    if (!ehci->running) {
        return;
    }
    
    /* Increment frame number */
    ehci->frame_number++;
    if (ehci->frame_number >= 0x4000) {
        ehci->frame_number = 0;
    }
    ehci->frindex = ehci->frame_number;
    
    /* Process periodic schedule */
    // usb_ehci_process_periodic(ehci);
    
    /* Schedule next frame */
    timer_mod(ehci->frame_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000);
}

/* ==================== OHCI CONTROLLER FUNCTIONS ==================== */

static void usb_ohci_start(struct ohci *ohci) {
    if (ohci->running) {
        return;
    }
    
    ohci->running = true;
    ohci->frame_number = 0;
    
    /* Start frame timer (1ms frames) */
    timer_mod(ohci->frame_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000);
    
    printf("[USB-OHCI] Controller started\n");
}

static void usb_ohci_stop(struct ohci *ohci) {
    if (!ohci->running) {
        return;
    }
    
    ohci->running = false;
    timer_del(ohci->frame_timer);
    
    printf("[USB-OHCI] Controller stopped\n");
}

static void usb_ohci_reset(struct ohci *ohci) {
    ohci->hccontrol = 0;
    ohci->hcstatus = 0;
    ohci->hcinterrupt = 0;
    ohci->hcinterruptenable = 0;
    
    for (int i = 0; i < USB_MAX_PORTS; i++) {
        ohci->ports[i].status = 0;
        ohci->ports[i].control = 0;
        ohci->ports[i].powered = false;
        ohci->ports[i].overcurrent = false;
    }
    
    timer_del(ohci->frame_timer);
    ohci->running = false;
    
    printf("[USB-OHCI] Controller reset\n");
}

static void usb_ohci_frame_timer(void *opaque) {
    struct ohci *ohci = opaque;
    
    if (!ohci->running) {
        return;
    }
    
    /* Increment frame number */
    ohci->frame_number++;
    if (ohci->frame_number >= 0x4000) {
        ohci->frame_number = 0;
    }
    
    /* Process control and bulk lists */
    // usb_ohci_process_lists(ohci);
    
    /* Schedule next frame */
    timer_mod(ohci->frame_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000);
}

/* ==================== DEVICE INITIALIZATION ==================== */

static void xenon_usb_realize(DeviceState *dev, Error **errp) {
    XenonUSBState *s = XENON_USB(dev);
    
    /* Initialize EHCI */
    s->ehci.usbcmd = 0;
    s->ehci.usbsts = 0;
    s->ehci.usbintr = 0;
    s->ehci.frindex = 0;
    s->ehci.configflag = 1;  /* Configured */
    
    for (int i = 0; i < USB_MAX_PORTS; i++) {
        s->ehci.ports[i].status = 0;
        s->ehci.ports[i].control = 0;
        s->ehci.ports[i].device = NULL;
        s->ehci.ports[i].powered = false;
        s->ehci.ports[i].overcurrent = false;
    }
    
    s->ehci.frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, 
                                       usb_ehci_frame_timer, &s->ehci);
    s->ehci.frame_number = 0;
    s->ehci.running = false;
    s->ehci.device_count = 0;
    
    /* Initialize OHCI */
    s->ohci.hccontrol = 0;
    s->ohci.hcstatus = 0;
    s->ohci.hcinterrupt = 0;
    s->ohci.hcinterruptenable = 0;
    
    for (int i = 0; i < USB_MAX_PORTS; i++) {
        s->ohci.ports[i].status = 0;
        s->ohci.ports[i].control = 0;
        s->ohci.ports[i].device = NULL;
        s->ohci.ports[i].powered = false;
        s->ohci.ports[i].overcurrent = false;
    }
    
    s->ohci.frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                       usb_ohci_frame_timer, &s->ohci);
    s->ohci.frame_number = 0;
    s->ohci.running = false;
    s->ohci.device_count = 0;
    
    /* Initialize statistics */
    s->packets_sent = 0;
    s->packets_received = 0;
    s->bytes_sent = 0;
    s->bytes_received = 0;
    
    /* Initialize memory region */
    memory_region_init_io(&s->iomem, OBJECT(s), &usb_ops, s,
                         "xenon.usb", USB_REGISTER_SIZE * 2);
    
    printf("[USB] Xenon USB Controller initialized\n");
    printf("[USB] EHCI at 0x%08X, OHCI at 0x%08X\n",
           USB_EHCI_BASE, USB_OHCI_BASE);
}

static void xenon_usb_reset(DeviceState *dev) {
    XenonUSBState *s = XENON_USB(dev);
    
    usb_ehci_reset(&s->ehci);
    usb_ohci_reset(&s->ohci);
    
    qemu_set_irq(s->irq, 0);
}

/* ==================== QEMU DEVICE ==================== */

static void xenon_usb_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    
    dc->realize = xenon_usb_realize;
    dc->reset = xenon_usb_reset;
    dc->desc = "Xenon USB Controller";
}

static const TypeInfo xenon_usb_type_info = {
    .name = TYPE_XENON_USB,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XenonUSBState),
    .class_init = xenon_usb_class_init,
};

static void xenon_usb_register_types(void) {
    type_register_static(&xenon_usb_type_info);
}

type_init(xenon_usb_register_types);

/* ==================== PUBLIC FUNCTIONS ==================== */

XenonUSBState *xenon_usb_create(MemoryRegion *parent) {
    DeviceState *dev;
    XenonUSBState *s;
    
    dev = qdev_new(TYPE_XENON_USB);
    s = XENON_USB(dev);
    
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    
    /* Map both controllers */
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, USB_EHCI_BASE);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 1, USB_OHCI_BASE);
    
    /* Connect interrupt */
    s->irq = qemu_allocate_irq(xenon_usb_irq_handler, s, 0);
    
    return s;
}

void xenon_usb_attach_device(XenonUSBState *s, int port, uint8_t speed) {
    if (port < 0 || port >= USB_MAX_PORTS) {
        return;
    }
    
    /* Create and attach device */
    USBDeviceState *dev = usb_create_device(speed);
    
    if (speed == USB_SPEED_HIGH) {
        usb_port_connect(&s->ehci.ports[port], dev, speed);
    } else {
        usb_port_connect(&s->ohci.ports[port], dev, speed);
    }
    
    /* Assign USB address */
    int addr = usb_assign_address(s, dev);
    if (addr > 0) {
        printf("[USB] Device assigned address %d on port %d\n", addr, port);
    }
}

void xenon_usb_detach_device(XenonUSBState *s, int port) {
    if (port < 0 || port >= USB_MAX_PORTS) {
        return;
    }
    
    /* Check both controllers */
    if (s->ehci.ports[port].device) {
        usb_port_disconnect(&s->ehci.ports[port]);
    }
    if (s->ohci.ports[port].device) {
        usb_port_disconnect(&s->ohci.ports[port]);
    }
}

void xenon_usb_dump_state(XenonUSBState *s) {
    printf("Xenon USB Controller State:\n");
    
    printf("  EHCI: %s, Frame=%u, Devices=%d\n",
           s->ehci.running ? "Running" : "Stopped",
           s->ehci.frame_number, s->ehci.device_count);
    printf("    Ports: ");
    for (int i = 0; i < USB_MAX_PORTS; i++) {
        if (s->ehci.ports[i].status & USB_PORT_CONNECT) {
            printf("%d ", i);
        }
    }
    printf("\n");
    
    printf("  OHCI: %s, Frame=%u, Devices=%d\n",
           s->ohci.running ? "Running" : "Stopped",
           s->ohci.frame_number, s->ohci.device_count);
    printf("    Ports: ");
    for (int i = 0; i < USB_MAX_PORTS; i++) {
        if (s->ohci.ports[i].status & USB_PORT_CONNECT) {
            printf("%d ", i);
        }
    }
    printf("\n");
    
    printf("  Statistics: %" PRIu64 " sent, %" PRIu64 " received\n",
           s->packets_sent, s->packets_received);
}
