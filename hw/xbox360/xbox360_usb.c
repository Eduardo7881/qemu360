#include "qemu/osdep.h"
#include "hw/xbox360/xbox360_usb.h"
#include "hw/xbox360/xbox360.h"
#include "hw/xbox360/xbox360_gic.h"
#include "hw/usb/hcd-ehci.h"
#include "hw/usb/hcd-ohci.h"
#include "hw/block/block.h"
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
    void *gic;
    int gic_irq;
    uint8_t hid_report[8];
    uint8_t msc_buf[4096];
    uint32_t msc_buf_len;
    uint32_t msc_block_size;
    uint32_t msc_last_lba;
    uint32_t msc_expected_len;
    uint32_t msc_tag;
    uint8_t msc_status;
    uint8_t msc_opcode;
    uint8_t msc_lun;
    uint32_t msc_lba;
    uint32_t msc_write_remaining;
    uint32_t msc_written;
    uint32_t msc_transferred;
    uint32_t msc_residue;
    bool msc_dir_in;
    uint8_t msc_sense_key;
    uint8_t msc_asc;
    uint8_t msc_ascq;
    BlockBackend *msc_blk;
    BlockBackend *msc_luns[2];
    bool msc_ua[2];
    uint32_t msc_read_remaining;
    uint32_t msc_read_offset;
} XenonUSBState;

static const uint8_t usb_device_descriptor_hs[] = {
    18, 1, 0x00, 0x02, 0x00, 0x00, 0x00, 64, 0x5D, 0x04, 0x23, 0xA1, 0x00, 0x01, 1, 2, 3, 1
};
static const uint8_t usb_configuration_descriptor_hs[] = {
    9, 2, 57, 0, 2, 1, 0, 0x80, 50,
    9, 4, 0, 0, 1, 0x03, 0x01, 0x01, 0,
    9, 0x21, 0x11, 0x01, 0, 1, 0x22, 63, 0,
    7, 5, 0x81, 0x03, 8, 0, 10,
    9, 4, 1, 0, 2, 0x08, 0x06, 0x50, 0,
    7, 5, 0x02, 0x02, 0x00, 0x02, 0,
    7, 5, 0x82, 0x02, 0x00, 0x02, 0
};
static const uint8_t usb_hid_report_descriptor_keyboard[] = {
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x05, 0x07,
    0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x95, 0x01,
    0x75, 0x08, 0x81, 0x03, 0x95, 0x05, 0x75, 0x01,
    0x05, 0x08, 0x19, 0x01, 0x29, 0x05, 0x91, 0x02,
    0x95, 0x01, 0x75, 0x03, 0x91, 0x03, 0x95, 0x06,
    0x75, 0x08, 0x15, 0x00, 0x25, 0x65, 0x05, 0x07,
    0x19, 0x00, 0x29, 0x65, 0x81, 0x00, 0xC0
};
static const uint8_t usb_string_langid[] = { 4, 3, 0x09, 0x04 };
static const uint8_t usb_string_mfr[] = { 16, 3, 'X', 0, 'e', 0, 'n', 0, 'o', 0, 'n', 0, ' ', 0, 'USB', 0 };
static const uint8_t usb_string_prod[] = { 22, 3, 'H',0,'I',0,'D',0,' ',0,'K',0,'e',0,'y',0,'b',0,'o',0,'a',0,'r',0,'d',0 };
static const uint8_t usb_string_ser[] = { 12, 3, '0',0,'0',0,'0',0,'1',0,'2',0 };

static void usb_write_descriptor(hwaddr buf, const uint8_t *data, uint32_t len) {
    uint32_t n = len;
    cpu_physical_memory_write(buf, data, n);
}

static void usb_handle_setup_ehci(XenonUSBState *s, uint32_t dev_addr, hwaddr setup_ptr, hwaddr data_ptr) {
    uint8_t setup[8];
    cpu_physical_memory_read(setup_ptr, setup, 8);
    uint8_t bm = setup[0];
    uint8_t b = setup[1];
    uint16_t wValue = setup[2] | (setup[3] << 8);
    uint16_t wIndex = setup[4] | (setup[5] << 8);
    uint16_t wLength = setup[6] | (setup[7] << 8);
    if ((bm & 0x60) == 0) {
        if (b == 6) {
            uint8_t dtype = wValue >> 8;
            uint8_t dindex = wValue & 0xFF;
            const uint8_t *src = NULL;
            uint32_t slen = 0;
            if (dtype == 1) { src = usb_device_descriptor_hs; slen = sizeof(usb_device_descriptor_hs); }
            else if (dtype == 2) { src = usb_configuration_descriptor_hs; slen = sizeof(usb_configuration_descriptor_hs); }
            else if (dtype == 3) {
                if (dindex == 0) { src = usb_string_langid; slen = sizeof(usb_string_langid); }
                else if (dindex == 1) { src = usb_string_mfr; slen = usb_string_mfr[0]; }
                else if (dindex == 2) { src = usb_string_prod; slen = usb_string_prod[0]; }
                else if (dindex == 3) { src = usb_string_ser; slen = usb_string_ser[0]; }
            } else if (dtype == 0x22) {
                src = usb_hid_report_descriptor_keyboard;
                slen = sizeof(usb_hid_report_descriptor_keyboard);
            }
            if (src && slen) {
                uint32_t n = slen;
                if (wLength && n > wLength) n = wLength;
                usb_write_descriptor(data_ptr, src, n);
            }
        } else if (b == 5) {
            uint8_t addr = wValue & 0x7F;
            for (int i = 1; i < USB_MAX_DEVICES; i++) {
                if (s->ehci.devices[i]) { s->ehci.devices[i]->address = addr; break; }
            }
        } else if (b == 9) {
            if (dev_addr && s->ehci.devices[dev_addr]) { s->ehci.devices[dev_addr]->configured = true; }
        } else if (b == 0x0A) {
            uint8_t idle = wValue >> 8;
            (void)idle;
        } else if (b == 0x0B) {
            uint8_t proto = wValue & 0xFF;
            (void)proto;
        }
    } else if ((bm & 0x60) == 0x20) {
        if (b == 1) {
            uint32_t n = wLength;
            if (n > sizeof(s->hid_report)) n = sizeof(s->hid_report);
            cpu_physical_memory_write(data_ptr, s->hid_report, n);
        } else if (b == 9) {
            uint8_t tmp[8];
            uint32_t n = wLength;
            if (n > sizeof(tmp)) n = sizeof(tmp);
            cpu_physical_memory_read(data_ptr, tmp, n);
            memcpy(s->hid_report, tmp, n);
        }
    }
}

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
    XenonUSBState *s = opaque;
    struct ehci *ehci = &s->ehci;
    
    if (!ehci->running) {
        return;
    }
    
    /* Increment frame number */
    ehci->frame_number++;
    if (ehci->frame_number >= 0x4000) {
        ehci->frame_number = 0;
    }
    ehci->frindex = ehci->frame_number;

    {
        hwaddr asynclist = ehci->asynclistaddr & ~0x1F;
        int qh_walk = 0;
        while (asynclist && qh_walk < 64) {
            uint32_t qh[8];
            cpu_physical_memory_read(asynclist, (uint8_t *)qh, sizeof(qh));
            hwaddr next = qh[0] & ~0x1F;
            hwaddr cur_qtd = qh[3] & ~0x1F;
            uint32_t devaddr = (qh[1] >> 0) & 0x7F;
            uint32_t epnum = (qh[1] >> 8) & 0xF;
            while (cur_qtd) {
                uint32_t qtd[8];
                cpu_physical_memory_read(cur_qtd, (uint8_t *)qtd, sizeof(qtd));
                uint32_t token = qtd[2];
                uint32_t bytes = (token >> 16) & 0x7FFF;
                uint32_t pid = (token >> 8) & 0x3;
                if (pid == 2) {
                    hwaddr setup_ptr = qtd[3] & ~0xFFF;
                    uint32_t nextptr = qtd[0] & ~0x1F;
                    hwaddr data_ptr = 0;
                    if (nextptr) {
                        uint32_t dn[8];
                        cpu_physical_memory_read(nextptr, (uint8_t *)dn, sizeof(dn));
                        data_ptr = dn[3] & ~0xFFF;
                    }
                    usb_handle_setup_ehci(s, devaddr, setup_ptr, data_ptr);
                }
                if (pid == 1 && epnum == 1 && bytes) {
                    uint32_t n = bytes;
                    if (n > sizeof(s->hid_report)) n = sizeof(s->hid_report);
                    hwaddr buf = qtd[3] & ~0xFFF;
                    cpu_physical_memory_write(buf, s->hid_report, n);
                }
                if (epnum == 2 && pid == 0 && bytes >= 31) {
                    uint8_t cbw[31];
                    hwaddr buf = qtd[3] & ~0xFFF;
                    cpu_physical_memory_read(buf, cbw, 31);
                    uint32_t sig = cbw[0] | (cbw[1] << 8) | (cbw[2] << 16) | (cbw[3] << 24);
                    if (sig == 0x43425355) {
                        s->msc_tag = cbw[4] | (cbw[5] << 8) | (cbw[6] << 16) | (cbw[7] << 24);
                        s->msc_expected_len = cbw[8] | (cbw[9] << 8) | (cbw[10] << 16) | (cbw[11] << 24);
                        s->msc_dir_in = (cbw[12] & 0x80) != 0;
                        s->msc_lun = cbw[13] & 0x0F;
                        s->msc_blk = (s->msc_lun < 2) ? s->msc_luns[s->msc_lun] : NULL;
                        s->msc_block_size = 512;
                        if (s->msc_blk) {
                            uint64_t size = blk_getlength(s->msc_blk);
                            s->msc_last_lba = (uint32_t)(size / s->msc_block_size) - 1;
                        } else {
                            s->msc_last_lba = 0;
                        }
                        uint8_t opcode = cbw[15];
                        s->msc_buf_len = 0;
                        s->msc_status = 0;
                        s->msc_opcode = opcode;
                        s->msc_written = 0;
                        s->msc_transferred = 0;
                        s->msc_sense_key = 0;
                        s->msc_asc = 0;
                        s->msc_ascq = 0;
                        s->msc_opcode = opcode;
                        s->msc_written = 0;
                        if (opcode == 0x12) {
                            uint8_t evpd = cbw[16];
                            uint8_t page = cbw[17];
                            if (evpd & 0x01) {
                                if (page == 0x00) {
                                    uint8_t vpd[8];
                                    memset(vpd, 0, sizeof(vpd));
                                    vpd[0] = 0x00; vpd[1] = 0x00; vpd[3] = 3;
                                    vpd[4] = 0x00; vpd[5] = 0x80; vpd[6] = 0x83;
                                    uint32_t n2 = sizeof(vpd);
                                    if (s->msc_expected_len && n2 > s->msc_expected_len) n2 = s->msc_expected_len;
                                    memcpy(s->msc_buf, vpd, n2);
                                    s->msc_buf_len = n2;
                                } else if (page == 0x80) {
                                    uint8_t vpd[32];
                                    memset(vpd, 0, sizeof(vpd));
                                    vpd[0] = 0x80; vpd[1] = 0x00; vpd[3] = 12;
                                    const char *ser = (s->msc_lun == 0) ? "XENONHDD0001" : "XENONMU0001";
                                    memcpy(vpd + 4, ser, 12);
                                    uint32_t n2 = 16;
                                    if (s->msc_expected_len && n2 > s->msc_expected_len) n2 = s->msc_expected_len;
                                    memcpy(s->msc_buf, vpd, n2);
                                    s->msc_buf_len = n2;
                                } else if (page == 0x83) {
                                    uint8_t vpd[64];
                                    memset(vpd, 0, sizeof(vpd));
                                    vpd[0] = 0x83; vpd[1] = 0x00; vpd[3] = 20;
                                    uint8_t *d = vpd + 4;
                                    d[0] = 0x01; d[1] = 0x01; d[3] = 16;
                                    const char *id = (s->msc_lun == 0) ? "MSFT-XENON-HDD" : "MSFT-XENON-MU";
                                    memcpy(d + 4, id, 12);
                                    uint32_t n2 = 24;
                                    if (s->msc_expected_len && n2 > s->msc_expected_len) n2 = s->msc_expected_len;
                                    memcpy(s->msc_buf, vpd, n2);
                                    s->msc_buf_len = n2;
                                } else {
                                    s->msc_buf_len = 0;
                                }
                            } else {
                                uint8_t inq[36];
                                memset(inq, 0, sizeof(inq));
                                inq[0] = 0x00;
                                inq[2] = 0x02;
                                inq[3] = 0x02;
                                inq[4] = 31;
                                const char *vend = "MSFT    ";
                                const char *prod = (s->msc_lun == 0) ? "Xenon HDD      " : "Xenon MU       ";
                                const char *rev = "0001";
                                memcpy(inq + 8, vend, 8);
                                memcpy(inq + 16, prod, 16);
                                memcpy(inq + 32, rev, 4);
                                uint32_t n2 = sizeof(inq);
                                if (s->msc_expected_len && n2 > s->msc_expected_len) n2 = s->msc_expected_len;
                                memcpy(s->msc_buf, inq, n2);
                                s->msc_buf_len = n2;
                            }
                        } else if (opcode == 0x25) {
                            uint8_t cap[8];
                            uint32_t last = s->msc_last_lba;
                            uint32_t blk = s->msc_block_size;
                            cap[0] = (last >> 24) & 0xFF; cap[1] = (last >> 16) & 0xFF; cap[2] = (last >> 8) & 0xFF; cap[3] = (last >> 0) & 0xFF;
                            cap[4] = (blk >> 24) & 0xFF; cap[5] = (blk >> 16) & 0xFF; cap[6] = (blk >> 8) & 0xFF; cap[7] = (blk >> 0) & 0xFF;
                            memcpy(s->msc_buf, cap, 8);
                            s->msc_buf_len = 8;
                        } else if (opcode == 0xA0) {
                            uint8_t rl[24];
                            memset(rl, 0, sizeof(rl));
                            rl[3] = 16;
                            rl[16] = 0x01;
                            uint32_t n2 = sizeof(rl);
                            if (s->msc_expected_len && n2 > s->msc_expected_len) n2 = s->msc_expected_len;
                            memcpy(s->msc_buf, rl, n2);
                            s->msc_buf_len = n2;
                        } else if (opcode == 0x1A) {
                            if (!s->msc_dir_in) { s->msc_status = 2; }
                            uint8_t page = cbw[16] & 0x3F;
                            uint8_t hdr[4]; memset(hdr, 0, sizeof(hdr));
                            uint8_t bdesc[8]; memset(bdesc, 0, sizeof(bdesc));
                            uint32_t blocks = s->msc_last_lba + 1;
                            bdesc[3] = (blocks >> 0) & 0xFF;
                            bdesc[2] = (blocks >> 8) & 0xFF;
                            bdesc[1] = (blocks >> 16) & 0xFF;
                            bdesc[0] = (blocks >> 24) & 0xFF;
                            bdesc[6] = (s->msc_block_size >> 8) & 0xFF;
                            bdesc[7] = (s->msc_block_size >> 0) & 0xFF;
                            hdr[0] = sizeof(bdesc);
                            uint8_t cache[12]; memset(cache, 0, sizeof(cache));
                            cache[0] = 0x08; cache[1] = 0x0A; /* caching page length */
                            cache[2] = 0x00; /* WCE=0 */
                            uint32_t n2 = sizeof(hdr) + sizeof(bdesc);
                            memcpy(s->msc_buf, hdr, sizeof(hdr));
                            memcpy(s->msc_buf + sizeof(hdr), bdesc, sizeof(bdesc));
                            if (page == 0x3F || page == 0x08) {
                                memcpy(s->msc_buf + n2, cache, sizeof(cache));
                                n2 += sizeof(cache);
                            }
                            if (s->msc_expected_len && n2 > s->msc_expected_len) n2 = s->msc_expected_len;
                            s->msc_buf_len = n2;
                        } else if (opcode == 0x5A) {
                            if (!s->msc_dir_in) { s->msc_status = 2; }
                            uint8_t page = cbw[17] & 0x3F;
                            uint8_t hdr10[8]; memset(hdr10, 0, sizeof(hdr10));
                            uint8_t bdesc10[16]; memset(bdesc10, 0, sizeof(bdesc10));
                            uint64_t blocks = (uint64_t)s->msc_last_lba + 1;
                            bdesc10[3] = (blocks >> 0) & 0xFF;
                            bdesc10[2] = (blocks >> 8) & 0xFF;
                            bdesc10[1] = (blocks >> 16) & 0xFF;
                            bdesc10[0] = (blocks >> 24) & 0xFF;
                            bdesc10[12] = (s->msc_block_size >> 24) & 0xFF;
                            bdesc10[13] = (s->msc_block_size >> 16) & 0xFF;
                            bdesc10[14] = (s->msc_block_size >> 8) & 0xFF;
                            bdesc10[15] = (s->msc_block_size >> 0) & 0xFF;
                            hdr10[0] = ((sizeof(bdesc10)) >> 8) & 0xFF;
                            hdr10[1] = ((sizeof(bdesc10)) >> 0) & 0xFF;
                            uint8_t cache[12]; memset(cache, 0, sizeof(cache)); cache[0] = 0x08; cache[1] = 0x0A; cache[2] = 0x00;
                            uint32_t n2 = sizeof(hdr10) + sizeof(bdesc10);
                            memcpy(s->msc_buf, hdr10, sizeof(hdr10));
                            memcpy(s->msc_buf + sizeof(hdr10), bdesc10, sizeof(bdesc10));
                            if (page == 0x3F || page == 0x08) {
                                memcpy(s->msc_buf + n2, cache, sizeof(cache));
                                n2 += sizeof(cache);
                            }
                            if (s->msc_expected_len && n2 > s->msc_expected_len) n2 = s->msc_expected_len;
                            s->msc_buf_len = n2;
                        } else if (opcode == 0x00) {
                            s->msc_buf_len = 0;
                        } else if (opcode == 0x1B) {
                            s->msc_buf_len = 0;
                        } else if (opcode == 0x1B) {
                            s->msc_buf_len = 0;
                        } else if (opcode == 0x03) {
                            uint8_t sense[18];
                            memset(sense, 0, sizeof(sense));
                            sense[0] = 0x70;
                            sense[2] = s->msc_sense_key;
                            sense[7] = 10;
                            sense[12] = s->msc_asc;
                            sense[13] = s->msc_ascq;
                            uint32_t n2 = sizeof(sense);
                            if (s->msc_expected_len && n2 > s->msc_expected_len) n2 = s->msc_expected_len;
                            memcpy(s->msc_buf, sense, n2);
                            s->msc_buf_len = n2;
                        } else if (opcode == 0x28) {
                            if (!s->msc_dir_in) {
                                s->msc_status = 2;
                                s->msc_sense_key = 0x05;
                                s->msc_asc = 0x24;
                                s->msc_ascq = 0x00;
                            }
                            uint32_t lba = (cbw[19] << 24) | (cbw[20] << 16) | (cbw[21] << 8) | cbw[22];
                            uint16_t cnt = (cbw[24] << 8) | cbw[25];
                            uint64_t end_lba = (uint64_t)lba + (uint64_t)cnt - 1;
                            if (!s->msc_blk || cnt == 0 || end_lba > (uint64_t)s->msc_last_lba) {
                                s->msc_status = 1;
                                s->msc_sense_key = 0x05;
                                s->msc_asc = 0x21;
                                s->msc_ascq = 0x00;
                                s->msc_buf_len = 0;
                                s->msc_read_remaining = 0;
                            } else {
                                s->msc_lba = lba;
                                s->msc_read_offset = 0;
                                s->msc_read_remaining = (uint32_t)cnt * s->msc_block_size;
                                uint32_t chunk = s->msc_read_remaining;
                                if (chunk > sizeof(s->msc_buf)) chunk = sizeof(s->msc_buf);
                                uint64_t off = (uint64_t)lba * s->msc_block_size;
                                int ret = blk_pread(s->msc_blk, off, s->msc_buf, chunk);
                                if (ret < 0) { s->msc_status = 1; s->msc_buf_len = 0; }
                                else { s->msc_buf_len = chunk; }
                            }
                        } else if (opcode == 0x2A) {
                            if (s->msc_dir_in) {
                                s->msc_status = 2;
                                s->msc_sense_key = 0x05;
                                s->msc_asc = 0x24;
                                s->msc_ascq = 0x00;
                            }
                            uint32_t lba = (cbw[19] << 24) | (cbw[20] << 16) | (cbw[21] << 8) | cbw[22];
                            uint16_t cnt = (cbw[24] << 8) | cbw[25];
                            uint64_t end_lba = (uint64_t)lba + (uint64_t)cnt - 1;
                            if (!s->msc_blk || cnt == 0 || end_lba > (uint64_t)s->msc_last_lba) {
                                s->msc_status = 1;
                                s->msc_sense_key = 0x05;
                                s->msc_asc = 0x21;
                                s->msc_ascq = 0x00;
                                s->msc_write_remaining = 0;
                                s->msc_buf_len = 0;
                            } else {
                                s->msc_lba = lba;
                                s->msc_write_remaining = (uint32_t)cnt * s->msc_block_size;
                                s->msc_buf_len = 0;
                            }
                        } else if (opcode == 0xA8) {
                            if (!s->msc_dir_in) { s->msc_status = 2; }
                            uint32_t lba = (cbw[19] << 24) | (cbw[20] << 16) | (cbw[21] << 8) | cbw[22];
                            uint32_t cnt = (cbw[23] << 24) | (cbw[24] << 16) | (cbw[25] << 8) | cbw[26];
                            uint64_t end_lba = (uint64_t)lba + (uint64_t)cnt - 1;
                            if (!s->msc_blk || cnt == 0 || end_lba > (uint64_t)s->msc_last_lba) {
                                s->msc_status = 1;
                                s->msc_sense_key = 0x05;
                                s->msc_asc = 0x21;
                                s->msc_ascq = 0x00;
                                s->msc_buf_len = 0;
                                s->msc_read_remaining = 0;
                            } else {
                                s->msc_lba = lba;
                                s->msc_read_offset = 0;
                                s->msc_read_remaining = cnt * s->msc_block_size;
                                uint32_t chunk = s->msc_read_remaining;
                                if (chunk > sizeof(s->msc_buf)) chunk = sizeof(s->msc_buf);
                                uint64_t off = (uint64_t)lba * s->msc_block_size;
                                int ret = blk_pread(s->msc_blk, off, s->msc_buf, chunk);
                                if (ret < 0) { s->msc_status = 1; s->msc_buf_len = 0; }
                                else { s->msc_buf_len = chunk; }
                            }
                        } else if (opcode == 0x88) {
                            if (!s->msc_dir_in) { s->msc_status = 2; }
                            uint64_t lba = ((uint64_t)cbw[19] << 56) | ((uint64_t)cbw[20] << 48) |
                                            ((uint64_t)cbw[21] << 40) | ((uint64_t)cbw[22] << 32) |
                                            ((uint64_t)cbw[23] << 24) | ((uint64_t)cbw[24] << 16) |
                                            ((uint64_t)cbw[25] << 8)  | ((uint64_t)cbw[26] << 0);
                            uint32_t cnt = (cbw[27] << 24) | (cbw[28] << 16) | (cbw[29] << 8) | cbw[30];
                            uint64_t end_lba = lba + (uint64_t)cnt - 1;
                            if (!s->msc_blk || cnt == 0 || end_lba > (uint64_t)s->msc_last_lba) {
                                s->msc_status = 1;
                                s->msc_sense_key = 0x05;
                                s->msc_asc = 0x21;
                                s->msc_ascq = 0x00;
                                s->msc_buf_len = 0;
                                s->msc_read_remaining = 0;
                            } else {
                                s->msc_lba = (uint32_t)lba;
                                s->msc_read_offset = 0;
                                s->msc_read_remaining = cnt * s->msc_block_size;
                                uint32_t chunk = s->msc_read_remaining;
                                if (chunk > sizeof(s->msc_buf)) chunk = sizeof(s->msc_buf);
                                uint64_t off = lba * s->msc_block_size;
                                int ret = blk_pread(s->msc_blk, off, s->msc_buf, chunk);
                                if (ret < 0) { s->msc_status = 1; s->msc_buf_len = 0; }
                                else { s->msc_buf_len = chunk; }
                            }
                        } else if (opcode == 0xAA) {
                            if (s->msc_dir_in) { s->msc_status = 2; }
                            uint32_t lba = (cbw[19] << 24) | (cbw[20] << 16) | (cbw[21] << 8) | cbw[22];
                            uint32_t cnt = (cbw[23] << 24) | (cbw[24] << 16) | (cbw[25] << 8) | cbw[26];
                            uint64_t end_lba = (uint64_t)lba + (uint64_t)cnt - 1;
                            if (!s->msc_blk || cnt == 0 || end_lba > (uint64_t)s->msc_last_lba) {
                                s->msc_status = 1;
                                s->msc_sense_key = 0x05;
                                s->msc_asc = 0x21;
                                s->msc_ascq = 0x00;
                                s->msc_write_remaining = 0;
                                s->msc_buf_len = 0;
                            } else {
                                s->msc_lba = lba;
                                s->msc_write_remaining = cnt * s->msc_block_size;
                                s->msc_buf_len = 0;
                            }
                        } else if (opcode == 0x8A) {
                            if (s->msc_dir_in) { s->msc_status = 2; }
                            uint64_t lba = ((uint64_t)cbw[19] << 56) | ((uint64_t)cbw[20] << 48) |
                                            ((uint64_t)cbw[21] << 40) | ((uint64_t)cbw[22] << 32) |
                                            ((uint64_t)cbw[23] << 24) | ((uint64_t)cbw[24] << 16) |
                                            ((uint64_t)cbw[25] << 8)  | ((uint64_t)cbw[26] << 0);
                            uint32_t cnt = (cbw[27] << 24) | (cbw[28] << 16) | (cbw[29] << 8) | cbw[30];
                            uint64_t end_lba = lba + (uint64_t)cnt - 1;
                            if (!s->msc_blk || cnt == 0 || end_lba > (uint64_t)s->msc_last_lba) {
                                s->msc_status = 1;
                                s->msc_sense_key = 0x05;
                                s->msc_asc = 0x21;
                                s->msc_ascq = 0x00;
                                s->msc_write_remaining = 0;
                                s->msc_buf_len = 0;
                            } else {
                                s->msc_lba = (uint32_t)lba; /* limited to 32-bit LBA for offset math */
                                s->msc_write_remaining = cnt * s->msc_block_size;
                                s->msc_buf_len = 0;
                            }
                        } else if (opcode == 0x15 || opcode == 0x55) {
                            if (s->msc_dir_in) { s->msc_status = 2; }
                            s->msc_buf_len = 0;
                        } else if (opcode == 0xA8) {
                            if (!s->msc_dir_in) { s->msc_status = 2; }
                            uint32_t lba = (cbw[19] << 24) | (cbw[20] << 16) | (cbw[21] << 8) | cbw[22];
                            uint32_t cnt = (cbw[23] << 24) | (cbw[24] << 16) | (cbw[25] << 8) | cbw[26];
                            uint64_t end_lba = (uint64_t)lba + (uint64_t)cnt - 1;
                            if (!s->msc_blk || cnt == 0 || end_lba > (uint64_t)s->msc_last_lba) {
                                s->msc_status = 1;
                                s->msc_sense_key = 0x05;
                                s->msc_asc = 0x21;
                                s->msc_ascq = 0x00;
                                s->msc_buf_len = 0;
                                s->msc_read_remaining = 0;
                            } else {
                                s->msc_lba = lba;
                                s->msc_read_offset = 0;
                                s->msc_read_remaining = cnt * s->msc_block_size;
                                uint32_t chunk = s->msc_read_remaining;
                                if (chunk > sizeof(s->msc_buf)) chunk = sizeof(s->msc_buf);
                                uint64_t off = (uint64_t)lba * s->msc_block_size;
                                int ret = blk_pread(s->msc_blk, off, s->msc_buf, chunk);
                                if (ret < 0) { s->msc_status = 1; s->msc_buf_len = 0; }
                                else { s->msc_buf_len = chunk; }
                            }
                        } else if (opcode == 0xAA) {
                            if (s->msc_dir_in) { s->msc_status = 2; }
                            uint32_t lba = (cbw[19] << 24) | (cbw[20] << 16) | (cbw[21] << 8) | cbw[22];
                            uint32_t cnt = (cbw[23] << 24) | (cbw[24] << 16) | (cbw[25] << 8) | cbw[26];
                            uint64_t end_lba = (uint64_t)lba + (uint64_t)cnt - 1;
                            if (!s->msc_blk || cnt == 0 || end_lba > (uint64_t)s->msc_last_lba) {
                                s->msc_status = 1;
                                s->msc_sense_key = 0x05;
                                s->msc_asc = 0x21;
                                s->msc_ascq = 0x00;
                                s->msc_write_remaining = 0;
                                s->msc_buf_len = 0;
                            } else {
                                s->msc_lba = lba;
                                s->msc_write_remaining = cnt * s->msc_block_size;
                                s->msc_buf_len = 0;
                            }
                        } else {
                            s->msc_status = 1;
                            s->msc_sense_key = 0x05;
                            s->msc_asc = 0x20;
                            s->msc_ascq = 0x00;
                        }
                    }
                }
                if (epnum == 2 && pid == 0 && (s->msc_opcode == 0x2A || s->msc_opcode == 0xAA || s->msc_opcode == 0x8A || s->msc_opcode == 0x0A) && bytes && s->msc_blk && s->msc_write_remaining) {
                    uint32_t n2 = bytes;
                    if (n2 > sizeof(s->msc_buf)) n2 = sizeof(s->msc_buf);
                    hwaddr buf = qtd[3] & ~0xFFF;
                    cpu_physical_memory_read(buf, s->msc_buf, n2);
                    uint64_t off = (uint64_t)s->msc_lba * s->msc_block_size + s->msc_written;
                    int ret = blk_pwrite(s->msc_blk, off, s->msc_buf, n2, 0);
                    if (ret < 0) { s->msc_status = 1; }
                    s->msc_written += n2;
                    s->msc_transferred += n2;
                    if (s->msc_write_remaining > n2) s->msc_write_remaining -= n2; else s->msc_write_remaining = 0;
                }
                if (epnum == 2 && pid == 1 && bytes) {
                    uint32_t n2 = bytes;
                    if (n2 > s->msc_buf_len) n2 = s->msc_buf_len;
                    hwaddr buf = qtd[3] & ~0xFFF;
                    cpu_physical_memory_write(buf, s->msc_buf, n2);
                    if (s->msc_buf_len >= n2) {
                        memmove(s->msc_buf, s->msc_buf + n2, s->msc_buf_len - n2);
                        s->msc_buf_len -= n2;
                    } else {
                        s->msc_buf_len = 0;
                    }
                    s->msc_transferred += n2;
                    if (s->msc_read_remaining > n2) s->msc_read_remaining -= n2; else s->msc_read_remaining = 0;
                    s->msc_read_offset += n2;
                    if (s->msc_buf_len == 0 && s->msc_read_remaining && s->msc_blk) {
                        uint32_t chunk = s->msc_read_remaining;
                        if (chunk > sizeof(s->msc_buf)) chunk = sizeof(s->msc_buf);
                        uint64_t off = (uint64_t)s->msc_lba * s->msc_block_size + s->msc_read_offset;
                        int ret = blk_pread(s->msc_blk, off, s->msc_buf, chunk);
                        if (ret >= 0) {
                            s->msc_buf_len = chunk;
                        }
                    }
                }
                if (epnum == 2 && pid == 1 && bytes == 13 && s->msc_tag) {
                    uint8_t csw[13];
                    uint32_t sig = 0x53425355;
                    csw[0] = sig & 0xFF; csw[1] = (sig >> 8) & 0xFF; csw[2] = (sig >> 16) & 0xFF; csw[3] = (sig >> 24) & 0xFF;
                    csw[4] = s->msc_tag & 0xFF; csw[5] = (s->msc_tag >> 8) & 0xFF; csw[6] = (s->msc_tag >> 16) & 0xFF; csw[7] = (s->msc_tag >> 24) & 0xFF;
                    uint32_t res = (s->msc_expected_len > s->msc_transferred) ? (s->msc_expected_len - s->msc_transferred) : 0;
                    csw[8] = res & 0xFF; csw[9] = (res >> 8) & 0xFF; csw[10] = (res >> 16) & 0xFF; csw[11] = (res >> 24) & 0xFF;
                    csw[12] = s->msc_status;
                    hwaddr buf = qtd[3] & ~0xFFF;
                    cpu_physical_memory_write(buf, csw, 13);
                    s->msc_tag = 0;
                }
                if (bytes) {
                    uint32_t remaining = bytes;
                    for (int bi = 0; bi < 5 && remaining; bi++) {
                        hwaddr buf = qtd[3 + bi] & ~0xFFF;
                        if (!buf) break;
                        uint32_t chunk = remaining;
                        if (pid == 1) {
                            uint8_t *tmp = g_malloc(chunk);
                            memset(tmp, 0, chunk);
                            cpu_physical_memory_write(buf, tmp, chunk);
                            g_free(tmp);
                        } else if (pid == 0) {
                            uint8_t *tmp = g_malloc(chunk);
                            cpu_physical_memory_read(buf, tmp, chunk);
                            g_free(tmp);
                        } else {
                            uint8_t setup[8];
                            cpu_physical_memory_read(buf, setup, MIN(chunk, (uint32_t)8));
                        }
                        remaining = 0;
                    }
                }
                token &= ~(1 << 7);
                token &= ~(0xFF << 16);
                qtd[2] = token;
                cpu_physical_memory_write(cur_qtd + 8, &qtd[2], 4);
                cur_qtd = qtd[0] & ~0x1F;
            }
            asynclist = next;
            qh_walk++;
        }
    }
    
    if (ehci->configflag) {
        int any_connected = 0;
        for (int i = 0; i < USB_MAX_PORTS; i++) {
            if (ehci->ports[i].status & USB_PORT_CONNECT) {
                any_connected = 1;
                break;
            }
        }
        if (any_connected) {
            ehci->usbsts |= 1;
            if (s->irq && (ehci->usbintr & 1)) {
                qemu_set_irq(s->irq, 1);
                qemu_set_irq(s->irq, 0);
            }
        }
    }

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
    XenonUSBState *s = opaque;
    struct ohci *ohci = &s->ohci;
    
    if (!ohci->running) {
        return;
    }
    
    /* Increment frame number */
    ohci->frame_number++;
    if (ohci->frame_number >= 0x4000) {
        ohci->frame_number = 0;
    }
    {
        hwaddr ed = ohci->hccontrolheaded & ~0xF;
        int ed_walk = 0;
        hwaddr last_done = 0;
        while (ed && ed_walk < 64) {
            uint32_t edw[4];
            cpu_physical_memory_read(ed, (uint8_t *)edw, sizeof(edw));
            uint32_t epnum = (edw[1] >> 7) & 0xF;
            uint32_t dir = (edw[1] >> 11) & 0x3;
            hwaddr headp = edw[2] & ~0xF;
            hwaddr tailp = edw[3] & ~0xF;
            while (headp && headp != tailp) {
                uint32_t tdw[4];
                cpu_physical_memory_read(headp, (uint8_t *)tdw, sizeof(tdw));
                hwaddr buf = tdw[2] & ~0xFFF;
                uint32_t len = (tdw[2] >> 16) & 0xFFFF;
                uint32_t pid = (tdw[3] >> 19) & 0x3;
                if (pid == 2) {
                    uint8_t setup[8];
                    cpu_physical_memory_read(buf, setup, 8);
                    uint8_t bm = setup[0];
                    uint8_t b = setup[1];
                    uint16_t wValue = setup[2] | (setup[3] << 8);
                    uint16_t wLength = setup[6] | (setup[7] << 8);
                    const uint8_t *src = NULL;
                    uint32_t slen = 0;
                    if ((bm & 0x60) == 0 && b == 6) {
                        uint8_t dtype = wValue >> 8;
                        uint8_t dindex = wValue & 0xFF;
                        if (dtype == 1) { src = usb_device_descriptor_hs; slen = sizeof(usb_device_descriptor_hs); }
                        else if (dtype == 2) { src = usb_configuration_descriptor_hs; slen = sizeof(usb_configuration_descriptor_hs); }
                        else if (dtype == 3) {
                            if (dindex == 0) { src = usb_string_langid; slen = sizeof(usb_string_langid); }
                            else if (dindex == 1) { src = usb_string_mfr; slen = usb_string_mfr[0]; }
                            else if (dindex == 2) { src = usb_string_prod; slen = usb_string_prod[0]; }
                            else if (dindex == 3) { src = usb_string_ser; slen = usb_string_ser[0]; }
                        } else if (dtype == 0x22) {
                            src = usb_hid_report_descriptor_keyboard;
                            slen = sizeof(usb_hid_report_descriptor_keyboard);
                        }
                    }
                    if (src && slen) {
                        uint32_t n = slen;
                        if (wLength && n > wLength) n = wLength;
                        cpu_physical_memory_write(buf, src, n);
                    }
                    if ((bm & 0x60) == 0x20 && b == 1) {
                        uint32_t n = wLength;
                        if (n > sizeof(s->hid_report)) n = sizeof(s->hid_report);
                        cpu_physical_memory_write(buf, s->hid_report, n);
                    }
                }
                if (pid == 1 && dir == 1 && epnum == 1 && len) {
                    uint32_t n = len;
                    if (n > sizeof(s->hid_report)) n = sizeof(s->hid_report);
                    cpu_physical_memory_write(buf, s->hid_report, n);
                }
                if (epnum == 2 && pid == 0 && len >= 31) {
                    uint8_t cbw[31];
                    cpu_physical_memory_read(buf, cbw, 31);
                    uint32_t sig = cbw[0] | (cbw[1] << 8) | (cbw[2] << 16) | (cbw[3] << 24);
                    if (sig == 0x43425355) {
                        s->msc_tag = cbw[4] | (cbw[5] << 8) | (cbw[6] << 16) | (cbw[7] << 24);
                        s->msc_expected_len = cbw[8] | (cbw[9] << 8) | (cbw[10] << 16) | (cbw[11] << 24);
                        s->msc_dir_in = (cbw[12] & 0x80) != 0;
                        s->msc_lun = cbw[13] & 0x0F;
                        s->msc_blk = (s->msc_lun < 2) ? s->msc_luns[s->msc_lun] : NULL;
                        s->msc_block_size = 512;
                        if (s->msc_blk) {
                            uint64_t size = blk_getlength(s->msc_blk);
                            s->msc_last_lba = (uint32_t)(size / s->msc_block_size) - 1;
                        } else {
                            s->msc_last_lba = 0;
                        }
                        uint8_t opcode = cbw[15];
                        s->msc_buf_len = 0;
                        s->msc_status = 0;
                        s->msc_opcode = opcode;
                        s->msc_transferred = 0;
                        s->msc_sense_key = 0;
                        s->msc_asc = 0;
                        s->msc_ascq = 0;
                        if (opcode == 0x12) {
                            uint8_t evpd = cbw[16];
                            uint8_t page = cbw[17];
                            if (evpd & 0x01) {
                                if (page == 0x00) {
                                    uint8_t vpd[8]; memset(vpd, 0, sizeof(vpd));
                                    vpd[0] = 0x00; vpd[1] = 0x00; vpd[3] = 3; vpd[4] = 0x00; vpd[5] = 0x80; vpd[6] = 0x83;
                                    uint32_t n2 = sizeof(vpd);
                                    if (s->msc_expected_len && n2 > s->msc_expected_len) n2 = s->msc_expected_len;
                                    memcpy(s->msc_buf, vpd, n2);
                                    s->msc_buf_len = n2;
                                } else if (page == 0x80) {
                                    uint8_t vpd[32]; memset(vpd, 0, sizeof(vpd));
                                    vpd[0] = 0x80; vpd[1] = 0x00; vpd[3] = 12;
                                    const char *ser = (s->msc_lun == 0) ? "XENONHDD0001" : "XENONMU0001";
                                    memcpy(vpd + 4, ser, 12);
                                    uint32_t n2 = 16; if (s->msc_expected_len && n2 > s->msc_expected_len) n2 = s->msc_expected_len;
                                    memcpy(s->msc_buf, vpd, n2); s->msc_buf_len = n2;
                                } else if (page == 0x83) {
                                    uint8_t vpd[64]; memset(vpd, 0, sizeof(vpd));
                                    vpd[0] = 0x83; vpd[1] = 0x00; vpd[3] = 20;
                                    uint8_t *d = vpd + 4; d[0] = 0x01; d[1] = 0x01; d[3] = 16;
                                    const char *id = (s->msc_lun == 0) ? "MSFT-XENON-HDD" : "MSFT-XENON-MU";
                                    memcpy(d + 4, id, 12);
                                    uint32_t n2 = 24; if (s->msc_expected_len && n2 > s->msc_expected_len) n2 = s->msc_expected_len;
                                    memcpy(s->msc_buf, vpd, n2); s->msc_buf_len = n2;
                                } else if (page == 0xB0) {
                                    uint8_t vpd[64]; memset(vpd, 0, sizeof(vpd));
                                    vpd[0] = 0xB0; vpd[1] = 0x00; vpd[3] = 0x3C;
                                    vpd[8] = 0x00; vpd[9] = 0x01; vpd[10] = 0x00; vpd[11] = 0x00; /* max transfer length */
                                    vpd[12] = 0x00; vpd[13] = 0x00; vpd[14] = 0x00; vpd[15] = 0x01; /* granularity */
                                    uint32_t n2 = 0x40; if (s->msc_expected_len && n2 > s->msc_expected_len) n2 = s->msc_expected_len;
                                    memcpy(s->msc_buf, vpd, n2); s->msc_buf_len = n2;
                                } else if (page == 0xB2) {
                                    uint8_t vpd[64]; memset(vpd, 0, sizeof(vpd));
                                    vpd[0] = 0xB2; vpd[1] = 0x00; vpd[3] = 0x3C; vpd[12] = 0x00; vpd[13] = 0x01;
                                    uint32_t n2 = 0x40; if (s->msc_expected_len && n2 > s->msc_expected_len) n2 = s->msc_expected_len;
                                    memcpy(s->msc_buf, vpd, n2); s->msc_buf_len = n2;
                                } else { s->msc_buf_len = 0; }
                            } else {
                                uint8_t inq[36]; memset(inq, 0, sizeof(inq));
                                inq[0] = 0x00; inq[2] = 0x02; inq[3] = 0x02; inq[4] = 31;
                                const char *vend = "MSFT    ";
                                const char *prod = (s->msc_lun == 0) ? "Xenon HDD      " : "Xenon MU       ";
                                const char *rev = "0001";
                                memcpy(inq + 8, vend, 8); memcpy(inq + 16, prod, 16); memcpy(inq + 32, rev, 4);
                                uint32_t n2 = sizeof(inq);
                                if (s->msc_expected_len && n2 > s->msc_expected_len) n2 = s->msc_expected_len;
                                memcpy(s->msc_buf, inq, n2); s->msc_buf_len = n2;
                            }
                        } else if (opcode == 0x25) {
                            uint8_t cap[8];
                            uint32_t last = s->msc_last_lba;
                            uint32_t blk = s->msc_block_size;
                            cap[0] = (last >> 24) & 0xFF; cap[1] = (last >> 16) & 0xFF; cap[2] = (last >> 8) & 0xFF; cap[3] = (last >> 0) & 0xFF;
                            cap[4] = (blk >> 24) & 0xFF; cap[5] = (blk >> 16) & 0xFF; cap[6] = (blk >> 8) & 0xFF; cap[7] = (blk >> 0) & 0xFF;
                            memcpy(s->msc_buf, cap, 8);
                            s->msc_buf_len = 8;
                        } else if (opcode == 0x1A) {
                            if (!s->msc_dir_in) { s->msc_status = 2; }
                            uint8_t page = cbw[16] & 0x3F;
                            uint8_t hdr[4]; memset(hdr, 0, sizeof(hdr));
                            uint8_t bdesc[8]; memset(bdesc, 0, sizeof(bdesc));
                            uint32_t blocks = s->msc_last_lba + 1;
                            bdesc[3] = (blocks >> 0) & 0xFF;
                            bdesc[2] = (blocks >> 8) & 0xFF;
                            bdesc[1] = (blocks >> 16) & 0xFF;
                            bdesc[0] = (blocks >> 24) & 0xFF;
                            bdesc[6] = (s->msc_block_size >> 8) & 0xFF;
                            bdesc[7] = (s->msc_block_size >> 0) & 0xFF;
                            hdr[0] = sizeof(bdesc);
                            uint8_t cache[12]; memset(cache, 0, sizeof(cache));
                            cache[0] = 0x08; cache[1] = 0x0A; cache[2] = 0x00;
                            uint32_t n2 = sizeof(hdr) + sizeof(bdesc);
                            memcpy(s->msc_buf, hdr, sizeof(hdr));
                            memcpy(s->msc_buf + sizeof(hdr), bdesc, sizeof(bdesc));
                            if (page == 0x3F || page == 0x08) {
                                memcpy(s->msc_buf + n2, cache, sizeof(cache));
                                n2 += sizeof(cache);
                            }
                            if (s->msc_expected_len && n2 > s->msc_expected_len) n2 = s->msc_expected_len;
                            s->msc_buf_len = n2;
                        } else if (opcode == 0x5A) {
                            if (!s->msc_dir_in) { s->msc_status = 2; }
                            uint8_t page = cbw[17] & 0x3F;
                            uint8_t hdr10[8]; memset(hdr10, 0, sizeof(hdr10));
                            uint8_t bdesc10[16]; memset(bdesc10, 0, sizeof(bdesc10));
                            uint64_t blocks = (uint64_t)s->msc_last_lba + 1;
                            bdesc10[3] = (blocks >> 0) & 0xFF;
                            bdesc10[2] = (blocks >> 8) & 0xFF;
                            bdesc10[1] = (blocks >> 16) & 0xFF;
                            bdesc10[0] = (blocks >> 24) & 0xFF;
                            bdesc10[12] = (s->msc_block_size >> 24) & 0xFF;
                            bdesc10[13] = (s->msc_block_size >> 16) & 0xFF;
                            bdesc10[14] = (s->msc_block_size >> 8) & 0xFF;
                            bdesc10[15] = (s->msc_block_size >> 0) & 0xFF;
                            hdr10[0] = ((sizeof(bdesc10)) >> 8) & 0xFF;
                            hdr10[1] = ((sizeof(bdesc10)) >> 0) & 0xFF;
                            uint8_t cache[12]; memset(cache, 0, sizeof(cache)); cache[0] = 0x08; cache[1] = 0x0A; cache[2] = 0x00;
                            uint32_t n2 = sizeof(hdr10) + sizeof(bdesc10);
                            memcpy(s->msc_buf, hdr10, sizeof(hdr10));
                            memcpy(s->msc_buf + sizeof(hdr10), bdesc10, sizeof(bdesc10));
                            if (page == 0x3F || page == 0x08) {
                                memcpy(s->msc_buf + n2, cache, sizeof(cache));
                                n2 += sizeof(cache);
                            }
                            if (s->msc_expected_len && n2 > s->msc_expected_len) n2 = s->msc_expected_len;
                            s->msc_buf_len = n2;
                        } else if (opcode == 0x00) {
                            s->msc_buf_len = 0;
                            if (s->msc_ua[s->msc_lun]) {
                                s->msc_status = 1;
                                s->msc_sense_key = 0x06;
                                s->msc_asc = 0x29;
                                s->msc_ascq = 0x00;
                            } else if (!s->msc_blk) {
                                s->msc_status = 1;
                                s->msc_sense_key = 0x02;
                                s->msc_asc = 0x04;
                                s->msc_ascq = 0x01;
                            }
                        } else if (opcode == 0x03) {
                            uint8_t sense[18];
                            memset(sense, 0, sizeof(sense));
                            sense[0] = 0x70;
                            sense[2] = s->msc_sense_key;
                            sense[7] = 10;
                            sense[12] = s->msc_asc;
                            sense[13] = s->msc_ascq;
                            uint32_t n2 = sizeof(sense);
                            if (s->msc_expected_len && n2 > s->msc_expected_len) n2 = s->msc_expected_len;
                            memcpy(s->msc_buf, sense, n2);
                            s->msc_buf_len = n2;
                            s->msc_ua[s->msc_lun] = false;
                            s->msc_sense_key = 0;
                            s->msc_asc = 0;
                            s->msc_ascq = 0;
                        } else if (opcode == 0x08) {
                            if (!s->msc_dir_in) { s->msc_status = 2; }
                            uint32_t lba = ((cbw[16] & 0x1F) << 16) | (cbw[17] << 8) | cbw[18];
                            uint32_t cnt = cbw[19] ? cbw[19] : 256;
                            uint64_t end_lba = (uint64_t)lba + (uint64_t)cnt - 1;
                            if (!s->msc_blk || end_lba > (uint64_t)s->msc_last_lba) {
                                s->msc_status = 1;
                                s->msc_sense_key = 0x05;
                                s->msc_asc = 0x21;
                                s->msc_ascq = 0x00;
                                s->msc_buf_len = 0;
                                s->msc_read_remaining = 0;
                            } else {
                                s->msc_lba = lba;
                                s->msc_read_offset = 0;
                                s->msc_read_remaining = cnt * s->msc_block_size;
                                uint32_t chunk = s->msc_read_remaining; if (chunk > sizeof(s->msc_buf)) chunk = sizeof(s->msc_buf);
                                uint64_t off = (uint64_t)lba * s->msc_block_size;
                                int ret = blk_pread(s->msc_blk, off, s->msc_buf, chunk);
                                if (ret < 0) { s->msc_status = 1; s->msc_buf_len = 0; } else { s->msc_buf_len = chunk; }
                            }
                        } else if (opcode == 0x0A) {
                            if (s->msc_dir_in) { s->msc_status = 2; }
                            uint32_t lba = ((cbw[16] & 0x1F) << 16) | (cbw[17] << 8) | cbw[18];
                            uint32_t cnt = cbw[19] ? cbw[19] : 256;
                            uint64_t end_lba = (uint64_t)lba + (uint64_t)cnt - 1;
                            if (!s->msc_blk || end_lba > (uint64_t)s->msc_last_lba) {
                                s->msc_status = 1;
                                s->msc_sense_key = 0x05;
                                s->msc_asc = 0x21;
                                s->msc_ascq = 0x00;
                                s->msc_write_remaining = 0; s->msc_buf_len = 0;
                            } else {
                                s->msc_lba = lba; s->msc_write_remaining = cnt * s->msc_block_size; s->msc_buf_len = 0;
                            }
                        } else if (opcode == 0x28) {
                            if (!s->msc_dir_in) {
                                s->msc_status = 2;
                                s->msc_sense_key = 0x05;
                                s->msc_asc = 0x24;
                                s->msc_ascq = 0x00;
                            }
                            uint32_t lba = (cbw[19] << 24) | (cbw[20] << 16) | (cbw[21] << 8) | cbw[22];
                            uint16_t cnt = (cbw[24] << 8) | cbw[25];
                            uint64_t end_lba = (uint64_t)lba + (uint64_t)cnt - 1;
                            if (!s->msc_blk || cnt == 0 || end_lba > (uint64_t)s->msc_last_lba) {
                                s->msc_status = 1;
                                s->msc_sense_key = 0x05;
                                s->msc_asc = 0x21;
                                s->msc_ascq = 0x00;
                                s->msc_buf_len = 0;
                                s->msc_read_remaining = 0;
                            } else {
                                s->msc_lba = lba;
                                s->msc_read_offset = 0;
                                s->msc_read_remaining = (uint32_t)cnt * s->msc_block_size;
                                uint32_t chunk = s->msc_read_remaining;
                                if (chunk > sizeof(s->msc_buf)) chunk = sizeof(s->msc_buf);
                                uint64_t off = (uint64_t)lba * s->msc_block_size;
                                int ret = blk_pread(s->msc_blk, off, s->msc_buf, chunk);
                                if (ret < 0) { s->msc_status = 1; s->msc_buf_len = 0; }
                                else { s->msc_buf_len = chunk; }
                            }
                        } else if (opcode == 0x2A) {
                            if (s->msc_dir_in) {
                                s->msc_status = 2;
                                s->msc_sense_key = 0x05;
                                s->msc_asc = 0x24;
                                s->msc_ascq = 0x00;
                            }
                            uint32_t lba = (cbw[19] << 24) | (cbw[20] << 16) | (cbw[21] << 8) | cbw[22];
                            uint16_t cnt = (cbw[24] << 8) | cbw[25];
                            uint64_t end_lba = (uint64_t)lba + (uint64_t)cnt - 1;
                            if (!s->msc_blk || cnt == 0 || end_lba > (uint64_t)s->msc_last_lba) {
                                s->msc_status = 1;
                                s->msc_sense_key = 0x05;
                                s->msc_asc = 0x21;
                                s->msc_ascq = 0x00;
                                s->msc_write_remaining = 0;
                                s->msc_buf_len = 0;
                            } else {
                                s->msc_lba = lba;
                                s->msc_write_remaining = (uint32_t)cnt * s->msc_block_size;
                                s->msc_buf_len = 0;
                            }
                        } else if (opcode == 0x1E) {
                            s->msc_buf_len = 0;
                        } else if (opcode == 0x2F) {
                            s->msc_buf_len = 0;
                        } else if (opcode == 0x35) {
                            s->msc_buf_len = 0;
                            if (s->msc_blk) {
                                blk_flush(s->msc_blk);
                            }
                        } else if (opcode == 0x91) {
                            s->msc_buf_len = 0; if (s->msc_blk) { blk_flush(s->msc_blk); }
                        } else if (opcode == 0x8F) {
                            s->msc_buf_len = 0;
                        } else if (opcode == 0x9E && cbw[16] == 0x10) {
                            uint8_t rc16[32]; memset(rc16, 0, sizeof(rc16));
                            uint64_t last = (uint64_t)s->msc_last_lba;
                            rc16[0] = (last >> 56) & 0xFF; rc16[1] = (last >> 48) & 0xFF; rc16[2] = (last >> 40) & 0xFF; rc16[3] = (last >> 32) & 0xFF;
                            rc16[4] = (last >> 24) & 0xFF; rc16[5] = (last >> 16) & 0xFF; rc16[6] = (last >> 8) & 0xFF; rc16[7] = (last >> 0) & 0xFF;
                            uint32_t bs = s->msc_block_size;
                            rc16[8] = (bs >> 24) & 0xFF; rc16[9] = (bs >> 16) & 0xFF; rc16[10] = (bs >> 8) & 0xFF; rc16[11] = (bs >> 0) & 0xFF;
                            uint32_t n2 = sizeof(rc16);
                            if (s->msc_expected_len && n2 > s->msc_expected_len) n2 = s->msc_expected_len;
                            memcpy(s->msc_buf, rc16, n2);
                            s->msc_buf_len = n2;
                        } else {
                            s->msc_status = 1;
                            s->msc_sense_key = 0x05;
                            s->msc_asc = 0x20;
                            s->msc_ascq = 0x00;
                        }
                    }
                }
                if (epnum == 2 && pid == 0 && (s->msc_opcode == 0x2A || s->msc_opcode == 0xAA || s->msc_opcode == 0x8A) && len && s->msc_blk && s->msc_write_remaining) {
                    uint32_t n3 = len;
                    if (n3 > sizeof(s->msc_buf)) n3 = sizeof(s->msc_buf);
                    cpu_physical_memory_read(buf, s->msc_buf, n3);
                    uint64_t off = (uint64_t)s->msc_lba * s->msc_block_size + s->msc_written;
                    int ret = blk_pwrite(s->msc_blk, off, s->msc_buf, n3, 0);
                    if (ret < 0) { s->msc_status = 1; }
                    s->msc_written += n3;
                    s->msc_transferred += n3;
                    if (s->msc_write_remaining > n3) s->msc_write_remaining -= n3; else s->msc_write_remaining = 0;
                }
                if (epnum == 2 && pid == 1 && len) {
                    uint32_t n3 = len;
                    if (n3 > s->msc_buf_len) n3 = s->msc_buf_len;
                    cpu_physical_memory_write(buf, s->msc_buf, n3);
                    if (s->msc_buf_len >= n3) {
                        memmove(s->msc_buf, s->msc_buf + n3, s->msc_buf_len - n3);
                        s->msc_buf_len -= n3;
                    } else {
                        s->msc_buf_len = 0;
                    }
                    s->msc_transferred += n3;
                    if (s->msc_read_remaining > n3) s->msc_read_remaining -= n3; else s->msc_read_remaining = 0;
                    s->msc_read_offset += n3;
                    if (s->msc_buf_len == 0 && s->msc_read_remaining && s->msc_blk) {
                        uint32_t chunk = s->msc_read_remaining;
                        if (chunk > sizeof(s->msc_buf)) chunk = sizeof(s->msc_buf);
                        uint64_t off = (uint64_t)s->msc_lba * s->msc_block_size + s->msc_read_offset;
                        int ret = blk_pread(s->msc_blk, off, s->msc_buf, chunk);
                        if (ret >= 0) {
                            s->msc_buf_len = chunk;
                        }
                    }
                }
                if (epnum == 2 && pid == 1 && len == 13 && s->msc_tag) {
                    uint8_t csw[13];
                    uint32_t sig = 0x53425355;
                    csw[0] = sig & 0xFF; csw[1] = (sig >> 8) & 0xFF; csw[2] = (sig >> 16) & 0xFF; csw[3] = (sig >> 24) & 0xFF;
                    csw[4] = s->msc_tag & 0xFF; csw[5] = (s->msc_tag >> 8) & 0xFF; csw[6] = (s->msc_tag >> 16) & 0xFF; csw[7] = (s->msc_tag >> 24) & 0xFF;
                    uint32_t res = (s->msc_expected_len > s->msc_transferred) ? (s->msc_expected_len - s->msc_transferred) : 0;
                    csw[8] = res & 0xFF; csw[9] = (res >> 8) & 0xFF; csw[10] = (res >> 16) & 0xFF; csw[11] = (res >> 24) & 0xFF;
                    csw[12] = s->msc_status;
                    cpu_physical_memory_write(buf, csw, 13);
                    s->msc_tag = 0;
                }
                last_done = headp;
                headp = tdw[0] & ~0xF;
                edw[2] = (edw[2] & 0xF) | headp;
                cpu_physical_memory_write(ed + 8, &edw[2], 4);
            }
            ed = edw[0] & ~0xF;
            ed_walk++;
        }
        if (last_done) {
            ohci->hcdonehead = last_done;
        }
    }

    int any_connected = 0;
    for (int i = 0; i < USB_MAX_PORTS; i++) {
        if (ohci->ports[i].status & USB_PORT_CONNECT) {
            any_connected = 1;
            break;
        }
    }
    if (any_connected) {
        ohci->hcstatus |= 1;
        if (s->irq && (ohci->hcinterruptenable & 1)) {
            qemu_set_irq(s->irq, 1);
            qemu_set_irq(s->irq, 0);
        }
    }
    
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
                                       usb_ehci_frame_timer, s);
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
                                       usb_ohci_frame_timer, s);
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
    s->irq = NULL;
    
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

void xenon_usb_attach_msc_backend(XenonUSBState *s, BlockBackend *blk) {
    xenon_usb_attach_msc_backend_lun(s, 0, blk);
}

void xenon_usb_attach_msc_backend_lun(XenonUSBState *s, int lun, BlockBackend *blk) {
    if (lun < 0 || lun >= 2) return;
    s->msc_luns[lun] = blk;
    if (blk) {
        uint64_t size = blk_getlength(blk);
        printf("[USB MSC] LUN %d backend attached: size=%" PRIu64 " bytes\n", lun, size);
        s->msc_ua[lun] = true;
    } else {
        printf("[USB MSC] LUN %d backend detached\n", lun);
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
static void xenon_usb_irq_handler(void *opaque, int n, int level) {
    XenonUSBState *s = opaque;
    if (!s->gic) {
        return;
    }
    if (level) {
        xenon_gic_assert_irq(s->gic, s->gic_irq, 0);
    } else {
        xenon_gic_deassert_irq(s->gic, s->gic_irq, 0);
    }
}

void xenon_usb_connect_irq(XenonUSBState *s, XenonGICState *gic, int irq_num) {
    s->gic = gic;
    s->gic_irq = irq_num;
    s->irq = qemu_allocate_irq(xenon_usb_irq_handler, s, 0);
}
