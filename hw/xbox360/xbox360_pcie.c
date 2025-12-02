#include "qemu/osdep.h"
#include "hw/xbox360/xbox360_pcie.h"
#include "hw/xbox360/xbox360.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_bridge.h"
#include "hw/pci/pci_host.h"
#include "hw/pci/msi.h"
#include "hw/irq.h"
#include "qemu/range.h"
#include "qemu/timer.h"
#include "migration/vmstate.h"

/* ==================== XENON PCIe CONFIGURATION ==================== */
#define PCIE_BASE_ADDRESS         0x80002000
#define PCIE_REGISTER_SIZE        0x2000

#define PCIE_MAX_DEVICES          32
#define PCIE_MAX_FUNCTIONS        8
#define PCIE_CONFIG_SIZE          0x1000

/* Xenon PCIe Controller Registers */
#define PCIE_REG_VERSION          0x0000
#define PCIE_REG_STATUS           0x0004
#define PCIE_REG_CONTROL          0x0008
#define PCIE_REG_CONFIG_ADDR      0x000C
#define PCIE_REG_CONFIG_DATA      0x0010
#define PCIE_REG_MSI_ADDR         0x0014
#define PCIE_REG_MSI_DATA         0x0018
#define PCIE_REG_MSI_MASK         0x001C
#define PCIE_REG_MSI_PENDING      0x0020
#define PCIE_REG_IRQ_STATUS       0x0024
#define PCIE_REG_IRQ_MASK         0x0028
#define PCIE_REG_IRQ_CLEAR        0x002C
#define PCIE_REG_MSI_TRIGGER      0x0030

/* PCIe Port Registers */
#define PCIE_PORT_BASE            0x0100
#define PCIE_PORT_SIZE            0x0040
#define PCIE_PORT_CTRL(n)         (PCIE_PORT_BASE + (n) * PCIE_PORT_SIZE + 0x00)
#define PCIE_PORT_STATUS(n)       (PCIE_PORT_BASE + (n) * PCIE_PORT_SIZE + 0x04)
#define PCIE_PORT_LINK_CTRL(n)    (PCIE_PORT_BASE + (n) * PCIE_PORT_SIZE + 0x08)
#define PCIE_PORT_LINK_STATUS(n)  (PCIE_PORT_BASE + (n) * PCIE_PORT_SIZE + 0x0C)

/* Xenon-specific PCIe ports */
#define PCIE_PORT_GPU             0
#define PCIE_PORT_EHCI            1  /* USB */
#define PCIE_PORT_OHCI            2  /* USB */
#define PCIE_PORT_SATA            3  /* HDD/DVD */
#define PCIE_PORT_SMBUS           4  /* System Management */
#define PCIE_PORT_AUDIO           5  /* XMA */
#define PCIE_PORT_ETHERNET        6  /* XNet */
#define PCIE_PORT_BRIDGE          7  /* Internal bridge */

/* PCIe Device IDs */
#define PCIE_VENDOR_ID_MICROSOFT  0x1414
#define PCIE_DEVICE_ID_XENON_PCIE 0x0001
#define PCIE_DEVICE_ID_GPU        0x0218
#define PCIE_DEVICE_ID_EHCI       0x0034
#define PCIE_DEVICE_ID_OHCI       0x0035
#define PCIE_DEVICE_ID_SATA       0x0056
#define PCIE_DEVICE_ID_SMBUS      0x0057
#define PCIE_DEVICE_ID_AUDIO      0x0058
#define PCIE_DEVICE_ID_ETHERNET   0x0059

/* ==================== PCIe DEVICE STRUCTURES ==================== */

typedef struct XenonPCIeDevice {
    PCIDevice pci_dev;
    uint32_t port_index;
    uint32_t config[256];  /* Configuration space */
    uint32_t msi_addr;
    uint32_t msi_data;
    uint32_t msi_mask;
    uint32_t msi_pending;
    bool msi_enabled;
} XenonPCIeDevice;

typedef struct XenonPCIePort {
    uint32_t control;
    uint32_t status;
    uint32_t link_control;
    uint32_t link_status;
    XenonPCIeDevice *device;
    bool enabled;
    bool link_up;
    uint8_t lane_count;
    uint32_t speed;  /* 1=2.5GT/s, 2=5GT/s */
} XenonPCIePort;

/* ==================== PCIe CONTROLLER STATE ==================== */

typedef struct XenonPCIeState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    MemoryRegion config_mem;
    MemoryRegion mmio_mem;
    
    /* Registers */
    uint32_t version;
    uint32_t status;
    uint32_t control;
    uint32_t config_addr;
    uint32_t config_data;
    uint32_t msi_addr;
    uint32_t msi_data;
    uint32_t msi_mask;
    uint32_t msi_pending;
    uint32_t irq_status;
    uint32_t irq_mask;
    
    /* Ports */
    XenonPCIePort ports[8];
    
    /* Devices */
    XenonPCIeDevice *devices[PCIE_MAX_DEVICES];
    int device_count;
    
    /* Interrupts */
    qemu_irq irq;
    qemu_irq msi_irq;
    
    /* PCI bus */
    PCIBus bus;
    
    /* DMA engine */
    struct {
        uint32_t control;
        uint32_t status;
        uint32_t source_addr;
        uint32_t dest_addr;
        uint32_t transfer_size;
        uint32_t command;
        QEMUTimer *timer;
        bool active;
    } dma;
    
    /* Xenon-specific */
    uint32_t xenon_features;
    uint32_t debug_ctrl;
} XenonPCIeState;

/* ==================== REGISTER ACCESS ==================== */

static uint64_t pcie_read(void *opaque, hwaddr offset, unsigned size) {
    XenonPCIeState *s = opaque;
    uint32_t value = 0;
    int port;
    
    if (offset >= PCIE_PORT_BASE && offset < PCIE_PORT_BASE + 8 * PCIE_PORT_SIZE) {
        port = (offset - PCIE_PORT_BASE) / PCIE_PORT_SIZE;
        offset = (offset - PCIE_PORT_BASE) % PCIE_PORT_SIZE;
        
        if (port < 8) {
            XenonPCIePort *p = &s->ports[port];
            switch (offset) {
                case 0x00: value = p->control; break;
                case 0x04: value = p->status; break;
                case 0x08: value = p->link_control; break;
                case 0x0C: value = p->link_status; break;
            }
        }
        return value;
    }
    
    switch (offset) {
        case PCIE_REG_VERSION:
            value = 0x00010000;  /* Version 1.0 */
            break;
        case PCIE_REG_STATUS:
            value = s->status;
            break;
        case PCIE_REG_CONTROL:
            value = s->control;
            break;
        case PCIE_REG_CONFIG_ADDR:
            value = s->config_addr;
            break;
        case PCIE_REG_CONFIG_DATA:
            value = pcie_config_read(s, s->config_addr);
            break;
        case PCIE_REG_MSI_ADDR:
            value = s->msi_addr;
            break;
        case PCIE_REG_MSI_DATA:
            value = s->msi_data;
            break;
        case PCIE_REG_MSI_MASK:
            value = s->msi_mask;
            break;
        case PCIE_REG_MSI_PENDING:
            value = s->msi_pending;
            break;
        case PCIE_REG_IRQ_STATUS:
            value = s->irq_status;
            break;
        case PCIE_REG_IRQ_MASK:
            value = s->irq_mask;
            break;
        default:
            break;
    }
    
    return value;
}

static void pcie_write(void *opaque, hwaddr offset, 
                      uint64_t value, unsigned size) {
    XenonPCIeState *s = opaque;
    int port;
    
    if (offset >= PCIE_PORT_BASE && offset < PCIE_PORT_BASE + 8 * PCIE_PORT_SIZE) {
        port = (offset - PCIE_PORT_BASE) / PCIE_PORT_SIZE;
        offset = (offset - PCIE_PORT_BASE) % PCIE_PORT_SIZE;
        
        if (port < 8) {
            XenonPCIePort *p = &s->ports[port];
            switch (offset) {
                case 0x00:
                    p->control = value;
                    p->enabled = (value & 1) != 0;
                    if (value & 2) {
                        pcie_port_reset(p);
                    }
                    break;
                case 0x04:
                    p->status = value & 0xFFFF;
                    break;
                case 0x08:
                    p->link_control = value & 0xFFFF;
                    p->lane_count = (value >> 4) & 0x1F;
                    p->speed = (value >> 8) & 0x7;
                    break;
                case 0x0C:
                    p->link_status = value & 0xFFFF;
                    p->link_up = (value & 1) != 0;
                    break;
            }
        }
        return;
    }
    
    switch (offset) {
        case PCIE_REG_CONTROL:
            s->control = value;
            if (value & 1) {
                pcie_reset(s);
            }
            break;
        case PCIE_REG_CONFIG_ADDR:
            s->config_addr = value;
            break;
        case PCIE_REG_CONFIG_DATA:
            pcie_config_write(s, s->config_addr, value);
            break;
        case PCIE_REG_MSI_ADDR:
            s->msi_addr = value;
            break;
        case PCIE_REG_MSI_DATA:
            s->msi_data = value;
            break;
        case PCIE_REG_MSI_MASK:
            s->msi_mask = value;
            break;
        case PCIE_REG_MSI_PENDING:
            s->msi_pending &= ~value;  /* Clear pending bits */
            break;
        case PCIE_REG_IRQ_STATUS:
            s->irq_status = value;
            if (s->irq_status & s->irq_mask) {
                qemu_set_irq(s->irq, 1);
            } else {
                qemu_set_irq(s->irq, 0);
            }
            break;
        case PCIE_REG_IRQ_MASK:
            s->irq_mask = value;
            break;
        case PCIE_REG_IRQ_CLEAR:
            s->irq_status &= ~value;
            if (!(s->irq_status & s->irq_mask)) {
                qemu_set_irq(s->irq, 0);
            }
            break;
        case PCIE_REG_MSI_TRIGGER:
            pcie_trigger_msi(s, value);
            break;
    }
}

static const MemoryRegionOps pcie_ops = {
    .read = pcie_read,
    .write = pcie_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

/* ==================== CONFIGURATION SPACE ACCESS ==================== */

static uint32_t pcie_config_read(XenonPCIeState *s, uint32_t addr) {
    uint32_t bus = (addr >> 16) & 0xFF;
    uint32_t dev = (addr >> 11) & 0x1F;
    uint32_t func = (addr >> 8) & 0x7;
    uint32_t reg = addr & 0xFF;
    
    if (bus != 0) {
        return 0xFFFFFFFF;  /* Only bus 0 supported */
    }
    
    /* Find device */
    for (int i = 0; i < s->device_count; i++) {
        XenonPCIeDevice *d = s->devices[i];
        if (d->pci_dev.devfn == PCI_DEVFN(dev, func)) {
            if (reg < 256) {
                return d->config[reg / 4];
            }
            break;
        }
    }
    
    return 0xFFFFFFFF;
}

static void pcie_config_write(XenonPCIeState *s, uint32_t addr, uint32_t value) {
    uint32_t bus = (addr >> 16) & 0xFF;
    uint32_t dev = (addr >> 11) & 0x1F;
    uint32_t func = (addr >> 8) & 0x7;
    uint32_t reg = addr & 0xFF;
    
    if (bus != 0) {
        return;  /* Only bus 0 supported */
    }
    
    /* Find device */
    for (int i = 0; i < s->device_count; i++) {
        XenonPCIeDevice *d = s->devices[i];
        if (d->pci_dev.devfn == PCI_DEVFN(dev, func)) {
            if (reg < 256) {
                int idx = reg / 4;
                
                /* Handle BAR writes */
                if (reg >= 0x10 && reg < 0x28) {
                    pcie_handle_bar_write(d, reg, value);
                }
                /* Handle command register */
                else if (reg == 0x04) {
                    d->config[idx] = value & 0xFFFF;
                }
                /* Handle MSI capability */
                else if (reg >= 0x50 && reg < 0x60) {
                    pcie_handle_msi_write(d, reg, value);
                }
                else {
                    d->config[idx] = value;
                }
            }
            break;
        }
    }
}

/* ==================== PCIe DEVICE MANAGEMENT ==================== */

static XenonPCIeDevice *pcie_create_device(XenonPCIeState *s, 
                                          uint32_t vendor_id, uint32_t device_id,
                                          uint32_t class_code, uint8_t dev, uint8_t func) {
    XenonPCIeDevice *d = g_new0(XenonPCIeDevice, 1);
    
    /* Initialize PCI device */
    pci_config_set_vendor_id(d->config, vendor_id);
    pci_config_set_device_id(d->config, device_id);
    pci_config_set_class(d->config, class_code);
    pci_config_set_revision(d->config, 0x00);
    
    /* Set up device function */
    d->pci_dev.devfn = PCI_DEVFN(dev, func);
    
    /* Add to device list */
    s->devices[s->device_count++] = d;
    
    return d;
}

static void pcie_handle_bar_write(XenonPCIeDevice *d, uint32_t reg, uint32_t value) {
    int bar_idx = (reg - 0x10) / 4;
    
    if (value == 0xFFFFFFFF) {
        /* BAR size probe */
        uint32_t size_mask = 0;
        switch (d->config[0x10/4 + bar_idx] & 1) {
            case 0:  /* 32-bit memory */
                size_mask = 0xFFFFFFF0;
                break;
            case 1:  /* I/O space */
                size_mask = 0xFFFFFFFC;
                break;
        }
        d->config[0x10/4 + bar_idx] = size_mask;
    } else {
        d->config[0x10/4 + bar_idx] = value;
    }
}

static void pcie_handle_msi_write(XenonPCIeDevice *d, uint32_t reg, uint32_t value) {
    int offset = reg - 0x50;
    
    switch (offset) {
        case 0x00:  /* MSI Capability ID */
            break;
        case 0x01:  /* MSI Control */
            d->msi_enabled = (value & 1) != 0;
            break;
        case 0x02:  /* MSI Address Low */
            d->msi_addr = (d->msi_addr & 0xFFFFFFFF00000000) | value;
            break;
        case 0x03:  /* MSI Address High */
            d->msi_addr = (d->msi_addr & 0xFFFFFFFF) | ((uint64_t)value << 32);
            break;
        case 0x04:  /* MSI Data */
            d->msi_data = value & 0xFFFF;
            break;
    }
}

/* ==================== MSI SUPPORT ==================== */

static void pcie_trigger_msi(XenonPCIeState *s, uint32_t msi_vector) {
    if (!(s->msi_mask & (1 << msi_vector))) {
        s->msi_pending |= (1 << msi_vector);
        
        /* Trigger MSI interrupt */
        if (s->msi_addr && s->msi_data) {
            /* Write MSI data to MSI address */
            cpu_physical_memory_write(s->msi_addr, &s->msi_data, sizeof(s->msi_data));
            qemu_set_irq(s->msi_irq, 1);
            qemu_set_irq(s->msi_irq, 0);
        }
    }
}

static void pcie_send_msi(XenonPCIeDevice *d, uint32_t vector) {
    if (d->msi_enabled && d->msi_addr) {
        /* Write MSI data to address */
        uint32_t data = d->msi_data | vector;
        cpu_physical_memory_write(d->msi_addr, &data, sizeof(data));
    }
}

/* ==================== PORT MANAGEMENT ==================== */

static void pcie_port_reset(XenonPCIePort *port) {
    port->control = 0;
    port->status = 0;
    port->link_control = 0;
    port->link_status = 0;
    port->enabled = false;
    port->link_up = false;
    port->lane_count = 1;
    port->speed = 1;  /* 2.5GT/s */
    
    if (port->device) {
        /* Reset connected device */
        memset(port->device->config, 0, 256);
    }
}

static void pcie_port_initialize(XenonPCIePort *port, uint32_t port_type) {
    port->control = 0x00000001;  /* Enable */
    port->status = 0x0010;       /* Link training */
    port->link_control = 0x0011; /* 1 lane, 2.5GT/s */
    port->link_status = 0x0001;  /* Link up */
    port->enabled = true;
    port->link_up = true;
    port->lane_count = 1;
    port->speed = 1;
    
    /* Set up port based on type */
    switch (port_type) {
        case PCIE_PORT_GPU:
            port->lane_count = 16;
            port->speed = 2;  /* 5GT/s for GPU */
            break;
        case PCIE_PORT_SATA:
            port->lane_count = 4;
            break;
        case PCIE_PORT_ETHERNET:
            port->lane_count = 1;
            break;
    }
}

/* ==================== DMA ENGINE ==================== */

static void dma_transfer_complete(void *opaque) {
    XenonPCIeState *s = opaque;
    
    s->dma.active = false;
    s->dma.status |= 0x00000001;  /* Transfer complete */
    s->dma.status &= ~0x00000002; /* Clear busy */
    
    /* Trigger interrupt */
    s->irq_status |= 0x00000002;  /* DMA complete interrupt */
    if (s->irq_status & s->irq_mask) {
        qemu_set_irq(s->irq, 1);
    }
}

static void dma_start_transfer(XenonPCIeState *s) {
    uint32_t src = s->dma.source_addr;
    uint32_t dst = s->dma.dest_addr;
    uint32_t size = s->dma.transfer_size;
    
    if (s->dma.active || size == 0) {
        return;
    }
    
    s->dma.active = true;
    s->dma.status |= 0x00000002;  /* Busy */
    s->dma.status &= ~0x00000001; /* Clear complete */
    
    /* Perform DMA transfer */
    uint8_t *buffer = g_malloc(size);
    
    /* Read from source */
    cpu_physical_memory_read(src, buffer, size);
    
    /* Write to destination */
    cpu_physical_memory_write(dst, buffer, size);
    
    g_free(buffer);
    
    /* Schedule completion */
    int64_t delay_ns = (size / 1024) * 100;  /* Simplified timing */
    timer_mod_ns(s->dma.timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + delay_ns);
}

/* ==================== DEVICE INITIALIZATION ==================== */

static void xenon_pcie_realize(DeviceState *dev, Error **errp) {
    XenonPCIeState *s = XENON_PCIE(dev);
    
    /* Initialize registers */
    s->version = 0x00010000;
    s->status = 0x00000000;
    s->control = 0x00000001;  /* Enabled */
    s->irq_mask = 0xFFFFFFFF;
    
    /* Initialize ports */
    for (int i = 0; i < 8; i++) {
        pcie_port_initialize(&s->ports[i], i);
    }
    
    /* Create GPU device on port 0 */
    XenonPCIeDevice *gpu_dev = pcie_create_device(s,
        PCIE_VENDOR_ID_MICROSOFT, PCIE_DEVICE_ID_GPU,
        PCI_CLASS_DISPLAY_VGA, 0, 0);
    
    /* Set up GPU BARs */
    gpu_dev->config[0x10/4] = 0x00000000;  /* BAR0: MMIO */
    gpu_dev->config[0x14/4] = 0x00000001;  /* BAR1: VRAM */
    gpu_dev->config[0x18/4] = 0x00000000;  /* BAR2: ROM */
    
    /* Set up MSI */
    gpu_dev->msi_addr = 0x80003000;
    gpu_dev->msi_data = 0x0000;
    gpu_dev->msi_enabled = true;
    
    s->ports[PCIE_PORT_GPU].device = gpu_dev;
    
    /* Create SATA controller on port 3 */
    XenonPCIeDevice *sata_dev = pcie_create_device(s,
        PCIE_VENDOR_ID_MICROSOFT, PCIE_DEVICE_ID_SATA,
        PCI_CLASS_STORAGE_SATA, 3, 0);
    
    /* Create audio device on port 5 */
    XenonPCIeDevice *audio_dev = pcie_create_device(s,
        PCIE_VENDOR_ID_MICROSOFT, PCIE_DEVICE_ID_AUDIO,
        PCI_CLASS_MULTIMEDIA_AUDIO, 5, 0);
    
    /* Create Ethernet device on port 6 */
    XenonPCIeDevice *net_dev = pcie_create_device(s,
        PCIE_VENDOR_ID_MICROSOFT, PCIE_DEVICE_ID_ETHERNET,
        PCI_CLASS_NETWORK_ETHERNET, 6, 0);
    
    /* Initialize DMA engine */
    s->dma.control = 0;
    s->dma.status = 0;
    s->dma.active = false;
    s->dma.timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, dma_transfer_complete, s);
    
    /* Initialize memory regions */
    memory_region_init_io(&s->iomem, OBJECT(s), &pcie_ops, s,
                         "xenon.pcie", PCIE_REGISTER_SIZE);
    
    memory_region_init_io(&s->config_mem, OBJECT(s), &pcie_config_ops, s,
                         "xenon.pcie.config", PCIE_CONFIG_SIZE);
    
    /* Initialize PCI bus */
    pci_bus_init(&s->bus, sizeof(s->bus), DEVICE(s), "pcie.0",
                 NULL, 0, TYPE_PCI_BUS);
    
    printf("[PCIE] Xenon PCIe Controller initialized\n");
    printf("[PCIE] Ports: GPU@%d, SATA@%d, Audio@%d, Ethernet@%d\n",
           PCIE_PORT_GPU, PCIE_PORT_SATA, PCIE_PORT_AUDIO, PCIE_PORT_ETHERNET);
}

static void xenon_pcie_reset(DeviceState *dev) {
    XenonPCIeState *s = XENON_PCIE(dev);
    
    /* Reset controller */
    s->control = 0x00000001;
    s->status = 0;
    s->irq_status = 0;
    s->msi_pending = 0;
    
    /* Reset ports */
    for (int i = 0; i < 8; i++) {
        pcie_port_reset(&s->ports[i]);
    }
    
    /* Reset DMA */
    s->dma.control = 0;
    s->dma.status = 0;
    s->dma.active = false;
    timer_del(s->dma.timer);
    
    qemu_set_irq(s->irq, 0);
    qemu_set_irq(s->msi_irq, 0);
}

/* ==================== QEMU DEVICE ==================== */

static void xenon_pcie_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    
    dc->realize = xenon_pcie_realize;
    dc->reset = xenon_pcie_reset;
    dc->desc = "Xenon PCIe Controller";
}

static const TypeInfo xenon_pcie_type_info = {
    .name = TYPE_XENON_PCIE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XenonPCIeState),
    .class_init = xenon_pcie_class_init,
};

static void xenon_pcie_register_types(void) {
    type_register_static(&xenon_pcie_type_info);
}

type_init(xenon_pcie_register_types);

/* ==================== PUBLIC FUNCTIONS ==================== */

XenonPCIeState *xenon_pcie_create(MemoryRegion *parent, hwaddr base) {
    DeviceState *dev;
    XenonPCIeState *s;
    
    dev = qdev_new(TYPE_XENON_PCIE);
    s = XENON_PCIE(dev);
    
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    
    /* Connect interrupts */
    s->irq = qemu_allocate_irq(xenon_pcie_irq_handler, s, 0);
    s->msi_irq = qemu_allocate_irq(xenon_pcie_msi_handler, s, 0);
    
    return s;
}

void xenon_pcie_connect_gpu(XenonPCIeState *s, Xbox360GPUState *gpu) {
    if (s->ports[PCIE_PORT_GPU].device) {
        /* Connect GPU to PCIe device */
        XenonPCIeDevice *dev = s->ports[PCIE_PORT_GPU].device;
        
        /* Set up GPU-specific configuration */
        dev->config[0x00/4] = PCIE_VENDOR_ID_MICROSOFT;
        dev->config[0x04/4] = PCIE_DEVICE_ID_GPU;
        dev->config[0x08/4] = PCI_CLASS_DISPLAY_VGA;
        
        printf("[PCIE] GPU connected to PCIe port 0\n");
    }
}

void xenon_pcie_dump_state(XenonPCIeState *s) {
    printf("Xenon PCIe Controller State:\n");
    printf("  Control: 0x%08X, Status: 0x%08X\n", s->control, s->status);
    printf("  IRQ Status: 0x%08X, Mask: 0x%08X\n", s->irq_status, s->irq_mask);
    printf("  MSI Addr: 0x%08X, Data: 0x%08X\n", s->msi_addr, s->msi_data);
    
    printf("  Ports:\n");
    for (int i = 0; i < 8; i++) {
        XenonPCIePort *p = &s->ports[i];
        printf("    Port %d: %s, Link: %s, Lanes: %d, Speed: %dGT/s\n",
               i, p->enabled ? "Enabled" : "Disabled",
               p->link_up ? "Up" : "Down",
               p->lane_count, p->speed == 1 ? 2 : 5);
    }
    
    printf("  Devices: %d\n", s->device_count);
    for (int i = 0; i < s->device_count; i++) {
        XenonPCIeDevice *d = s->devices[i];
        printf("    Device %d: VID:DID = %04X:%04X\n",
               i, d->config[0] & 0xFFFF, (d->config[0] >> 16) & 0xFFFF);
    }
}
