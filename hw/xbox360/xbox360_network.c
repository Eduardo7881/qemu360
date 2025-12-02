#include "qemu/osdep.h"
#include "hw/xbox360/xbox360_network.h"
#include "hw/xbox360/xbox360.h"
#include "hw/net/ne2000.h"
#include "hw/pci/pci.h"
#include "hw/irq.h"
#include "net/net.h"
#include "net/eth.h"
#include "qemu/timer.h"
#include "qemu/sockets.h"
#include "migration/vmstate.h"

/* ==================== XNET CONFIGURATION ==================== */
#define XNET_BASE_ADDRESS         0x8000A000
#define XNET_REGISTER_SIZE        0x1000

#define XNET_MAX_PACKET_SIZE      1514
#define XNET_RX_RING_SIZE         256
#define XNET_TX_RING_SIZE         256
#define XNET_MAC_ADDRESS_LEN      6

/* XNet Registers */
#define XNET_REG_CONTROL          0x0000
#define XNET_REG_STATUS           0x0004
#define XNET_REG_INTERRUPT        0x0008
#define XNET_REG_MAC_LOW          0x000C
#define XNET_REG_MAC_HIGH         0x0010
#define XNET_REG_RX_DESC_BASE     0x0014
#define XNET_REG_TX_DESC_BASE     0x0018
#define XNET_REG_RX_DESC_LEN      0x001C
#define XNET_REG_TX_DESC_LEN      0x0020
#define XNET_REG_RX_PRODUCER      0x0024
#define XNET_REG_RX_CONSUMER      0x0028
#define XNET_REG_TX_PRODUCER      0x002C
#define XNET_REG_TX_CONSUMER      0x0030
#define XNET_REG_PHY_CONTROL      0x0034
#define XNET_REG_PHY_STATUS       0x0038
#define XNET_REG_PHY_ADVERTISE    0x003C
#define XNET_REG_PHY_LINK_PARTNER 0x0040

/* Control Bits */
#define XNET_CTRL_ENABLE          (1 << 0)
#define XNET_CTRL_RESET           (1 << 1)
#define XNET_CTRL_PROMISC         (1 << 2)
#define XNET_CTRL_MULTICAST       (1 << 3)
#define XNET_CTRL_BROADCAST       (1 << 4)
#define XNET_CTRL_RX_ENABLE       (1 << 5)
#define XNET_CTRL_TX_ENABLE       (1 << 6)
#define XNET_CTRL_LOOPBACK        (1 << 7)

/* Status Bits */
#define XNET_STATUS_LINK_UP       (1 << 0)
#define XNET_STATUS_RX_ACTIVE     (1 << 1)
#define XNET_STATUS_TX_ACTIVE     (1 << 2)
#define XNET_STATUS_RX_EMPTY      (1 << 3)
#define XNET_STATUS_TX_FULL       (1 << 4)
#define XNET_STATUS_ERROR         (1 << 5)

/* Interrupt Bits */
#define XNET_INT_RX_COMPLETE      (1 << 0)
#define XNET_INT_TX_COMPLETE      (1 << 1)
#define XNET_INT_RX_ERROR         (1 << 2)
#define XNET_INT_TX_ERROR         (1 << 3)
#define XNET_INT_LINK_CHANGE      (1 << 4)
#define XNET_INT_OVERFLOW         (1 << 5)

/* Descriptor Flags */
#define XNET_DESC_OWN             (1 << 31)  /* Owner (1=HW, 0=SW) */
#define XNET_DESC_EOP             (1 << 30)  /* End of Packet */
#define XNET_DESC_SOP             (1 << 29)  /* Start of Packet */
#define XNET_DESC_ERROR           (1 << 28)  /* Error */
#define XNET_DESC_CRC             (1 << 27)  /* CRC Present */

/* PHY Registers */
#define XNET_PHY_BMCR             0x00  /* Basic Mode Control */
#define XNET_PHY_BMSR             0x01  /* Basic Mode Status */
#define XNET_PHY_ID1              0x02  /* PHY Identifier 1 */
#define XNET_PHY_ID2              0x03  /* PHY Identifier 2 */
#define XNET_PHY_ANAR             0x04  /* Auto-Negotiation Advertisement */
#define XNET_PHY_ANLPAR           0x05  /* Auto-Negotiation Link Partner */
#define XNET_PHY_ANER             0x06  /* Auto-Negotiation Expansion */

/* ==================== NETWORK STRUCTURES ==================== */

typedef struct XNetDescriptor {
    uint32_t buffer_addr;
    uint32_t buffer_addr_high;
    uint32_t length:16;
    uint32_t flags:16;
    uint32_t status;
    uint32_t next_desc;
} XNetDescriptor;

typedef struct XNetRing {
    XNetDescriptor *descriptors;
    uint32_t base_addr;
    uint32_t length;
    uint32_t producer;
    uint32_t consumer;
    uint32_t size;
} XNetRing;

/* ==================== NETWORK CONTROLLER STATE ==================== */

typedef struct XenonNetworkState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    
    /* Registers */
    uint32_t control;
    uint32_t status;
    uint32_t interrupt;
    uint32_t interrupt_mask;
    uint32_t mac_low;
    uint32_t mac_high;
    uint32_t rx_desc_base;
    uint32_t tx_desc_base;
    uint32_t rx_desc_len;
    uint32_t tx_desc_len;
    uint32_t rx_producer;
    uint32_t rx_consumer;
    uint32_t tx_producer;
    uint32_t tx_consumer;
    
    /* PHY registers */
    uint32_t phy_control;
    uint32_t phy_status;
    uint32_t phy_advertise;
    uint32_t phy_link_partner;
    
    /* Rings */
    XNetRing rx_ring;
    XNetRing tx_ring;
    
    /* Buffers */
    uint8_t *rx_buffers[XNET_RX_RING_SIZE];
    uint8_t *tx_buffers[XNET_TX_RING_SIZE];
    
    /* MAC address */
    uint8_t mac_addr[XNET_MAC_ADDRESS_LEN];
    
    /* Network backend */
    NICState *nic;
    NICConf conf;
    bool link_up;
    
    /* Interrupts */
    qemu_irq irq;
    
    /* Timers */
    QEMUTimer *link_timer;
    QEMUTimer *rx_timer;
    QEMUTimer *tx_timer;
    
    /* Statistics */
    uint64_t rx_packets;
    uint64_t tx_packets;
    uint64_t rx_bytes;
    uint64_t tx_bytes;
    uint64_t rx_errors;
    uint64_t tx_errors;
    
    /* Xenon-specific */
    uint32_t xenon_features;
    uint32_t debug;
} XenonNetworkState;

/* ==================== REGISTER ACCESS ==================== */

static uint64_t xnet_read(void *opaque, hwaddr offset, unsigned size) {
    XenonNetworkState *s = opaque;
    uint32_t value = 0;
    
    switch (offset) {
        case XNET_REG_CONTROL:
            value = s->control;
            break;
        case XNET_REG_STATUS:
            value = s->status;
            if (s->link_up) {
                value |= XNET_STATUS_LINK_UP;
            }
            if (s->rx_ring.producer != s->rx_ring.consumer) {
                value |= XNET_STATUS_RX_ACTIVE;
            }
            if (s->tx_ring.producer != s->tx_ring.consumer) {
                value |= XNET_STATUS_TX_ACTIVE;
            }
            break;
        case XNET_REG_INTERRUPT:
            value = s->interrupt;
            break;
        case XNET_REG_MAC_LOW:
            value = (s->mac_addr[3] << 24) | (s->mac_addr[2] << 16) |
                    (s->mac_addr[1] << 8) | s->mac_addr[0];
            break;
        case XNET_REG_MAC_HIGH:
            value = (s->mac_addr[5] << 8) | s->mac_addr[4];
            break;
        case XNET_REG_RX_DESC_BASE:
            value = s->rx_desc_base;
            break;
        case XNET_REG_TX_DESC_BASE:
            value = s->tx_desc_base;
            break;
        case XNET_REG_RX_DESC_LEN:
            value = s->rx_desc_len;
            break;
        case XNET_REG_TX_DESC_LEN:
            value = s->tx_desc_len;
            break;
        case XNET_REG_RX_PRODUCER:
            value = s->rx_producer;
            break;
        case XNET_REG_RX_CONSUMER:
            value = s->rx_consumer;
            break;
        case XNET_REG_TX_PRODUCER:
            value = s->tx_producer;
            break;
        case XNET_REG_TX_CONSUMER:
            value = s->tx_consumer;
            break;
        case XNET_REG_PHY_CONTROL:
            value = s->phy_control;
            break;
        case XNET_REG_PHY_STATUS:
            value = s->phy_status;
            if (s->link_up) {
                value |= (1 << 2);  /* Link status */
                value |= (1 << 1);  /* Jabber detect */
                value |= (1 << 0);  /* Extended capabilities */
            }
            break;
        case XNET_REG_PHY_ADVERTISE:
            value = s->phy_advertise;
            break;
        case XNET_REG_PHY_LINK_PARTNER:
            value = s->phy_link_partner;
            break;
    }
    
    return value;
}

static void xnet_write(void *opaque, hwaddr offset, 
                      uint64_t value, unsigned size) {
    XenonNetworkState *s = opaque;
    
    switch (offset) {
        case XNET_REG_CONTROL:
            s->control = value;
            if (value & XNET_CTRL_RESET) {
                xnet_reset(s);
            }
            if (value & XNET_CTRL_ENABLE) {
                xnet_start(s);
            } else {
                xnet_stop(s);
            }
            break;
        case XNET_REG_INTERRUPT:
            s->interrupt_mask = value;
            break;
        case XNET_REG_MAC_LOW:
            s->mac_addr[0] = (value >> 0) & 0xFF;
            s->mac_addr[1] = (value >> 8) & 0xFF;
            s->mac_addr[2] = (value >> 16) & 0xFF;
            s->mac_addr[3] = (value >> 24) & 0xFF;
            break;
        case XNET_REG_MAC_HIGH:
            s->mac_addr[4] = (value >> 0) & 0xFF;
            s->mac_addr[5] = (value >> 8) & 0xFF;
            break;
        case XNET_REG_RX_DESC_BASE:
            s->rx_desc_base = value & ~0x3;
            xnet_setup_rx_ring(s);
            break;
        case XNET_REG_TX_DESC_BASE:
            s->tx_desc_base = value & ~0x3;
            xnet_setup_tx_ring(s);
            break;
        case XNET_REG_RX_DESC_LEN:
            s->rx_desc_len = value & 0xFFF;
            s->rx_ring.size = 1 << ((value >> 12) & 0xF);
            break;
        case XNET_REG_TX_DESC_LEN:
            s->tx_desc_len = value & 0xFFF;
            s->tx_ring.size = 1 << ((value >> 12) & 0xF);
            break;
        case XNET_REG_RX_CONSUMER:
            s->rx_consumer = value & (s->rx_ring.size - 1);
            xnet_process_rx(s);
            break;
        case XNET_REG_TX_PRODUCER:
            s->tx_producer = value & (s->tx_ring.size - 1);
            xnet_process_tx(s);
            break;
        case XNET_REG_PHY_CONTROL:
            s->phy_control = value;
            if (value & (1 << 15)) {
                /* Software reset */
                s->phy_status = 0;
                s->link_up = false;
                timer_mod(s->link_timer,
                         qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 100);
            }
            if (value & (1 << 12)) {
                /* Auto-negotiation restart */
                s->phy_status |= (1 << 5);  /* Auto-negotiation complete */
                s->link_up = true;
            }
            break;
        case XNET_REG_PHY_ADVERTISE:
            s->phy_advertise = value;
            break;
    }
}

static const MemoryRegionOps xnet_ops = {
    .read = xnet_read,
    .write = xnet_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

/* ==================== RING MANAGEMENT ==================== */

static void xnet_setup_rx_ring(XenonNetworkState *s) {
    if (s->rx_ring.descriptors) {
        g_free(s->rx_ring.descriptors);
    }
    
    s->rx_ring.base_addr = s->rx_desc_base;
    s->rx_ring.length = s->rx_desc_len;
    s->rx_ring.size = 1 << ((s->rx_desc_len >> 12) & 0xF);
    s->rx_ring.producer = 0;
    s->rx_ring.consumer = 0;
    
    s->rx_ring.descriptors = g_malloc0(s->rx_ring.size * sizeof(XNetDescriptor));
    
    /* Initialize descriptors */
    for (uint32_t i = 0; i < s->rx_ring.size; i++) {
        XNetDescriptor *desc = &s->rx_ring.descriptors[i];
        desc->buffer_addr = s->rx_desc_base + 0x1000 + i * XNET_MAX_PACKET_SIZE;
        desc->length = XNET_MAX_PACKET_SIZE;
        desc->flags = XNET_DESC_OWN;  /* Owned by hardware */
        desc->status = 0;
        desc->next_desc = s->rx_desc_base + (i + 1) % s->rx_ring.size * sizeof(XNetDescriptor);
    }
    
    /* Allocate buffers */
    for (uint32_t i = 0; i < XNET_RX_RING_SIZE; i++) {
        if (!s->rx_buffers[i]) {
            s->rx_buffers[i] = g_malloc(XNET_MAX_PACKET_SIZE);
        }
    }
}

static void xnet_setup_tx_ring(XenonNetworkState *s) {
    if (s->tx_ring.descriptors) {
        g_free(s->tx_ring.descriptors);
    }
    
    s->tx_ring.base_addr = s->tx_desc_base;
    s->tx_ring.length = s->tx_desc_len;
    s->tx_ring.size = 1 << ((s->tx_desc_len >> 12) & 0xF);
    s->tx_ring.producer = 0;
    s->tx_ring.consumer = 0;
    
    s->tx_ring.descriptors = g_malloc0(s->tx_ring.size * sizeof(XNetDescriptor));
    
    /* Allocate buffers */
    for (uint32_t i = 0; i < XNET_TX_RING_SIZE; i++) {
        if (!s->tx_buffers[i]) {
            s->tx_buffers[i] = g_malloc(XNET_MAX_PACKET_SIZE);
        }
    }
}

/* ==================== PACKET PROCESSING ==================== */

static void xnet_receive_packet(XenonNetworkState *s, const uint8_t *buf, int size) {
    if (!(s->control & XNET_CTRL_RX_ENABLE) || size > XNET_MAX_PACKET_SIZE) {
        return;
    }
    
    /* Find free RX descriptor */
    uint32_t idx = s->rx_ring.consumer;
    XNetDescriptor *desc = &s->rx_ring.descriptors[idx];
    
    if (!(desc->flags & XNET_DESC_OWN)) {
        /* No free descriptors */
        s->rx_errors++;
        return;
    }
    
    /* Copy packet to buffer */
    uint8_t *buffer = s->rx_buffers[idx];
    memcpy(buffer, buf, size);
    
    /* Update descriptor */
    desc->length = size;
    desc->flags &= ~XNET_DESC_OWN;  /* Owned by software now */
    desc->flags |= XNET_DESC_EOP | XNET_DESC_SOP;
    desc->status = 0;
    
    /* Update consumer index */
    s->rx_ring.consumer = (idx + 1) % s->rx_ring.size;
    s->rx_consumer = s->rx_ring.consumer;
    
    /* Update statistics */
    s->rx_packets++;
    s->rx_bytes += size;
    
    /* Generate interrupt */
    s->interrupt |= XNET_INT_RX_COMPLETE;
    if (s->interrupt & s->interrupt_mask) {
        qemu_set_irq(s->irq, 1);
        timer_mod(s->rx_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000);
    }
    
    printf("[XNET] RX: %d bytes, descriptor %d\n", size, idx);
}

static void xnet_process_tx(XenonNetworkState *s) {
    if (!(s->control & XNET_CTRL_TX_ENABLE)) {
        return;
    }
    
    while (s->tx_ring.producer != s->tx_consumer) {
        uint32_t idx = s->tx_ring.producer;
        XNetDescriptor *desc = &s->tx_ring.descriptors[idx];
        
        if (desc->flags & XNET_DESC_OWN) {
            /* Still owned by hardware */
            break;
        }
        
        /* Get packet from buffer */
        uint8_t *buffer = s->tx_buffers[idx];
        uint32_t length = desc->length & 0xFFFF;
        
        if (length > 0 && length <= XNET_MAX_PACKET_SIZE) {
            /* Send packet to network */
            if (s->nic && s->nic->nc.peer) {
                qemu_send_packet(&s->nic->nc, buffer, length);
                
                /* Update statistics */
                s->tx_packets++;
                s->tx_bytes += length;
                
                printf("[XNET] TX: %d bytes, descriptor %d\n", length, idx);
            }
        }
        
        /* Return descriptor to hardware */
        desc->flags |= XNET_DESC_OWN;
        desc->status = 0;
        
        /* Update producer index */
        s->tx_ring.producer = (idx + 1) % s->tx_ring.size;
        s->tx_producer = s->tx_ring.producer;
        
        /* Generate interrupt */
        s->interrupt |= XNET_INT_TX_COMPLETE;
        if (s->interrupt & s->interrupt_mask) {
            qemu_set_irq(s->irq, 1);
            timer_mod(s->tx_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000);
        }
    }
}

static void xnet_process_rx(XenonNetworkState *s) {
    /* Process any pending RX operations */
    while (s->rx_consumer != s->rx_producer) {
        uint32_t idx = s->rx_consumer;
        XNetDescriptor *desc = &s->rx_ring.descriptors[idx];
        
        if (desc->flags & XNET_DESC_OWN) {
            /* Still owned by hardware */
            break;
        }
        
        /* Return descriptor to hardware */
        desc->flags |= XNET_DESC_OWN;
        desc->length = XNET_MAX_PACKET_SIZE;
        
        /* Update consumer index */
        s->rx_consumer = (idx + 1) % s->rx_ring.size;
        s->rx_consumer = s->rx_consumer;
    }
}

/* ==================== NETWORK CALLBACKS ==================== */

static ssize_t xnet_receive(NetClientState *nc, const uint8_t *buf, size_t size) {
    XenonNetworkState *s = qemu_get_nic_opaque(nc);
    
    if (!s || !(s->control & XNET_CTRL_ENABLE)) {
        return -1;
    }
    
    /* Filter packets based on mode */
    if (!(s->control & XNET_CTRL_PROMISC)) {
        /* Check destination MAC */
        const uint8_t *dest_mac = buf;
        
        /* Broadcast */
        if (dest_mac[0] == 0xFF && dest_mac[1] == 0xFF && 
            dest_mac[2] == 0xFF && dest_mac[3] == 0xFF &&
            dest_mac[4] == 0xFF && dest_mac[5] == 0xFF) {
            if (!(s->control & XNET_CTRL_BROADCAST)) {
                return size;
            }
        }
        /* Multicast */
        else if (dest_mac[0] & 0x01) {
            if (!(s->control & XNET_CTRL_MULTICAST)) {
                return size;
            }
        }
        /* Unicast to us */
        else if (memcmp(dest_mac, s->mac_addr, XNET_MAC_ADDRESS_LEN) != 0) {
            return size;
        }
    }
    
    /* Receive packet */
    xnet_receive_packet(s, buf, size);
    
    return size;
}

static void xnet_link_status_changed(NetClientState *nc) {
    XenonNetworkState *s = qemu_get_nic_opaque(nc);
    
    if (!s) {
        return;
    }
    
    bool old_link = s->link_up;
    s->link_up = nc->link_down ? false : true;
    
    if (old_link != s->link_up) {
        printf("[XNET] Link %s\n", s->link_up ? "UP" : "DOWN");
        
        /* Update PHY status */
        if (s->link_up) {
            s->phy_status |= (1 << 2);  /* Link up */
            s->status |= XNET_STATUS_LINK_UP;
        } else {
            s->phy_status &= ~(1 << 2);
            s->status &= ~XNET_STATUS_LINK_UP;
        }
        
        /* Generate interrupt */
        s->interrupt |= XNET_INT_LINK_CHANGE;
        if (s->interrupt & s->interrupt_mask) {
            qemu_set_irq(s->irq, 1);
            timer_mod(s->link_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000);
        }
    }
}

/* ==================== DEVICE INITIALIZATION ==================== */

static void xenon_network_realize(DeviceState *dev, Error **errp) {
    XenonNetworkState *s = XENON_NETWORK(dev);
    
    /* Initialize registers */
    s->control = 0;
    s->status = 0;
    s->interrupt = 0;
    s->interrupt_mask = 0xFFFFFFFF;
    
    /* Generate MAC address from CPU key or random */
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    memcpy(s->mac_addr, s->conf.macaddr.a, XNET_MAC_ADDRESS_LEN);
    
    s->mac_low = (s->mac_addr[3] << 24) | (s->mac_addr[2] << 16) |
                 (s->mac_addr[1] << 8) | s->mac_addr[0];
    s->mac_high = (s->mac_addr[5] << 8) | s->mac_addr[4];
    
    /* Initialize rings */
    s->rx_ring.descriptors = NULL;
    s->tx_ring.descriptors = NULL;
    s->rx_desc_len = 0x1000;  /* 256 descriptors */
    s->tx_desc_len = 0x1000;
    s->rx_ring.size = 256;
    s->tx_ring.size = 256;
    
    /* Initialize buffers */
    for (int i = 0; i < XNET_RX_RING_SIZE; i++) {
        s->rx_buffers[i] = NULL;
    }
    for (int i = 0; i < XNET_TX_RING_SIZE; i++) {
        s->tx_buffers[i] = NULL;
    }
    
    /* Initialize PHY */
    s->phy_control = 0x1140;  /* Auto-negotiation, 100Mbps FD */
    s->phy_status = 0x7849;   /* 100Mbps FD capable, auto-negotiation complete */
    s->phy_advertise = 0x01E1; /* Advertise all capabilities */
    s->phy_link_partner = 0xCDE1; /* Link partner capabilities */
    
    /* Initialize timers */
    s->link_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xnet_link_timer, s);
    s->rx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xnet_rx_timer, s);
    s->tx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xnet_tx_timer, s);
    
    /* Initialize statistics */
    s->rx_packets = 0;
    s->tx_packets = 0;
    s->rx_bytes = 0;
    s->tx_bytes = 0;
    s->rx_errors = 0;
    s->tx_errors = 0;
    
    /* Create network device */
    s->nic = qemu_new_nic(&net_xenon_info, &s->conf,
                         object_get_typename(OBJECT(dev)),
                         dev->id, s);
    
    /* Set link initially down */
    s->link_up = false;
    s->status &= ~XNET_STATUS_LINK_UP;
    
    /* Initialize memory region */
    memory_region_init_io(&s->iomem, OBJECT(s), &xnet_ops, s,
                         "xenon.network", XNET_REGISTER_SIZE);
    
    printf("[XNET] Xenon Network Controller initialized\n");
    printf("[XNET] MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
           s->mac_addr[0], s->mac_addr[1], s->mac_addr[2],
           s->mac_addr[3], s->mac_addr[4], s->mac_addr[5]);
}

static void xenon_network_reset(DeviceState *dev) {
    XenonNetworkState *s = XENON_NETWORK(dev);
    
    s->control = 0;
    s->status = 0;
    s->interrupt = 0;
    
    /* Reset rings */
    s->rx_ring.producer = 0;
    s->rx_ring.consumer = 0;
    s->tx_ring.producer = 0;
    s->tx_ring.consumer = 0;
    
    s->rx_producer = 0;
    s->rx_consumer = 0;
    s->tx_producer = 0;
    s->tx_consumer = 0;
    
    /* Reset PHY */
    s->phy_control = 0x1140;
    s->link_up = false;
    
    /* Reset timers */
    timer_del(s->link_timer);
    timer_del(s->rx_timer);
    timer_del(s->tx_timer);
    
    qemu_set_irq(s->irq, 0);
    
    /* Reset link status */
    if (s->nic) {
        xnet_link_status_changed(&s->nic->nc);
    }
}

static void xnet_link_timer(void *opaque) {
    XenonNetworkState *s = opaque;
    
    /* Clear interrupt */
    s->interrupt &= ~XNET_INT_LINK_CHANGE;
    if (!(s->interrupt & s->interrupt_mask)) {
        qemu_set_irq(s->irq, 0);
    }
}

static void xnet_rx_timer(void *opaque) {
    XenonNetworkState *s = opaque;
    
    /* Clear interrupt */
    s->interrupt &= ~XNET_INT_RX_COMPLETE;
    if (!(s->interrupt & s->interrupt_mask)) {
        qemu_set_irq(s->irq, 0);
    }
}

static void xnet_tx_timer(void *opaque) {
    XenonNetworkState *s = opaque;
    
    /* Clear interrupt */
    s->interrupt &= ~XNET_INT_TX_COMPLETE;
    if (!(s->interrupt & s->interrupt_mask)) {
        qemu_set_irq(s->irq, 0);
    }
}

static void xnet_start(XenonNetworkState *s) {
    printf("[XNET] Controller started\n");
}

static void xnet_stop(XenonNetworkState *s) {
    printf("[XNET] Controller stopped\n");
}

static void xnet_reset(XenonNetworkState *s) {
    printf("[XNET] Controller reset\n");
    xenon_network_reset(DEVICE(s));
}

/* ==================== QEMU NETWORK INFO ==================== */

static NetClientInfo net_xenon_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .receive = xnet_receive,
    .link_status_changed = xnet_link_status_changed,
};

/* ==================== QEMU DEVICE ==================== */

static Property xenon_network_properties[] = {
    DEFINE_NIC_PROPERTIES(XenonNetworkState, conf),
    DEFINE_PROP_END_OF_LIST(),
};

static void xenon_network_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    
    dc->realize = xenon_network_realize;
    dc->reset = xenon_network_reset;
    dc->desc = "Xenon Network Controller (XNet)";
    device_class_set_props(dc, xenon_network_properties);
}

static const TypeInfo xenon_network_type_info = {
    .name = TYPE_XENON_NETWORK,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XenonNetworkState),
    .class_init = xenon_network_class_init,
};

static void xenon_network_register_types(void) {
    type_register_static(&xenon_network_type_info);
}

type_init(xenon_network_register_types);

/* ==================== PUBLIC FUNCTIONS ==================== */

XenonNetworkState *xenon_network_create(MemoryRegion *parent, hwaddr base) {
    DeviceState *dev;
    XenonNetworkState *s;
    
    dev = qdev_new(TYPE_XENON_NETWORK);
    s = XENON_NETWORK(dev);
    
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    
    /* Connect interrupt */
    s->irq = qemu_allocate_irq(xenon_network_irq_handler, s, 0);
    
    return s;
}

void xenon_network_dump_state(XenonNetworkState *s) {
    printf("Xenon Network Controller State:\n");
    printf("  Control: 0x%08X, Status: 0x%08X, Interrupt: 0x%08X\n",
           s->control, s->status, s->interrupt);
    printf("  MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
           s->mac_addr[0], s->mac_addr[1], s->mac_addr[2],
           s->mac_addr[3], s->mac_addr[4], s->mac_addr[5]);
    printf("  Link: %s, RX: %d/%d, TX: %d/%d\n",
           s->link_up ? "UP" : "DOWN",
           s->rx_ring.consumer, s->rx_ring.producer,
           s->tx_ring.consumer, s->tx_ring.producer);
    printf("  Statistics: RX %" PRIu64 " packets, %" PRIu64 " bytes\n",
           s->rx_packets, s->rx_bytes);
    printf("              TX %" PRIu64 " packets, %" PRIu64 " bytes\n",
           s->tx_packets, s->tx_bytes);
}
