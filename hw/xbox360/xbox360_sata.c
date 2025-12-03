// do we really need this sata emulator? (or i'm wasting my time?) //
//i hope we do... //
#include "qemu/osdep.h"
#include "hw/xbox360/xbox360_sata.h"
#include "hw/xbox360/xbox360.h"
#include "hw/xbox360/xbox360_gic.h"
#include "hw/block/block.h"
#include "hw/ide/internal.h"
#include "hw/pci/pci.h"
#include "hw/irq.h"
#include "qemu/timer.h"
#include "qemu/error-report.h"
#include "migration/vmstate.h"

/* ==================== XENON SATA CONFIGURATION ==================== */
#define SATA_BASE_ADDRESS         0x80007000
#define SATA_REGISTER_SIZE        0x1000

#define SATA_MAX_PORTS            2  /* Port 0: HDD, Port 1: DVD */
#define SATA_MAX_DEVICES          2  /* Max devices per port */
#define SATA_SECTOR_SIZE          512
#define SATA_MAX_SECTORS          256  /* Max sectors per transfer */

/* SATA Controller Registers */
#define SATA_REG_VERSION          0x0000
#define SATA_REG_CONTROL          0x0004
#define SATA_REG_STATUS           0x0008
#define SATA_REG_INTERRUPT        0x000C
#define SATA_REG_ERROR            0x0010
#define SATA_REG_ACTIVE           0x0014
#define SATA_REG_COMMAND_LIST     0x0018
#define SATA_REG_FIS_LIST         0x001C
#define SATA_REG_DEVICE_SIG       0x0020

/* Port Registers (per port) */
#define SATA_PORT_BASE            0x0100
#define SATA_PORT_SIZE            0x0080
#define SATA_PORT_CLB(n)          (SATA_PORT_BASE + (n) * SATA_PORT_SIZE + 0x00)
#define SATA_PORT_CLBU(n)         (SATA_PORT_BASE + (n) * SATA_PORT_SIZE + 0x04)
#define SATA_PORT_FB(n)           (SATA_PORT_BASE + (n) * SATA_PORT_SIZE + 0x08)
#define SATA_PORT_FBU(n)          (SATA_PORT_BASE + (n) * SATA_PORT_SIZE + 0x0C)
#define SATA_PORT_IS(n)           (SATA_PORT_BASE + (n) * SATA_PORT_SIZE + 0x10)
#define SATA_PORT_IE(n)           (SATA_PORT_BASE + (n) * SATA_PORT_SIZE + 0x14)
#define SATA_PORT_CMD(n)          (SATA_PORT_BASE + (n) * SATA_PORT_SIZE + 0x18)
#define SATA_PORT_TFD(n)          (SATA_PORT_BASE + (n) * SATA_PORT_SIZE + 0x1C)
#define SATA_PORT_SIG(n)          (SATA_PORT_BASE + (n) * SATA_PORT_SIZE + 0x20)
#define SATA_PORT_SSTS(n)         (SATA_PORT_BASE + (n) * SATA_PORT_SIZE + 0x24)
#define SATA_PORT_SCTL(n)         (SATA_PORT_BASE + (n) * SATA_PORT_SIZE + 0x28)
#define SATA_PORT_SERR(n)         (SATA_PORT_BASE + (n) * SATA_PORT_SIZE + 0x2C)
#define SATA_PORT_SACT(n)         (SATA_PORT_BASE + (n) * SATA_PORT_SIZE + 0x30)
#define SATA_PORT_CI(n)           (SATA_PORT_BASE + (n) * SATA_PORT_SIZE + 0x34)
#define SATA_PORT_PND(n)          (SATA_PORT_BASE + (n) * SATA_PORT_SIZE + 0x38)

/* SATA Command Bits */
#define SATA_CMD_ST               (1 << 0)  /* Start */
#define SATA_CMD_FRE              (1 << 4)  /* FIS Receive Enable */
#define SATA_CMD_FR               (1 << 14) /* FIS Receive Running */
#define SATA_CMD_CR               (1 << 15) /* Command List Running */

/* SATA Status Bits */
#define SATA_STS_BSY              (1 << 7)  /* Busy */
#define SATA_STS_DRQ              (1 << 3)  /* Data Request */
#define SATA_STS_ERR              (1 << 0)  /* Error */

/* SATA Interrupt Bits */
#define SATA_INT_DHRS             (1 << 0)  /* Device to Host Register FIS */
#define SATA_INT_PS               (1 << 1)  /* PIO Setup FIS */
#define SATA_INT_DSS              (1 << 2)  /* DMA Setup FIS */
#define SATA_INT_SDBS             (1 << 3)  /* Set Device Bits FIS */
#define SATA_INT_UFS              (1 << 4)  /* Unknown FIS */
#define SATA_INT_DPRS             (1 << 5)  /* D2H Register FIS */
#define SATA_INT_PCS              (1 << 6)  /* Port Change */
#define SATA_INT_DMPS             (1 << 7)  /* Device Mechanical Presence */

/* SATA FIS Types */
#define FIS_TYPE_REG_H2D          0x27  /* Host to Device */
#define FIS_TYPE_REG_D2H          0x34  /* Device to Host */
#define FIS_TYPE_DMA_ACTIVATE     0x39  /* DMA Activate */
#define FIS_TYPE_DMA_SETUP        0x41  /* DMA Setup */
#define FIS_TYPE_DATA             0x46  /* Data */
#define FIS_TYPE_BIST             0x58  /* BIST */
#define FIS_TYPE_PIO_SETUP        0x5F  /* PIO Setup */
#define FIS_TYPE_DEVICE_BITS      0xA1  /* Set Device Bits */

/* ==================== SATA STRUCTURES ==================== */

/* FIS (Frame Information Structure) */
typedef struct SATAFIS {
    uint8_t type;
    uint8_t pmport:4;
    uint8_t reserved0:3;
    uint8_t c:1;
    uint8_t command;
    uint8_t feature_low;
    
    uint8_t lba_low;
    uint8_t lba_mid;
    uint8_t lba_high;
    uint8_t device;
    
    uint8_t lba_low_exp;
    uint8_t lba_mid_exp;
    uint8_t lba_high_exp;
    uint8_t feature_high;
    
    uint8_t count_low;
    uint8_t count_high;
    uint8_t icc;
    uint8_t control;
    
    uint8_t reserved1[4];
} SATAFIS;

/* Command Header */
typedef struct SATACommandHeader {
    uint8_t cfl:5;
    uint8_t a:1;
    uint8_t w:1;
    uint8_t p:1;
    
    uint8_t r:1;
    uint8_t b:1;
    uint8_t c:1;
    uint8_t reserved0:1;
    uint8_t pmp:4;
    
    uint16_t prdtl;
    uint32_t prdbc;
    uint32_t ctba;
    uint32_t ctbau;
    uint32_t reserved1[4];
} SATACommandHeader;

/* PRD (Physical Region Descriptor) */
typedef struct SATAPRD {
    uint32_t dba;
    uint32_t dbau;
    uint32_t reserved0;
    uint32_t dbc:22;
    uint32_t reserved1:9;
    uint32_t i:1;
} SATAPRD;

/* SATA Port State */
typedef struct SATAPortState {
    uint32_t clb;          /* Command List Base */
    uint32_t clbu;         /* Command List Base Upper */
    uint32_t fb;           /* FIS Base */
    uint32_t fbu;          /* FIS Base Upper */
    uint32_t is;           /* Interrupt Status */
    uint32_t ie;           /* Interrupt Enable */
    uint32_t cmd;          /* Command */
    uint32_t tfd;          /* Task File Data */
    uint32_t sig;          /* Signature */
    uint32_t ssts;         /* SATA Status */
    uint32_t sctl;         /* SATA Control */
    uint32_t serr;         /* SATA Error */
    uint32_t sact;         /* SATA Active */
    uint32_t ci;           /* Command Issue */
    uint32_t pnd;          /* Pending */
    
    bool running;
    bool dma_active;
    uint32_t active_slot;
    
    BlockBackend *blk;
    char *serial;
    char *model;
    uint64_t capacity;
    
    QEMUTimer *timer;
} SATAPortState;

/* ==================== SATA CONTROLLER STATE ==================== */

typedef struct XenonSATAState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    
    /* Controller registers */
    uint32_t version;
    uint32_t control;
    uint32_t status;
    uint32_t interrupt;
    uint32_t error;
    uint32_t active;
    uint32_t command_list;
    uint32_t fis_list;
    uint32_t device_sig;
    
    /* Ports */
    SATAPortState ports[SATA_MAX_PORTS];
    
    /* Interrupts */
    qemu_irq irq;
    
    /* DMA */
    struct {
        uint32_t control;
        uint32_t source;
        uint32_t dest;
        uint32_t count;
        bool active;
        QEMUTimer *timer;
    } dma;
    
    /* Statistics */
    uint64_t reads;
    uint64_t writes;
    uint64_t bytes_read;
    uint64_t bytes_written;
    
    /* Xenon-specific */
    uint32_t xenon_features;
    uint32_t debug;
    void *gic;
    int hdd_irq;
    int dvd_irq;
} XenonSATAState;

/* ==================== REGISTER ACCESS ==================== */

static uint64_t sata_read(void *opaque, hwaddr offset, unsigned size) {
    XenonSATAState *s = opaque;
    uint32_t value = 0;
    int port;
    
    if (offset >= SATA_PORT_BASE && offset < SATA_PORT_BASE + SATA_MAX_PORTS * SATA_PORT_SIZE) {
        port = (offset - SATA_PORT_BASE) / SATA_PORT_SIZE;
        offset = (offset - SATA_PORT_BASE) % SATA_PORT_SIZE;
        
        if (port < SATA_MAX_PORTS) {
            SATAPortState *p = &s->ports[port];
            switch (offset) {
                case 0x00: value = p->clb; break;
                case 0x04: value = p->clbu; break;
                case 0x08: value = p->fb; break;
                case 0x0C: value = p->fbu; break;
                case 0x10: value = p->is; break;
                case 0x14: value = p->ie; break;
                case 0x18: value = p->cmd; break;
                case 0x1C: value = p->tfd; break;
                case 0x20: value = p->sig; break;
                case 0x24: value = p->ssts; break;
                case 0x28: value = p->sctl; break;
                case 0x2C: value = p->serr; break;
                case 0x30: value = p->sact; break;
                case 0x34: value = p->ci; break;
                case 0x38: value = p->pnd; break;
            }
        }
        return value;
    }
    
    switch (offset) {
        case SATA_REG_VERSION:
            value = 0x00010000;  /* Version 1.0 */
            break;
        case SATA_REG_CONTROL:
            value = s->control;
            break;
        case SATA_REG_STATUS:
            value = s->status;
            break;
        case SATA_REG_INTERRUPT:
            value = s->interrupt;
            break;
        case SATA_REG_ERROR:
            value = s->error;
            break;
        case SATA_REG_ACTIVE:
            value = s->active;
            break;
        case SATA_REG_COMMAND_LIST:
            value = s->command_list;
            break;
        case SATA_REG_FIS_LIST:
            value = s->fis_list;
            break;
        case SATA_REG_DEVICE_SIG:
            value = s->device_sig;
            break;
    }
    
    return value;
}

static void sata_write(void *opaque, hwaddr offset, 
                      uint64_t value, unsigned size) {
    XenonSATAState *s = opaque;
    int port;
    
    if (offset >= SATA_PORT_BASE && offset < SATA_PORT_BASE + SATA_MAX_PORTS * SATA_PORT_SIZE) {
        port = (offset - SATA_PORT_BASE) / SATA_PORT_SIZE;
        offset = (offset - SATA_PORT_BASE) % SATA_PORT_SIZE;
        
        if (port < SATA_MAX_PORTS) {
            SATAPortState *p = &s->ports[port];
            switch (offset) {
                case 0x00:  /* CLB */
                    p->clb = value;
                    break;
                case 0x04:  /* CLBU */
                    p->clbu = value;
                    break;
                case 0x08:  /* FB */
                    p->fb = value;
                    break;
                case 0x0C:  /* FBU */
                    p->fbu = value;
                    break;
                case 0x10:  /* IS */
                    p->is = value;
                    if (p->is & p->ie) {
                        sata_trigger_interrupt(s, port);
                    }
                    break;
                case 0x14:  /* IE */
                    p->ie = value;
                    break;
                case 0x18:  /* CMD */
                    p->cmd = value;
                    if (value & SATA_CMD_ST) {
                        sata_port_start(p);
                    }
                    if (!(value & SATA_CMD_ST)) {
                        sata_port_stop(p);
                    }
                    break;
                case 0x1C:  /* TFD */
                    p->tfd = value;
                    break;
                case 0x20:  /* SIG */
                    p->sig = value;
                    break;
                case 0x28:  /* SCTL */
                    p->sctl = value;
                    break;
                case 0x2C:  /* SERR */
                    p->serr = value;
                    break;
                case 0x34:  /* CI */
                    p->ci = value;
                    sata_process_command(p, value);
                    break;
                case 0x38:  /* PND */
                    p->pnd = value;
                    break;
            }
        }
        return;
    }
    
    switch (offset) {
        case SATA_REG_CONTROL:
            s->control = value;
            if (value & 1) {
                sata_reset(s);
            }
            break;
        case SATA_REG_INTERRUPT:
            s->interrupt = value;
            break;
        case SATA_REG_ERROR:
            s->error = value;
            break;
    }
}

static const MemoryRegionOps sata_ops = {
    .read = sata_read,
    .write = sata_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

/* ==================== SATA COMMAND PROCESSING ==================== */

static void sata_process_command(SATAPortState *p, uint32_t ci) {
    if (!p->running || ci == 0) {
        return;
    }
    for (uint32_t slot = 0; slot < 32; slot++) {
        if (!(ci & (1u << slot))) {
            continue;
        }
        p->active_slot = slot;
        uint64_t cmd_list_addr = ((uint64_t)p->clbu << 32) | p->clb;
        uint64_t cmd_header_addr = cmd_list_addr + slot * sizeof(SATACommandHeader);
        SATACommandHeader cmd_header;
        cpu_physical_memory_read(cmd_header_addr, (uint8_t*)&cmd_header, sizeof(cmd_header));
        uint64_t fis_addr = ((uint64_t)cmd_header.ctbau << 32) | cmd_header.ctba;
        SATAFIS fis;
        cpu_physical_memory_read(fis_addr, (uint8_t*)&fis, sizeof(fis));
        switch (fis.command) {
            case 0x20:
            case 0x21:
                sata_read_sectors(p, &fis, &cmd_header);
                break;
            case 0x30:
            case 0x31:
                sata_write_sectors(p, &fis, &cmd_header);
                break;
            case 0xEC:
                sata_identify_device(p, &fis, &cmd_header);
                break;
            default:
                p->tfd = SATA_STS_ERR;
                break;
        }
        p->ci &= ~(1u << slot);
        p->sact &= ~(1u << slot);
        p->is |= SATA_INT_DHRS;
        {
            XenonSATAState *s = container_of(p, XenonSATAState, ports[p - s->ports]);
            sata_trigger_interrupt(s, p - s->ports);
        }
    }
}

static void sata_read_sectors(SATAPortState *p, SATAFIS *fis, 
                             SATACommandHeader *cmd_header) {
    if (!p->blk) {
        p->tfd = SATA_STS_ERR;
        p->serr |= 1;
        return;
    }
    
    /* Calculate LBA and sector count */
    uint64_t lba = ((uint64_t)fis->lba_high_exp << 40) |
                   ((uint64_t)fis->lba_mid_exp << 32) |
                   ((uint64_t)fis->lba_high << 24) |
                   ((uint64_t)fis->lba_mid << 16) |
                   ((uint64_t)fis->lba_low << 8) |
                   fis->lba_low_exp;
    
    uint32_t count = ((uint32_t)fis->count_high << 8) | fis->count_low;
    if (count == 0) {
        count = 256;
    }
    
    /* Read PRDs */
    uint64_t prd_addr = ((uint64_t)cmd_header->ctbau << 32) | cmd_header->ctba;
    prd_addr += sizeof(SATAFIS);  /* Skip FIS */
    
    uint32_t total_bytes = count * SATA_SECTOR_SIZE;
    uint32_t bytes_transferred = 0;
    
    for (uint16_t i = 0; i < cmd_header->prdtl && bytes_transferred < total_bytes; i++) {
        SATAPRD prd;
        cpu_physical_memory_read(prd_addr + i * sizeof(SATAPRD),
                                (uint8_t*)&prd, sizeof(prd));
        
        uint32_t prd_bytes = prd.dbc & 0x3FFFFF;
        if (prd_bytes == 0) {
            prd_bytes = 0x400000;  /* 4MB max */
        }
        
        uint32_t to_transfer = MIN(prd_bytes, total_bytes - bytes_transferred);
        
        /* Read sectors into buffer */
        uint8_t *buffer = g_malloc(to_transfer);
        
        uint32_t sectors_to_read = to_transfer / SATA_SECTOR_SIZE;
        uint64_t current_lba = lba + (bytes_transferred / SATA_SECTOR_SIZE);
        
        for (uint32_t s = 0; s < sectors_to_read; s++) {
            int ret = blk_pread(p->blk, (current_lba + s) * SATA_SECTOR_SIZE,
                               buffer + s * SATA_SECTOR_SIZE, SATA_SECTOR_SIZE);
            if (ret < 0) {
                p->tfd = SATA_STS_ERR;
                p->serr |= 2;
                g_free(buffer);
                return;
            }
        }
        
        /* Write to PRD destination */
        uint64_t dest_addr = ((uint64_t)prd.dbau << 32) | prd.dba;
        cpu_physical_memory_write(dest_addr, buffer, to_transfer);
        
        bytes_transferred += to_transfer;
        g_free(buffer);
    }
    
    /* Update PRD byte count */
    cmd_header->prdbc = bytes_transferred;
    
    /* Update status */
    p->tfd = SATA_STS_DRQ;
    if (lba + count > p->capacity) { p->serr |= 4; p->tfd = SATA_STS_ERR; }
    
    /* Update statistics */
    XenonSATAState *s = container_of(p, XenonSATAState, ports[p - s->ports]);
    s->reads++;
    s->bytes_read += bytes_transferred;
}

static void sata_write_sectors(SATAPortState *p, SATAFIS *fis,
                              SATACommandHeader *cmd_header) {
    if (!p->blk) {
        p->tfd = SATA_STS_ERR;
        p->serr |= 1;
        return;
    }
    
    /* Calculate LBA and sector count */
    uint64_t lba = ((uint64_t)fis->lba_high_exp << 40) |
                   ((uint64_t)fis->lba_mid_exp << 32) |
                   ((uint64_t)fis->lba_high << 24) |
                   ((uint64_t)fis->lba_mid << 16) |
                   ((uint64_t)fis->lba_low << 8) |
                   fis->lba_low_exp;
    
    uint32_t count = ((uint32_t)fis->count_high << 8) | fis->count_low;
    if (count == 0) {
        count = 256;
    }
    
    /* Read PRDs */
    uint64_t prd_addr = ((uint64_t)cmd_header->ctbau << 32) | cmd_header->ctba;
    prd_addr += sizeof(SATAFIS);  /* Skip FIS */
    
    uint32_t total_bytes = count * SATA_SECTOR_SIZE;
    uint32_t bytes_transferred = 0;
    
    for (uint16_t i = 0; i < cmd_header->prdtl && bytes_transferred < total_bytes; i++) {
        SATAPRD prd;
        cpu_physical_memory_read(prd_addr + i * sizeof(SATAPRD),
                                (uint8_t*)&prd, sizeof(prd));
        
        uint32_t prd_bytes = prd.dbc & 0x3FFFFF;
        if (prd_bytes == 0) {
            prd_bytes = 0x400000;  /* 4MB max */
        }
        
        uint32_t to_transfer = MIN(prd_bytes, total_bytes - bytes_transferred);
        
        /* Read data from PRD source */
        uint8_t *buffer = g_malloc(to_transfer);
        uint64_t src_addr = ((uint64_t)prd.dbau << 32) | prd.dba;
        cpu_physical_memory_read(src_addr, buffer, to_transfer);
        
        /* Write sectors to disk */
        uint32_t sectors_to_write = to_transfer / SATA_SECTOR_SIZE;
        uint64_t current_lba = lba + (bytes_transferred / SATA_SECTOR_SIZE);
        
        for (uint32_t s = 0; s < sectors_to_write; s++) {
            int ret = blk_pwrite(p->blk, (current_lba + s) * SATA_SECTOR_SIZE,
                                buffer + s * SATA_SECTOR_SIZE, SATA_SECTOR_SIZE, 0);
            if (ret < 0) {
                p->tfd = SATA_STS_ERR;
                p->serr |= 2;
                g_free(buffer);
                return;
            }
        }
        
        bytes_transferred += to_transfer;
        g_free(buffer);
    }
    
    /* Update PRD byte count */
    cmd_header->prdbc = bytes_transferred;
    
    /* Update status */
    p->tfd = SATA_STS_DRQ;
    if (lba + count > p->capacity) { p->serr |= 4; p->tfd = SATA_STS_ERR; }
    
    /* Update statistics */
    XenonSATAState *s = container_of(p, XenonSATAState, ports[p - s->ports]);
    s->writes++;
    s->bytes_written += bytes_transferred;
}

static void sata_identify_device(SATAPortState *p, SATAFIS *fis,
                                SATACommandHeader *cmd_header) {
    /* Create IDENTIFY DEVICE data */
    uint16_t identify[256];
    memset(identify, 0, sizeof(identify));
    
    if (p->blk) {
        /* Device is present */
        identify[0] = 0x0040;  /* Fixed disk */
        identify[1] = 0x3FFF;  /* Default cylinders */
        identify[3] = 0x0010;  /* Default heads */
        identify[6] = 0x003F;  /* Default sectors per track */
        
        /* Serial number */
        if (p->serial) {
            for (int i = 0; i < 10; i++) {
                uint16_t val = 0;
                if (i * 2 < strlen(p->serial)) {
                    val = p->serial[i * 2];
                }
                if (i * 2 + 1 < strlen(p->serial)) {
                    val |= p->serial[i * 2 + 1] << 8;
                }
                identify[10 + i] = val;
            }
        }
        
        /* Model number */
        if (p->model) {
            for (int i = 0; i < 20; i++) {
                uint16_t val = 0;
                if (i * 2 < strlen(p->model)) {
                    val = p->model[i * 2];
                }
                if (i * 2 + 1 < strlen(p->model)) {
                    val |= p->model[i * 2 + 1] << 8;
                }
                identify[27 + i] = val;
            }
        }
        
        /* Capabilities */
        identify[49] = 0x0F00;  /* LBA supported */
        identify[53] = 0x0007;  /* Words 64-70, 88 valid */
        identify[60] = p->capacity & 0xFFFF;  /* LBA sectors */
        identify[61] = (p->capacity >> 16) & 0xFFFF;
        identify[100] = p->capacity & 0xFFFF;  /* LBA48 sectors */
        identify[101] = (p->capacity >> 16) & 0xFFFF;
        identify[102] = (p->capacity >> 32) & 0xFFFF;
        identify[103] = (p->capacity >> 48) & 0xFFFF;
    } else {
        /* No device */
        identify[0] = 0x7FFF;  /* No device */
    }
    
    /* Write IDENTIFY data to PRD */
    if (cmd_header->prdtl > 0) {
        SATAPRD prd;
        uint64_t prd_addr = ((uint64_t)cmd_header->ctbau << 32) | cmd_header->ctba;
        prd_addr += sizeof(SATAFIS);  /* Skip FIS */
        
        cpu_physical_memory_read(prd_addr, (uint8_t*)&prd, sizeof(prd));
        
        uint64_t dest_addr = ((uint64_t)prd.dbau << 32) | prd.dba;
        cpu_physical_memory_write(dest_addr, (uint8_t*)identify, sizeof(identify));
        
        cmd_header->prdbc = sizeof(identify);
    }
    
    p->tfd = SATA_STS_DRQ;
}

/* ==================== PORT MANAGEMENT ==================== */

static void sata_port_start(SATAPortState *p) {
    if (p->running) {
        return;
    }
    
    p->running = true;
    p->cmd |= SATA_CMD_CR | SATA_CMD_FR;
    
    /* Set device signature */
    if (p->blk) {
        p->sig = 0x00000101;  /* SATA device present */
    } else {
        p->sig = 0xFFFFFFFF;  /* No device */
    }
}

static void sata_port_stop(SATAPortState *p) {
    if (!p->running) {
        return;
    }
    
    p->running = false;
    p->cmd &= ~(SATA_CMD_CR | SATA_CMD_FR);
}

static void sata_trigger_interrupt(XenonSATAState *s, int port) {
    s->interrupt |= (1 << port);
    qemu_set_irq(s->irq, 1);
    
    /* Clear after 1us */
    timer_mod_ns(s->ports[port].timer,
                qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000);
}

static void sata_interrupt_clear(void *opaque) {
    SATAPortState *p = opaque;
    XenonSATAState *s = container_of(p, XenonSATAState, ports[p - s->ports]);
    
    p->is = 0;
    s->interrupt &= ~(1 << (p - s->ports));
    
    if (s->interrupt == 0) {
        qemu_set_irq(s->irq, 0);
    }
}

static void sata_dma_complete(void *opaque) {
    XenonSATAState *s = opaque;
    s->dma.active = false;
    s->active = 0;
    /* Pulse per-port interrupts if any DMA completion affects them */
    for (int i = 0; i < SATA_MAX_PORTS; i++) {
        if (s->ports[i].running) {
            s->ports[i].is |= SATA_INT_DSS;
            sata_trigger_interrupt(s, i);
        }
    }
}

/* ==================== DEVICE INITIALIZATION ==================== */

static void xenon_sata_realize(DeviceState *dev, Error **errp) {
    XenonSATAState *s = XENON_SATA(dev);
    
    /* Initialize registers */
    s->version = 0x00010000;
    s->control = 0x00000001;  /* Enabled */
    s->status = 0;
    s->interrupt = 0;
    s->error = 0;
    s->active = 0;
    
    /* Initialize ports */
    for (int i = 0; i < SATA_MAX_PORTS; i++) {
        SATAPortState *p = &s->ports[i];
        
        p->clb = 0;
        p->clbu = 0;
        p->fb = 0;
        p->fbu = 0;
        p->is = 0;
        p->ie = 0xFFFFFFFF;  /* Enable all interrupts */
        p->cmd = 0;
        p->tfd = 0;
        p->sig = 0xFFFFFFFF;  /* No device by default */
        p->ssts = 0;
        p->sctl = 0;
        p->serr = 0;
        p->sact = 0;
        p->ci = 0;
        p->pnd = 0;
        
        p->running = false;
        p->dma_active = false;
        p->active_slot = 0;
        
        p->blk = NULL;
        p->serial = NULL;
        p->model = NULL;
        p->capacity = 0;
        
        p->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sata_interrupt_clear, p);
    }
    
    /* Initialize DMA */
    s->dma.control = 0;
    s->dma.active = false;
    s->dma.timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sata_dma_complete, s);
    
    /* Initialize statistics */
    s->reads = 0;
    s->writes = 0;
    s->bytes_read = 0;
    s->bytes_written = 0;
    
    /* Initialize memory region */
    memory_region_init_io(&s->iomem, OBJECT(s), &sata_ops, s,
                         "xenon.sata", SATA_REGISTER_SIZE);
    
    printf("[SATA] Xenon SATA Controller initialized: %d ports\n", SATA_MAX_PORTS);
}

static void xenon_sata_reset(DeviceState *dev) {
    XenonSATAState *s = XENON_SATA(dev);
    
    s->control = 0x00000001;
    s->interrupt = 0;
    s->error = 0;
    
    /* Reset ports */
    for (int i = 0; i < SATA_MAX_PORTS; i++) {
        SATAPortState *p = &s->ports[i];
        
        p->is = 0;
        p->cmd = 0;
        p->tfd = 0;
        p->ci = 0;
        p->pnd = 0;
        
        p->running = false;
        p->dma_active = false;
        p->active_slot = 0;
        
        timer_del(p->timer);
    }
    
    /* Reset DMA */
    s->dma.control = 0;
    s->dma.active = false;
    timer_del(s->dma.timer);
    
    qemu_set_irq(s->irq, 0);
}

/* ==================== QEMU DEVICE ==================== */

static void xenon_sata_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    
    dc->realize = xenon_sata_realize;
    dc->reset = xenon_sata_reset;
    dc->desc = "Xenon SATA Controller";
}

static const TypeInfo xenon_sata_type_info = {
    .name = TYPE_XENON_SATA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XenonSATAState),
    .class_init = xenon_sata_class_init,
};

static void xenon_sata_register_types(void) {
    type_register_static(&xenon_sata_type_info);
}

type_init(xenon_sata_register_types);

/* ==================== PUBLIC FUNCTIONS ==================== */

XenonSATAState *xenon_sata_create(MemoryRegion *parent, hwaddr base) {
    DeviceState *dev;
    XenonSATAState *s;
    
    dev = qdev_new(TYPE_XENON_SATA);
    s = XENON_SATA(dev);
    
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    
    /* Connect interrupt */
    s->irq = NULL;
    
    return s;
}

void xenon_sata_attach_drive(XenonSATAState *s, int port, BlockBackend *blk,
                            const char *serial, const char *model) {
    if (port < 0 || port >= SATA_MAX_PORTS) {
        return;
    }
    
    SATAPortState *p = &s->ports[port];
    
    if (p->blk) {
        blk_unref(p->blk);
    }
    
    p->blk = blk;
    
    if (serial) {
        g_free(p->serial);
        p->serial = g_strdup(serial);
    }
    
    if (model) {
        g_free(p->model);
        p->model = g_strdup(model);
    }
    
    if (blk) {
        p->capacity = blk_getlength(blk) / SATA_SECTOR_SIZE;
        p->sig = 0x00000101;  /* SATA device present */
        
        printf("[SATA] Port %d: %s (%s), %" PRIu64 " sectors\n",
               port, model ? model : "Unknown",
               serial ? serial : "Unknown", p->capacity);
    } else {
        p->capacity = 0;
        p->sig = 0xFFFFFFFF;  /* No device */
        printf("[SATA] Port %d: No device\n", port);
    }
}

void xenon_sata_dump_state(XenonSATAState *s) {
    printf("Xenon SATA Controller State:\n");
    printf("  Control: 0x%08X, Status: 0x%08X, Interrupt: 0x%08X\n",
           s->control, s->status, s->interrupt);
    printf("  Error: 0x%08X, Active: 0x%08X\n", s->error, s->active);
    printf("  Statistics: %" PRIu64 " reads, %" PRIu64 " writes, %" PRIu64 " bytes\n",
           s->reads, s->writes, s->bytes_read + s->bytes_written);
    
    printf("  Ports:\n");
    for (int i = 0; i < SATA_MAX_PORTS; i++) {
        SATAPortState *p = &s->ports[i];
        printf("    Port %d: %s, CMD=0x%08X, TFD=0x%08X\n",
               i, p->running ? "Running" : "Stopped",
               p->cmd, p->tfd);
        if (p->blk) {
            printf("      Device: %s, %" PRIu64 " sectors\n",
                   p->model ? p->model : "Unknown", p->capacity);
        }
    }
}
static void xenon_sata_irq_handler(void *opaque, int n, int level) {
    XenonSATAState *s = opaque;
    if (!s->gic) {
        return;
    }
    if (level) {
        if (s->interrupt & (1 << SATA_PORT_HDD)) {
            xenon_gic_assert_irq(s->gic, s->hdd_irq, 0);
        }
        if (s->interrupt & (1 << SATA_PORT_DVD)) {
            xenon_gic_assert_irq(s->gic, s->dvd_irq, 0);
        }
    } else {
        xenon_gic_deassert_irq(s->gic, s->hdd_irq, 0);
        xenon_gic_deassert_irq(s->gic, s->dvd_irq, 0);
    }
}

void xenon_sata_connect_irq(XenonSATAState *s, XenonGICState *gic, int hdd_irq, int dvd_irq) {
    s->gic = gic;
    s->hdd_irq = hdd_irq;
    s->dvd_irq = dvd_irq;
    s->irq = qemu_allocate_irq(xenon_sata_irq_handler, s, 0);
}

BlockBackend *xenon_sata_get_backend(XenonSATAState *s, int port) {
    if (!s || port < 0 || port >= SATA_MAX_PORTS) {
        return NULL;
    }
    return s->ports[port].blk;
}
