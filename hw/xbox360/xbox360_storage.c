#include "qemu/osdep.h"
#include "hw/xbox360/xbox360_storage.h"
#include "hw/xbox360/xbox360.h"
#include "hw/block/block.h"
#include "sysemu/blockdev.h"
#include "qapi/error.h"
#include "qemu/error-report.h"

/* ==================== STORAGE CONFIGURATION ==================== */
#define XBOX360_HDD_SIZE          (7 * 1024 * 1024 * 1024ULL)  /* 120GB is recommended */
#define XBOX360_DVD_SIZE          (8 * 1024 * 1024 * 1024ULL)    /* 8.5GB DL */
#define XBOX360_MU_SIZE           (512 * 1024 * 1024ULL)         /* 512MB */

/* Xbox 360 HDD Partition Layout */
typedef struct XBOX360_HDD_PARTITION {
    uint32_t start_sector;
    uint32_t sector_count;
    const char *name;
    const char *type;
} XBOX360_HDD_PARTITION;

static const XBOX360_HDD_PARTITION xbox360_partitions[] = {
    {0x000800, 0x0FF800, "System", "FATX"},      /* System cache */
    {0x100000, 0x1F0000, "Cache1", "FATX"},      /* Game cache 1 */
    {0x2F0000, 0x1F0000, "Cache2", "FATX"},      /* Game cache 2 */
    {0x4E0000, 0x5D2000, "Content", "FATX"},     /* User content */
    {0xAB2000, 0x154E000, "Games", "FATX"},      /* Installed games */
    {0, 0, NULL, NULL}
};

/* ==================== VIRTUAL DISK CREATION ==================== */

static BlockBackend *create_xbox360_hdd(const char *filename, Error **errp) {
    BlockBackend *blk = NULL;
    BlockDriverState *bs = NULL;
    uint64_t total_size = XBOX360_HDD_SIZE;
    
    /* Create or open HDD image */
    if (filename && *filename) {
        /* Open existing image */
        bs = bdrv_open(filename, NULL, NULL, BDRV_O_RDWR, errp);
        if (!bs) {
            return NULL;
        }
        
        /* Verify size */
        uint64_t size = bdrv_getlength(bs);
        if (size < total_size) {
            error_setg(errp, "HDD image too small: %" PRIu64 " < %" PRIu64,
                       size, total_size);
            bdrv_unref(bs);
            return NULL;
        }
    } else {
        /* Create new image */
        const char *fmt = "raw";
        QDict *opts = qdict_new();
        
        qdict_put_str(opts, "driver", fmt);
        qdict_put_str(opts, "file.driver", "file");
        qdict_put_str(opts, "file.filename", "xbox360_hdd.img");
        qdict_put_int(opts, "size", total_size);
        
        bs = bdrv_open("xbox360_hdd.img", NULL, opts, BDRV_O_RDWR | BDRV_O_CREAT, errp);
        if (!bs) {
            return NULL;
        }
        
        /* Initialize partition table */
        initialize_xbox360_partitions(bs, errp);
        if (*errp) {
            bdrv_unref(bs);
            return NULL;
        }
    }
    
    blk = blk_new(BLK_PERM_ALL, BLK_PERM_ALL);
    blk_insert_bs(blk, bs, errp);
    
    if (*errp) {
        blk_unref(blk);
        bdrv_unref(bs);
        return NULL;
    }
    
    printf("[HDD] Xbox 360 HDD initialized: %" PRIu64 " bytes\n", total_size);
    return blk;
}

static BlockBackend *create_xbox360_dvd(const char *filename, Error **errp) {
    BlockBackend *blk = NULL;
    BlockDriverState *bs = NULL;
    
    if (!filename || !*filename) {
        error_setg(errp, "DVD image filename required");
        return NULL;
    }
    
    /* Open DVD image */
    bs = bdrv_open(filename, NULL, NULL, BDRV_O_RDONLY, errp);
    if (!bs) {
        return NULL;
    }
    
    /* Verify it's a DVD image */
    const char *drv_name = bdrv_get_format_name(bs);
    if (!drv_name || (strcmp(drv_name, "raw") != 0 && 
                      strcmp(drv_name, "iso") != 0)) {
        error_setg(errp, "Invalid DVD image format: %s", drv_name ? drv_name : "unknown");
        bdrv_unref(bs);
        return NULL;
    }
    
    blk = blk_new(BLK_PERM_CONSISTENT_READ, BLK_PERM_ALL);
    blk_insert_bs(blk, bs, errp);
    
    if (*errp) {
        blk_unref(blk);
        bdrv_unref(bs);
        return NULL;
    }
    
    uint64_t size = blk_getlength(blk);
    printf("[DVD] Xbox 360 DVD initialized: %" PRIu64 " bytes\n", size);
    return blk;
}

BlockBackend *xenon_create_mu(Error **errp) {
    BlockBackend *blk = NULL;
    BlockDriverState *bs = NULL;
    uint64_t total_size = XBOX360_MU_SIZE;
    const char *fmt = "raw";
    QDict *opts = qdict_new();
    qdict_put_str(opts, "driver", fmt);
    qdict_put_str(opts, "file.driver", "file");
    qdict_put_str(opts, "file.filename", "xbox360_mu.img");
    qdict_put_int(opts, "size", total_size);
    bs = bdrv_open("xbox360_mu.img", NULL, opts, BDRV_O_RDWR | BDRV_O_CREAT, errp);
    if (!bs) {
        return NULL;
    }
    blk = blk_new(BLK_PERM_ALL, BLK_PERM_ALL);
    blk_insert_bs(blk, bs, errp);
    if (*errp) {
        blk_unref(blk);
        bdrv_unref(bs);
        return NULL;
    }
    return blk;
}

static void initialize_xbox360_partitions(BlockDriverState *bs, Error **errp) {
    /* Create FATX partition table */
    uint8_t *mbr = g_malloc0(512);
    
    /* MBR signature */
    mbr[510] = 0x55;
    mbr[511] = 0xAA;
    
    /* Create partitions */
    uint32_t offset = 0;
    const XBOX360_HDD_PARTITION *part = xbox360_partitions;
    
    while (part->name) {
        /* Create partition entry */
        uint8_t *entry = mbr + 0x1BE + (offset * 16);
        
        /* Boot flag (0x80 = bootable) */
        entry[0] = (offset == 0) ? 0x80 : 0x00;
        
        /* Start CHS (simplified) */
        entry[1] = 0x01;  /* Head */
        entry[2] = 0x01;  /* Sector */
        entry[3] = 0x00;  /* Cylinder */
        
        /* Partition type (0xEB = FATX) */
        entry[4] = 0xEB;
        
        /* End CHS */
        entry[5] = 0xFF;
        entry[6] = 0xFF;
        entry[7] = 0xFF;
        
        /* Start LBA */
        entry[8] = (part->start_sector >> 0) & 0xFF;
        entry[9] = (part->start_sector >> 8) & 0xFF;
        entry[10] = (part->start_sector >> 16) & 0xFF;
        entry[11] = (part->start_sector >> 24) & 0xFF;
        
        /* Sector count */
        entry[12] = (part->sector_count >> 0) & 0xFF;
        entry[13] = (part->sector_count >> 8) & 0xFF;
        entry[14] = (part->sector_count >> 16) & 0xFF;
        entry[15] = (part->sector_count >> 24) & 0xFF;
        
        printf("[HDD] Partition %d: %s, %u sectors\n",
               offset + 1, part->name, part->sector_count);
        
        part++;
        offset++;
    }
    
    /* Write MBR */
    bdrv_pwrite(bs, 0, mbr, 512, 0);
    
    g_free(mbr);
}

/* ==================== STORAGE INTEGRATION ==================== */

void xenon_storage_init(XenonState *s, const char *hdd_path, 
                       const char *dvd_path, Error **errp) {
    BlockBackend *hdd_blk = NULL;
    BlockBackend *dvd_blk = NULL;
    
    /* Initialize SATA controller */
    s->sata = xenon_sata_create(get_system_memory(), 0x80007000);
    printf("[SATA] Initialized at 0x80007000\n");
    
    /* Attach HDD to port 0 */
    if (hdd_path) {
        hdd_blk = create_xbox360_hdd(hdd_path, errp);
        if (*errp) {
            return;
        }
        
        xenon_sata_attach_drive(s->sata, SATA_PORT_HDD, hdd_blk,
                               "XBOX360HDD001", "Microsoft Xbox 360 HDD");
    } else {
        printf("[HDD] No HDD image specified, using empty drive\n");
    }
    
    /* Attach DVD to port 1 */
    if (dvd_path) {
        dvd_blk = create_xbox360_dvd(dvd_path, errp);
        if (*errp) {
            if (hdd_blk) {
                blk_unref(hdd_blk);
            }
            return;
        }
        
        xenon_sata_attach_drive(s->sata, SATA_PORT_DVD, dvd_blk,
                               "XBOX360DVD001", "Microsoft Xbox 360 DVD");
    } else {
        printf("[DVD] No DVD image specified\n");
    }
    
    /* Connect SATA interrupts to GIC */
    if (s->sata && s->gic) {
        xenon_sata_connect_irq(s->sata, s->gic, IRQ_HDD, IRQ_DVD);
    }
    
    printf("[STORAGE] Storage system initialized\n");
}

void xenon_storage_cleanup(XenonState *s) {
    if (s->sata) {
        /* Clean up drives */
        for (int i = 0; i < 2; i++) {
            SATAPortState *port = &s->sata->ports[i];
            if (port->blk) {
                blk_unref(port->blk);
                port->blk = NULL;
            }
        }
    }
}
