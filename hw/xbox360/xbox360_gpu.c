#include "hw/xbox360/xbox360_gpu.h"
#include "hw/xbox360/xbox360.h"
#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/pci/pci.h"
#include "hw/display/ramfb.h"
#include "ui/console.h"
#include "migration/vmstate.h"
#include "trace.h"

/* ==================== GPU REGISTERS ==================== */
#define GPU_VENDOR_ID         0x1002
#define GPU_DEVICE_ID         0x0218
#define GPU_CLASS_CODE        0x030000
#define GPU_REVISION_ID       0x00
#define GPU_SUBSYSTEM_VENDOR  0x10DE
#define GPU_SUBSYSTEM_ID      0x02A1

#define VBLANK_PERIOD_NS      16666666  // 60Hz

#define DEFAULT_WIDTH         1280
#define DEFAULT_HEIGHT        720
#define DEFAULT_BPP           32
#define DEFAULT_PITCH         (DEFAULT_WIDTH * 4)

/* ==================== MEMORY REGION OPS ==================== */
static uint64_t gpu_mmio_read(void *opaque, hwaddr addr, unsigned size) {
    Xbox360GPUState *s = opaque;
    uint32_t value = 0;
    uint32_t reg = addr / 4;
    
    if (addr >= GPU_REGISTER_SIZE) {
        return 0;
    }
    
    switch (addr) {
        case GPU_REG_ID:
            value = GPU_XENOS_ID;
            break;
            
        case GPU_REG_STATUS:
            value = (s->state << 0) |
                    ((s->interrupt_status & s->interrupt_mask) << 8) |
                    (s->display.enabled << 16) |
                    (s->cp_ring.enabled << 17);
            break;
            
        case GPU_REG_INTERRUPT:
            value = s->interrupt_status;
            break;
            
        case GPU_REG_CP_RB_BASE:
            value = s->cp_ring.base;
            break;
            
        case GPU_REG_CP_RB_RPTR:
            value = s->cp_ring.read_ptr;
            break;
            
        case GPU_REG_CP_RB_WPTR:
            value = s->cp_ring.write_ptr;
            break;
            
        case GPU_REG_CP_RB_CNTL:
            value = s->cp_ring.control;
            break;
            
        case GPU_REG_DISPLAY_CTL:
            value = (s->display.enabled << 0) |
                    (s->display.vsync_enabled << 1) |
                    (s->display.hdmi_connected << 2) |
                    (s->display.mode << 8) |
                    (s->display.resolution << 12);
            break;
            
        case GPU_REG_DISPLAY_SIZE:
            value = (s->display.width << 0) |
                    (s->display.height << 16);
            break;
            
        case GPU_REG_PERF_COUNTERS:
            value = s->frame_count & 0xFFFFFFFF;
            break;
            
        default:
            if (reg < GPU_REGISTER_SIZE / 4) {
                value = s->registers[reg];
            }
            break;
    }
    
    return value;
}

static void gpu_mmio_write(void *opaque, hwaddr addr,
                          uint64_t value, unsigned size) {
    Xbox360GPUState *s = opaque;
    uint32_t reg = addr / 4;
    
    switch (addr) {
        case GPU_REG_RESET:
            if (value == 0xDEADBEEF) {
                device_reset(DEVICE(s));
            }
            break;
            
        case GPU_REG_INTERRUPT:
            if (value & 0x80000000) {
                s->interrupt_status &= ~(value & 0x7FFFFFFF);
                xbox360_gpu_update_irq(s);
            } else {
                s->interrupt_mask = value;
            }
            break;
            
        case GPU_REG_CP_RB_BASE:
            s->cp_ring.base = value;
            break;
            
        case GPU_REG_CP_RB_RPTR:
            s->cp_ring.read_ptr = value & (s->cp_ring.size - 1);
            break;
            
        case GPU_REG_CP_RB_WPTR:
            s->cp_ring.write_ptr = value & (s->cp_ring.size - 1);
            if (s->cp_ring.read_ptr != s->cp_ring.write_ptr) {
                xbox360_gpu_ring_buffer_step(s);
            }
            break;
            
        case GPU_REG_CP_RB_CNTL:
            s->cp_ring.control = value;
            s->cp_ring.enabled = (value & 1) != 0;
            if (value & 2) {
                s->cp_ring.size = 1 << ((value >> 8) & 0xF);
            }
            break;
            
        case GPU_REG_3D_SCISSOR:
            s->state_3d.scissor_x = (value >> 0) & 0xFFFF;
            s->state_3d.scissor_y = (value >> 16) & 0xFFFF;
            s->state_3d.scissor_w = 4096;
            s->state_3d.scissor_h = 4096;
            break;
            
        case GPU_REG_3D_VIEWPORT:
            s->state_3d.viewport_x = (value >> 0) & 0xFFFF;
            s->state_3d.viewport_y = (value >> 16) & 0xFFFF;
            s->state_3d.viewport_w = s->display.width;
            s->state_3d.viewport_h = s->display.height;
            break;
            
        case GPU_REG_2D_SRC:
            s->state_2d.src_addr = value;
            break;
            
        case GPU_REG_2D_DST:
            s->state_2d.dst_addr = value;
            break;
            
        case GPU_REG_2D_SIZE:
            s->state_2d.width = (value >> 0) & 0xFFFF;
            s->state_2d.height = (value >> 16) & 0xFFFF;
            break;
            
        case GPU_REG_2D_CTL:
            if (value & 1) {
                xbox360_gpu_blit(s, s->state_2d.src_addr, s->state_2d.dst_addr,
                               s->state_2d.width, s->state_2d.height);
            }
            break;
            
        case GPU_REG_DISPLAY_CTL:
            s->display.enabled = (value & 1) != 0;
            s->display.vsync_enabled = (value & 2) != 0;
            s->display.mode = (value >> 8) & 0xF;
            s->display.resolution = (value >> 12) & 0xF;
            
            switch (s->display.resolution) {
                case RESOLUTION_640x480:
                    s->display.width = 640;
                    s->display.height = 480;
                    break;
                case RESOLUTION_1280x720:
                    s->display.width = 1280;
                    s->display.height = 720;
                    break;
                case RESOLUTION_1920x1080:
                    s->display.width = 1920;
                    s->display.height = 1080;
                    break;
                default:
                    s->display.width = 1280;
                    s->display.height = 720;
                    break;
            }
            
            if (s->display.enabled) {
                xbox360_gpu_display_init(s, s->display.width, s->display.height, 32);
            }
            break;
            
        default:
            if (reg < GPU_REGISTER_SIZE / 4) {
                s->registers[reg] = value;
            }
            break;
    }
}

static const MemoryRegionOps gpu_mmio_ops = {
    .read = gpu_mmio_read,
    .write = gpu_mmio_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

/* ==================== VRAM OPERATIONS ==================== */
static uint64_t gpu_vram_read(void *opaque, hwaddr addr, unsigned size) {
    Xbox360GPUState *s = opaque;
    
    if (addr >= s->vram_size) {
        return 0;
    }
    
    uint8_t *ptr = s->vram_ptr + addr;
    
    switch (size) {
        case 1: return *(uint8_t*)ptr;
        case 2: return *(uint16_t*)ptr;
        case 4: return *(uint32_t*)ptr;
        case 8: return *(uint64_t*)ptr;
        default: return 0;
    }
}

static void gpu_vram_write(void *opaque, hwaddr addr,
                          uint64_t value, unsigned size) {
    Xbox360GPUState *s = opaque;
    
    if (addr >= s->vram_size) {
        return;
    }
    
    uint8_t *ptr = s->vram_ptr + addr;
    
    switch (size) {
        case 1: *(uint8_t*)ptr = value; break;
        case 2: *(uint16_t*)ptr = value; break;
        case 4: *(uint32_t*)ptr = value; break;
        case 8: *(uint64_t*)ptr = value; break;
    }
}

static const MemoryRegionOps gpu_vram_ops = {
    .read = gpu_vram_read,
    .write = gpu_vram_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
    .impl = { .min_access_size = 1, .max_access_size = 8 },
};

/* ==================== VBLANK TIMER ==================== */
static void vblank_callback(void *opaque) {
    Xbox360GPUState *s = opaque;
    
    s->interrupt_status |= GPU_INTR_VBLANK;
    xbox360_gpu_update_irq(s);
    
    s->frame_count++;
    
    if (s->display.enabled) {
        xbox360_gpu_display_update(s);
    }
    
    timer_mod(s->vblank_timer, 
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->vblank_period_ns);
}

/* ==================== COMMAND PROCESSING ==================== */
void xbox360_gpu_process_command(Xbox360GPUState *s, uint32_t cmd) {
    uint32_t type = cmd & 0xFFFF0000;
    uint32_t opcode = cmd & 0x0000FFFF;
    
    switch (type) {
        case 0x00010000: // 3D command
            switch (opcode) {
                case 0x0100: // Draw
                    s->interrupt_status |= GPU_INTR_3D;
                    break;
                case 0x0200: // State update
                    break;
            }
            break;
            
        case 0x00020000: // 2D command
            s->interrupt_status |= GPU_INTR_2D;
            break;
            
        case 0x00030000: // Packet3
            s->interrupt_status |= GPU_INTR_CP;
            break;
            
        default:
            s->interrupt_status |= GPU_INTR_ERROR;
            break;
    }
    
    xbox360_gpu_update_irq(s);
}

void xbox360_gpu_ring_buffer_step(Xbox360GPUState *s) {
    if (!s->cp_ring.enabled || s->cp_ring.read_ptr == s->cp_ring.write_ptr) {
        return;
    }
    
    uint32_t cmd_addr = s->cp_ring.base + s->cp_ring.read_ptr;
    uint32_t cmd = xbox360_gpu_vram_read(s, cmd_addr, 4);
    
    xbox360_gpu_process_command(s, cmd);
    s->cp_ring.read_ptr = (s->cp_ring.read_ptr + 4) & (s->cp_ring.size - 1);
}

void xbox360_gpu_process_command_buffer(Xbox360GPUState *s, uint32_t addr, uint32_t size) {
    for (uint32_t i = 0; i < size; i += 4) {
        uint32_t cmd = xbox360_gpu_vram_read(s, addr + i, 4);
        xbox360_gpu_process_command(s, cmd);
    }
    
    s->interrupt_status |= GPU_INTR_CP;
    xbox360_gpu_update_irq(s);
}

/* ==================== 2D OPERATIONS ==================== */
void xbox360_gpu_blit(Xbox360GPUState *s, uint32_t src, uint32_t dst,
                     uint32_t width, uint32_t height) {
    if (src >= s->vram_size || dst >= s->vram_size) {
        s->interrupt_status |= GPU_INTR_ERROR;
        return;
    }
    
    uint8_t *src_ptr = s->vram_ptr + src;
    uint8_t *dst_ptr = s->vram_ptr + dst;
    uint32_t bytes_per_line = width * 4;
    
    for (uint32_t y = 0; y < height; y++) {
        memcpy(dst_ptr + y * bytes_per_line, 
               src_ptr + y * bytes_per_line, 
               bytes_per_line);
    }
    
    s->interrupt_status |= GPU_INTR_2D;
    xbox360_gpu_update_irq(s);
}

void xbox360_gpu_fill_rect(Xbox360GPUState *s, uint32_t addr, uint32_t color, 
                          uint32_t width, uint32_t height) {
    if (addr >= s->vram_size) {
        s->interrupt_status |= GPU_INTR_ERROR;
        return;
    }
    
    uint32_t *ptr = (uint32_t*)(s->vram_ptr + addr);
    
    for (uint32_t y = 0; y < height; y++) {
        for (uint32_t x = 0; x < width; x++) {
            ptr[y * width + x] = color;
        }
    }
    
    s->interrupt_status |= GPU_INTR_2D;
    xbox360_gpu_update_irq(s);
}

/* ==================== DISPLAY OPERATIONS ==================== */
void xbox360_gpu_display_init(Xbox360GPUState *s, uint32_t width, uint32_t height, uint32_t bpp) {
    s->fb_width = width;
    s->fb_height = height;
    s->fb_pitch = width * (bpp / 8);
    
    if (!s->framebuffer) {
        s->framebuffer = g_malloc0(width * height * sizeof(uint32_t));
    }
    
    s->display.width = width;
    s->display.height = height;
    s->display.pitch = s->fb_pitch;
    s->display.base_addr = 0;
    s->display.format = PIXEL_FORMAT_ARGB8888;
    s->display.enabled = true;
    
    if (!s->ramfb) {
        s->ramfb = ramfb_setup(DEVICE(s), NULL);
    }
}

void xbox360_gpu_display_update(Xbox360GPUState *s) {
    if (!s->display.enabled || !s->framebuffer) {
        return;
    }
    
    uint8_t *vram_fb = s->vram_ptr + s->display.base_addr;
    uint32_t *fb = s->framebuffer;
    
    for (uint32_t y = 0; y < s->display.height; y++) {
        uint32_t *src = (uint32_t*)(vram_fb + y * s->display.pitch);
        uint32_t *dst = fb + y * s->display.width;
        
        for (uint32_t x = 0; x < s->display.width; x++) {
            dst[x] = src[x];
        }
    }
    
    if (s->ramfb && s->console) {
        DisplaySurface *surface = qemu_console_surface(s->console);
        if (surface) {
            ramfb_display_update(s->console, s->ramfb);
        }
    }
}

void xbox360_gpu_display_set_mode(Xbox360GPUState *s, DISPLAY_MODE mode, DISPLAY_RESOLUTION res) {
    s->display.mode = mode;
    s->display.resolution = res;
    
    switch (res) {
        case RESOLUTION_640x480:
            xbox360_gpu_display_init(s, 640, 480, 32);
            break;
        case RESOLUTION_1280x720:
            xbox360_gpu_display_init(s, 1280, 720, 32);
            break;
        case RESOLUTION_1920x1080:
            xbox360_gpu_display_init(s, 1920, 1080, 32);
            break;
        default:
            xbox360_gpu_display_init(s, 1280, 720, 32);
            break;
    }
}

/* ==================== QEMU CONSOLE ==================== */
static const GraphicHwOps gpu_ops = {
    .gfx_update = xbox360_gpu_display_update,
};

/* ==================== DEVICE INITIALIZATION ==================== */
static void gpu_realize(PCIDevice *dev, Error **errp) {
    Xbox360GPUState *s = XBOX360_GPU(dev);
    
    // PCI Configuration
    pci_config_set_vendor_id(dev->config, GPU_VENDOR_ID);
    pci_config_set_device_id(dev->config, GPU_DEVICE_ID);
    pci_config_set_class(dev->config, GPU_CLASS_CODE);
    pci_config_set_revision(dev->config, GPU_REVISION_ID);
    pci_config_set_subsystem_vendor_id(dev->config, GPU_SUBSYSTEM_VENDOR);
    pci_config_set_subsystem_id(dev->config, GPU_SUBSYSTEM_ID);
    
    // BAR0: MMIO registers
    memory_region_init_io(&s->mmio, OBJECT(s), &gpu_mmio_ops, s,
                         "xbox360.gpu.mmio", GPU_REGISTER_SIZE);
    pci_register_bar(dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->mmio);
    
    // BAR1: VRAM
    s->vram_size = VRAM_SIZE;
    s->vram_ptr = g_malloc0(s->vram_size);
    memory_region_init_io(&s->vram, OBJECT(s), &gpu_vram_ops, s,
                         "xbox360.gpu.vram", s->vram_size);
    pci_register_bar(dev, 1, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->vram);
    
    // Initialize state
    s->state = GPU_STATE_RESET;
    s->interrupt_status = 0;
    s->interrupt_mask = 0;
    
    s->cp_ring.base = 0;
    s->cp_ring.size = 4096;
    s->cp_ring.read_ptr = 0;
    s->cp_ring.write_ptr = 0;
    s->cp_ring.control = 0;
    s->cp_ring.enabled = false;
    
    memset(&s->display, 0, sizeof(s->display));
    s->display.mode = DISPLAY_MODE_HDMI;
    s->display.resolution = RESOLUTION_1280x720;
    s->display.enabled = false;
    
    memset(&s->state_3d, 0, sizeof(s->state_3d));
    s->state_3d.scissor_w = 4096;
    s->state_3d.scissor_h = 4096;
    s->state_3d.viewport_w = 1280;
    s->state_3d.viewport_h = 720;
    
    memset(&s->state_2d, 0, sizeof(s->state_2d));
    
    s->frame_count = 0;
    
    // Setup display
    s->console = graphic_console_init(DEVICE(dev), 0, &gpu_ops, s);
    s->vblank_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, vblank_callback, s);
    s->vblank_period_ns = VBLANK_PERIOD_NS;
    
    // Start vblank timer
    timer_mod(s->vblank_timer, 
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->vblank_period_ns);
}

static void gpu_reset(DeviceState *dev) {
    Xbox360GPUState *s = XBOX360_GPU(dev);
    
    memset(s->registers, 0, sizeof(s->registers));
    
    s->state = GPU_STATE_RESET;
    s->interrupt_status = 0;
    s->interrupt_mask = 0;
    
    s->cp_ring.read_ptr = 0;
    s->cp_ring.write_ptr = 0;
    s->cp_ring.enabled = false;
    
    s->frame_count = 0;
    
    timer_del(s->vblank_timer);
    timer_mod(s->vblank_timer, 
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->vblank_period_ns);
    
    s->state = GPU_STATE_READY;
}

/* ==================== PUBLIC FUNCTIONS ==================== */
void xbox360_gpu_update_irq(Xbox360GPUState *s) {
    uint32_t pending = s->interrupt_status & s->interrupt_mask;
    
    if (pending) {
        pci_irq_assert(PCI_DEVICE(s));
        
        if (s->interrupt_callback) {
            s->interrupt_callback(s->callback_opaque, pending);
        }
    } else {
        pci_irq_deassert(PCI_DEVICE(s));
    }
}

void *xbox360_gpu_vram_ptr(Xbox360GPUState *s, uint32_t offset) {
    if (offset >= s->vram_size) {
        return NULL;
    }
    return s->vram_ptr + offset;
}

uint32_t xbox360_gpu_vram_read(Xbox360GPUState *s, uint32_t offset, unsigned size) {
    if (offset >= s->vram_size) {
        return 0;
    }
    
    uint8_t *ptr = s->vram_ptr + offset;
    
    switch (size) {
        case 1: return *(uint8_t*)ptr;
        case 2: return *(uint16_t*)ptr;
        case 4: return *(uint32_t*)ptr;
        default: return 0;
    }
}

void xbox360_gpu_vram_write(Xbox360GPUState *s, uint32_t offset,
                           uint32_t value, unsigned size) {
    if (offset >= s->vram_size) {
        return;
    }
    
    uint8_t *ptr = s->vram_ptr + offset;
    
    switch (size) {
        case 1: *(uint8_t*)ptr = value; break;
        case 2: *(uint16_t*)ptr = value; break;
        case 4: *(uint32_t*)ptr = value; break;
    }
}

void xbox360_gpu_set_interrupt_callback(Xbox360GPUState *s,
                                       void (*callback)(void *, uint32_t),
                                       void *opaque) {
    s->interrupt_callback = callback;
    s->callback_opaque = opaque;
}

Xbox360GPUState *xbox360_gpu_create(PCIBus *bus, 
                                    MemoryRegion *parent_mem,
                                    MemoryRegion *parent_io) {
    PCIDevice *dev;
    Xbox360GPUState *s;
    
    dev = pci_create_simple(bus, -1, TYPE_XBOX360_GPU);
    s = XBOX360_GPU(dev);
    
    dev->config[PCI_INTERRUPT_PIN] = 1;
    
    return s;
}

/* ==================== QEMU DEVICE ==================== */
static void gpu_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);
    
    k->realize = gpu_realize;
    k->vendor_id = GPU_VENDOR_ID;
    k->device_id = GPU_DEVICE_ID;
    k->revision = GPU_REVISION_ID;
    k->class_id = GPU_CLASS_CODE;
    k->subsystem_vendor_id = GPU_SUBSYSTEM_VENDOR;
    k->subsystem_id = GPU_SUBSYSTEM_ID;
    
    dc->reset = gpu_reset;
    dc->desc = "Xbox 360 GPU (Xenos)";
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
}

static const TypeInfo gpu_type_info = {
    .name = TYPE_XBOX360_GPU,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(Xbox360GPUState),
    .class_init = gpu_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { }
    },
};

static void gpu_register_types(void) {
    type_register_static(&gpu_type_info);
}

type_init(gpu_register_types);
