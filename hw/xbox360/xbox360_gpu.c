// QEMU360 - the gratest emulator of all the time (ironic)
// building high quality content lol (900 lines)
#include "hw/xbox360/xbox360_gpu.h"
#include "hw/xbox360/xbox360.h"
#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "hw/irq.h"
#include "hw/pci/pci.h"
#include "hw/display/ramfb.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "migration/vmstate.h"
#include "trace.h"
#include "sysemu/reset.h"

/* ==================== DEFINATIONS ==================== */
#define GPU_VENDOR_ID         0x1002    // ATI/AMD
#define GPU_DEVICE_ID         0x0218    // Xenos (R500)

#define GPU_CLASS_CODE        0x030000  // Display controller
#define GPU_REVISION_ID       0x00
#define GPU_SUBSYSTEM_VENDOR  0x10DE    // Microsoft
#define GPU_SUBSYSTEM_ID      0x02A1

#define VBLANK_PERIOD_NS      16666666  // 16.67ms

#define DEFAULT_WIDTH         1280
#define DEFAULT_HEIGHT        720
#define DEFAULT_BPP           32
#define DEFAULT_PITCH         (DEFAULT_WIDTH * 4)

#define CMD_TYPE_NOP          0x00000000
#define CMD_TYPE_3D           0x00010000
#define CMD_TYPE_2D           0x00020000
#define CMD_TYPE_PACKET3      0x00030000
#define CMD_TYPE_INDIRECT     0x00040000

#define CMD_3D_DRAW           0x00000100
#define CMD_3D_STATE          0x00000200
#define CMD_3D_TEXTURE        0x00000300
#define CMD_3D_SHADER         0x00000400

#define CMD_2D_BLIT           0x00000100
#define CMD_2D_FILL           0x00000200
#define CMD_2D_STRETCH        0x00000300

#define COLOR_BLACK           0xFF000000
#define COLOR_WHITE           0xFFFFFFFF
#define COLOR_RED             0xFFFF0000
#define COLOR_GREEN           0xFF00FF00
#define COLOR_BLUE            0xFF0000FF
#define COLOR_GRAY            0xFF808080

/* ==================== MEMORY REGION OPS ==================== */
static uint64_t xbox360_gpu_mmio_read(void *opaque, hwaddr addr, unsigned size) {
    Xbox360GPUState *s = opaque;
    uint32_t value = 0;
    uint32_t reg = addr / 4;
    
    if (addr >= GPU_REGISTER_SIZE) {
        trace_xbox360_gpu_mmio_read_invalid(addr);
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
            
        case GPU_REG_3D_SCISSOR:
            value = (s->state_3d.scissor_x << 0) |
                    (s->state_3d.scissor_y << 16);
            break;
            
        case GPU_REG_3D_VIEWPORT:
            value = (s->state_3d.viewport_x << 0) |
                    (s->state_3d.viewport_y << 16);
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
            
        case GPU_REG_DISPLAY_MODE:
            value = (s->display.refresh_rate << 0) |
                    (s->display.format << 8);
            break;
            
        case GPU_REG_PERF_COUNTERS:
            if (addr == GPU_REG_PERF_COUNTERS) {
                value = s->frame_count & 0xFFFFFFFF;
            } else if (addr == GPU_REG_PERF_COUNTERS + 4) {
                value = (s->frame_count >> 32) & 0xFFFFFFFF;
            }
            break;
            
        default:
            // Normal Register
            if (reg < GPU_REGISTER_SIZE / 4) {
                value = s->registers[reg];
            }
            break;
    }
    
    trace_xbox360_gpu_mmio_read(addr, size, value);
    return value;
}

static void xbox360_gpu_mmio_write(void *opaque, hwaddr addr,
                                  uint64_t value, unsigned size) {
    Xbox360GPUState *s = opaque;
    uint32_t reg = addr / 4;
    
    trace_xbox360_gpu_mmio_write(addr, value, size);
    
    switch (addr) {
        case GPU_REG_RESET:
            if (value == 0xDEADBEEF) {
                trace_xbox360_gpu_reset();
                xbox360_gpu_reset(s);
            }
            break;
            
        case GPU_REG_INTERRUPT:
            if (value & 0x80000000) {
                // Clear interrupts
                s->interrupt_status &= ~(value & 0x7FFFFFFF);
                xbox360_gpu_update_irq(s);
            } else {
                // Set interrupt mask
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
            if (value & 2) { // Buffer size
                s->cp_ring.size = 1 << ((value >> 8) & 0xF);
            }
            break;
            
        case GPU_REG_3D_SCISSOR:
            s->state_3d.scissor_x = (value >> 0) & 0xFFFF;
            s->state_3d.scissor_y = (value >> 16) & 0xFFFF;
            s->state_3d.scissor_w = 4096; // Default
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
            if (value & 1) { // Execute blit
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
                case RESOLUTION_720x480:
                    s->display.width = 720;
                    s->display.height = 480;
                    break;
                case RESOLUTION_720x576:
                    s->display.width = 720;
                    s->display.height = 576;
                    break;
                case RESOLUTION_1280x720:
                    s->display.width = 1280;
                    s->display.height = 720;
                    break;
                case RESOLUTION_1920x1080:
                    s->display.width = 1920;
                    s->display.height = 1080;
                    break;
            }
            
            if (s->display.enabled) {
                xbox360_gpu_display_init(s, s->display.width, s->display.height, 32);
            }
            break;
            
        case GPU_REG_DISPLAY_SIZE:
            s->display.width = (value >> 0) & 0xFFFF;
            s->display.height = (value >> 16) & 0xFFFF;
            break;
            
        case GPU_REG_DISPLAY_MODE:
            s->display.refresh_rate = (value >> 0) & 0xFF;
            s->display.format = (value >> 8) & 0xFF;
            break;
            
        case GPU_REG_SCRATCH:
            if (value == 0x12345678) {
                printf("[GPU] Debug command received\n");
                xbox360_gpu_print_state(s);
            }
            break;
            
        default:
            // Normal Register
            if (reg < GPU_REGISTER_SIZE / 4) {
                s->registers[reg] = value;
            }
            break;
    }
}

static const MemoryRegionOps xbox360_gpu_mmio_ops = {
    .read = xbox360_gpu_mmio_read,
    .write = xbox360_gpu_mmio_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/* ==================== VRAM OPERATIONS ==================== */
static uint64_t xbox360_gpu_vram_read(void *opaque, hwaddr addr, unsigned size) {
    Xbox360GPUState *s = opaque;
    uint64_t value = 0;
    
    if (addr >= s->vram_size) {
        trace_xbox360_gpu_vram_read_invalid(addr);
        return 0;
    }
    
    uint8_t *ptr = s->vram_ptr + addr;
    
    switch (size) {
        case 1:
            value = *(uint8_t*)ptr;
            break;
        case 2:
            value = *(uint16_t*)ptr;
            break;
        case 4:
            value = *(uint32_t*)ptr;
            break;
        case 8:
            value = *(uint64_t*)ptr;
            break;
        default:
            for (unsigned i = 0; i < size; i++) {
                value |= (uint64_t)ptr[i] << (i * 8);
            }
            break;
    }
    
    trace_xbox360_gpu_vram_read(addr, size, value);
    return value;
}

static void xbox360_gpu_vram_write(void *opaque, hwaddr addr,
                                  uint64_t value, unsigned size) {
    Xbox360GPUState *s = opaque;
    
    if (addr >= s->vram_size) {
        trace_xbox360_gpu_vram_write_invalid(addr, value, size);
        return;
    }
    
    uint8_t *ptr = s->vram_ptr + addr;
    trace_xbox360_gpu_vram_write(addr, value, size);
    
    switch (size) {
        case 1:
            *(uint8_t*)ptr = value;
            break;
        case 2:
            *(uint16_t*)ptr = value;
            break;
        case 4:
            *(uint32_t*)ptr = value;
            break;
        case 8:
            *(uint64_t*)ptr = value;
            break;
        default:
            for (unsigned i = 0; i < size; i++) {
                ptr[i] = (value >> (i * 8)) & 0xFF;
            }
            break;
    }
    
    if (s->display.enabled && addr >= s->display.base_addr && 
        addr < s->display.base_addr + s->display.height * s->display.pitch) {
        // Mark to Update
        // xbox360_gpu_display_update(s);
    }
}

static const MemoryRegionOps xbox360_gpu_vram_ops = {
    .read = xbox360_gpu_vram_read,
    .write = xbox360_gpu_vram_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

/* ==================== VBLANK TIMER ==================== */
static void xbox360_gpu_vblank_callback(void *opaque) {
    Xbox360GPUState *s = opaque;
    
    s->interrupt_status |= GPU_INTR_VBLANK;
    xbox360_gpu_update_irq(s);
    
    s->frame_count++;
    
    if (s->display.enabled) {
        xbox360_gpu_display_update(s);
    }
    
    timer_mod(s->vblank_timer, 
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->vblank_period_ns);
    
    trace_xbox360_gpu_vblank(s->frame_count);
}

/* ==================== COMMAND PROCESSING ==================== */
void xbox360_gpu_process_command(Xbox360GPUState *s, uint32_t cmd) {
    uint32_t type = cmd & 0xFFFF0000;
    uint32_t opcode = cmd & 0x0000FFFF;
    
    trace_xbox360_gpu_command(cmd);
    
    switch (type) {
        case CMD_TYPE_NOP:
            // No operation
            break;
            
        case CMD_TYPE_3D:
            switch (opcode) {
                case CMD_3D_DRAW:
                    // Stub - only log
                    printf("[GPU] 3D Draw command\n");
                    s->interrupt_status |= GPU_INTR_3D;
                    break;
                    
                case CMD_3D_STATE:
                    printf("[GPU] 3D State update\n");
                    break;
                    
                default:
                    printf("[GPU] Unknown 3D command: 0x%08X\n", cmd);
                    break;
            }
            break;
            
        case CMD_TYPE_2D:
            switch (opcode) {
                case CMD_2D_BLIT:
                    s->interrupt_status |= GPU_INTR_2D;
                    break;
                    
                case CMD_2D_FILL:
                    printf("[GPU] 2D Fill command\n");
                    s->interrupt_status |= GPU_INTR_2D;
                    break;
                    
                default:
                    printf("[GPU] Unknown 2D command: 0x%08X\n", cmd);
                    break;
            }
            break;
            
        case CMD_TYPE_PACKET3:
            // Packet3 commands (more complex*ity* *side*)
            printf("[GPU] Packet3 command: 0x%08X\n", cmd);
            s->interrupt_status |= GPU_INTR_CP;
            break;
            
        default:
            printf("[GPU] Unknown command type: 0x%08X\n", cmd);
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
    
    if (s->cp_ring.read_ptr != s->cp_ring.write_ptr) {
        // (Simulated)
        timer_mod(s->vblank_timer, 
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000); // 1ms
    }
}

void xbox360_gpu_process_command_buffer(Xbox360GPUState *s, uint32_t addr, uint32_t size) {
    printf("[GPU] Processing command buffer @ 0x%08X, size: %d\n", addr, size);
    
    for (uint32_t i = 0; i < size; i += 4) {
        uint32_t cmd = xbox360_gpu_vram_read(s, addr + i, 4);
        xbox360_gpu_process_command(s, cmd);
    }
    
    s->interrupt_status |= GPU_INTR_CP;
    xbox360_gpu_update_irq(s);
}

/* ==================== 2D OPERATIONS ==================== */
void xbox360_gpu_blit(Xbox360GPUState *s, 
                     uint32_t src, uint32_t dst,
                     uint32_t width, uint32_t height) {
    if (src >= s->vram_size || dst >= s->vram_size) {
        printf("[GPU] Invalid blit addresses: src=0x%08X, dst=0x%08X\n", src, dst);
        s->interrupt_status |= GPU_INTR_ERROR;
        return;
    }
    
    uint8_t *src_ptr = s->vram_ptr + src;
    uint8_t *dst_ptr = s->vram_ptr + dst;
    
    // Assume 32bpp (4 bytes per pixel)
    uint32_t bytes_per_line = width * 4;
    
    for (uint32_t y = 0; y < height; y++) {
        memcpy(dst_ptr + y * bytes_per_line, 
               src_ptr + y * bytes_per_line, 
               bytes_per_line);
    }
    
    printf("[GPU] Blit: %dx%d from 0x%08X to 0x%08X\n", 
           width, height, src, dst);
    
    s->interrupt_status |= GPU_INTR_2D;
    xbox360_gpu_update_irq(s);
}

void xbox360_gpu_fill_rect(Xbox360GPUState *s, uint32_t addr, uint32_t color, uint32_t width, uint32_t height) {
    if (addr >= s->vram_size) {
        printf("[GPU] Invalid fill address: 0x%08X\n", addr);
        s->interrupt_status |= GPU_INTR_ERROR;
        return;
    }
    
    uint32_t *ptr = (uint32_t*)(s->vram_ptr + addr);
    
    for (uint32_t y = 0; y < height; y++) {
        for (uint32_t x = 0; x < width; x++) {
            ptr[y * width + x] = color;
        }
    }
    
    printf("[GPU] Fill rect: %dx%d @ 0x%08X with 0x%08X\n", 
           width, height, addr, color);
    
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
    
    printf("[GPU] Display initialized: %dx%d, %dbpp\n", width, height, bpp);
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
            uint32_t pixel = src[x];
            // Format: 0xAARRGGBB
            dst[x] = pixel;
        }
    }
    
    if (s->ramfb && s->console) {
        DisplaySurface *surface = qemu_console_surface(s->console);
        if (surface) {
            ramfb_display_update(s->console, s->ramfb);
        }
    }
    
    trace_xbox360_gpu_display_update(s->display.width, s->display.height);
}

void xbox360_gpu_display_set_mode(Xbox360GPUState *s, DISPLAY_MODE mode, DISPLAY_RESOLUTION res) {
    s->display.mode = mode;
    s->display.resolution = res;
    
    // Update Dimensions
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
    
    printf("[GPU] Display mode set: %d, resolution: %d\n", mode, res);
}

/* ==================== QEMU CONSOLE CALLBACKS ==================== */
static const GraphicHwOps xbox360_gpu_ops = {
    .gfx_update = xbox360_gpu_display_update,
    .text_update = NULL,
    .ui_info = NULL,
    .get_resolution = NULL,
    .get_flags = NULL,
};

/* ==================== INITIALIZATION ==================== */
static void xbox360_gpu_realize(PCIDevice *dev, Error **errp) {
    Xbox360GPUState *s = XBOX360_GPU(dev);
    Error *local_err = NULL;
    
    printf("[GPU] Xbox 360 GPU (Xenos) initializing...\n");
    
    pci_config_set_vendor_id(dev->config, GPU_VENDOR_ID);
    pci_config_set_device_id(dev->config, GPU_DEVICE_ID);
    pci_config_set_class(dev->config, GPU_CLASS_CODE);
    pci_config_set_revision(dev->config, GPU_REVISION_ID);
    pci_config_set_subsystem_vendor_id(dev->config, GPU_SUBSYSTEM_VENDOR);
    pci_config_set_subsystem_id(dev->config, GPU_SUBSYSTEM_ID);
    
    // BAR0: MMIO registers (256MB)
    memory_region_init_io(&s->mmio, OBJECT(s), &xbox360_gpu_mmio_ops, s,
                          "xbox360.gpu.mmio", GPU_REGISTER_SIZE);
    pci_register_bar(dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->mmio);
    
    s->vram_size = VRAM_SIZE;
    s->vram_ptr = g_malloc0(s->vram_size);
    memory_region_init_io(&s->vram, OBJECT(s), &xbox360_gpu_vram_ops, s, "xbox360.gpu.vram", s->vram_size);
    pci_register_bar(dev, 1, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->vram);
    
    s->state = GPU_STATE_RESET;
    s->interrupt_status = 0;
    s->interrupt_mask = 0;
    
    s->cp_ring.base = 0;
    s->cp_ring.size = 4096; // 4KB default
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
    s->state_3d.depth_near = 0.0f;
    s->state_3d.depth_far = 1.0f;
    
    memset(&s->state_2d, 0, sizeof(s->state_2d));
    
    memset(s->surfaces, 0, sizeof(s->surfaces));
    s->current_surface = 0;
    
    s->frame_count = 0;
    s->vertex_count = 0;
    s->triangle_count = 0;
    s->pixel_count = 0;
    
    s->framebuffer = NULL;
    s->fb_width = 0;
    s->fb_height = 0;
    s->fb_pitch = 0;
    
    s->console = graphic_console_init(DEVICE(dev), 0, &xbox360_gpu_ops, s);
    s->vblank_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xbox360_gpu_vblank_callback, s);
    s->vblank_period_ns = VBLANK_PERIOD_NS;
    
    // Debug
    s->debug_enabled = false;
    s->cmd_log = NULL;
    
    // Callbacks
    s->interrupt_callback = NULL;
    s->callback_opaque = NULL;
    
    printf("[GPU] Initialized successfully\n");
    printf("      VRAM: %zu MB, MMIO: %u MB\n", 
           s->vram_size / (1024*1024), GPU_REGISTER_SIZE / (1024*1024));
}

static void xbox360_gpu_reset(DeviceState *dev) {
    Xbox360GPUState *s = XBOX360_GPU(dev);
    printf("[GPU] Reset\n");
    
    memset(s->registers, 0, sizeof(s->registers));
    
    s->state = GPU_STATE_RESET;
    s->interrupt_status = 0;
    s->interrupt_mask = 0;
    
    s->cp_ring.read_ptr = 0;
    s->cp_ring.write_ptr = 0;
    s->cp_ring.enabled = false;
    
    s->frame_count = 0;
    s->vertex_count = 0;
    s->triangle_count = 0;
    s->pixel_count = 0;
    
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
        
        trace_xbox360_gpu_irq_assert(pending);
    } else {
        pci_irq_deassert(PCI_DEVICE(s));
        trace_xbox360_gpu_irq_deassert();
    }
}

void *xbox360_gpu_vram_ptr(Xbox360GPUState *s, uint32_t offset) {
    if (offset >= s->vram_size) {
        return NULL;
    }
    return s->vram_ptr + offset;
}

uint32_t xbox360_gpu_vram_read(Xbox360GPUState *s, uint32_t offset, 
                              unsigned size) {
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

void xbox360_gpu_dump_registers(Xbox360GPUState *s) {
    printf("\n=== GPU REGISTER DUMP ===\n");
    printf("GPU ID:       0x%08X\n", GPU_XENOS_ID);
    printf("Status:       0x%08X (state=%d)\n", s->registers[GPU_REG_STATUS / 4], s->state);
    printf("Interrupt:    0x%08X (mask=0x%08X)\n", s->interrupt_status, s->interrupt_mask);
    printf("CP Ring:      base=0x%08X, rptr=0x%08X, wptr=0x%08X\n", s->cp_ring.base, s->cp_ring.read_ptr, s->cp_ring.write_ptr);
    printf("Display:      %dx%d, enabled=%d, mode=%d\n", s->display.width, s->display.height, s->display.enabled, s->display.mode);
    printf("Frame count:  %" PRIu64 "\n", s->frame_count);
    printf("=======================\n");
}

void xbox360_gpu_print_state(Xbox360GPUState *s) {
    printf("[GPU] State: ");
    switch (s->state) {
        case GPU_STATE_RESET: printf("RESET"); break;
        case GPU_STATE_IDLE: printf("IDLE"); break;
        case GPU_STATE_INITIALIZING: printf("INIT"); break;
        case GPU_STATE_READY: printf("READY"); break;
        case GPU_STATE_RENDERING: printf("RENDERING"); break;
        case GPU_STATE_ERROR: printf("ERROR"); break;
    }
    
    printf(", IRQ: 0x%08X, Display: %dx%d, Frames: %" PRIu64 "\n",
           s->interrupt_status & s->interrupt_mask,
           s->display.width, s->display.height,
           s->frame_count);
}

void xbox360_gpu_set_interrupt_callback(Xbox360GPUState *s,
                                       void (*callback)(void *, uint32_t),
                                       void *opaque) {
    s->interrupt_callback = callback;
    s->callback_opaque = opaque;
}

void xbox360_gpu_reset_stats(Xbox360GPUState *s) {
    s->frame_count = 0;
    s->vertex_count = 0;
    s->triangle_count = 0;
    s->pixel_count = 0;
}

void xbox360_gpu_print_stats(Xbox360GPUState *s) {
    printf("[GPU] Stats: Frames=%" PRIu64 ", Vertices=%" PRIu64 
           ", Triangles=%" PRIu64 ", Pixels=%" PRIu64 "\n",
           s->frame_count, s->vertex_count, 
           s->triangle_count, s->pixel_count);
}

/* ==================== QEMU DEVICE ==================== */
static void xbox360_gpu_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);
    
    k->realize = xbox360_gpu_realize;
    k->vendor_id = GPU_VENDOR_ID;
    k->device_id = GPU_DEVICE_ID;
    k->revision = GPU_REVISION_ID;
    k->class_id = GPU_CLASS_CODE;
    k->subsystem_vendor_id = GPU_SUBSYSTEM_VENDOR;
    k->subsystem_id = GPU_SUBSYSTEM_ID;
    
    dc->reset = xbox360_gpu_reset;
    dc->desc = "Xbox 360 GPU (Xenos)";
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
}

static const TypeInfo xbox360_gpu_type_info = {
    .name = TYPE_XBOX360_GPU,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(Xbox360GPUState),
    .class_init = xbox360_gpu_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { }
    },
};

static void xbox360_gpu_register_types(void) {
    type_register_static(&xbox360_gpu_type_info);
}

type_init(xbox360_gpu_register_types);

/* ==================== CREATION FUNCTION ==================== */
Xbox360GPUState *xbox360_gpu_create(PCIBus *bus, 
                                    MemoryRegion *parent_mem,
                                    MemoryRegion *parent_io) {
    PCIDevice *dev;
    Xbox360GPUState *s;
    
    dev = pci_create_simple(bus, -1, TYPE_XBOX360_GPU);
    s = XBOX360_GPU(dev);
    
    pci_dev->config[PCI_INTERRUPT_PIN] = 1; // Use INTA
    
    printf("[GPU] Created on PCI bus\n");
    return s;
}
