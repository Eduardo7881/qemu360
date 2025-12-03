#ifndef HW_XBOX360_GPU_H
#define HW_XBOX360_GPU_H

#include "hw/sysbus.h"
#include "qom/object.h"
#include "hw/pci/pci.h"
#include "hw/display/ramfb.h"
#include "ui/console.h"

/* ==================== GPU REGISTERS ==================== */
#define GPU_BASE_ADDRESS          0xC0000000
#define GPU_REGISTER_SIZE         0x10000000  /* 256MB space of registers  */

// Registers (offsets of GPU_BASE_ADDRESS)
#define GPU_REG_ID                0x0000      /* GPU ID */
#define GPU_REG_STATUS            0x0004      /* Status */
#define GPU_REG_INTERRUPT         0x0008      /* Interrupt control */
#define GPU_REG_RESET             0x000C      /* Reset control */
#define GPU_REG_CONFIG            0x0010      /* Configuration */
#define GPU_REG_CLOCK             0x0014      /* Clock control */
#define GPU_REG_MEMORY            0x0018      /* Memory controller */
#define GPU_REG_SCRATCH           0x001C      /* Scratch register */

// Command processor
#define GPU_REG_CP_RB_BASE        0x0100      /* Ring buffer base */
#define GPU_REG_CP_RB_RPTR        0x0104      /* Read pointer */
#define GPU_REG_CP_RB_WPTR        0x0108      /* Write pointer */
#define GPU_REG_CP_RB_CNTL        0x010C      /* Control */

// 3D engine
#define GPU_REG_3D_SCISSOR        0x0200      /* Scissor rectangle */
#define GPU_REG_3D_VIEWPORT       0x0204      /* Viewport */
#define GPU_REG_3D_DEPTH          0x0208      /* Depth control */
#define GPU_REG_3D_STENCIL        0x020C      /* Stencil control */
#define GPU_REG_3D_ALPHA          0x0210      /* Alpha blend */
#define GPU_REG_3D_COLOR          0x0214      /* Color control */

// 2D engine (BLT)
#define GPU_REG_2D_SRC            0x0300      /* Source address */
#define GPU_REG_2D_DST            0x0304      /* Destination address */
#define GPU_REG_2D_SIZE           0x0308      /* Size */
#define GPU_REG_2D_CTL            0x030C      /* Control */

// Display controller
#define GPU_REG_DISPLAY_BASE      0x0400
#define GPU_REG_DISPLAY_CTL       0x0400      /* Display control */
#define GPU_REG_DISPLAY_MODE      0x0404      /* Display mode */
#define GPU_REG_DISPLAY_SIZE      0x0408      /* Display size */
#define GPU_REG_DISPLAY_SYNC      0x040C      /* Sync control */
#define GPU_REG_DISPLAY_HDMI      0x0410      /* HDMI control */

// Surface/Texture
#define GPU_REG_SURFACE_BASE      0x0500
#define GPU_REG_TEXTURE_BASE      0x0600

// Performance counters
#define GPU_REG_PERF_COUNTERS     0x0700

// VRAM Address (512MB)
#define VRAM_BASE_ADDRESS         0xC0000000
#define VRAM_SIZE                 (512 * 1024 * 1024)  // 512MB

/* ==================== GPU CONSTANTS ==================== */
#define GPU_XENOS_ID              0x02180002

typedef enum {
  GPU_STATE_RESET = 0,
  GPU_STATE_IDLE,
  GPU_STATE_INITIALIZING,
  GPU_STATE_READY,
  GPU_STATE_RENDERING,
  GPU_STATE_ERROR,
} GPU_STATE;

typedef enum {
    GPU_INTR_NONE = 0,
    GPU_INTR_VBLANK = 1 << 0,        // Vertical blank
    GPU_INTR_HBLANK = 1 << 1,        // Horizontal blank
    GPU_INTR_DMA = 1 << 2,           // DMA complete
    GPU_INTR_CP = 1 << 3,            // Command processor
    GPU_INTR_3D = 1 << 4,            // 3D engine
    GPU_INTR_2D = 1 << 5,            // 2D engine
    GPU_INTR_ERROR = 1 << 6,         // Error
    GPU_INTR_USER = 1 << 7,          // User interrupt
} GPU_INTERRUPT;

typedef enum {
    DISPLAY_MODE_NONE = 0,
    DISPLAY_MODE_VGA = 1,
    DISPLAY_MODE_DVI = 2,
    DISPLAY_MODE_HDMI = 3,
    DISPLAY_MODE_COMPOSITE = 4,
    DISPLAY_MODE_COMPONENT = 5,
} DISPLAY_MODE;

typedef enum {
    RESOLUTION_640x480 = 0,
    RESOLUTION_720x480 = 1,
    RESOLUTION_720x576 = 2,
    RESOLUTION_1280x720 = 3,
    RESOLUTION_1920x1080 = 4,
} DISPLAY_RESOLUTION;

typedef enum {
    PIXEL_FORMAT_ARGB8888 = 0,
    PIXEL_FORMAT_RGB565 = 1,
    PIXEL_FORMAT_DXT1 = 2,
    PIXEL_FORMAT_DXT5 = 3,
} PIXEL_FORMAT;

/* ==================== STRUCTURES ==================== */
typedef struct GPU_RING_BUFFER {
    uint32_t base;          // Base address in VRAM
    uint32_t size;          // Size in bytes
    uint32_t read_ptr;      // Read pointer
    uint32_t write_ptr;     // Write pointer
    uint32_t control;       // Control register
    bool enabled;           // Buffer enabled
} GPU_RING_BUFFER;

typedef struct GPU_DISPLAY {
    DISPLAY_MODE mode;
    DISPLAY_RESOLUTION resolution;
    uint32_t width;
    uint32_t height;
    uint32_t refresh_rate;
    uint32_t base_addr;     // Framebuffer address in VRAM
    uint32_t pitch;         // Bytes per line
    PIXEL_FORMAT format;
    bool enabled;
    bool vsync_enabled;
    bool hdmi_connected;
} GPU_DISPLAY;

typedef struct GPU_3D_STATE {
    uint32_t scissor_x, scissor_y, scissor_w, scissor_h;
    uint32_t viewport_x, viewport_y, viewport_w, viewport_h;
    float depth_near, depth_far;
    uint32_t stencil_func, stencil_ref;
    uint32_t blend_src, blend_dst;
    uint32_t color_mask;
    bool depth_test;
    bool stencil_test;
    bool alpha_test;
    bool blending;
} GPU_3D_STATE;

typedef struct GPU_2D_STATE {
    uint32_t src_addr;
    uint32_t dst_addr;
    uint32_t width, height;
    uint32_t src_pitch, dst_pitch;
    PIXEL_FORMAT src_format, dst_format;
    bool blending;
    bool color_key;
    uint32_t color_key_value;
} GPU_2D_STATE;

typedef struct GPU_SURFACE {
    uint32_t address;
    uint32_t pitch;
    uint32_t width, height;
    PIXEL_FORMAT format;
    uint32_t mip_levels;
    bool render_target;
    bool depth_stencil;
} GPU_SURFACE;

/* ==================== GPU STATE ==================== */
#define TYPE_XBOX360_GPU "xbox360.gpu"
OBJECT_DECLARE_SIMPLE_TYPE(Xbox360GPUState, XBOX360_GPU)

struct Xbox360GPUState {
    /*< private >*/
    PCIDevice parent_obj;
    
    /*< public >*/
    // Memory regions
    MemoryRegion mmio;
    MemoryRegion vram;
    MemoryRegion rom;       // GPU ROM (microcode)
    
    // VRAM
    uint8_t *vram_ptr;
    size_t vram_size;
    
    // Registers
    uint32_t registers[GPU_REGISTER_SIZE / 4];
    
    // State
    GPU_STATE state;
    uint32_t interrupt_status;
    uint32_t interrupt_mask;
    
    // Components
    GPU_RING_BUFFER cp_ring;    // Command processor ring buffer
    GPU_DISPLAY display;
    GPU_3D_STATE state_3d;
    GPU_2D_STATE state_2d;
    
    // Surfaces (simplified)
    GPU_SURFACE surfaces[16];
    uint32_t current_surface;
    
    // Performance
    uint64_t frame_count;
    uint64_t vertex_count;
    uint64_t triangle_count;
    uint64_t pixel_count;
    
    // Framebuffer for display
    uint32_t *framebuffer;
    uint32_t fb_width;
    uint32_t fb_height;
    uint32_t fb_pitch;
    
    // QEMU display
    QemuConsole *console;
    RAMFBState *ramfb;
    
    // Timing
    QEMUTimer *vblank_timer;
    uint64_t vblank_period_ns;  // Nanoseconds per vblank
    
    // Debug
    bool debug_enabled;
    FILE *cmd_log;
    
    // Callbacks
    void (*interrupt_callback)(void *opaque, uint32_t interrupts);
    void *callback_opaque;
};

/* ==================== PUBLIC FUNCTIONS ==================== */
Xbox360GPUState *xbox360_gpu_create(PCIBus *bus, MemoryRegion *parent_mem, MemoryRegion *parent_io);

void xbox360_gpu_reset(Xbox360GPUState *s);
void xbox360_gpu_update_irq(Xbox360GPUState *s);

void *xbox360_gpu_vram_ptr(Xbox360GPUState *s, uint32_t offset);
uint32_t xbox360_gpu_vram_read(Xbox360GPUState *s, uint32_t offset, unsigned size);
void xbox360_gpu_vram_write(Xbox360GPUState *s, uint32_t offset, uint32_t value, unsigned size);

void xbox360_gpu_process_command(Xbox360GPUState *s, uint32_t cmd);
void xbox360_gpu_process_command_buffer(Xbox360GPUState *s, uint32_t addr, uint32_t size);
void xbox360_gpu_ring_buffer_step(Xbox360GPUState *s);

void xbox360_gpu_display_init(Xbox360GPUState *s, uint32_t width, uint32_t height, uint32_t bpp);
void xbox360_gpu_display_update(Xbox360GPUState *s);
void xbox360_gpu_display_set_mode(Xbox360GPUState *s, DISPLAY_MODE mode, DISPLAY_RESOLUTION res);

void xbox360_gpu_blit(Xbox360GPUState *s, uint32_t src, uint32_t dst, uint32_t width, uint32_t height);
void xbox360_gpu_fill_rect(Xbox360GPUState *s, uint32_t addr, uint32_t color, uint32_t width, uint32_t height);

void xbox360_gpu_draw_triangles(Xbox360GPUState *s, uint32_t vertex_addr, uint32_t vertex_count, uint32_t index_addr, uint32_t index_count);

void xbox360_gpu_dump_registers(Xbox360GPUState *s);
void xbox360_gpu_dump_vram(Xbox360GPUState *s, uint32_t addr, uint32_t size);
void xbox360_gpu_print_state(Xbox360GPUState *s);

void xbox360_gpu_set_interrupt_callback(Xbox360GPUState *s, void (*callback)(void *, uint32_t), void *opaque);

void xbox360_gpu_reset_stats(Xbox360GPUState *s);
void xbox360_gpu_print_stats(Xbox360GPUState *s);

#endif
