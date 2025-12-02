#ifndef HW_XBOX360_KERNEL_H
#define HW_XBOX360_KERNEL_H

#include "hw/xbox360/xbox360.h"
#include "hw/xbox360/crypto/xe_crypto.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== SYSCALL NUMBERS ==================== */
#define SYSCALL_GET_TICK_COUNT           0x00000001
#define SYSCALL_QUERY_PERFORMANCE_COUNTER 0x00000002
#define SYSCALL_QUERY_PERFORMANCE_FREQUENCY 0x00000003
#define SYSCALL_KE_DELAY_EXECUTION_THREAD 0x00000004
#define SYSCALL_KE_SET_TIMER             0x00000005
#define SYSCALL_KE_CANCEL_TIMER          0x00000006
#define SYSCALL_EX_CREATE_THREAD         0x00000007
#define SYSCALL_KE_RESUME_THREAD         0x00000008
#define SYSCALL_KE_SUSPEND_THREAD        0x00000009
#define SYSCALL_KE_TERMINATE_THREAD      0x0000000A
#define SYSCALL_KE_GET_CURRENT_PROCESSOR_NUMBER 0x0000000B
#define SYSCALL_KE_SET_PROCESSOR_AFFINITY 0x0000000C
#define SYSCALL_KE_QUERY_ACTIVE_PROCESSORS 0x0000000D

#define SYSCALL_MM_ALLOCATE_PHYSICAL_MEMORY 0x00000020
#define SYSCALL_MM_FREE_PHYSICAL_MEMORY   0x00000021
#define SYSCALL_MM_MAP_IO_SPACE           0x00000022
#define SYSCALL_MM_UNMAP_IO_SPACE         0x00000023
#define SYSCALL_MM_QUERY_ALLOCATION_SIZE  0x00000024
#define SYSCALL_MM_LOCK_PAGES             0x00000025
#define SYSCALL_MM_UNLOCK_PAGES           0x00000026
#define SYSCALL_MM_GET_PHYSICAL_ADDRESS   0x00000027

#define SYSCALL_NT_CREATE_FILE           0x00000040
#define SYSCALL_NT_OPEN_FILE             0x00000041
#define SYSCALL_NT_READ_FILE             0x00000042
#define SYSCALL_NT_WRITE_FILE            0x00000043
#define SYSCALL_NT_CLOSE                 0x00000044
#define SYSCALL_NT_QUERY_INFORMATION_FILE 0x00000045
#define SYSCALL_NT_SET_INFORMATION_FILE  0x00000046
#define SYSCALL_NT_QUERY_VOLUME_INFO     0x00000047
#define SYSCALL_NT_DEVICE_IO_CONTROL     0x00000048

#define SYSCALL_VGPUHD_CREATE_DEVICE     0x00000060
#define SYSCALL_VGPUHD_DESTROY_DEVICE    0x00000061
#define SYSCALL_VGPUHD_FLIP              0x00000062
#define SYSCALL_VGPUHD_SETMODE           0x00000063
#define SYSCALL_VGPUHD_SETSCANLINE       0x00000064
#define SYSCALL_VGPUHD_WAITFORVSYNC      0x00000065

#define SYSCALL_XAUDIO_CREATE            0x00000080
#define SYSCALL_XAUDIO_DESTROY           0x00000081
#define SYSCALL_XAUDIO_SUBMIT_BUFFER     0x00000082
#define SYSCALL_XAUDIO_FLUSH_BUFFERS     0x00000083

#define SYSCALL_XECRYPT_SHA_INIT         0x000000A0
#define SYSCALL_XECRYPT_SHA_UPDATE       0x000000A1
#define SYSCALL_XECRYPT_SHA_FINAL        0x000000A2
#define SYSCALL_XECRYPT_AES_CBC          0x000000A3
#define SYSCALL_XECRYPT_RSA_VERIFY       0x000000A4
#define SYSCALL_XECRYPT_RC4              0x000000A5

#define SYSCALL_XAM_GET_CONSOLETYPE      0x000000C0
#define SYSCALL_XAM_GET_SYSTEMINFO       0x000000C1
#define SYSCALL_XAM_GET_CURRENTTITLEID   0x000000C2
#define SYSCALL_XAM_GET_CURRENTTITLEINFO 0x000000C3
#define SYSCALL_XAM_LAUNCH_TITLE         0x000000C4
#define SYSCALL_XAM_TERMINATE_TITLE      0x000000C5
#define SYSCALL_XAM_LOAD_GAME            0x000000C6

#define SYSCALL_DASHBOARD_GET_VERSION    0x000000E0
#define SYSCALL_DASHBOARD_SHOW_UI        0x000000E1
#define SYSCALL_DASHBOARD_HIDE_UI        0x000000E2
#define SYSCALL_DASHBOARD_GET_STATE      0x000000E3

#define SYSCALL_DBG_PRINT                0x000000F0
#define SYSCALL_DBG_BREAK                0x000000F1
#define SYSCALL_TEST_FUNCTION            0x000000F2

/* ==================== STATUS CODES ==================== */
// NTSTATUS codes (simplified)
#define STATUS_SUCCESS                   0x00000000
#define STATUS_PENDING                   0x00000103
#define STATUS_NOT_IMPLEMENTED           0xC0000002
#define STATUS_INVALID_PARAMETER         0xC000000D
#define STATUS_ACCESS_DENIED             0xC0000022
#define STATUS_OBJECT_NAME_NOT_FOUND     0xC0000034
#define STATUS_OBJECT_PATH_NOT_FOUND     0xC000003A
#define STATUS_END_OF_FILE               0xC0000011
#define STATUS_NO_MEMORY                 0xC0000017
#define STATUS_IO_TIMEOUT                0xC00000B5
#define STATUS_CANCELLED                 0xC0000120
#define STATUS_NOT_FOUND                 0xC0000225
#define STATUS_UNSUCCESSFUL              0xC0000001

/* ==================== STRUCTURES ==================== */
typedef struct XTHREAD_CONTEXT {
    uint32_t gpr[32];      // General purpose registers
    uint32_t lr;           // Link register
    uint32_t ctr;          // Count register
    uint32_t cr;           // Condition register
    uint32_t xer;          // Fixed-point exception register
    uint32_t fpscr;        // Floating-point status/control
    uint32_t srr0;         // Saved return address
    uint32_t srr1;         // Saved machine state
    uint32_t pid;          // Process ID
    uint32_t tid;          // Thread ID
    uint32_t state;        // Thread state
    uint32_t priority;     // Thread priority
    uint64_t create_time;
    uint64_t exit_time;
} XTHREAD_CONTEXT;

typedef struct XPROCESS_CONTEXT {
    uint32_t pid;          // Process ID
    uint32_t parent_pid;
    uint32_t title_id;     // Xbox title ID
    uint32_t base_address; // Process base address
    uint32_t size;         // Process size
    uint32_t thread_count;
    uint32_t handle_table[256]; // Object handles
    char name[64];         // Process name
} XPROCESS_CONTEXT;

typedef struct XALLOCATION_INFO {
    uint32_t base_address;
    uint32_t size;
    uint32_t type;         // 0=Physical, 1=Virtual, 2=IO
    uint32_t protection;   // Read/Write/Execute
    uint32_t alignment;
    uint32_t handle;
} XALLOCATION_INFO;

typedef struct XFILE_INFO {
    uint32_t attributes;
    uint64_t creation_time;
    uint64_t last_access_time;
    uint64_t last_write_time;
    uint64_t file_size;
    uint64_t allocation_size;
    uint32_t file_index;
} XFILE_INFO;

typedef struct XSYSTEM_INFO {
    uint32_t console_type;      // 0=Devkit, 1=Retail, 2=Recovery
    uint32_t cpu_count;         // Number of CPU cores
    uint32_t cpu_speed_mhz;     // CPU speed in MHz
    uint32_t memory_size_mb;    // Total RAM in MB
    uint32_t av_region;         // AV region (0=NTSC-U, 1=PAL, etc)
    uint32_t game_region;       // Game region
    uint32_t system_version;    // System version
    uint32_t dashboard_version; // Dashboard version
    uint32_t kernel_version;    // Kernel version
    uint32_t build_date;        // Build date
    char serial_number[12];     // Console serial
    char console_id[5];         // Console ID
} XSYSTEM_INFO;

typedef struct XTITLE_INFO {
    uint32_t title_id;
    uint32_t title_version;
    uint32_t media_type;    // 0=HDD, 1=DVD, 2=MU, 3=ODD
    uint32_t executable_type;
    char title_name[64];
    char publisher[32];
    char developer[32];
    uint32_t ratings[16];   // Age ratings per region
} XTITLE_INFO;

typedef struct XDISPLAY_MODE {
    uint32_t width;
    uint32_t height;
    uint32_t refresh_rate;
    uint32_t interlaced;
    uint32_t format;        // Pixel format
    uint32_t scanline_pitch;
} XDISPLAY_MODE;

typedef struct XAUDIO_BUFFER {
    uint32_t flags;
    uint32_t audio_bytes;
    uint32_t play_begin;
    uint32_t play_length;
    uint32_t loop_begin;
    uint32_t loop_length;
    uint32_t loop_count;
    uint32_t context;
    uint32_t pcm_buffer;    // Pointer to PCM data
} XAUDIO_BUFFER;

/* ==================== KERNEL STATE ==================== */
typedef struct XBOX360_KERNEL_STATE {
    // Process/Thread management
    XPROCESS_CONTEXT processes[32];
    XTHREAD_CONTEXT threads[256];
    uint32_t current_process_id;
    uint32_t current_thread_id;
    uint32_t next_process_id;
    uint32_t next_thread_id;
    
    // Memory management
    XALLOCATION_INFO allocations[1024];
    uint32_t next_allocation_handle;
    
    // File system state
    struct {
        char path[256];
        uint32_t handle;
        uint64_t position;
        XFILE_INFO info;
    } open_files[64];
    uint32_t next_file_handle;
    
    // Timing
    uint64_t boot_time_ns;
    uint64_t tick_count;
    uint64_t performance_frequency;
    uint64_t performance_counter;
    
    // System info
    XSYSTEM_INFO system_info;
    XTITLE_INFO current_title;
    
    // Display state
    XDISPLAY_MODE display_mode;
    uint32_t display_enabled;
    
    // Audio state
    uint32_t audio_initialized;
    
    // Crypto state
    XECRYPT_SHA_STATE sha_state;
    XECRYPT_AES_STATE aes_state;
    XECRYPT_RC4_STATE rc4_state;
    
    // Callbacks
    void (*syscall_callback)(void *opaque, uint32_t syscall, uint32_t result);
    void *callback_opaque;
    
    // Debug
    bool debug_enabled;
    FILE *log_file;
} XBOX360_KERNEL_STATE;

/* ==================== PUBLIC FUNCTIONS ==================== */
void xbox360_kernel_init(XBOX360_KERNEL_STATE *ks, XenonState *xenon);
void xbox360_kernel_reset(XBOX360_KERNEL_STATE *ks);

uint32_t xbox360_handle_syscall(XBOX360_KERNEL_STATE *ks, uint32_t syscall_number, uint32_t *parameters, uint32_t parameter_count);

uint32_t xbox360_create_thread(XBOX360_KERNEL_STATE *ks, uint32_t start_address, uint32_t stack_size, uint32_t priority);
uint32_t xbox360_terminate_thread(XBOX360_KERNEL_STATE *ks, uint32_t thread_id);
uint32_t xbox360_get_current_thread(XBOX360_KERNEL_STATE *ks);

uint32_t xbox360_allocate_memory(XBOX360_KERNEL_STATE *ks, uint32_t size, uint32_t alignment, uint32_t type);
uint32_t xbox360_free_memory(XBOX360_KERNEL_STATE *ks, uint32_t handle);
void *xbox360_get_memory_ptr(XBOX360_KERNEL_STATE *ks, uint32_t handle);

uint32_t xbox360_open_file(XBOX360_KERNEL_STATE *ks, const char *path, uint32_t access_mask);
uint32_t xbox360_read_file(XBOX360_KERNEL_STATE *ks, uint32_t handle, void *buffer, uint32_t size, uint32_t *bytes_read);
uint32_t xbox360_close_file(XBOX360_KERNEL_STATE *ks, uint32_t handle);

uint32_t xbox360_get_tick_count(XBOX360_KERNEL_STATE *ks);
uint64_t xbox360_query_performance_counter(XBOX360_KERNEL_STATE *ks);
uint64_t xbox360_query_performance_frequency(XBOX360_KERNEL_STATE *ks);
void xbox360_update_timing(XBOX360_KERNEL_STATE *ks);

const XSYSTEM_INFO *xbox360_get_system_info(XBOX360_KERNEL_STATE *ks);
uint32_t xbox360_get_console_type(XBOX360_KERNEL_STATE *ks);
uint32_t xbox360_get_title_id(XBOX360_KERNEL_STATE *ks);

void xbox360_kernel_debug_print(XBOX360_KERNEL_STATE *ks, const char *format, ...);
void xbox360_kernel_dump_state(XBOX360_KERNEL_STATE *ks);
void xbox360_kernel_set_debug(XBOX360_KERNEL_STATE *ks, bool enabled);

void xbox360_kernel_set_callback(XBOX360_KERNEL_STATE *ks, void (*callback)(void *, uint32_t, uint32_t), void *opaque);

#endif
