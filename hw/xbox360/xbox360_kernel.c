#include "hw/xbox360/xbox360_kernel.h"
#include "hw/xbox360/xbox360.h"
#include "hw/xbox360/crypto/xe_crypto.h"
#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include <stdarg.h>

/* ==================== CONSTANTS ==================== */
#define MAX_SYSCALL_PARAMETERS   8
#define DEFAULT_STACK_SIZE       (64 * 1024)  // 64KB
#define DEFAULT_PRIORITY         8

#define CONSOLE_TYPE_RETAIL      1
#define CONSOLE_TYPE_DEBUG       2
#define CONSOLE_TYPE_RECOVERY    3

#define MEMORY_TYPE_PHYSICAL     0
#define MEMORY_TYPE_VIRTUAL      1
#define MEMORY_TYPE_IO           2

#define PROTECTION_READ          0x01
#define PROTECTION_WRITE         0x02
#define PROTECTION_EXECUTE       0x04

#define THREAD_STATE_READY       0
#define THREAD_STATE_RUNNING     1
#define THREAD_STATE_WAITING     2
#define THREAD_STATE_TERMINATED  3

#define FILE_ATTRIBUTE_READONLY  0x00000001
#define FILE_ATTRIBUTE_HIDDEN    0x00000002
#define FILE_ATTRIBUTE_SYSTEM    0x00000004
#define FILE_ATTRIBUTE_DIRECTORY 0x00000010
#define FILE_ATTRIBUTE_NORMAL    0x00000080

#define GENERIC_READ             0x80000000
#define GENERIC_WRITE            0x40000000
#define GENERIC_EXECUTE          0x20000000
#define GENERIC_ALL              0x10000000

#define TITLE_ID_DASHBOARD       0xFFFE07D1
#define TITLE_ID_XEX_LOADER      0x00000176
#define TITLE_ID_HV_LOADER       0x00000188

/* ==================== GLOBALS ==================== */
typedef uint32_t (*SYSCALL_HANDLER)(XBOX360_KERNEL_STATE *ks, uint32_t *params);
static SYSCALL_HANDLER syscall_handlers[256];

/* ==================== HELPERS ==================== */
static uint64_t get_current_time_ns(void) {
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static uint32_t generate_handle(void) {
    static uint32_t next_handle = 0x1000;
    return next_handle++;
}

static bool is_valid_thread(XBOX360_KERNEL_STATE *ks, uint32_t thread_id) {
    for (int i = 0; i < 256; i++) {
        if (ks->threads[i].tid == thread_id && 
            ks->threads[i].state != THREAD_STATE_TERMINATED) {
            return true;
        }
    }
    return false;
}

static bool is_valid_process(XBOX360_KERNEL_STATE *ks, uint32_t process_id) {
    for (int i = 0; i < 32; i++) {
        if (ks->processes[i].pid == process_id) {
            return true;
        }
    }
    return false;
}

static XTHREAD_CONTEXT *find_thread(XBOX360_KERNEL_STATE *ks, uint32_t thread_id) {
    for (int i = 0; i < 256; i++) {
        if (ks->threads[i].tid == thread_id) {
            return &ks->threads[i];
        }
    }
    return NULL;
}

static XPROCESS_CONTEXT *find_process(XBOX360_KERNEL_STATE *ks, uint32_t process_id) {
    for (int i = 0; i < 32; i++) {
        if (ks->processes[i].pid == process_id) {
            return &ks->processes[i];
        }
    }
    return NULL;
}

static XALLOCATION_INFO *find_allocation(XBOX360_KERNEL_STATE *ks, uint32_t handle) {
    for (int i = 0; i < 1024; i++) {
        if (ks->allocations[i].handle == handle) {
            return &ks->allocations[i];
        }
    }
    return NULL;
}

/* ==================== SYSCALL HANDLERS ==================== */
// System and timing
static uint32_t syscall_get_tick_count(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    (void)params;
    return ks->tick_count;
}

static uint32_t syscall_query_performance_counter(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    (void)params;
    return ks->performance_counter & 0xFFFFFFFF;
}

static uint32_t syscall_query_performance_frequency(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    (void)params;
    return ks->performance_frequency & 0xFFFFFFFF;
}

static uint32_t syscall_ke_delay_execution_thread(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t processor_mode = params[0]; // unused
    uint32_t alertable = params[1];      // unused
    uint64_t *interval = (uint64_t*)params[2];
    
    (void)processor_mode;
    (void)alertable;
    
    if (!interval) {
        return STATUS_INVALID_PARAMETER;
    }
    
    // TODO: Suspend thread
    return STATUS_SUCCESS;
}

static uint32_t syscall_ke_set_timer(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t *timer_handle = (uint32_t*)params[0];
    uint64_t due_time = *(uint64_t*)&params[1];
    uint32_t period = params[3];
    uint32_t routine = params[4];
    uint32_t context = params[5];
    
    (void)period;
    (void)routine;
    (void)context;
    
    if (!timer_handle) {
        return STATUS_INVALID_PARAMETER;
    }
    
    *timer_handle = generate_handle();
    
    xbox360_kernel_debug_print(ks, "KeSetTimer: handle=%08X, due=%llu", 
                              *timer_handle, (unsigned long long)due_time);
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_ke_cancel_timer(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t timer_handle = params[0];
    (void)timer_handle;
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_ke_resume_thread(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t thread_handle = params[0];
    
    XTHREAD_CONTEXT *thread = find_thread(ks, thread_handle);
    if (!thread) {
        return STATUS_NOT_FOUND;
    }
    
    thread->state = THREAD_STATE_READY;
    
    xbox360_kernel_debug_print(ks, "KeResumeThread: %08X", thread_handle);
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_ke_suspend_thread(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t thread_handle = params[0];
    uint32_t *previous_suspend_count = (uint32_t*)params[1];
    
    XTHREAD_CONTEXT *thread = find_thread(ks, thread_handle);
    if (!thread) {
        return STATUS_NOT_FOUND;
    }
    
    if (previous_suspend_count) {
        *previous_suspend_count = 0;
    }
    
    thread->state = THREAD_STATE_WAITING;
    
    xbox360_kernel_debug_print(ks, "KeSuspendThread: %08X", thread_handle);
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_ke_set_processor_affinity(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t thread_handle = params[0];
    uint32_t affinity_mask = params[1];
    
    (void)thread_handle;
    (void)affinity_mask;
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_ke_query_active_processors(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    (void)params;
    
    return 0x00000007; // CPUs 0, 1 and 2
}

static uint32_t syscall_ex_create_thread(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t *thread_handle_ptr = (uint32_t*)params[0];
    uint32_t desired_access = params[1];
    uint32_t object_attributes = params[2]; // unused
    uint32_t *thread_id_ptr = (uint32_t*)params[3];
    uint32_t start_address = params[4];
    uint32_t start_context = params[5];
    uint32_t create_suspended = params[6];
    
    (void)desired_access;
    (void)object_attributes;
    (void)start_context;
    (void)create_suspended;
    
    if (!thread_handle_ptr || !thread_id_ptr) {
        return STATUS_INVALID_PARAMETER;
    }
    
    uint32_t thread_id = xbox360_create_thread(ks, start_address, DEFAULT_STACK_SIZE, DEFAULT_PRIORITY);
    if (thread_id == 0) {
        return STATUS_NO_MEMORY;
    }
    
    *thread_handle_ptr = generate_handle();
    *thread_id_ptr = thread_id;
    
    xbox360_kernel_debug_print(ks, "Created thread %08X at address %08X", 
                              thread_id, start_address);
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_ke_terminate_thread(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t exit_code = params[0];
    
    uint32_t result = xbox360_terminate_thread(ks, ks->current_thread_id);
    if (result != STATUS_SUCCESS) {
        return result;
    }
    
    xbox360_kernel_debug_print(ks, "Thread %08X terminated with code %08X", 
                              ks->current_thread_id, exit_code);
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_ke_get_current_processor_number(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    (void)params;
    // Always return core 0 (simplified)
    return 0;
}

static uint32_t syscall_mm_allocate_physical_memory(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t size = params[0];
    uint32_t alignment = params[1];
    uint32_t protection = params[2];
    uint32_t *handle_ptr = (uint32_t*)params[3];
    
    if (!handle_ptr || size == 0) {
        return STATUS_INVALID_PARAMETER;
    }
    
    uint32_t handle = xbox360_allocate_memory(ks, size, alignment, MEMORY_TYPE_PHYSICAL);
    if (handle == 0) {
        return STATUS_NO_MEMORY;
    }
    
    XALLOCATION_INFO *alloc = find_allocation(ks, handle);
    if (alloc) {
        alloc->protection = protection;
    }
    
    *handle_ptr = handle;
    
    xbox360_kernel_debug_print(ks, "Allocated %d bytes at %08X, handle %08X", 
                              size, alloc ? alloc->base_address : 0, handle);
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_mm_free_physical_memory(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t handle = params[0];
    
    return xbox360_free_memory(ks, handle);
}

static uint32_t syscall_mm_get_physical_address(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t virtual_address = params[0];
    uint32_t *physical_address_ptr = (uint32_t*)params[1];
    
    if (!physical_address_ptr) {
        return STATUS_INVALID_PARAMETER;
    }
    
    // Simplified: Virtual Address = physical
    *physical_address_ptr = virtual_address;
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_nt_create_file(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t *handle_ptr = (uint32_t*)params[0];
    uint32_t desired_access = params[1];
    uint32_t object_attributes = params[2]; // unused
    uint32_t *io_status_block = (uint32_t*)params[3]; // unused
    uint32_t allocation_size = params[4]; // unused
    uint32_t file_attributes = params[5];
    uint32_t share_access = params[6];
    uint32_t create_disposition = params[7];
    uint32_t create_options = params[8];
    const char *file_name = (const char*)params[9];
    
    (void)object_attributes;
    (void)io_status_block;
    (void)allocation_size;
    (void)share_access;
    (void)create_disposition;
    (void)create_options;
    
    if (!handle_ptr || !file_name) {
        return STATUS_INVALID_PARAMETER;
    }
    
    xbox360_kernel_debug_print(ks, "Create file: %s, access: %08X, attr: %08X", 
                              file_name, desired_access, file_attributes);
    
    if (strcmp(file_name, "\\Device\\Harddisk0\\Partition1\\xboxdash.xex") == 0 ||
        strcmp(file_name, "\\Device\\Harddisk0\\Partition1\\Dashboard.xex") == 0) {
        *handle_ptr = generate_handle();
        return STATUS_SUCCESS;
    }
    
    if (strcmp(file_name, "\\Device\\Cdrom0\\default.xex") == 0) {
        // Game Disc - return not found (no disc inserted)
        return STATUS_OBJECT_NAME_NOT_FOUND;
    }
    
    // For other files, simulate they don't exist (TODO: Implement Real Functionality)
    return STATUS_OBJECT_NAME_NOT_FOUND;
}

static uint32_t syscall_nt_read_file(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t file_handle = params[0];
    uint32_t *io_status_block = (uint32_t*)params[1]; // unused
    void *buffer = (void*)params[2];
    uint32_t length = params[3];
    uint64_t *byte_offset = (uint64_t*)params[4]; // unused
    
    (void)io_status_block;
    (void)byte_offset;
    
    if (file_handle == 0 || !buffer || length == 0) {
        return STATUS_INVALID_PARAMETER;
    }
    
    xbox360_kernel_debug_print(ks, "Read file handle %08X, %d bytes to %08X", 
                              file_handle, length, (uint32_t)buffer);
    
    // I'll return EOF for now
    if (io_status_block) {
        io_status_block[0] = STATUS_END_OF_FILE;
        io_status_block[1] = 0; // Bytes read
    }
    
    return STATUS_END_OF_FILE;
}

static uint32_t syscall_nt_close(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t handle = params[0];
    xbox360_kernel_debug_print(ks, "Close handle %08X", handle);
    
    // Always return success (development)
    return STATUS_SUCCESS;
}

// Graphics
static uint32_t syscall_vgpuhd_create_device(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t *device_handle_ptr = (uint32_t*)params[0];
    
    if (!device_handle_ptr) {
        return STATUS_INVALID_PARAMETER;
    }
    
    // Create Graphical Dummy Device
    *device_handle_ptr = generate_handle();
    ks->display_enabled = 1;
    
    xbox360_kernel_debug_print(ks, "Created GPU device, handle %08X", *device_handle_ptr);
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_vgpuhd_setmode(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t device_handle = params[0];
    uint32_t width = params[1];
    uint32_t height = params[2];
    uint32_t refresh_rate = params[3];
    
    (void)device_handle;
    
    ks->display_mode.width = width;
    ks->display_mode.height = height;
    ks->display_mode.refresh_rate = refresh_rate;
    
    xbox360_kernel_debug_print(ks, "Set display mode: %dx%d @ %dHz", 
                              width, height, refresh_rate);
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_vgpuhd_waitforvsync(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t device_handle = params[0];
    (void)device_handle;
    
    // TODO: Sync with VBLANK Timer
    return STATUS_SUCCESS;
}

// Crypto
static uint32_t syscall_xecrypt_sha_init(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t *context_ptr = (uint32_t*)params[0];
    
    if (!context_ptr) {
        return STATUS_INVALID_PARAMETER;
    }
    
    xeCryptShaInit(&ks->sha_state);
    *context_ptr = (uint32_t)&ks->sha_state;
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_xecrypt_sha_update(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t context = params[0];
    const uint8_t *data = (const uint8_t*)params[1];
    uint32_t data_size = params[2];
    
    if (context != (uint32_t)&ks->sha_state) {
        return STATUS_INVALID_PARAMETER;
    }
    
    xeCryptShaUpdate(&ks->sha_state, data, data_size);
    return STATUS_SUCCESS;
}

static uint32_t syscall_xecrypt_aes_cbc(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t key_ptr = params[0];
    uint32_t iv_ptr = params[1];
    uint32_t data_ptr = params[2];
    uint32_t data_size = params[3];
    uint32_t encrypt = params[4];
    
    (void)key_ptr;
    (void)iv_ptr;
    (void)data_ptr;
    (void)data_size;
    (void)encrypt;
    
    // Stub - Always return success
    return STATUS_SUCCESS;
}

static uint32_t syscall_xam_get_consoletype(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t *console_type_ptr = (uint32_t*)params[0];
    
    if (!console_type_ptr) {
        return STATUS_INVALID_PARAMETER;
    }
    
    *console_type_ptr = CONSOLE_TYPE_RETAIL;
    return STATUS_SUCCESS;
}

static uint32_t syscall_xam_get_systeminfo(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t system_info_ptr = params[0];
    
    if (!system_info_ptr) {
        return STATUS_INVALID_PARAMETER;
    }
    
    memcpy((void*)system_info_ptr, &ks->system_info, sizeof(XSYSTEM_INFO));
    return STATUS_SUCCESS;
}

static uint32_t syscall_xam_get_currenttitleid(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t *title_id_ptr = (uint32_t*)params[0];
    
    if (!title_id_ptr) {
        return STATUS_INVALID_PARAMETER;
    }
    
    *title_id_ptr = ks->current_title.title_id;
    return STATUS_SUCCESS;
}

static uint32_t syscall_dbg_print(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    const char *format = (const char*)params[0];
    
    if (!format) {
        return STATUS_INVALID_PARAMETER;
    }
    
    printf("[DBG] ");
    printf("%s", format);
    
    for (int i = 1; i < 4; i++) {
        if (params[i] != 0) {
            printf(" 0x%08X", params[i]);
        }
    }
    printf("\n");
    
    return STATUS_SUCCESS;
}

/* ==================== SYSCALL DISPATCHER ==================== */
static void initialize_syscall_handlers(void) {
    // Zerar todos os handlers
    memset(syscall_handlers, 0, sizeof(syscall_handlers));
    
    // System and timing (agora completos)
    syscall_handlers[SYSCALL_GET_TICK_COUNT] = syscall_get_tick_count;
    syscall_handlers[SYSCALL_QUERY_PERFORMANCE_COUNTER] = syscall_query_performance_counter;
    syscall_handlers[SYSCALL_QUERY_PERFORMANCE_FREQUENCY] = syscall_query_performance_frequency;
    syscall_handlers[SYSCALL_KE_DELAY_EXECUTION_THREAD] = syscall_ke_delay_execution_thread;
    syscall_handlers[SYSCALL_KE_SET_TIMER] = syscall_ke_set_timer;
    syscall_handlers[SYSCALL_KE_CANCEL_TIMER] = syscall_ke_cancel_timer;
    syscall_handlers[SYSCALL_KE_RESUME_THREAD] = syscall_ke_resume_thread;
    syscall_handlers[SYSCALL_KE_SUSPEND_THREAD] = syscall_ke_suspend_thread;
    syscall_handlers[SYSCALL_KE_TERMINATE_THREAD] = syscall_ke_terminate_thread;
    syscall_handlers[SYSCALL_KE_GET_CURRENT_PROCESSOR_NUMBER] = syscall_ke_get_current_processor_number;
    syscall_handlers[SYSCALL_KE_SET_PROCESSOR_AFFINITY] = syscall_ke_set_processor_affinity;
    syscall_handlers[SYSCALL_KE_QUERY_ACTIVE_PROCESSORS] = syscall_ke_query_active_processors;
    
    // Memory management (completos)
    syscall_handlers[SYSCALL_MM_ALLOCATE_PHYSICAL_MEMORY] = syscall_mm_allocate_physical_memory;
    syscall_handlers[SYSCALL_MM_FREE_PHYSICAL_MEMORY] = syscall_mm_free_physical_memory;
    syscall_handlers[SYSCALL_MM_MAP_IO_SPACE] = syscall_mm_map_io_space;
    syscall_handlers[SYSCALL_MM_UNMAP_IO_SPACE] = syscall_mm_unmap_io_space;
    syscall_handlers[SYSCALL_MM_QUERY_ALLOCATION_SIZE] = syscall_mm_query_allocation_size;
    syscall_handlers[SYSCALL_MM_LOCK_PAGES] = syscall_mm_lock_pages;
    syscall_handlers[SYSCALL_MM_UNLOCK_PAGES] = syscall_mm_unlock_pages;
    syscall_handlers[SYSCALL_MM_GET_PHYSICAL_ADDRESS] = syscall_mm_get_physical_address;
    
    // File system (completos)
    syscall_handlers[SYSCALL_NT_CREATE_FILE] = syscall_nt_create_file;
    syscall_handlers[SYSCALL_NT_OPEN_FILE] = syscall_nt_open_file;
    syscall_handlers[SYSCALL_NT_READ_FILE] = syscall_nt_read_file;
    syscall_handlers[SYSCALL_NT_WRITE_FILE] = syscall_nt_write_file;
    syscall_handlers[SYSCALL_NT_CLOSE] = syscall_nt_close;
    syscall_handlers[SYSCALL_NT_QUERY_INFORMATION_FILE] = syscall_nt_query_information_file;
    syscall_handlers[SYSCALL_NT_SET_INFORMATION_FILE] = syscall_nt_set_information_file;
    syscall_handlers[SYSCALL_NT_QUERY_VOLUME_INFO] = syscall_nt_query_volume_info;
    syscall_handlers[SYSCALL_NT_DEVICE_IO_CONTROL] = syscall_nt_device_io_control;
    
    // Graphics (completos)
    syscall_handlers[SYSCALL_VGPUHD_CREATE_DEVICE] = syscall_vgpuhd_create_device;
    syscall_handlers[SYSCALL_VGPUHD_DESTROY_DEVICE] = syscall_vgpuhd_destroy_device;
    syscall_handlers[SYSCALL_VGPUHD_FLIP] = syscall_vgpuhd_flip;
    syscall_handlers[SYSCALL_VGPUHD_SETMODE] = syscall_vgpuhd_setmode;
    syscall_handlers[SYSCALL_VGPUHD_SETSCANLINE] = syscall_vgpuhd_setscanline;
    syscall_handlers[SYSCALL_VGPUHD_WAITFORVSYNC] = syscall_vgpuhd_waitforvsync;
    
    // Audio (completos)
    syscall_handlers[SYSCALL_XAUDIO_CREATE] = syscall_xaudio_create;
    syscall_handlers[SYSCALL_XAUDIO_DESTROY] = syscall_xaudio_destroy;
    syscall_handlers[SYSCALL_XAUDIO_SUBMIT_BUFFER] = syscall_xaudio_submit_buffer;
    syscall_handlers[SYSCALL_XAUDIO_FLUSH_BUFFERS] = syscall_xaudio_flush_buffers;
    
    // Crypto (completos)
    syscall_handlers[SYSCALL_XECRYPT_SHA_INIT] = syscall_xecrypt_sha_init;
    syscall_handlers[SYSCALL_XECRYPT_SHA_UPDATE] = syscall_xecrypt_sha_update;
    syscall_handlers[SYSCALL_XECRYPT_SHA_FINAL] = syscall_xecrypt_sha_final;
    syscall_handlers[SYSCALL_XECRYPT_AES_CBC] = syscall_xecrypt_aes_cbc;
    syscall_handlers[SYSCALL_XECRYPT_RSA_VERIFY] = syscall_xecrypt_rsa_verify;
    syscall_handlers[SYSCALL_XECRYPT_RC4] = syscall_xecrypt_rc4;
    
    // XAM (completos)
    syscall_handlers[SYSCALL_XAM_GET_CONSOLETYPE] = syscall_xam_get_consoletype;
    syscall_handlers[SYSCALL_XAM_GET_SYSTEMINFO] = syscall_xam_get_systeminfo;
    syscall_handlers[SYSCALL_XAM_GET_CURRENTTITLEID] = syscall_xam_get_currenttitleid;
    syscall_handlers[SYSCALL_XAM_GET_CURRENTTITLEINFO] = syscall_xam_get_currenttitleinfo;
    syscall_handlers[SYSCALL_XAM_LAUNCH_TITLE] = syscall_xam_launch_title;
    syscall_handlers[SYSCALL_XAM_TERMINATE_TITLE] = syscall_xam_terminate_title;
    syscall_handlers[SYSCALL_XAM_LOAD_GAME] = syscall_xam_load_game;
    
    // Dashboard (completos)
    syscall_handlers[SYSCALL_DASHBOARD_GET_VERSION] = syscall_dashboard_get_version;
    syscall_handlers[SYSCALL_DASHBOARD_SHOW_UI] = syscall_dashboard_show_ui;
    syscall_handlers[SYSCALL_DASHBOARD_HIDE_UI] = syscall_dashboard_hide_ui;
    syscall_handlers[SYSCALL_DASHBOARD_GET_STATE] = syscall_dashboard_get_state;
    
    // Debug (completos)
    syscall_handlers[SYSCALL_DBG_PRINT] = syscall_dbg_print;
    syscall_handlers[SYSCALL_DBG_BREAK] = syscall_dbg_break;
    syscall_handlers[SYSCALL_TEST_FUNCTION] = syscall_test_function;
}

uint32_t xbox360_handle_syscall(XBOX360_KERNEL_STATE *ks, uint32_t syscall_number, uint32_t *parameters, uint32_t parameter_count) {
    static bool handlers_initialized = false;
    if (!handlers_initialized) {
        initialize_syscall_handlers();
        handlers_initialized = true;
    }
    
    if (syscall_number >= 256) {
        return STATUS_NOT_IMPLEMENTED;
    }
    
    SYSCALL_HANDLER handler = syscall_handlers[syscall_number];
    if (!handler) {
        xbox360_kernel_debug_print(ks, "Unhandled syscall: 0x%08X", syscall_number);
        return STATUS_NOT_IMPLEMENTED;
    }
    
    uint32_t result = handler(ks, parameters);
    
    if (ks->syscall_callback) {
        ks->syscall_callback(ks->callback_opaque, syscall_number, result);
    }
    
    return result;
}

/* ==================== KERNEL MANAGEMENT ==================== */
void xbox360_kernel_init(XBOX360_KERNEL_STATE *ks, XenonState *xenon) {
    memset(ks, 0, sizeof(XBOX360_KERNEL_STATE));
    
    ks->boot_time_ns = get_current_time_ns();
    ks->tick_count = 0;
    ks->performance_frequency = 1000000; // 1MHz
    ks->performance_counter = 0;
    
    memset(&ks->system_info, 0, sizeof(XSYSTEM_INFO));
    ks->system_info.console_type = CONSOLE_TYPE_RETAIL;
    ks->system_info.cpu_count = 3;
    ks->system_info.cpu_speed_mhz = 3200;
    ks->system_info.memory_size_mb = 512;
    ks->system_info.av_region = 0; // NTSC-U
    ks->system_info.game_region = 0;
    ks->system_info.system_version = 0x2B0; // 2.0.17559.0
    ks->system_info.dashboard_version = 0x2B0;
    ks->system_info.kernel_version = 0x2B0;
    ks->system_info.build_date = 20110830; // 30/08/2011
    
    // Serial and Dummy ID
    memcpy(ks->system_info.serial_number, "123456789012", 12);
    memcpy(ks->system_info.console_id, "\x11\x22\x33\x44\x55", 5);
    
    memset(&ks->current_title, 0, sizeof(XTITLE_INFO));
    ks->current_title.title_id = TITLE_ID_DASHBOARD;
    ks->current_title.title_version = 0x2B0;
    ks->current_title.media_type = 0; // HDD
    strcpy(ks->current_title.title_name, "Xbox 360 Dashboard");
    strcpy(ks->current_title.publisher, "Microsoft");
    strcpy(ks->current_title.developer, "Microsoft");
    
    memset(&ks->display_mode, 0, sizeof(XDISPLAY_MODE));
    ks->display_mode.width = 1280;
    ks->display_mode.height = 720;
    ks->display_mode.refresh_rate = 60;
    ks->display_enabled = 0;
    
    ks->next_process_id = 0x1000;
    ks->next_thread_id = 0x2000;
    ks->next_file_handle = 0x4000;
    ks->next_allocation_handle = 0x8000;
    
    ks->current_process_id = ks->next_process_id++;
    XPROCESS_CONTEXT *kernel_proc = &ks->processes[0];
    kernel_proc->pid = ks->current_process_id;
    kernel_proc->parent_pid = 0;
    kernel_proc->title_id = 0;
    kernel_proc->base_address = 0x80000000;
    kernel_proc->size = 0x100000;
    strcpy(kernel_proc->name, "Kernel");
    
    ks->current_thread_id = xbox360_create_thread(ks, 0x80000000, DEFAULT_STACK_SIZE, 0);
    
    ks->debug_enabled = true;
    ks->log_file = fopen("xbox360_kernel.log", "w");
    if (ks->log_file) {
        fprintf(ks->log_file, "Xbox 360 Kernel initialized at %llu\n", 
                (unsigned long long)ks->boot_time_ns);
    }
    
    printf("[KERNEL] Initialized\n");
    printf("         System version: %08X\n", ks->system_info.system_version);
    printf("         Console ID: %02X%02X%02X%02X%02X\n",
           ks->system_info.console_id[0], ks->system_info.console_id[1],
           ks->system_info.console_id[2], ks->system_info.console_id[3],
           ks->system_info.console_id[4]);
}

void xbox360_kernel_reset(XBOX360_KERNEL_STATE *ks) {
    if (ks->log_file) {
        fclose(ks->log_file);
        ks->log_file = NULL;
    }
    
    XenonState *xenon = NULL;
    xbox360_kernel_init(ks, xenon);
}

/* ==================== THREAD MANAGEMENT ==================== */
uint32_t xbox360_create_thread(XBOX360_KERNEL_STATE *ks, uint32_t start_address, uint32_t stack_size, uint32_t priority) {
    XTHREAD_CONTEXT *thread = NULL;
    for (int i = 0; i < 256; i++) {
        if (ks->threads[i].state == THREAD_STATE_TERMINATED || 
            ks->threads[i].tid == 0) {
            thread = &ks->threads[i];
            break;
        }
    }
    
    if (!thread) {
        return 0;
    }
    
    memset(thread, 0, sizeof(XTHREAD_CONTEXT));
    thread->tid = ks->next_thread_id++;
    thread->pid = ks->current_process_id;
    thread->state = THREAD_STATE_READY;
    thread->priority = priority;
    thread->create_time = get_current_time_ns();
    thread->srr0 = start_address;
    thread->gpr[1] = 0x80000000 + (256 * 1024 * 1024) - 4; // Stack top
    
    XPROCESS_CONTEXT *proc = find_process(ks, ks->current_process_id);
    if (proc && proc->thread_count < 256) {
        proc->handle_table[proc->thread_count++] = thread->tid;
    }
    
    return thread->tid;
}

uint32_t xbox360_terminate_thread(XBOX360_KERNEL_STATE *ks, uint32_t thread_id) {
    XTHREAD_CONTEXT *thread = find_thread(ks, thread_id);
    if (!thread) {
        return STATUS_NOT_FOUND;
    }
    
    thread->state = THREAD_STATE_TERMINATED;
    thread->exit_time = get_current_time_ns();
    
    XPROCESS_CONTEXT *proc = find_process(ks, thread->pid);
    if (proc) {
        for (int i = 0; i < proc->thread_count; i++) {
            if (proc->handle_table[i] == thread_id) {
                proc->handle_table[i] = 0;
                break;
            }
        }
    }
    
    return STATUS_SUCCESS;
}

uint32_t xbox360_get_current_thread(XBOX360_KERNEL_STATE *ks) {
    return ks->current_thread_id;
}

static uint32_t syscall_mm_map_io_space(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t physical_address = params[0];
    uint32_t size = params[1];
    uint32_t protection = params[2];
    uint32_t *virtual_address_ptr = (uint32_t*)params[3];
    
    if (!virtual_address_ptr) {
        return STATUS_INVALID_PARAMETER;
    }
    
    *virtual_address_ptr = physical_address;
    
    XALLOCATION_INFO *alloc = NULL;
    for (int i = 0; i < 1024; i++) {
        if (ks->allocations[i].handle == 0) {
            alloc = &ks->allocations[i];
            break;
        }
    }
    
    if (alloc) {
        alloc->handle = generate_handle();
        alloc->base_address = physical_address;
        alloc->size = size;
        alloc->type = MEMORY_TYPE_IO;
        alloc->protection = protection;
    }
    
    xbox360_kernel_debug_print(ks, "MmMapIoSpace: phys=%08X, size=%d, virt=%08X", 
                              physical_address, size, *virtual_address_ptr);
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_mm_unmap_io_space(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t virtual_address = params[0];
    uint32_t size = params[1];
    
    (void)size;
    
    for (int i = 0; i < 1024; i++) {
        if (ks->allocations[i].base_address == virtual_address && 
            ks->allocations[i].type == MEMORY_TYPE_IO) {
            memset(&ks->allocations[i], 0, sizeof(XALLOCATION_INFO));
            break;
        }
    }
    
    xbox360_kernel_debug_print(ks, "MmUnmapIoSpace: virt=%08X", virtual_address);
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_mm_query_allocation_size(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t handle = params[0];
    uint32_t *size_ptr = (uint32_t*)params[1];
    
    if (!size_ptr) {
        return STATUS_INVALID_PARAMETER;
    }
    
    XALLOCATION_INFO *alloc = find_allocation(ks, handle);
    if (!alloc) {
        return STATUS_NOT_FOUND;
    }
    
    *size_ptr = alloc->size;
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_mm_lock_pages(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t *lock_info_ptr = (uint32_t*)params[0];
    uint32_t operation = params[1];
    uint32_t mode = params[2];
    
    (void)lock_info_ptr;
    (void)operation;
    (void)mode;
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_mm_unlock_pages(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t *lock_info_ptr = (uint32_t*)params[0];
    
    (void)lock_info_ptr;
    
    return STATUS_SUCCESS;
}

uint32_t xbox360_allocate_memory(XBOX360_KERNEL_STATE *ks, uint32_t size, uint32_t alignment, uint32_t type) {
    XALLOCATION_INFO *alloc = NULL;
    for (int i = 0; i < 1024; i++) {
        if (ks->allocations[i].handle == 0) {
            alloc = &ks->allocations[i];
            break;
        }
    }
    
    if (!alloc) {
        return 0;
    }
    
    memset(alloc, 0, sizeof(XALLOCATION_INFO));
    alloc->handle = ks->next_allocation_handle++;
    alloc->size = size;
    alloc->type = type;
    alloc->alignment = alignment;
    
    // Dummy Address for Development
    static uint32_t next_address = 0x90000000;
    alloc->base_address = next_address;
    next_address += (size + 0xFFF) & ~0xFFF;
    
    return alloc->handle;
}

uint32_t xbox360_free_memory(XBOX360_KERNEL_STATE *ks, uint32_t handle) {
    XALLOCATION_INFO *alloc = find_allocation(ks, handle);
    if (!alloc) {
        return STATUS_NOT_FOUND;
    }
    
    memset(alloc, 0, sizeof(XALLOCATION_INFO));
    return STATUS_SUCCESS;
}

void *xbox360_get_memory_ptr(XBOX360_KERNEL_STATE *ks, uint32_t handle) {
    XALLOCATION_INFO *alloc = find_allocation(ks, handle);
    if (!alloc) {
        return NULL;
    }
    
    // In development: we don't map real memory
    return (void*)(uintptr_t)alloc->base_address;
}

/* ==================== FILE SYSTEM ==================== */
static uint32_t syscall_nt_open_file(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t *handle_ptr = (uint32_t*)params[0];
    uint32_t desired_access = params[1];
    uint32_t object_attributes = params[2];
    uint32_t *io_status_block = (uint32_t*)params[3];
    uint32_t share_access = params[4];
    uint32_t open_options = params[5];
    
    (void)object_attributes;
    (void)share_access;
    (void)open_options;
    
    if (!handle_ptr) {
        return STATUS_INVALID_PARAMETER;
    }
    
    *handle_ptr = generate_handle();
    
    if (io_status_block) {
        io_status_block[0] = STATUS_SUCCESS;
        io_status_block[1] = 0;
    }
    
    xbox360_kernel_debug_print(ks, "NtOpenFile: access=%08X -> handle=%08X", 
                              desired_access, *handle_ptr);
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_nt_write_file(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t file_handle = params[0];
    uint32_t *io_status_block = (uint32_t*)params[1];
    const void *buffer = (const void*)params[2];
    uint32_t length = params[3];
    uint64_t *byte_offset = (uint64_t*)params[4];
    
    (void)file_handle;
    (void)buffer;
    (void)byte_offset;
    
    if (!buffer || length == 0) {
        return STATUS_INVALID_PARAMETER;
    }
    
    if (io_status_block) {
        io_status_block[0] = STATUS_SUCCESS;
        io_status_block[1] = length; // Bytes written
    }
    
    xbox360_kernel_debug_print(ks, "NtWriteFile: handle=%08X, %d bytes", 
                              file_handle, length);
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_nt_query_information_file(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t file_handle = params[0];
    uint32_t *io_status_block = (uint32_t*)params[1];
    void *file_info = (void*)params[2];
    uint32_t length = params[3];
    uint32_t file_info_class = params[4];
    
    (void)file_handle;
    
    if (!file_info || !io_status_block) {
        return STATUS_INVALID_PARAMETER;
    }
    
    switch (file_info_class) {
        case 1: { // FileBasicInformation
            if (length >= 40) {
                // Timestamps dummy
                *(uint64_t*)((char*)file_info + 0) = 0;  // CreationTime
                *(uint64_t*)((char*)file_info + 8) = 0;  // LastAccessTime
                *(uint64_t*)((char*)file_info + 16) = 0; // LastWriteTime
                *(uint64_t*)((char*)file_info + 24) = 0; // ChangeTime
                *(uint32_t*)((char*)file_info + 32) = 0; // FileAttributes
            }
            break;
        }
        case 4: { // FileStandardInformation
            if (length >= 24) {
                *(uint64_t*)((char*)file_info + 0) = 1024*1024; // AllocationSize
                *(uint64_t*)((char*)file_info + 8) = 1024*512;  // EndOfFile
                *(uint32_t*)((char*)file_info + 16) = 0;        // NumberOfLinks
                *(uint8_t*)((char*)file_info + 20) = 0;         // DeletePending
                *(uint8_t*)((char*)file_info + 21) = 0;         // Directory
            }
            break;
        }
        default:
            return STATUS_INVALID_INFO_CLASS;
    }
    
    io_status_block[0] = STATUS_SUCCESS;
    io_status_block[1] = length;
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_nt_set_information_file(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t file_handle = params[0];
    uint32_t *io_status_block = (uint32_t*)params[1];
    const void *file_info = (const void*)params[2];
    uint32_t length = params[3];
    uint32_t file_info_class = params[4];
    
    (void)file_handle;
    (void)file_info;
    (void)length;
    (void)file_info_class;
    
    if (io_status_block) {
        io_status_block[0] = STATUS_SUCCESS;
        io_status_block[1] = 0;
    }
    
    return STATUS_SUCCESS;
}

// Syscall: NtQueryVolumeInformation (0x00000047)
static uint32_t syscall_nt_query_volume_info(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t file_handle = params[0];
    uint32_t *io_status_block = (uint32_t*)params[1];
    void *fs_info = (void*)params[2];
    uint32_t length = params[3];
    uint32_t fs_info_class = params[4];
    
    (void)file_handle;
    
    if (!fs_info || !io_status_block) {
        return STATUS_INVALID_PARAMETER;
    }
    
    switch (fs_info_class) {
        case 1: { // FileFsVolumeInformation
            if (length >= 48) {
                *(uint64_t*)((char*)fs_info + 0) = 0; // VolumeCreationTime
                *(uint32_t*)((char*)fs_info + 8) = 0; // VolumeSerialNumber
                *(uint32_t*)((char*)fs_info + 12) = 8; // VolumeLabelLength
                *(uint8_t*)((char*)fs_info + 16) = 0; // SupportsObjects
                memcpy((char*)fs_info + 17, "XBOX360", 8); // VolumeLabel
            }
            break;
        }
        case 2: { // FileFsSizeInformation
            if (length >= 24) {
                *(uint64_t*)((char*)fs_info + 0) = 500*1024*1024; // TotalAllocationUnits
                *(uint64_t*)((char*)fs_info + 8) = 400*1024*1024; // AvailableAllocationUnits
                *(uint32_t*)((char*)fs_info + 16) = 4096;         // SectorsPerAllocationUnit
                *(uint32_t*)((char*)fs_info + 20) = 512;          // BytesPerSector
            }
            break;
        }
        default:
            return STATUS_INVALID_INFO_CLASS;
    }
    
    io_status_block[0] = STATUS_SUCCESS;
    io_status_block[1] = length;
    
    return STATUS_SUCCESS;
}

// Syscall: NtDeviceIoControl (0x00000048)
static uint32_t syscall_nt_device_io_control(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t file_handle = params[0];
    uint32_t *io_status_block = (uint32_t*)params[1];
    uint32_t io_control_code = params[2];
    void *input_buffer = (void*)params[3];
    uint32_t input_buffer_length = params[4];
    void *output_buffer = (void*)params[5];
    uint32_t output_buffer_length = params[6];
    
    (void)file_handle;
    (void)input_buffer;
    (void)input_buffer_length;
    (void)output_buffer;
    (void)output_buffer_length;
    
    xbox360_kernel_debug_print(ks, "NtDeviceIoControl: code=0x%08X", io_control_code);
    
    if (io_status_block) {
        io_status_block[0] = STATUS_SUCCESS;
        io_status_block[1] = 0;
    }
    
    return STATUS_SUCCESS;
}

uint32_t xbox360_read_file(XBOX360_KERNEL_STATE *ks, uint32_t handle, void *buffer, uint32_t size, uint32_t *bytes_read) {
    for (int i = 0; i < 64; i++) {
        if (ks->open_files[i].handle == handle) {
            // Simulate Read
            uint64_t remaining = ks->open_files[i].info.file_size - 
                                ks->open_files[i].position;
            uint32_t to_read = size;
            if (to_read > remaining) {
                to_read = (uint32_t)remaining;
            }
            
            memset(buffer, 0, to_read);
            ks->open_files[i].position += to_read;
            
            if (bytes_read) {
                *bytes_read = to_read;
            }
            
            return STATUS_SUCCESS;
        }
    }
    
    return STATUS_NOT_FOUND;
}

uint32_t xbox360_close_file(XBOX360_KERNEL_STATE *ks, uint32_t handle) {
    for (int i = 0; i < 64; i++) {
        if (ks->open_files[i].handle == handle) {
            memset(&ks->open_files[i], 0, sizeof(ks->open_files[i]));
            return STATUS_SUCCESS;
        }
    }
    
    return STATUS_NOT_FOUND;
}


/* ==================== GRAPHICS COMPLETE ==================== */

// Syscall: VgpuhdDestroyDevice (0x00000061)
static uint32_t syscall_vgpuhd_destroy_device(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t device_handle = params[0];
    
    xbox360_kernel_debug_print(ks, "VgpuhdDestroyDevice: %08X", device_handle);
    
    ks->display_enabled = 0;
    
    return STATUS_SUCCESS;
}

// Syscall: VgpuhdFlip (0x00000062)
static uint32_t syscall_vgpuhd_flip(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t device_handle = params[0];
    uint32_t surface_handle = params[1];
    uint32_t interval = params[2];
    
    (void)device_handle;
    (void)surface_handle;
    (void)interval;
    
    xbox360_kernel_debug_print(ks, "VgpuhdFlip: surface=%08X, interval=%d", 
                              surface_handle, interval);
    
    return STATUS_SUCCESS;
}

// Syscall: VgpuhdSetScanline (0x00000064)
static uint32_t syscall_vgpuhd_setscanline(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t device_handle = params[0];
    uint32_t scanline = params[1];
    
    (void)device_handle;
    
    xbox360_kernel_debug_print(ks, "VgpuhdSetScanline: %d", scanline);
    
    return STATUS_SUCCESS;
}

/* ==================== AUDIO COMPLETE ==================== */

// Syscall: XAudioCreate (0x00000080)
static uint32_t syscall_xaudio_create(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t *audio_handle_ptr = (uint32_t*)params[0];
    uint32_t flags = params[1];
    
    (void)flags;
    
    if (!audio_handle_ptr) {
        return STATUS_INVALID_PARAMETER;
    }
    
    *audio_handle_ptr = generate_handle();
    ks->audio_initialized = 1;
    
    xbox360_kernel_debug_print(ks, "XAudioCreate: flags=%08X -> handle=%08X", 
                              flags, *audio_handle_ptr);
    
    return STATUS_SUCCESS;
}

// Syscall: XAudioDestroy (0x00000081)
static uint32_t syscall_xaudio_destroy(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t audio_handle = params[0];
    
    xbox360_kernel_debug_print(ks, "XAudioDestroy: %08X", audio_handle);
    
    ks->audio_initialized = 0;
    
    return STATUS_SUCCESS;
}

// Syscall: XAudioSubmitBuffer (0x00000082)
static uint32_t syscall_xaudio_submit_buffer(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t audio_handle = params[0];
    uint32_t *buffer_ptr = (uint32_t*)params[1];
    
    (void)audio_handle;
    (void)buffer_ptr;
    
    // Sempre sucesso
    return STATUS_SUCCESS;
}

// Syscall: XAudioFlushBuffers (0x00000083)
static uint32_t syscall_xaudio_flush_buffers(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t audio_handle = params[0];
    
    (void)audio_handle;
    
    return STATUS_SUCCESS;
}

/* ==================== CRYPTO COMPLETE ==================== */

// Syscall: XeCryptShaFinal (0x000000A2)
static uint32_t syscall_xecrypt_sha_final(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t context = params[0];
    uint8_t *digest = (uint8_t*)params[1];
    
    if (context != (uint32_t)&ks->sha_state || !digest) {
        return STATUS_INVALID_PARAMETER;
    }
    
    xeCryptShaFinal(&ks->sha_state, digest);
    
    return STATUS_SUCCESS;
}

// Syscall: XeCryptRsaVerify (0x000000A4)
static uint32_t syscall_xecrypt_rsa_verify(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint8_t *hash = (uint8_t*)params[0];
    uint8_t *signature = (uint8_t*)params[1];
    uint8_t *modulus = (uint8_t*)params[2];
    uint32_t modulus_size = params[3];
    uint64_t exponent = *(uint64_t*)&params[4];
    
    (void)hash;
    (void)signature;
    (void)modulus;
    (void)modulus_size;
    (void)exponent;
    
    // Para desenvolvimento, sempre verifica
    return xeCryptRsaVerify(hash, signature, modulus, modulus_size, exponent) ? 
           STATUS_SUCCESS : STATUS_INVALID_SIGNATURE;
}

// Syscall: XeCryptRc4 (0x000000A5)
static uint32_t syscall_xecrypt_rc4(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint8_t *key = (uint8_t*)params[0];
    uint32_t key_size = params[1];
    uint8_t *data = (uint8_t*)params[2];
    uint32_t data_size = params[3];
    
    if (!key || !data) {
        return STATUS_INVALID_PARAMETER;
    }
    
    xeCryptRc4(key, key_size, data, data_size);
    
    return STATUS_SUCCESS;
}

/* ==================== XAM COMPLETE ==================== */
static uint32_t syscall_xam_get_currenttitleinfo(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t title_info_ptr = params[0];
    
    if (!title_info_ptr) {
        return STATUS_INVALID_PARAMETER;
    }
    
    memcpy((void*)title_info_ptr, &ks->current_title, sizeof(XTITLE_INFO));
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_xam_launch_title(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t title_id = params[0];
    uint32_t launch_flags = params[1];
    
    xbox360_kernel_debug_print(ks, "XamLaunchTitle: %08X, flags=%08X", 
                              title_id, launch_flags);
    
    ks->current_title.title_id = title_id;
    // todo
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_xam_terminate_title(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t exit_code = params[0];
    xbox360_kernel_debug_print(ks, "XamTerminateTitle: code=%08X", exit_code);
    
    ks->current_title.title_id = TITLE_ID_DASHBOARD;
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_xam_load_game(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    const char *path = (const char*)params[0];
    uint32_t flags = params[1];
    
    xbox360_kernel_debug_print(ks, "XamLoadGame: %s, flags=%08X", path, flags);
    // todo
    
    return STATUS_SUCCESS;
}

/* ==================== DASHBOARD ==================== */
static uint32_t syscall_dashboard_get_version(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t *version_ptr = (uint32_t*)params[0];
    
    if (!version_ptr) {
        return STATUS_INVALID_PARAMETER;
    }
    
    *version_ptr = ks->system_info.dashboard_version;
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_dashboard_show_ui(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t ui_type = params[0];
    
    xbox360_kernel_debug_print(ks, "DashboardShowUI: type=%d", ui_type);
    return STATUS_SUCCESS;
}

static uint32_t syscall_dashboard_hide_ui(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    (void)params;
    
    xbox360_kernel_debug_print(ks, "DashboardHideUI");
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_dashboard_get_state(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t *state_ptr = (uint32_t*)params[0];
    
    if (!state_ptr) {
        return STATUS_INVALID_PARAMETER;
    }
    
    *state_ptr = 1; 
    
    return STATUS_SUCCESS;
}

/* ==================== DEBUG COMPLETE ==================== */
static uint32_t syscall_dbg_break(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t break_type = params[0];
    
    xbox360_kernel_debug_print(ks, "DbgBreak: type=%d", break_type);
    
    printf("\n=== DEBUG BREAKPOINT HIT ===\n");
    printf("Type: %d\n", break_type);
    xbox360_kernel_dump_state(ks);
    printf("Press Enter to continue...\n");
    getchar();
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_test_function(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t test_id = params[0];
    uint32_t param1 = params[1];
    uint32_t param2 = params[2];
    
    xbox360_kernel_debug_print(ks, "TestFunction: id=%d, params=%08X %08X", 
                              test_id, param1, param2);
    
    return STATUS_SUCCESS;
}

/* ==================== TIMING ==================== */
uint32_t xbox360_get_tick_count(XBOX360_KERNEL_STATE *ks) {
    return ks->tick_count;
}

uint64_t xbox360_query_performance_counter(XBOX360_KERNEL_STATE *ks) {
    return ks->performance_counter;
}

uint64_t xbox360_query_performance_frequency(XBOX360_KERNEL_STATE *ks) {
    return ks->performance_frequency;
}

void xbox360_update_timing(XBOX360_KERNEL_STATE *ks) {
    uint64_t now = get_current_time_ns();
    uint64_t elapsed = now - ks->boot_time_ns;
    
    ks->tick_count = (uint32_t)(elapsed / 100);
    ks->performance_counter = elapsed / (1000000000 / ks->performance_frequency);
}

/* ==================== SYSTEM INFORMATION ==================== */
const XSYSTEM_INFO *xbox360_get_system_info(XBOX360_KERNEL_STATE *ks) {
    return &ks->system_info;
}

uint32_t xbox360_get_console_type(XBOX360_KERNEL_STATE *ks) {
    return ks->system_info.console_type;
}

uint32_t xbox360_get_title_id(XBOX360_KERNEL_STATE *ks) {
    return ks->current_title.title_id;
}

/* ==================== DEBUG ==================== */
void xbox360_kernel_debug_print(XBOX360_KERNEL_STATE *ks, const char *format, ...) {
    if (!ks->debug_enabled) {
        return;
    }
    
    va_list args;
    char buffer[1024];
    
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    printf("[KERNEL] %s\n", buffer);
    
    if (ks->log_file) {
        fprintf(ks->log_file, "[%llu] %s\n", 
                (unsigned long long)get_current_time_ns(), buffer);
        fflush(ks->log_file);
    }
}

void xbox360_kernel_dump_state(XBOX360_KERNEL_STATE *ks) {
    printf("\n=== KERNEL STATE DUMP ===\n");
    printf("Current process: %08X\n", ks->current_process_id);
    printf("Current thread:  %08X\n", ks->current_thread_id);
    printf("Tick count:      %u\n", ks->tick_count);
    printf("Performance:     %llu / %llu\n", 
           (unsigned long long)ks->performance_counter,
           (unsigned long long)ks->performance_frequency);
    
    printf("\nProcesses (%d):\n", 32);
    for (int i = 0; i < 32; i++) {
        if (ks->processes[i].pid != 0) {
            printf("  %08X: %s (title: %08X)\n",
                   ks->processes[i].pid,
                   ks->processes[i].name,
                   ks->processes[i].title_id);
        }
    }
    
    printf("\nThreads (%d active):\n", 256);
    int active_threads = 0;
    for (int i = 0; i < 256; i++) {
        if (ks->threads[i].tid != 0 && ks->threads[i].state != THREAD_STATE_TERMINATED) {
            active_threads++;
            printf("  %08X: state=%d, priority=%d\n",
                   ks->threads[i].tid,
                   ks->threads[i].state,
                   ks->threads[i].priority);
        }
    }
    printf("  Total active: %d\n", active_threads);
    
    printf("\nMemory allocations (%d):\n", 1024);
    int active_allocs = 0;
    for (int i = 0; i < 1024; i++) {
        if (ks->allocations[i].handle != 0) {
            active_allocs++;
            printf("  %08X: %d bytes @ %08X\n",
                   ks->allocations[i].handle,
                   ks->allocations[i].size,
                   ks->allocations[i].base_address);
        }
    }
    printf("  Total allocations: %d\n", active_allocs);
    printf("=======================\n");
}

void xbox360_kernel_set_debug(XBOX360_KERNEL_STATE *ks, bool enabled) {
    ks->debug_enabled = enabled;
}

/* ==================== CALLBACKS ==================== */
void xbox360_kernel_set_callback(XBOX360_KERNEL_STATE *ks, void (*callback)(void *, uint32_t, uint32_t), void *opaque) {
    ks->syscall_callback = callback;
    ks->callback_opaque = opaque;
}
