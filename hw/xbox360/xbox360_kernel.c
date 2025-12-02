#include "hw/xbox360/xbox360_kernel.h"
#include "hw/xbox360/xbox360.h"
#include "hw/xbox360/crypto/xe_crypto.h"
#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "qemu/error-report.h"
#include <stdarg.h>
#include <string.h>
#include <time.h>

/* ==================== SYSCALL HANDLERS ==================== */

typedef uint32_t (*SyscallHandler)(XBOX360_KERNEL_STATE *ks, uint32_t *params);
static SyscallHandler syscall_handlers[256];

/* ==================== HELPER FUNCTIONS ==================== */

static uint64_t get_current_time_ns(void) {
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static uint32_t generate_handle(void) {
    static uint32_t next_handle = 0x1000;
    return next_handle++;
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

/* ==================== SYSTEM SYSCALLS ==================== */

static uint32_t syscall_get_tick_count(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    return ks->tick_count;
}

static uint32_t syscall_query_performance_counter(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    return ks->performance_counter & 0xFFFFFFFF;
}

static uint32_t syscall_query_performance_frequency(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    return ks->performance_frequency & 0xFFFFFFFF;
}

static uint32_t syscall_ke_delay_execution_thread(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint64_t *interval = (uint64_t*)params[2];
    if (!interval) return STATUS_INVALID_PARAMETER;
    return STATUS_SUCCESS;
}

static uint32_t syscall_ke_set_timer(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t *timer_handle = (uint32_t*)params[0];
    if (!timer_handle) return STATUS_INVALID_PARAMETER;
    *timer_handle = generate_handle();
    return STATUS_SUCCESS;
}

static uint32_t syscall_ke_cancel_timer(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    return STATUS_SUCCESS;
}

/* ==================== THREAD MANAGEMENT ==================== */

static uint32_t syscall_ex_create_thread(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t *thread_handle_ptr = (uint32_t*)params[0];
    uint32_t *thread_id_ptr = (uint32_t*)params[3];
    uint32_t start_address = params[4];
    
    if (!thread_handle_ptr || !thread_id_ptr) {
        return STATUS_INVALID_PARAMETER;
    }
    
    uint32_t thread_id = xbox360_create_thread(ks, start_address, 64 * 1024, 8);
    if (thread_id == 0) {
        return STATUS_NO_MEMORY;
    }
    
    *thread_handle_ptr = generate_handle();
    *thread_id_ptr = thread_id;
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_ke_resume_thread(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t thread_handle = params[0];
    XTHREAD_CONTEXT *thread = find_thread(ks, thread_handle);
    if (!thread) return STATUS_NOT_FOUND;
    
    thread->state = 0; // THREAD_STATE_READY
    return STATUS_SUCCESS;
}

static uint32_t syscall_ke_suspend_thread(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t thread_handle = params[0];
    XTHREAD_CONTEXT *thread = find_thread(ks, thread_handle);
    if (!thread) return STATUS_NOT_FOUND;
    
    thread->state = 2; // THREAD_STATE_WAITING
    return STATUS_SUCCESS;
}

static uint32_t syscall_ke_terminate_thread(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t exit_code = params[0];
    uint32_t result = xbox360_terminate_thread(ks, ks->current_thread_id);
    return result;
}

static uint32_t syscall_ke_get_current_processor_number(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    return 0;
}

static uint32_t syscall_ke_set_processor_affinity(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    return STATUS_SUCCESS;
}

static uint32_t syscall_ke_query_active_processors(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    return 0x00000007; // CPUs 0, 1 and 2
}

/* ==================== MEMORY MANAGEMENT ==================== */

static uint32_t syscall_mm_allocate_physical_memory(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t size = params[0];
    uint32_t alignment = params[1];
    uint32_t protection = params[2];
    uint32_t *handle_ptr = (uint32_t*)params[3];
    
    if (!handle_ptr || size == 0) {
        return STATUS_INVALID_PARAMETER;
    }
    
    uint32_t handle = xbox360_allocate_memory(ks, size, alignment, 0);
    if (handle == 0) {
        return STATUS_NO_MEMORY;
    }
    
    XALLOCATION_INFO *alloc = find_allocation(ks, handle);
    if (alloc) {
        alloc->protection = protection;
    }
    
    *handle_ptr = handle;
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
    
    *physical_address_ptr = virtual_address;
    return STATUS_SUCCESS;
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
        alloc->type = 2; // MEMORY_TYPE_IO
        alloc->protection = protection;
    }
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_mm_unmap_io_space(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t virtual_address = params[0];
    
    for (int i = 0; i < 1024; i++) {
        if (ks->allocations[i].base_address == virtual_address && 
            ks->allocations[i].type == 2) {
            memset(&ks->allocations[i], 0, sizeof(XALLOCATION_INFO));
            break;
        }
    }
    
    return STATUS_SUCCESS;
}

/* ==================== FILE SYSTEM ==================== */

static uint32_t syscall_nt_create_file(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t *handle_ptr = (uint32_t*)params[0];
    const char *file_name = (const char*)params[9];
    
    if (!handle_ptr || !file_name) {
        return STATUS_INVALID_PARAMETER;
    }
    
    if (strcmp(file_name, "\\Device\\Harddisk0\\Partition1\\xboxdash.xex") == 0 ||
        strcmp(file_name, "\\Device\\Harddisk0\\Partition1\\Dashboard.xex") == 0) {
        *handle_ptr = generate_handle();
        return STATUS_SUCCESS;
    }
    
    return STATUS_OBJECT_NAME_NOT_FOUND;
}

static uint32_t syscall_nt_open_file(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t *handle_ptr = (uint32_t*)params[0];
    
    if (!handle_ptr) {
        return STATUS_INVALID_PARAMETER;
    }
    
    *handle_ptr = generate_handle();
    return STATUS_SUCCESS;
}

static uint32_t syscall_nt_read_file(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t file_handle = params[0];
    void *buffer = (void*)params[2];
    uint32_t length = params[3];
    uint32_t *io_status_block = (uint32_t*)params[1];
    
    if (file_handle == 0 || !buffer || length == 0) {
        return STATUS_INVALID_PARAMETER;
    }
    
    if (io_status_block) {
        io_status_block[0] = STATUS_END_OF_FILE;
        io_status_block[1] = 0;
    }
    
    return STATUS_END_OF_FILE;
}

static uint32_t syscall_nt_write_file(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    void *buffer = (void*)params[2];
    uint32_t length = params[3];
    uint32_t *io_status_block = (uint32_t*)params[1];
    
    if (!buffer || length == 0) {
        return STATUS_INVALID_PARAMETER;
    }
    
    if (io_status_block) {
        io_status_block[0] = STATUS_SUCCESS;
        io_status_block[1] = length;
    }
    
    return STATUS_SUCCESS;
}

static uint32_t syscall_nt_close(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    return STATUS_SUCCESS;
}

/* ==================== GRAPHICS ==================== */

static uint32_t syscall_vgpuhd_create_device(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t *device_handle_ptr = (uint32_t*)params[0];
    
    if (!device_handle_ptr) {
        return STATUS_INVALID_PARAMETER;
    }
    
    *device_handle_ptr = generate_handle();
    ks->display_enabled = 1;
    return STATUS_SUCCESS;
}

static uint32_t syscall_vgpuhd_setmode(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    ks->display_mode.width = params[1];
    ks->display_mode.height = params[2];
    ks->display_mode.refresh_rate = params[3];
    return STATUS_SUCCESS;
}

static uint32_t syscall_vgpuhd_waitforvsync(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    return STATUS_SUCCESS;
}

static uint32_t syscall_vgpuhd_destroy_device(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    ks->display_enabled = 0;
    return STATUS_SUCCESS;
}

static uint32_t syscall_vgpuhd_flip(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    return STATUS_SUCCESS;
}

/* ==================== AUDIO ==================== */

static uint32_t syscall_xaudio_create(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t *audio_handle_ptr = (uint32_t*)params[0];
    
    if (!audio_handle_ptr) {
        return STATUS_INVALID_PARAMETER;
    }
    
    *audio_handle_ptr = generate_handle();
    ks->audio_initialized = 1;
    return STATUS_SUCCESS;
}

static uint32_t syscall_xaudio_destroy(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    ks->audio_initialized = 0;
    return STATUS_SUCCESS;
}

static uint32_t syscall_xaudio_submit_buffer(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    return STATUS_SUCCESS;
}

static uint32_t syscall_xaudio_flush_buffers(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    return STATUS_SUCCESS;
}

/* ==================== CRYPTO ==================== */

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

static uint32_t syscall_xecrypt_sha_final(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t context = params[0];
    uint8_t *digest = (uint8_t*)params[1];
    
    if (context != (uint32_t)&ks->sha_state || !digest) {
        return STATUS_INVALID_PARAMETER;
    }
    
    xeCryptShaFinal(&ks->sha_state, digest);
    return STATUS_SUCCESS;
}

static uint32_t syscall_xecrypt_aes_cbc(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    return STATUS_SUCCESS;
}

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

/* ==================== XAM ==================== */

static uint32_t syscall_xam_get_consoletype(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t *console_type_ptr = (uint32_t*)params[0];
    
    if (!console_type_ptr) {
        return STATUS_INVALID_PARAMETER;
    }
    
    *console_type_ptr = 1; // CONSOLE_TYPE_RETAIL
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

static uint32_t syscall_xam_get_currenttitleinfo(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    uint32_t title_info_ptr = params[0];
    
    if (!title_info_ptr) {
        return STATUS_INVALID_PARAMETER;
    }
    
    memcpy((void*)title_info_ptr, &ks->current_title, sizeof(XTITLE_INFO));
    return STATUS_SUCCESS;
}

static uint32_t syscall_xam_launch_title(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    ks->current_title.title_id = params[0];
    return STATUS_SUCCESS;
}

static uint32_t syscall_xam_terminate_title(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    ks->current_title.title_id = 0xFFFE07D1; // Dashboard
    return STATUS_SUCCESS;
}

/* ==================== DEBUG ==================== */

static uint32_t syscall_dbg_print(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    const char *format = (const char*)params[0];
    
    if (!format) {
        return STATUS_INVALID_PARAMETER;
    }
    
    vprintf(format, (va_list)&params[1]);
    return STATUS_SUCCESS;
}

static uint32_t syscall_dbg_break(XBOX360_KERNEL_STATE *ks, uint32_t *params) {
    return STATUS_SUCCESS;
}

/* ==================== SYSCALL DISPATCHER ==================== */

static void initialize_syscall_handlers(void) {
    memset(syscall_handlers, 0, sizeof(syscall_handlers));
    
    // System
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
    syscall_handlers[SYSCALL_EX_CREATE_THREAD] = syscall_ex_create_thread;
    
    // Memory
    syscall_handlers[SYSCALL_MM_ALLOCATE_PHYSICAL_MEMORY] = syscall_mm_allocate_physical_memory;
    syscall_handlers[SYSCALL_MM_FREE_PHYSICAL_MEMORY] = syscall_mm_free_physical_memory;
    syscall_handlers[SYSCALL_MM_MAP_IO_SPACE] = syscall_mm_map_io_space;
    syscall_handlers[SYSCALL_MM_UNMAP_IO_SPACE] = syscall_mm_unmap_io_space;
    syscall_handlers[SYSCALL_MM_GET_PHYSICAL_ADDRESS] = syscall_mm_get_physical_address;
    
    // Filesystem
    syscall_handlers[SYSCALL_NT_CREATE_FILE] = syscall_nt_create_file;
    syscall_handlers[SYSCALL_NT_OPEN_FILE] = syscall_nt_open_file;
    syscall_handlers[SYSCALL_NT_READ_FILE] = syscall_nt_read_file;
    syscall_handlers[SYSCALL_NT_WRITE_FILE] = syscall_nt_write_file;
    syscall_handlers[SYSCALL_NT_CLOSE] = syscall_nt_close;
    
    // Graphics
    syscall_handlers[SYSCALL_VGPUHD_CREATE_DEVICE] = syscall_vgpuhd_create_device;
    syscall_handlers[SYSCALL_VGPUHD_DESTROY_DEVICE] = syscall_vgpuhd_destroy_device;
    syscall_handlers[SYSCALL_VGPUHD_FLIP] = syscall_vgpuhd_flip;
    syscall_handlers[SYSCALL_VGPUHD_SETMODE] = syscall_vgpuhd_setmode;
    syscall_handlers[SYSCALL_VGPUHD_WAITFORVSYNC] = syscall_vgpuhd_waitforvsync;
    
    // Audio
    syscall_handlers[SYSCALL_XAUDIO_CREATE] = syscall_xaudio_create;
    syscall_handlers[SYSCALL_XAUDIO_DESTROY] = syscall_xaudio_destroy;
    syscall_handlers[SYSCALL_XAUDIO_SUBMIT_BUFFER] = syscall_xaudio_submit_buffer;
    syscall_handlers[SYSCALL_XAUDIO_FLUSH_BUFFERS] = syscall_xaudio_flush_buffers;
    
    // Crypto
    syscall_handlers[SYSCALL_XECRYPT_SHA_INIT] = syscall_xecrypt_sha_init;
    syscall_handlers[SYSCALL_XECRYPT_SHA_UPDATE] = syscall_xecrypt_sha_update;
    syscall_handlers[SYSCALL_XECRYPT_SHA_FINAL] = syscall_xecrypt_sha_final;
    syscall_handlers[SYSCALL_XECRYPT_AES_CBC] = syscall_xecrypt_aes_cbc;
    syscall_handlers[SYSCALL_XECRYPT_RC4] = syscall_xecrypt_rc4;
    
    // XAM
    syscall_handlers[SYSCALL_XAM_GET_CONSOLETYPE] = syscall_xam_get_consoletype;
    syscall_handlers[SYSCALL_XAM_GET_SYSTEMINFO] = syscall_xam_get_systeminfo;
    syscall_handlers[SYSCALL_XAM_GET_CURRENTTITLEID] = syscall_xam_get_currenttitleid;
    syscall_handlers[SYSCALL_XAM_GET_CURRENTTITLEINFO] = syscall_xam_get_currenttitleinfo;
    syscall_handlers[SYSCALL_XAM_LAUNCH_TITLE] = syscall_xam_launch_title;
    syscall_handlers[SYSCALL_XAM_TERMINATE_TITLE] = syscall_xam_terminate_title;
    
    // Debug
    syscall_handlers[SYSCALL_DBG_PRINT] = syscall_dbg_print;
    syscall_handlers[SYSCALL_DBG_BREAK] = syscall_dbg_break;
}

uint32_t xbox360_handle_syscall(XBOX360_KERNEL_STATE *ks, uint32_t syscall_number, 
                               uint32_t *parameters, uint32_t parameter_count) {
    static bool initialized = false;
    if (!initialized) {
        initialize_syscall_handlers();
        initialized = true;
    }
    
    if (syscall_number >= 256) {
        return STATUS_NOT_IMPLEMENTED;
    }
    
    SyscallHandler handler = syscall_handlers[syscall_number];
    if (!handler) {
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
    ks->performance_frequency = 1000000;
    ks->performance_counter = 0;
    
    // System info
    ks->system_info.console_type = 1;
    ks->system_info.cpu_count = 3;
    ks->system_info.cpu_speed_mhz = 3200;
    ks->system_info.memory_size_mb = 512;
    ks->system_info.av_region = 0;
    ks->system_info.game_region = 0;
    ks->system_info.system_version = 0x2B0;
    ks->system_info.dashboard_version = 0x2B0;
    ks->system_info.kernel_version = 0x2B0;
    ks->system_info.build_date = 20110830;
    
    memcpy(ks->system_info.serial_number, "123456789012", 12);
    memcpy(ks->system_info.console_id, "\x11\x22\x33\x44\x55", 5);
    
    // Current title
    ks->current_title.title_id = 0xFFFE07D1;
    ks->current_title.title_version = 0x2B0;
    ks->current_title.media_type = 0;
    strcpy(ks->current_title.title_name, "Xbox 360 Dashboard");
    strcpy(ks->current_title.publisher, "Microsoft");
    strcpy(ks->current_title.developer, "Microsoft");
    
    // Display
    ks->display_mode.width = 1280;
    ks->display_mode.height = 720;
    ks->display_mode.refresh_rate = 60;
    ks->display_enabled = 0;
    
    // IDs
    ks->next_process_id = 0x1000;
    ks->next_thread_id = 0x2000;
    ks->next_file_handle = 0x4000;
    ks->next_allocation_handle = 0x8000;

    ks->timer_callback = xenon_timer_get_callback(xenon->timer);
    ks->timer_opaque = xenon;

    ks->tick_period_ns = 1000000;
    ks->last_tick_update = get_current_time_ns();
    
    // Create kernel process
    ks->current_process_id = ks->next_process_id++;
    XPROCESS_CONTEXT *kernel_proc = &ks->processes[0];
    kernel_proc->pid = ks->current_process_id;
    kernel_proc->parent_pid = 0;
    kernel_proc->title_id = 0;
    kernel_proc->base_address = 0x80000000;
    kernel_proc->size = 0x100000;
    strcpy(kernel_proc->name, "Kernel");
    
    // Create initial thread
    ks->current_thread_id = xbox360_create_thread(ks, 0x80000000, 64 * 1024, 0);
    
    // Debug
    ks->debug_enabled = true;
    ks->log_file = fopen("xbox360_kernel.log", "w");
}

void xbox360_kernel_reset(XBOX360_KERNEL_STATE *ks) {
    if (ks->log_file) {
        fclose(ks->log_file);
        ks->log_file = NULL;
    }
    
    XenonState *xenon = NULL;
    xbox360_kernel_init(ks, xenon);
}

void xbox360_update_timing(XBOX360_KERNEL_STATE *ks) {
    uint64_t now = get_current_time_ns();
    uint64_t elapsed = now - ks->last_tick_update;
    
    /* Update tick count based on elapsed time */
    ks->tick_count += elapsed / ks->tick_period_ns;
    ks->last_tick_update = now;
    
    /* Update performance counter */
    ks->performance_counter = now;
}

uint64_t xbox360_get_system_time(XBOX360_KERNEL_STATE *ks) {
    return ks->performance_counter;
}

uint64_t xbox360_get_timer_value(XBOX360_KERNEL_STATE *ks, int timer) {
    if (!ks->xenon_state || !ks->xenon_state->timer) {
        return 0;
    }
    
    if (timer == 0) {
        return xenon_timer_get_global_counter(ks->xenon_state->timer);
    }
    
    return 0;
}

/* ==================== THREAD MANAGEMENT ==================== */

uint32_t xbox360_create_thread(XBOX360_KERNEL_STATE *ks, uint32_t start_address, 
                              uint32_t stack_size, uint32_t priority) {
    XTHREAD_CONTEXT *thread = NULL;
    for (int i = 0; i < 256; i++) {
        if (ks->threads[i].state == 3 || ks->threads[i].tid == 0) {
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
    thread->state = 0;
    thread->priority = priority;
    thread->create_time = get_current_time_ns();
    thread->srr0 = start_address;
    thread->gpr[1] = 0x80000000 + (256 * 1024 * 1024) - 4;
    
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
    
    thread->state = 3;
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

/* ==================== MEMORY MANAGEMENT ==================== */

uint32_t xbox360_allocate_memory(XBOX360_KERNEL_STATE *ks, uint32_t size, 
                                uint32_t alignment, uint32_t type) {
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
    
    return (void*)(uintptr_t)alloc->base_address;
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

/* ==================== SYSTEM INFO ==================== */

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
    printf("KERNEL STATE:\n");
    printf("  Current PID: %08X\n", ks->current_process_id);
    printf("  Current TID: %08X\n", ks->current_thread_id);
    printf("  Tick Count: %u\n", ks->tick_count);
    printf("  Title ID: %08X\n", ks->current_title.title_id);
}

/* ==================== CALLBACKS ==================== */
void xbox360_kernel_set_callback(XBOX360_KERNEL_STATE *ks,
                                void (*callback)(void *, uint32_t, uint32_t),
                                void *opaque) {
    ks->syscall_callback = callback;
    ks->callback_opaque = opaque;
}
