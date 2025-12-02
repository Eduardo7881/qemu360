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
    memset(syscall_handlers, 0, sizeof(syscall_handlers));
    
    syscall_handlers[SYSCALL_GET_TICK_COUNT] = syscall_get_tick_count;
    syscall_handlers[SYSCALL_QUERY_PERFORMANCE_COUNTER] = syscall_query_performance_counter;
    syscall_handlers[SYSCALL_QUERY_PERFORMANCE_FREQUENCY] = syscall_query_performance_frequency;
    syscall_handlers[SYSCALL_KE_DELAY_EXECUTION_THREAD] = syscall_ke_delay_execution_thread;
    
    syscall_handlers[SYSCALL_EX_CREATE_THREAD] = syscall_ex_create_thread;
    syscall_handlers[SYSCALL_KE_TERMINATE_THREAD] = syscall_ke_terminate_thread;
    syscall_handlers[SYSCALL_KE_GET_CURRENT_PROCESSOR_NUMBER] = syscall_ke_get_current_processor_number;
    
    syscall_handlers[SYSCALL_MM_ALLOCATE_PHYSICAL_MEMORY] = syscall_mm_allocate_physical_memory;
    syscall_handlers[SYSCALL_MM_FREE_PHYSICAL_MEMORY] = syscall_mm_free_physical_memory;
    syscall_handlers[SYSCALL_MM_GET_PHYSICAL_ADDRESS] = syscall_mm_get_physical_address;
    
    syscall_handlers[SYSCALL_NT_CREATE_FILE] = syscall_nt_create_file;
    syscall_handlers[SYSCALL_NT_READ_FILE] = syscall_nt_read_file;
    syscall_handlers[SYSCALL_NT_CLOSE] = syscall_nt_close;
    
    syscall_handlers[SYSCALL_VGPUHD_CREATE_DEVICE] = syscall_vgpuhd_create_device;
    syscall_handlers[SYSCALL_VGPUHD_SETMODE] = syscall_vgpuhd_setmode;
    syscall_handlers[SYSCALL_VGPUHD_WAITFORVSYNC] = syscall_vgpuhd_waitforvsync;

    syscall_handlers[SYSCALL_XECRYPT_SHA_INIT] = syscall_xecrypt_sha_init;
    syscall_handlers[SYSCALL_XECRYPT_SHA_UPDATE] = syscall_xecrypt_sha_update;
    syscall_handlers[SYSCALL_XECRYPT_AES_CBC] = syscall_xecrypt_aes_cbc;
    
    syscall_handlers[SYSCALL_XAM_GET_CONSOLETYPE] = syscall_xam_get_consoletype;
    syscall_handlers[SYSCALL_XAM_GET_SYSTEMINFO] = syscall_xam_get_systeminfo;
    syscall_handlers[SYSCALL_XAM_GET_CURRENTTITLEID] = syscall_xam_get_currenttitleid;
    
    syscall_handlers[SYSCALL_DBG_PRINT] = syscall_dbg_print;
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
uint32_t xbox360_open_file(XBOX360_KERNEL_STATE *ks, const char *path, uint32_t access_mask) {
    (void)access_mask;
    
    for (int i = 0; i < 64; i++) {
        if (ks->open_files[i].handle == 0) {
            ks->open_files[i].handle = ks->next_file_handle++;
            strncpy(ks->open_files[i].path, path, 255);
            ks->open_files[i].path[255] = '\0';
            ks->open_files[i].position = 0;
            
            // Fill with dummy data
            ks->open_files[i].info.file_size = 1024 * 1024; // 1MB
            ks->open_files[i].info.attributes = FILE_ATTRIBUTE_NORMAL;
            
            return ks->open_files[i].handle;
        }
    }
    
    return 0;
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
    
    // Atualizar tick count (100ns ticks)
    ks->tick_count = (uint32_t)(elapsed / 100);
    
    // Atualizar performance counter
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
