#include "hw/xbox360/xbox360_kernel.h"
#include "hw/xbox360/xbox360.h"
#include "qemu/osdep.h"
#include "cpu.h"

static void log_syscall_details(uint32_t syscall_number, uint32_t *params, uint32_t result) {
    const char *syscall_names[] = {
        [0x01] = "GetTickCount",
        [0x02] = "QueryPerformanceCounter",
        [0x03] = "QueryPerformanceFrequency",
        [0x04] = "KeDelayExecutionThread",
        [0x05] = "KeSetTimer",
        [0x06] = "KeCancelTimer",
        [0x07] = "ExCreateThread",
        [0x08] = "KeResumeThread",
        [0x09] = "KeSuspendThread",
        [0x0A] = "KeTerminateThread",
        [0x0B] = "KeGetCurrentProcessorNumber",
        [0x0C] = "KeSetProcessorAffinity",
        [0x0D] = "KeQueryActiveProcessors",
        [0x20] = "MmAllocatePhysicalMemory",
        [0x21] = "MmFreePhysicalMemory",
        [0x22] = "MmMapIoSpace",
        [0x23] = "MmUnmapIoSpace",
        [0x24] = "MmQueryAllocationSize",
        [0x25] = "MmLockPages",
        [0x26] = "MmUnlockPages",
        [0x27] = "MmGetPhysicalAddress",
        [0x40] = "NtCreateFile",
        [0x41] = "NtOpenFile",
        [0x42] = "NtReadFile",
        [0x43] = "NtWriteFile",
        [0x44] = "NtClose",
        [0x45] = "NtQueryInformationFile",
        [0x46] = "NtSetInformationFile",
        [0x47] = "NtQueryVolumeInformation",
        [0x48] = "NtDeviceIoControl",
        [0x60] = "VgpuhdCreateDevice",
        [0x61] = "VgpuhdDestroyDevice",
        [0x62] = "VgpuhdFlip",
        [0x63] = "VgpuhdSetMode",
        [0x64] = "VgpuhdSetScanline",
        [0x65] = "VgpuhdWaitForVSync",
        [0x80] = "XAudioCreate",
        [0x81] = "XAudioDestroy",
        [0x82] = "XAudioSubmitBuffer",
        [0x83] = "XAudioFlushBuffers",
        [0xA0] = "XeCryptShaInit",
        [0xA1] = "XeCryptShaUpdate",
        [0xA2] = "XeCryptShaFinal",
        [0xA3] = "XeCryptAesCbc",
        [0xA4] = "XeCryptRsaVerify",
        [0xA5] = "XeCryptRc4",
        [0xC0] = "XamGetConsoleType",
        [0xC1] = "XamGetSystemInfo",
        [0xC2] = "XamGetCurrentTitleId",
        [0xC3] = "XamGetCurrentTitleInfo",
        [0xC4] = "XamLaunchTitle",
        [0xC5] = "XamTerminateTitle",
        [0xC6] = "XamLoadGame",
        [0xE0] = "DashboardGetVersion",
        [0xE1] = "DashboardShowUI",
        [0xE2] = "DashboardHideUI",
        [0xE3] = "DashboardGetState",
        [0xF0] = "DbgPrint",
        [0xF1] = "DbgBreak",
        [0xF2] = "TestFunction",
    };
    
    const char *name = "Unknown";
    if (syscall_number < sizeof(syscall_names)/sizeof(syscall_names[0]) && 
        syscall_names[syscall_number]) {
        name = syscall_names[syscall_number];
    }
    
    printf("[SYSCALL] %s (0x%08X) -> 0x%08X\n", name, syscall_number, result);
    
    if (syscall_number <= 0x0D) { // System calls
        printf("  Params:");
        for (int i = 0; i < 4 && i < 8; i++) {
            printf(" 0x%08X", params[i]);
        }
        printf("\n");
    }
}

/* ==================== SYSCALL INTERCEPT ==================== */
static bool xbox360_syscall_intercept(CPUState *cs, uint32_t opcode) {
    PowerPCCPU *cpu = POWERPC_CPU(cs);
    CPUPPCState *env = &cpu->env;
    
    if ((opcode >> 26) != 17) { // 17 = sc instruction
        return false;
    }
    
    XenonState *xenon = (XenonState *)cs->opaque;
    if (!xenon || !xenon->kernel_state) {
        return false;
    }
    
    XBOX360_KERNEL_STATE *ks = xenon->kernel_state;
    
    uint32_t syscall_number = env->gpr[0];
    uint32_t parameters[MAX_SYSCALL_PARAMETERS];
    
    for (int i = 0; i < MAX_SYSCALL_PARAMETERS; i++) {
        parameters[i] = env->gpr[3 + i];
    }

    xbox360_update_timing(ks);
    
    uint32_t result = xbox360_handle_syscall(ks, syscall_number, parameters, MAX_SYSCALL_PARAMETERS);
    log_syscall_details(syscall_number, parameters, result);
    
    env->gpr[3] = result;
    env->nip = env->lr;
    
    return true;
}

/* ==================== CPU HOOKS ==================== */
void xbox360_install_syscall_handler(XenonState *s) {
    for (int i = 0; i < 3; i++) {
        if (s->cpu[i]) {
            CPUState *cs = CPU(s->cpu[i]);
            
            // TODO: Configure callback to instruction interception
            // i think it's a custom QEMU api
            // boutta use CPU_SET_INTERRUPT_CALLBACK or simillar
            cs->opaque = s;
            
            printf("[SYSCALL] Handler installed for CPU%d\n", i);
        }
    }
}

/* ==================== SYSCALL EMULATION HELPERS ==================== */
uint32_t xbox360_emulate_syscall(XenonState *s, 
                                uint32_t syscall_number,
                                uint32_t param1, uint32_t param2,
                                uint32_t param3, uint32_t param4) {
    if (!s->kernel_state) {
        return STATUS_UNSUCCESSFUL;
    }
    
    uint32_t params[MAX_SYSCALL_PARAMETERS] = {0};
    params[0] = param1;
    params[1] = param2;
    params[2] = param3;
    params[3] = param4;
    
    return xbox360_handle_syscall(s->kernel_state, syscall_number, 
                                 params, MAX_SYSCALL_PARAMETERS);
}

void xbox360_register_syscall_callback(XenonState *s, void (*callback)(void *, uint32_t, uint32_t), void *opaque) {
    if (s->kernel_state) {
        xbox360_kernel_set_callback(s->kernel_state, callback, opaque);
    }
}

/* ==================== KERNEL INTEGRATION ==================== */
void xbox360_kernel_integration_init(XenonState *s) {
    printf("[KERNEL] Initializing kernel integration...\n");
    
    s->kernel_state = g_malloc0(sizeof(XBOX360_KERNEL_STATE));
    if (!s->kernel_state) {
        error_report("Failed to allocate kernel state");
        return;
    }
    
    xbox360_kernel_init(s->kernel_state, s);
    xbox360_install_syscall_handler(s);
    
    // TODO: Configure periodic timing update (with QEMU timer)
    
    printf("[KERNEL] Integration complete\n");
}

void xbox360_kernel_integration_cleanup(XenonState *s) {
    if (s->kernel_state) {
        if (s->kernel_state->log_file) {
            fclose(s->kernel_state->log_file);
        }
        g_free(s->kernel_state);
        s->kernel_state = NULL;
    }
}

/* ==================== DEBUG FUNCTIONS ==================== */
void xbox360_dump_kernel_state(XenonState *s) {
    if (s->kernel_state) {
        xbox360_kernel_dump_state(s->kernel_state);
    } else {
        printf("[KERNEL] No kernel state available\n");
    }
}
