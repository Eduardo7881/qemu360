#include "hw/xbox360/xbox360_kernel.h"
#include "hw/xbox360/xbox360.h"
#include "qemu/osdep.h"
#include "cpu.h"

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
    
    printf("[SYSCALL] Call %08X from 0x%08X\n", 
           syscall_number, env->nip - 4);
    
    uint32_t result = xbox360_handle_syscall(ks, syscall_number, parameters, MAX_SYSCALL_PARAMETERS);
    
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
