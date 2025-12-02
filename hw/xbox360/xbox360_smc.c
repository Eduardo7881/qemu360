#include "hw/xbox360/xbox360_smc.h"
#include "hw/xbox360/xbox360_nand.h"
#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"

/* ==================== SMC REGISTERS ==================== */

#define SMC_FIRMWARE_VERSION     0x00020000
#define DEFAULT_CPU_TEMP         450
#define DEFAULT_GPU_TEMP         500
#define DEFAULT_FAN_SPEED        30
#define DEFAULT_POWER_LIMIT      150
#define DEFAULT_TEMP_LIMIT_CPU   800
#define DEFAULT_TEMP_LIMIT_GPU   850

static uint64_t smc_read(void *opaque, hwaddr addr, unsigned size) {
    Xbox360SMCState *s = opaque;
    uint32_t value = 0;
    uint32_t reg = addr / 4;
    
    if (reg >= SMC_REGISTER_SIZE / 4) {
        return 0;
    }
    
    switch (addr) {
        case SMC_REG_VERSION:
            value = SMC_FIRMWARE_VERSION;
            break;
            
        case SMC_REG_STATUS:
            value = (s->power_state << 0) |
                    ((s->error_code != 0) << 8) |
                    (s->boot_reason << 16);
            break;
            
        case SMC_REG_CPU_TEMP:
            value = s->cpu_temp;
            break;
            
        case SMC_REG_GPU_TEMP:
            value = s->gpu_temp;
            break;
            
        case SMC_REG_FAN_SPEED:
            value = s->fan_speed;
            break;
            
        case SMC_REG_POWER_STATE:
            value = s->power_state;
            break;
            
        case SMC_REG_CPU_KEY:
            if (size == 4) {
                uint32_t offset = addr - SMC_REG_CPU_KEY;
                if (offset + 4 <= sizeof(s->cpu_key)) {
                    value = (s->cpu_key[offset] << 24) |
                            (s->cpu_key[offset + 1] << 16) |
                            (s->cpu_key[offset + 2] << 8) |
                            s->cpu_key[offset + 3];
                }
            }
            break;
            
        case SMC_REG_CONSOLE_ID:
            if (size == 4) {
                if (addr == SMC_REG_CONSOLE_ID) {
                    value = (s->console_id[0] << 24) |
                            (s->console_id[1] << 16) |
                            (s->console_id[2] << 8) |
                            s->console_id[3];
                } else if (addr == SMC_REG_CONSOLE_ID + 4) {
                    value = s->console_id[4] << 24;
                }
            }
            break;
            
        case SMC_REG_BOOT_REASON:
            value = s->boot_reason;
            break;
            
        case SMC_REG_POST_CODE:
            value = s->post_code;
            break;
            
        case SMC_REG_ERROR_CODE:
            value = s->error_code;
            break;
            
        case SMC_REG_AV_PACK:
            value = s->av_pack;
            break;
            
        case SMC_REG_REGION:
            value = s->region;
            break;
            
        case SMC_REG_SERIAL:
            if (size == 4) {
                uint32_t offset = addr - SMC_REG_SERIAL;
                if (offset + 4 <= sizeof(s->serial_number)) {
                    value = (s->serial_number[offset] << 24) |
                            (s->serial_number[offset + 1] << 16) |
                            (s->serial_number[offset + 2] << 8) |
                            s->serial_number[offset + 3];
                }
            }
            break;
            
        case SMC_REG_MAC_ADDR:
            if (size == 4) {
                uint32_t offset = addr - SMC_REG_MAC_ADDR;
                if (offset + 4 <= sizeof(s->mac_address)) {
                    value = (s->mac_address[offset] << 24) |
                            (s->mac_address[offset + 1] << 16) |
                            (s->mac_address[offset + 2] << 8) |
                            s->mac_address[offset + 3];
                }
            }
            break;
            
        case SMC_REG_CONFIG_FLAGS:
            value = s->config_flags;
            break;
            
        case SMC_REG_POWER_LIMIT:
            value = s->power_limit;
            break;
            
        case SMC_REG_TEMP_LIMITS:
            value = (s->temp_limit_cpu << 0) |
                    (s->temp_limit_gpu << 16);
            break;
            
        case SMC_REG_COMMAND:
            value = s->last_command;
            break;
            
        case SMC_REG_RESPONSE:
            value = s->command_result;
            break;
            
        default:
            value = s->registers[reg];
            break;
    }
    
    return value;
}

static void smc_write(void *opaque, hwaddr addr, 
                     uint64_t value, unsigned size) {
    Xbox360SMCState *s = opaque;
    uint32_t reg = addr / 4;
    
    if (reg >= SMC_REGISTER_SIZE / 4) {
        return;
    }
    
    switch (addr) {
        case SMC_REG_FAN_SPEED:
            if (size == 4) {
                s->fan_speed = value & 0xFF;
                if (s->fan_speed > 100) s->fan_speed = 100;
            }
            break;
            
        case SMC_REG_POWER_STATE:
            if (size == 4) {
                xbox360_smc_set_power_state(s, value & 0xFF);
            }
            break;
            
        case SMC_REG_POST_CODE:
            if (size == 4) {
                s->post_code = value;
            }
            break;
            
        case SMC_REG_ERROR_CODE:
            if (size == 4) {
                if (value == 0) {
                    xbox360_smc_clear_errors(s);
                } else {
                    s->error_code = value;
                }
            }
            break;
            
        case SMC_REG_CONFIG_FLAGS:
            if (size == 4) {
                s->config_flags = value;
            }
            break;
            
        case SMC_REG_POWER_LIMIT:
            if (size == 4) {
                s->power_limit = value;
                if (s->power_limit > 200) s->power_limit = 200;
            }
            break;
            
        case SMC_REG_TEMP_LIMITS:
            if (size == 4) {
                s->temp_limit_cpu = (value >> 0) & 0xFFFF;
                s->temp_limit_gpu = (value >> 16) & 0xFFFF;
                if (s->temp_limit_cpu > 950) s->temp_limit_cpu = 950;
                if (s->temp_limit_gpu > 950) s->temp_limit_gpu = 950;
            }
            break;
            
        case SMC_REG_COMMAND:
            if (size == 4) {
                SMC_COMMAND cmd = value & 0xFF;
                s->last_command = cmd;
                
                uint8_t response[32];
                uint32_t resp_len = 0;
                s->command_result = xbox360_smc_execute_command(
                    s, cmd, s->scratch, sizeof(s->scratch),
                    response, &resp_len);
                
                if (resp_len > 0 && resp_len <= sizeof(s->scratch)) {
                    memcpy(s->scratch, response, resp_len);
                }
            }
            break;
            
        case SMC_REG_RESET:
            if (size == 4 && value == 0xDEADBEEF) {
                if (s->reset_callback) {
                    s->reset_callback(s->callback_opaque);
                }
            }
            break;
            
        default:
            s->registers[reg] = value;
            break;
    }
}

static const MemoryRegionOps smc_ops = {
    .read = smc_read,
    .write = smc_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

/* ==================== SMC COMMANDS ==================== */

uint32_t xbox360_smc_execute_command(Xbox360SMCState *s, SMC_COMMAND cmd,
                                     const uint8_t *params, uint32_t param_len,
                                     uint8_t *response, uint32_t *resp_len) {
    uint32_t result = 0;
    
    if (resp_len) *resp_len = 0;
    
    switch (cmd) {
        case SMC_CMD_GET_VERSION:
            if (response && resp_len && *resp_len >= 4) {
                *(uint32_t*)response = SMC_FIRMWARE_VERSION;
                *resp_len = 4;
            }
            result = 0;
            break;
            
        case SMC_CMD_GET_TEMPERATURE:
            if (response && resp_len && *resp_len >= 8) {
                *(uint32_t*)(response + 0) = s->cpu_temp;
                *(uint32_t*)(response + 4) = s->gpu_temp;
                *resp_len = 8;
            }
            result = 0;
            break;
            
        case SMC_CMD_GET_FAN_SPEED:
            if (response && resp_len && *resp_len >= 1) {
                response[0] = s->fan_speed;
                *resp_len = 1;
            }
            result = 0;
            break;
            
        case SMC_CMD_SET_FAN_SPEED:
            if (param_len >= 1) {
                s->fan_speed = params[0];
                if (s->fan_speed > 100) s->fan_speed = 100;
                result = 0;
            } else {
                result = 0x80000001;
            }
            break;
            
        case SMC_CMD_GET_POWER_STATE:
            if (response && resp_len && *resp_len >= 1) {
                response[0] = s->power_state;
                *resp_len = 1;
            }
            result = 0;
            break;
            
        case SMC_CMD_SET_POWER_STATE:
            if (param_len >= 1) {
                xbox360_smc_set_power_state(s, params[0]);
                result = 0;
            } else {
                result = 0x80000001;
            }
            break;
            
        case SMC_CMD_GET_BOOT_REASON:
            if (response && resp_len && *resp_len >= 1) {
                response[0] = s->boot_reason;
                *resp_len = 1;
            }
            result = 0;
            break;
            
        case SMC_CMD_GET_CPU_KEY:
            if (response && resp_len && *resp_len >= 16) {
                memcpy(response, s->cpu_key, 16);
                *resp_len = 16;
            }
            result = 0;
            break;
            
        case SMC_CMD_GET_CONSOLE_ID:
            if (response && resp_len && *resp_len >= 5) {
                memcpy(response, s->console_id, 5);
                *resp_len = 5;
            }
            result = 0;
            break;
            
        case SMC_CMD_GET_SERIAL:
            if (response && resp_len && *resp_len >= 12) {
                memcpy(response, s->serial_number, 12);
                *resp_len = 12;
            }
            result = 0;
            break;
            
        case SMC_CMD_GET_REGION:
            if (response && resp_len && *resp_len >= 1) {
                response[0] = s->region;
                *resp_len = 1;
            }
            result = 0;
            break;
            
        case SMC_CMD_GET_AV_PACK:
            if (response && resp_len && *resp_len >= 1) {
                response[0] = s->av_pack;
                *resp_len = 1;
            }
            result = 0;
            break;
            
        case SMC_CMD_GET_POST_CODE:
            if (response && resp_len && *resp_len >= 4) {
                *(uint32_t*)response = s->post_code;
                *resp_len = 4;
            }
            result = 0;
            break;
            
        case SMC_CMD_CLEAR_ERRORS:
            xbox360_smc_clear_errors(s);
            result = 0;
            break;
            
        case SMC_CMD_ENTER_STANDBY:
            xbox360_smc_set_power_state(s, POWER_STATE_STANDBY);
            result = 0;
            break;
            
        case SMC_CMD_EXIT_STANDBY:
            xbox360_smc_set_power_state(s, POWER_STATE_ON);
            result = 0;
            break;
            
        case SMC_CMD_SHUTDOWN:
            xbox360_smc_set_power_state(s, POWER_STATE_OFF);
            result = 0;
            break;
            
        case SMC_CMD_REBOOT:
            xbox360_smc_set_power_state(s, POWER_STATE_REBOOT);
            result = 0;
            break;
            
        default:
            result = 0x80000000;
            break;
    }
    
    return result;
}

/* ==================== SMC FUNCTIONS ==================== */

void xbox360_smc_set_power_state(Xbox360SMCState *s, POWER_STATE state) {
    if (s->power_state == state) {
        return;
    }
    
    s->power_state = state;
    
    if (s->power_callback) {
        s->power_callback(s->callback_opaque, state);
    }
    
    switch (state) {
        case POWER_STATE_OFF:
            break;
            
        case POWER_STATE_STANDBY:
            s->fan_speed = 10;
            break;
            
        case POWER_STATE_ON:
            s->fan_speed = 30;
            break;
            
        case POWER_STATE_REBOOT:
            s->power_state = POWER_STATE_ON;
            if (s->reset_callback) {
                s->reset_callback(s->callback_opaque);
            }
            break;
    }
}

void xbox360_smc_set_boot_reason(Xbox360SMCState *s, BOOT_REASON reason) {
    s->boot_reason = reason;
}

void xbox360_smc_set_error(Xbox360SMCState *s, SMC_ERROR_CODE error) {
    s->error_code |= error;
}

void xbox360_smc_clear_errors(Xbox360SMCState *s) {
    s->error_code = SMC_ERROR_NONE;
}

void xbox360_smc_set_temperature(Xbox360SMCState *s, int32_t cpu_temp, int32_t gpu_temp) {
    s->cpu_temp = cpu_temp;
    s->gpu_temp = gpu_temp;
    
    if (cpu_temp > s->temp_limit_cpu) {
        xbox360_smc_set_error(s, SMC_ERROR_OVERHEAT_CPU);
    }
    
    if (gpu_temp > s->temp_limit_gpu) {
        xbox360_smc_set_error(s, SMC_ERROR_OVERHEAT_GPU);
    }
    
    int max_temp = cpu_temp > gpu_temp ? cpu_temp : gpu_temp;
    max_temp /= 10;
    
    // Simple fan control
    if (max_temp > 80) s->fan_speed = 100;
    else if (max_temp > 70) s->fan_speed = 80;
    else if (max_temp > 60) s->fan_speed = 60;
    else if (max_temp > 50) s->fan_speed = 40;
    else s->fan_speed = 30;
}

void xbox360_smc_set_fan_speed(Xbox360SMCState *s, uint8_t speed) {
    if (speed > 100) speed = 100;
    s->fan_speed = speed;
}

void xbox360_smc_set_power_callback(Xbox360SMCState *s, void (*callback)(void *, POWER_STATE), void *opaque) {
    s->power_callback = callback;
    s->callback_opaque = opaque;
}

void xbox360_smc_set_reset_callback(Xbox360SMCState *s, void (*callback)(void *), void *opaque) {
    s->reset_callback = callback;
    s->callback_opaque = opaque;
}

/* ==================== SMC INITIALIZATION ==================== */

static void smc_realize(DeviceState *dev, Error **errp) {
    Xbox360SMCState *s = XBOX360_SMC(dev);
    
    memset(s->registers, 0, sizeof(s->registers));
    
    s->console_type = SMC_CONSOLE_TRINITY;
    s->power_state = POWER_STATE_ON;
    s->boot_reason = BOOT_REASON_POWER_ON;
    s->error_code = SMC_ERROR_NONE;
    s->av_pack = AV_PACK_HDMI;
    s->region = REGION_NTSC_U;
    
    s->cpu_temp = DEFAULT_CPU_TEMP;
    s->gpu_temp = DEFAULT_GPU_TEMP;
    s->fan_speed = DEFAULT_FAN_SPEED;
    
    time_t now = time(NULL);
    struct tm *tm = localtime(&now);
    s->rtc.seconds = tm->tm_sec;
    s->rtc.minutes = tm->tm_min;
    s->rtc.hours = tm->tm_hour;
    s->rtc.day = tm->tm_mday;
    s->rtc.month = tm->tm_mon + 1;
    s->rtc.year = tm->tm_year + 1900;
    
    s->post_code = 0xAA;
    s->config_flags = 0;
    s->power_limit = DEFAULT_POWER_LIMIT;
    s->temp_limit_cpu = DEFAULT_TEMP_LIMIT_CPU;
    s->temp_limit_gpu = DEFAULT_TEMP_LIMIT_GPU;
    
    s->last_command = SMC_CMD_NOP;
    s->command_result = 0;
    
    if (s->nand_state) {
        memcpy(s->cpu_key, s->nand_state->key_vault.cpu_key, 16);
        memcpy(s->console_id, s->nand_state->key_vault.console_id, 5);
        memcpy(s->serial_number, s->nand_state->key_vault.serial_number, 12);
        memcpy(s->mac_address, s->nand_state->key_vault.mac_address, 6);
    } else {
        memset(s->cpu_key, 0xAA, 16);
        memset(s->console_id, 0xBB, 5);
        memset(s->serial_number, '0', 12);
        memset(s->mac_address, 0xCC, 6);
    }
    
    memory_region_init_io(&s->iomem, OBJECT(s), &smc_ops, s,
                         "xbox360.smc", SMC_REGISTER_SIZE);
}

/* ==================== QEMU DEVICE ==================== */

static Property smc_properties[] = {
    DEFINE_PROP_PTR("nand-state", Xbox360SMCState, nand_state),
    DEFINE_PROP_END_OF_LIST(),
};

static void smc_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    
    dc->realize = smc_realize;
    dc->desc = "Xbox 360 System Management Controller";
    device_class_set_props(dc, smc_properties);
}

static const TypeInfo smc_type_info = {
    .name = TYPE_XBOX360_SMC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Xbox360SMCState),
    .class_init = smc_class_init,
};

static void smc_register_types(void) {
    type_register_static(&smc_type_info);
}

type_init(smc_register_types);

/* ==================== CREATE FUNCTION ==================== */

Xbox360SMCState *xbox360_smc_create(MemoryRegion *parent, hwaddr base, 
                                    XBOX360_NAND_STATE *nand_state) {
    DeviceState *dev;
    Xbox360SMCState *s;
    
    dev = qdev_new(TYPE_XBOX360_SMC);
    s = XBOX360_SMC(dev);
    
    s->nand_state = nand_state;
    
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    
    return s;
}
