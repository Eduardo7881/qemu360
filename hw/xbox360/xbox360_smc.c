#include "hw/xbox360/xbox360_smc.h"
#include "hw/xbox360/xbox360_nand.h"
#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "trace.h"

/* ==================== DEFINATIONS ==================== */
#define SMC_FIRMWARE_VERSION     0x00020000 /* 2.0 for trinity */

#define DEFAULT_CPU_TEMP         450     // 45.0°C
#define DEFAULT_GPU_TEMP         500     // 50.0°C
#define DEFAULT_EDRAM_TEMP       480     // 48.0°C
#define DEFAULT_VRM_TEMP         420     // 42.0°C

#define DEFAULT_VOLTAGE_12V      12000
#define DEFAULT_VOLTAGE_5V       5000
#define DEFAULT_VOLTAGE_3V3      3300
#define DEFAULT_VOLTAGE_1V8      1800
#define DEFAULT_VOLTAGE_CORE     1100
#define DEFAULT_VOLTAGE_MEMORY   1800

#define DEFAULT_CURRENT_12V      5000
#define DEFAULT_CURRENT_5V       2000
#define DEFAULT_CURRENT_3V3      3000

#define MAX_CPU_TEMP             850     // 85.0°C
#define MAX_GPU_TEMP             900     // 90.0°C
#define DEFAULT_POWER_LIMIT      150     // 150W
#define DEFAULT_TEMP_LIMIT_CPU   800     // 80.0°C
#define DEFAULT_TEMP_LIMIT_GPU   850     // 85.0°C

static const uint8_t default_fan_curve[8][2] = {
    {40, 10},   // 40°C -> 10%
    {50, 20},   // 50°C -> 20%
    {60, 35},   // 60°C -> 35%
    {65, 50},   // 65°C -> 50%
    {70, 65},   // 70°C -> 65%
    {75, 80},   // 75°C -> 80%
    {80, 95},   // 80°C -> 95%
    {85, 100},  // 85°C -> 100%
};

/* ==================== MEMORY REGION OPS ==================== */
static uint64_t xbox360_smc_read(void *opaque, hwaddr addr, unsigned size) {
    Xbox360SMCState *s = opaque;
    uint32_t value = 0;
    uint32_t reg = addr / 4;
    
    if (reg >= SMC_REGISTER_SIZE / 4) {
        trace_xbox360_smc_read_invalid(addr);
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
            
        case SMC_REG_FUSE_DATA:
            if (size == 4) {
                uint32_t offset = addr - SMC_REG_FUSE_DATA;
                if (offset + 4 <= sizeof(s->fuse_data)) {
                    value = (s->fuse_data[offset] << 24) |
                            (s->fuse_data[offset + 1] << 16) |
                            (s->fuse_data[offset + 2] << 8) |
                            s->fuse_data[offset + 3];
                }
            }
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
            
        case SMC_REG_RTC_SECONDS:
            value = s->rtc.seconds;
            break;
            
        case SMC_REG_RTC_MINUTES:
            value = s->rtc.minutes;
            break;
            
        case SMC_REG_RTC_HOURS:
            value = s->rtc.hours;
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
            
        case SMC_REG_DVD_KEY:
            if (size == 4) {
                uint32_t offset = addr - SMC_REG_DVD_KEY;
                if (offset + 4 <= sizeof(s->dvd_key)) {
                    value = (s->dvd_key[offset] << 24) |
                            (s->dvd_key[offset + 1] << 16) |
                            (s->dvd_key[offset + 2] << 8) |
                            s->dvd_key[offset + 3];
                }
            }
            break;
            
        case SMC_REG_CONFIG_FLAGS:
            value = s->config_flags;
            break;
            
        case SMC_REG_GPIO_STATE:
            value = s->gpio_state;
            break;
            
        case SMC_REG_POWER_LIMIT:
            value = s->power_limit;
            break;
            
        case SMC_REG_VOLTAGES:
            if (addr < SMC_REG_VOLTAGES + 16) {
                uint32_t index = (addr - SMC_REG_VOLTAGES) / 4;
                switch (index) {
                    case 0: value = s->voltage_12v; break;
                    case 1: value = s->voltage_5v; break;
                    case 2: value = s->voltage_3v3; break;
                    case 3: value = s->voltage_1v8; break;
                }
            }
            break;
            
        case SMC_REG_CURRENT_DRAW:
            value = s->current_12v + s->current_5v + s->current_3v3;
            break;
            
        case SMC_REG_ENERGY_USAGE:
            // Placeholder - Emulation doesn't use real power
            value = s->power_limit * 1000; // mWh
            break;
            
        case SMC_REG_TEMP_LIMITS:
            value = (s->temp_limit_cpu << 0) |
                    (s->temp_limit_gpu << 16);
            break;
            
        case SMC_REG_FAN_CURVE:
            if (addr < SMC_REG_FAN_CURVE + 32) {
                uint32_t index = (addr - SMC_REG_FAN_CURVE) / 4;
                if (index < 8) {
                    value = (s->fan_curve[index].temp << 0) |
                            (s->fan_curve[index].speed << 8);
                }
            }
            break;
            
        case SMC_REG_SCRATCH:
            if (addr < SMC_REG_SCRATCH + 32) {
                uint32_t offset = addr - SMC_REG_SCRATCH;
                value = (s->scratch[offset] << 24) |
                        (s->scratch[offset + 1] << 16) |
                        (s->scratch[offset + 2] << 8) |
                        s->scratch[offset + 3];
            }
            break;
            
        case SMC_REG_COMMAND:
            value = s->last_command;
            break;
            
        case SMC_REG_RESPONSE:
            value = s->command_result;
            break;
            
        case SMC_REG_RESET:
            value = 0; // Always returns 0
            break;
            
        default:
            // Default Register
            value = s->registers[reg];
            break;
    }
    
    trace_xbox360_smc_read(addr, size, value);
    return value;
}

static void xbox360_smc_write(void *opaque, hwaddr addr, 
                             uint64_t value, unsigned size) {
    Xbox360SMCState *s = opaque;
    uint32_t reg = addr / 4;
    
    if (reg >= SMC_REGISTER_SIZE / 4) {
        trace_xbox360_smc_write_invalid(addr, value, size);
        return;
    }
    
    trace_xbox360_smc_write(addr, value, size);
    
    switch (addr) {
        case SMC_REG_FAN_SPEED:
            if (size == 4) {
                s->fan_speed = value & 0xFF;
                if (s->fan_speed > 100) s->fan_speed = 100;
                trace_xbox360_smc_fan_speed(s->fan_speed);
            }
            break;
            
        case SMC_REG_POWER_STATE:
            if (size == 4) {
                POWER_STATE new_state = value & 0xFF;
                xbox360_smc_set_power_state(s, new_state);
            }
            break;
            
        case SMC_REG_POST_CODE:
            if (size == 4) {
                s->post_code = value;
                trace_xbox360_smc_post_code(value);
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
            
        case SMC_REG_AV_PACK:
            if (size == 4) {
                s->av_pack = value & 0xFF;
            }
            break;
            
        case SMC_REG_CONFIG_FLAGS:
            if (size == 4) {
                s->config_flags = value;
            }
            break;
            
        case SMC_REG_GPIO_STATE:
            if (size == 4) {
                s->gpio_state = value;
            }
            break;
            
        case SMC_REG_POWER_LIMIT:
            if (size == 4) {
                s->power_limit = value;
                if (s->power_limit > 200) s->power_limit = 200;
                trace_xbox360_smc_power_limit(value);
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
            
        case SMC_REG_FAN_CURVE:
            if (size == 4 && addr < SMC_REG_FAN_CURVE + 32) {
                uint32_t index = (addr - SMC_REG_FAN_CURVE) / 4;
                if (index < 8) {
                    s->fan_curve[index].temp = (value >> 0) & 0xFF;
                    s->fan_curve[index].speed = (value >> 8) & 0xFF;
                    if (s->fan_curve[index].speed > 100) {
                        s->fan_curve[index].speed = 100;
                    }
                }
            }
            break;
            
        case SMC_REG_SCRATCH:
            if (addr < SMC_REG_SCRATCH + 32) {
                uint32_t offset = addr - SMC_REG_SCRATCH;
                if (offset + 4 <= sizeof(s->scratch)) {
                    s->scratch[offset] = (value >> 24) & 0xFF;
                    s->scratch[offset + 1] = (value >> 16) & 0xFF;
                    s->scratch[offset + 2] = (value >> 8) & 0xFF;
                    s->scratch[offset + 3] = value & 0xFF;
                }
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
                
                trace_xbox360_smc_command(cmd, s->command_result);
            }
            break;
            
        case SMC_REG_RESET:
            if (size == 4 && value == 0xDEADBEEF) {
                trace_xbox360_smc_reset();
                if (s->reset_callback) {
                    s->reset_callback(s->callback_opaque);
                }
            }
            break;
            
        case SMC_REG_RTC_SECONDS:
            if (size == 4) {
                s->rtc.seconds = value & 0x3F; // 0-59
            }
            break;
            
        case SMC_REG_RTC_MINUTES:
            if (size == 4) {
                s->rtc.minutes = value & 0x3F; // 0-59
            }
            break;
            
        case SMC_REG_RTC_HOURS:
            if (size == 4) {
                s->rtc.hours = value & 0x1F; // 0-23
            }
            break;
            
        default:
            // Normal Register
            s->registers[reg] = value;
            break;
    }
}

static const MemoryRegionOps xbox360_smc_ops = {
    .read = xbox360_smc_read,
    .write = xbox360_smc_write,
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

/* ==================== INITIALIZATION ==================== */
static void xbox360_smc_init_state(Xbox360SMCState *s) {
    memset(s->registers, 0, sizeof(s->registers));
    
    s->console_type = SMC_CONSOLE_TRINITY; // Emulating: Trinity
    s->power_state = POWER_STATE_ON;
    s->boot_reason = BOOT_REASON_POWER_ON;
    s->error_code = SMC_ERROR_NONE;
    s->av_pack = AV_PACK_HDMI; // Assume HDMI
    s->region = REGION_NTSC_U;
    
    s->cpu_temp = DEFAULT_CPU_TEMP;
    s->gpu_temp = DEFAULT_GPU_TEMP;
    s->edram_temp = DEFAULT_EDRAM_TEMP;
    s->vrm_temp = DEFAULT_VRM_TEMP;
    s->fan_speed = 30; // 30%
    
    s->voltage_12v = DEFAULT_VOLTAGE_12V;
    s->voltage_5v = DEFAULT_VOLTAGE_5V;
    s->voltage_3v3 = DEFAULT_VOLTAGE_3V3;
    s->voltage_1v8 = DEFAULT_VOLTAGE_1V8;
    s->voltage_core = DEFAULT_VOLTAGE_CORE;
    s->voltage_memory = DEFAULT_VOLTAGE_MEMORY;
    
    s->current_12v = DEFAULT_CURRENT_12V;
    s->current_5v = DEFAULT_CURRENT_5V;
    s->current_3v3 = DEFAULT_CURRENT_3V3;
    
    time_t now = time(NULL);
    struct tm *tm = localtime(&now);
    s->rtc.seconds = tm->tm_sec;
    s->rtc.minutes = tm->tm_min;
    s->rtc.hours = tm->tm_hour;
    s->rtc.day = tm->tm_mday;
    s->rtc.month = tm->tm_mon + 1;
    s->rtc.year = tm->tm_year + 1900;
    
    s->post_code = 0xAA; // POST OK
    
    s->config_flags = 0;
    s->power_limit = DEFAULT_POWER_LIMIT;
    s->temp_limit_cpu = DEFAULT_TEMP_LIMIT_CPU;
    s->temp_limit_gpu = DEFAULT_TEMP_LIMIT_GPU;
    
    for (int i = 0; i < 8; i++) {
        s->fan_curve[i].temp = default_fan_curve[i][0];
        s->fan_curve[i].speed = default_fan_curve[i][1];
    }
    
    s->gpio_state = 0;
    memset(s->scratch, 0, sizeof(s->scratch));
    
    s->last_command = SMC_CMD_NOP;
    s->command_result = 0;
    
    // Callbacks
    s->power_callback = NULL;
    s->reset_callback = NULL;
    s->callback_opaque = NULL;
}

static void xbox360_smc_realize(DeviceState *dev, Error **errp) {
    Xbox360SMCState *s = XBOX360_SMC(dev);
    xbox360_smc_init_state(s);
    
    if (s->nand_state) {
        xbox360_smc_load_from_nand(s);
    }
    
    memory_region_init_io(&s->iomem, OBJECT(s), &xbox360_smc_ops, s,
                          "xbox360.smc", SMC_REGISTER_SIZE);
    
    printf("[SMC] System Management Controller initialized\n");
    printf("      Console: Trinity, Region: %d\n", s->region);
}

/* ==================== SMC COMMANDS ==================== */
uint32_t xbox360_smc_execute_command(Xbox360SMCState *s, SMC_COMMAND cmd,
                                     const uint8_t *params, uint32_t param_len,
                                     uint8_t *response, uint32_t *resp_len) {
    uint32_t result = 0;
    
    if (resp_len) *resp_len = 0;
    
    switch (cmd) {
        case SMC_CMD_NOP:
            result = 0;
            break;
            
        case SMC_CMD_GET_VERSION:
            if (response && resp_len && *resp_len >= 4) {
                *(uint32_t*)response = SMC_FIRMWARE_VERSION;
                *resp_len = 4;
            }
            result = 0;
            break;
            
        case SMC_CMD_GET_TEMPERATURE:
            if (response && resp_len && *resp_len >= 16) {
                *(uint32_t*)(response + 0) = s->cpu_temp;
                *(uint32_t*)(response + 4) = s->gpu_temp;
                *(uint32_t*)(response + 8) = s->edram_temp;
                *(uint32_t*)(response + 12) = s->vrm_temp;
                *resp_len = 16;
            }
            result = 0;
            break;
            
        case SMC_CMD_GET_VOLTAGES:
            if (response && resp_len && *resp_len >= 24) {
                *(uint32_t*)(response + 0) = s->voltage_12v;
                *(uint32_t*)(response + 4) = s->voltage_5v;
                *(uint32_t*)(response + 8) = s->voltage_3v3;
                *(uint32_t*)(response + 12) = s->voltage_1v8;
                *(uint32_t*)(response + 16) = s->voltage_core;
                *(uint32_t*)(response + 20) = s->voltage_memory;
                *resp_len = 24;
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
                result = 0x80000001; // Insufficient Parameters
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
            
        case SMC_CMD_GET_RTC:
            if (response && resp_len && *resp_len >= 6) {
                response[0] = s->rtc.seconds;
                response[1] = s->rtc.minutes;
                response[2] = s->rtc.hours;
                response[3] = s->rtc.day;
                response[4] = s->rtc.month;
                *(uint16_t*)(response + 4) = s->rtc.year;
                *resp_len = 6;
            }
            result = 0;
            break;
            
        case SMC_CMD_SET_RTC:
            if (param_len >= 6) {
                s->rtc.seconds = params[0];
                s->rtc.minutes = params[1];
                s->rtc.hours = params[2];
                s->rtc.day = params[3];
                s->rtc.month = params[4];
                s->rtc.year = *(uint16_t*)(params + 4);
                result = 0;
            } else {
                result = 0x80000001;
            }
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
            
        case SMC_CMD_GET_DVD_KEY:
            if (response && resp_len && *resp_len >= 16) {
                memcpy(response, s->dvd_key, 16);
                *resp_len = 16;
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
            
        case SMC_CMD_GET_MAC_ADDR:
            if (response && resp_len && *resp_len >= 6) {
                memcpy(response, s->mac_address, 6);
                *resp_len = 6;
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
            
        case SMC_CMD_GET_CONFIG:
            if (response && resp_len && *resp_len >= 4) {
                *(uint32_t*)response = s->config_flags;
                *resp_len = 4;
            }
            result = 0;
            break;
            
        case SMC_CMD_SET_CONFIG:
            if (param_len >= 4) {
                s->config_flags = *(uint32_t*)params;
                result = 0;
            } else {
                result = 0x80000001;
            }
            break;
            
        case SMC_CMD_GET_POST_CODE:
            if (response && resp_len && *resp_len >= 4) {
                *(uint32_t*)response = s->post_code;
                *resp_len = 4;
            }
            result = 0;
            break;
            
        case SMC_CMD_GET_ERROR_CODE:
            if (response && resp_len && *resp_len >= 4) {
                *(uint32_t*)response = s->error_code;
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
            
        case SMC_CMD_SCRATCH_READ:
            if (response && resp_len) {
                uint32_t len = sizeof(s->scratch);
                if (*resp_len < len) len = *resp_len;
                memcpy(response, s->scratch, len);
                *resp_len = len;
            }
            result = 0;
            break;
            
        case SMC_CMD_SCRATCH_WRITE:
            if (param_len > 0) {
                uint32_t len = param_len;
                if (len > sizeof(s->scratch)) len = sizeof(s->scratch);
                memcpy(s->scratch, params, len);
                result = 0;
            } else {
                result = 0x80000001;
            }
            break;
            
        default:
            result = 0x80000000; // Command not supported
            trace_xbox360_smc_unknown_command(cmd);
            break;
    }
    
    return result;
}

/* ==================== PUBLIC FUNCTIONS =================== */
void xbox360_smc_set_power_state(Xbox360SMCState *s, POWER_STATE state) {
    if (s->power_state == state) {
        return;
    }
    
    POWER_STATE old_state = s->power_state;
    s->power_state = state;
    
    trace_xbox360_smc_power_state(old_state, state);
    
    if (s->power_callback) {
        s->power_callback(s->callback_opaque, state);
    }
    
    switch (state) {
        case POWER_STATE_OFF:
            printf("[SMC] System powered off\n");
            break;
            
        case POWER_STATE_STANDBY:
            printf("[SMC] System entering standby\n");
            s->fan_speed = 10;
            break;
            
        case POWER_STATE_ON:
            printf("[SMC] System powered on\n");
            s->fan_speed = 30;
            break;
            
        case POWER_STATE_REBOOT:
            printf("[SMC] System rebooting\n");
            s->power_state = POWER_STATE_ON;
            if (s->reset_callback) {
                s->reset_callback(s->callback_opaque);
            }
            break;
            
        default:
            break;
    }
}

void xbox360_smc_set_boot_reason(Xbox360SMCState *s, BOOT_REASON reason) {
    s->boot_reason = reason;
    trace_xbox360_smc_boot_reason(reason);
}

void xbox360_smc_set_error(Xbox360SMCState *s, SMC_ERROR_CODE error) {
    s->error_code |= error;
    trace_xbox360_smc_error_set(error, s->error_code);
    
    if (error & (SMC_ERROR_OVERHEAT_CPU | SMC_ERROR_OVERHEAT_GPU)) {
        printf("[SMC] Overheat detected, shutting down (shouldn't happen... WHAT YOU DID?)\n");
        xbox360_smc_set_power_state(s, POWER_STATE_OFF);
    }
}

void xbox360_smc_clear_errors(Xbox360SMCState *s) {
    s->error_code = SMC_ERROR_NONE;
    trace_xbox360_smc_error_clear();
}

void xbox360_smc_set_temperature(Xbox360SMCState *s, int32_t cpu_temp, int32_t gpu_temp) {
    s->cpu_temp = cpu_temp;
    s->gpu_temp = gpu_temp;
    
    s->edram_temp = cpu_temp + 30; // EDRAM commonly heats more
    s->vrm_temp = (cpu_temp + gpu_temp) / 2;
    
    if (cpu_temp > s->temp_limit_cpu) {
        xbox360_smc_set_error(s, SMC_ERROR_OVERHEAT_CPU);
    }
    
    if (gpu_temp > s->temp_limit_gpu) {
        xbox360_smc_set_error(s, SMC_ERROR_OVERHEAT_GPU);
    }
    
    int max_temp = cpu_temp > gpu_temp ? cpu_temp : gpu_temp;
    max_temp /= 10;
    
    for (int i = 7; i >= 0; i--) {
        if (max_temp >= s->fan_curve[i].temp) {
            s->fan_speed = s->fan_curve[i].speed;
            break;
        }
    }
    
    trace_xbox360_smc_temperature(cpu_temp, gpu_temp, s->fan_speed);
}

void xbox360_smc_set_fan_speed(Xbox360SMCState *s, uint8_t speed) {
    if (speed > 100) speed = 100;
    s->fan_speed = speed;
    trace_xbox360_smc_fan_speed(speed);
}

void xbox360_smc_set_voltages(Xbox360SMCState *s, int32_t v12, int32_t v5, int32_t v3v3) {
    s->voltage_12v = v12;
    s->voltage_5v = v5;
    s->voltage_3v3 = v3v3;
    
    if (v12 < 11000) xbox360_smc_set_error(s, SMC_ERROR_UNDERVOLT_12V);
    if (v12 > 13000) xbox360_smc_set_error(s, SMC_ERROR_OVERVOLT_12V);
    if (v5 < 4750) xbox360_smc_set_error(s, SMC_ERROR_UNDERVOLT_5V);
    if (v5 > 5250) xbox360_smc_set_error(s, SMC_ERROR_OVERVOLT_5V);
    if (v3v3 < 3100) xbox360_smc_set_error(s, SMC_ERROR_UNDERVOLT_3V3);
    if (v3v3 > 3500) xbox360_smc_set_error(s, SMC_ERROR_OVERVOLT_3V3);
    
    trace_xbox360_smc_voltages(v12, v5, v3v3);
}

void xbox360_smc_set_rtc(Xbox360SMCState *s, uint8_t hours, uint8_t minutes, uint8_t seconds) {
    s->rtc.hours = hours & 0x1F;
    s->rtc.minutes = minutes & 0x3F;
    s->rtc.seconds = seconds & 0x3F;
    trace_xbox360_smc_rtc_set(hours, minutes, seconds);
}

void xbox360_smc_set_date(Xbox360SMCState *s, uint8_t day, uint8_t month, uint16_t year) {
    s->rtc.day = day;
    s->rtc.month = month;
    s->rtc.year = year;
    trace_xbox360_smc_date_set(day, month, year);
}

void xbox360_smc_dump_registers(Xbox360SMCState *s) {
    printf("\n=== SMC REGISTER DUMP ===\n");
    printf("Version:     0x%08X\n", SMC_FIRMWARE_VERSION);
    printf("Status:      0x%08X\n", (s->power_state << 0) | ((s->error_code != 0) << 8) | (s->boot_reason << 16));
    printf("CPU Temp:    %d.%d°C\n", s->cpu_temp / 10, s->cpu_temp % 10);
    printf("GPU Temp:    %d.%d°C\n", s->gpu_temp / 10, s->gpu_temp % 10);
    printf("Fan Speed:   %d%%\n", s->fan_speed);
    printf("Power State: %d\n", s->power_state);
    printf("Boot Reason: %d\n", s->boot_reason);
    printf("Error Code:  0x%08X\n", s->error_code);
    printf("POST Code:   0x%08X\n", s->post_code);
    printf("Region:      %d\n", s->region);
    printf("AV Pack:     %d\n", s->av_pack);
    printf("Config:      0x%08X\n", s->config_flags);
    printf("Power Limit: %dW\n", s->power_limit);
    printf("Last Cmd:    0x%02X\n", s->last_command);
    printf("Cmd Result:  0x%08X\n", s->command_result);
    printf("=======================\n");
}

void xbox360_smc_print_state(Xbox360SMCState *s) {
    printf("[SMC] State: ");
    switch (s->power_state) {
        case POWER_STATE_OFF: printf("OFF"); break;
        case POWER_STATE_STANDBY: printf("STANDBY"); break;
        case POWER_STATE_ON: printf("ON"); break;
        default: printf("UNKNOWN"); break;
    }
    
    printf(", CPU: %d.%d°C, GPU: %d.%d°C, Fan: %d%%, Errors: 0x%08X\n",
           s->cpu_temp / 10, s->cpu_temp % 10,
           s->gpu_temp / 10, s->gpu_temp % 10,
           s->fan_speed, s->error_code);
}

void xbox360_smc_set_power_callback(Xbox360SMCState *s, void (*callback)(void *, POWER_STATE), void *opaque) {
    s->power_callback = callback;
    s->callback_opaque = opaque;
}

void xbox360_smc_set_reset_callback(Xbox360SMCState *s, void (*callback)(void *), void *opaque) {
    s->reset_callback = callback;
    s->callback_opaque = opaque;
}

// big file yeah
// oops, i forgot to implement QEMU DEVICE support

/* ==================== QEMU DEVICE ==================== */
static Property xbox360_smc_properties[] = {
    DEFINE_PROP_PTR("nand-state", Xbox360SMCState, nand_state),
    DEFINE_PROP_END_OF_LIST(),
};

static void xbox360_smc_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    
    dc->realize = xbox360_smc_realize;
    dc->desc = "Xbox 360 System Management Controller";
    device_class_set_props(dc, xbox360_smc_properties);
}

static const TypeInfo xbox360_smc_type_info = {
    .name = TYPE_XBOX360_SMC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Xbox360SMCState),
    .class_init = xbox360_smc_class_init,
};

static void xbox360_smc_register_types(void) {
    type_register_static(&xbox360_smc_type_info);
}

type_init(xbox360_smc_register_types);

/* ==================== CREATE FUNCTION ==================== */
Xbox360SMCState *xbox360_smc_create(MemoryRegion *parent, hwaddr base, XBOX360_NAND_STATE *nand_state) {
    DeviceState *dev;
    Xbox360SMCState *s;
    
    dev = qdev_new(TYPE_XBOX360_SMC);
    s = XBOX360_SMC(dev);
    
    s->nand_state = nand_state;
    
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    
    printf("[SMC] Created at 0x%08" HWADDR_PRIx "\n", base);
    return s;
}
