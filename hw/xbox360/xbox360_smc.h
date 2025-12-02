// based on QEMU and Xenia
// help me its big code time oh nooooooooo
#ifndef HW_XBOX360_SMC_H
#define HW_XBOX360_SMC_H

#include "hw/sysbus.h"
#include "qom/object.h"
#include "hw/xbox360/xbox360_nand.h"

/* ==================== CONSOLE TYPES ==================== */
typedef enum {
  SMC_CONSOLE_XENON = 0x01,    // Xenon (90nm)
  SMC_CONSOLE_ZEPHYR = 0x02,   // Zephyr/Opus
  SMC_CONSOLE_FALCON = 0x03,   // Falcon/Jasper
  SMC_CONSOLE_TRINITY = 0x04,  // Trinity (my console lol)
  SMC_CONSOLE_CORONA = 0x05,   // Corona
} SMC_CONSOLE_TYPE;

/* ==================== SMC REGISTERS ==================== */
#define SMC_BASE_ADDRESS          0x80000200 /* fake address (TODO) */
#define SMC_REGISTER_SIZE         0x100 /* idk if it's fake */

// Offset Registers (from SMC_BASE_ADDRESS)
#define SMC_REG_VERSION           0x00    // SMC Version
#define SMC_REG_STATUS            0x04    // System Status
#define SMC_REG_CPU_TEMP          0x08    // CPU Temperature (Simulated) (°C)
#define SMC_REG_GPU_TEMP          0x0C    // GPU Temperature (Simulated) (°C)
#define SMC_REG_FAN_SPEED         0x10    // FAN Speed (Simulated) (0-100%)
#define SMC_REG_POWER_STATE       0x14    // Energy State
#define SMC_REG_FUSE_DATA         0x18    // Fuse Data (12 bytes)
#define SMC_REG_CPU_KEY           0x24    // CPU Key (16 bytes)
#define SMC_REG_CONSOLE_ID        0x34    // Console ID (5 bytes)
#define SMC_REG_BOOT_REASON       0x3C    // Boot Reason
#define SMC_REG_RTC_SECONDS       0x40    // RTC seconds
#define SMC_REG_RTC_MINUTES       0x44    // RTC minutes
#define SMC_REG_RTC_HOURS         0x48    // RTC hours
#define SMC_REG_RTC_DAYS          0x4C    // RTC days
#define SMC_REG_RTC_MONTH         0x50    // RTC month
#define SMC_REG_RTC_YEAR          0x54    // RTC year
#define SMC_REG_POST_CODE         0x58    // Post Code
#define SMC_REG_ERROR_CODE        0x5C    // Error Code
#define SMC_REG_AV_PACK           0x60    // AV pack type
#define SMC_REG_REGION            0x64    // Console Region
#define SMC_REG_SERIAL            0x68    // Serial Number (12 bytes)
#define SMC_REG_MAC_ADDR          0x74    // MAC Address (6 bytes)
#define SMC_REG_DVD_KEY           0x7C    // DVD Key (16 bytes)
#define SMC_REG_CONFIG_FLAGS      0x8C    // Configuration Flags
#define SMC_REG_GPIO_STATE        0x90    // GPIO State
#define SMC_REG_POWER_LIMIT       0x94    // Power Limit (Simulated)
#define SMC_REG_VOLTAGES          0x98    // Voltage Reads (array) (Simulated)
#define SMC_REG_CURRENT_DRAW      0xA8    // Current Energy Draw (Simulated)
#define SMC_REG_ENERGY_USAGE      0xAC    // Energy Usage (Simulated)
#define SMC_REG_TEMP_LIMITS       0xB0    // Temperature Limits (Simulated)
#define SMC_REG_FAN_CURVE         0xB4    // FAN Curve (8 entries) (Simulated)
#define SMC_REG_SCRATCH           0xD4    // Scratch Register (32 bytes)
#define SMC_REG_COMMAND           0xF4    // Command to SMC
#define SMC_REG_RESPONSE          0xF8    // SMC Response
#define SMC_REG_RESET             0xFC    // Reset SMC

/* ==================== POWER STATES ==================== */
typedef enum {
    POWER_STATE_OFF = 0x00,           // Off
    POWER_STATE_STANDBY = 0x01,       // Standby (orange light)
    POWER_STATE_ON = 0x02,            // On (green light)
    POWER_STATE_SHUTDOWN = 0x03,      // Shutdown in progress
    POWER_STATE_REBOOT = 0x04,        // Reboot in progress
    POWER_STATE_ERROR = 0x05,         // Error State
} POWER_STATE;

/* ==================== BOOT REASONS ==================== */
typedef enum {
    BOOT_REASON_POWER_ON = 0x00,      // Power button
    BOOT_REASON_EJECT = 0x01,         // Eject button
    BOOT_REASON_CONTROLLER = 0x02,    // Controller guide button
    BOOT_REASON_KINECT = 0x03,        // Kinect wake
    BOOT_REASON_IR = 0x04,            // IR remote
    BOOT_REASON_TIMER = 0x05,         // Timer wake
    BOOT_REASON_ERROR = 0x06,         // Error recovery
    BOOT_REASON_UPDATE = 0x07,        // System update
    BOOT_REASON_DIAG = 0x08,          // Diagnostic boot
    BOOT_REASON_RGH = 0x09,           // RGH boot (glitch)
} BOOT_REASON;

/* ==================== ERROR CODES ==================== */
typedef enum {
    SMC_ERROR_NONE = 0x00000000,
    SMC_ERROR_OVERHEAT_CPU = 0x00000001,
    SMC_ERROR_OVERHEAT_GPU = 0x00000002,
    SMC_ERROR_OVERHEAT_EDRAM = 0x00000004,
    SMC_ERROR_OVERHEAT_VRM = 0x00000008,
    SMC_ERROR_UNDERVOLT_12V = 0x00000010,
    SMC_ERROR_OVERVOLT_12V = 0x00000020,
    SMC_ERROR_UNDERVOLT_5V = 0x00000040,
    SMC_ERROR_OVERVOLT_5V = 0x00000080,
    SMC_ERROR_UNDERVOLT_3V3 = 0x00000100,
    SMC_ERROR_OVERVOLT_3V3 = 0x00000200,
    SMC_ERROR_FAN_FAILURE = 0x00000400,
    SMC_ERROR_POWER_SUPPLY = 0x00000800,
    SMC_ERROR_RTC_BATTERY = 0x00001000,
    SMC_ERROR_EEPROM = 0x00002000,
    SMC_ERROR_SMC_FW = 0x00004000,
    SMC_ERROR_AV_PACK = 0x00008000,
    SMC_ERROR_POST = 0x00010000,
    SMC_ERROR_HARDWARE = 0x00020000,
} SMC_ERROR_CODE;

/* ==================== AV PACK TYPES ==================== */
typedef enum {
    AV_PACK_NONE = 0x00,              // Sem AV pack
    AV_PACK_COMPOSITE = 0x01,         // Composite (SD)
    AV_PACK_COMPONENT = 0x02,         // Component (HD)
    AV_PACK_VGA = 0x03,               // VGA
    AV_PACK_HDMI = 0x04,              // HDMI
    AV_PACK_DVI = 0x05,               // DVI
} AV_PACK_TYPE;

/* ==================== REGION CODES ==================== */
typedef enum {
    REGION_NTSC_U = 0x00,             // North America
    REGION_PAL = 0x01,                // Europe/Australia
    REGION_NTSC_J = 0x02,             // Japan
    REGION_CHINA = 0x03,              // China
    REGION_KOREA = 0x04,              // Korea
} REGION_CODE;

/* ==================== SMC STATE STRUCTURE ==================== */
#define TYPE_XBOX360_SMC "xbox360.smc"
OBJECT_DECLARE_SIMPLE_TYPE(Xbox360SMCState, XBOX360_SMC)

struct Xbox360SMCState {
  SysBusDevice parent_obj;
  MemoryRegion iomem;
  uint32_t registers[SMC_REGISTER_SIZE / 4];
  SMC_CONSOLE_TYPE console_type;
  POWER_STATE power_state;
  BOOT_REASON boot_reason; // the only boot reason really used is pressed the power button, as there is no physical eject button, etc...
  SMC_ERROR_CODE error_code;
  AV_PACK_TYPE av_pack;
  REGION_CODE region;
  uint8_t cpu_key[16];
  uint8_t console_id[5];
  uint8_t dvd_key[16];
  uint8_t serial_number[12];
  uint8_t mac_address[6];
  uint8_t fuse_data[12];
  int32_t cpu_temp;  // ºC * 10 (e.g.: 450 = 45.0ºC)
  int32_t gpu_temp;  // ºC * 10
  int32_t edram_temp;// ºC * 10
  int32_t vrm_temp;  // ºC * 10
  uint8_t fan_speed; // 0-100%
  int32_t voltage_12v;
  int32_t voltage_5v;
  int32_t voltage_3v3;
  int32_t voltage_1v8;
  int32_t voltage_core;
  int32_t voltage_memory;
  int32_t current_12v;
  int32_t current_5v;
  int32_t current_3v3;

  struct {
      uint8_t seconds;
      uint8_t minutes;
      uint8_t hours;
      uint8_t day;
      uint8_t month;
      uint16_t year;
  } rtc;

  uint32_t post_code;
  SMC_COMMAND last_command;
  uint32_t command_result;
  uint32_t config_flags;
  uint32_t power_limit;  // watts
  uint32_t temp_limit_cpu; // ºC * 10
  uint32_t temp_limit_gpu; // ºC * 10

  // Fan curve
  struct {
      uint8_t temp;
      uint8_t speed;
  } fan_curve[8];

  uint32_t gpio_state;
  uint8_t scratch[32];

  void (*power_callback)(void *opaque, POWER_STATE state);
  void (*reset_callback)(void *opaque);
  void *callback_opaque;

  // reference to NAND
  XBOX360_NAND_STATE *nand_state;
}

/* ==================== PUBLIC FUNCTIONS ==================== */
Xbox360SMCState *xbox360_smc_create(MemoryRegion *parent, hwaddr base, XBOX360_NAND_STATE *nand_state);
void xbox360_smc_set_power_state(Xbox360SMCState *s, POWER_STATE state);
void xbox360_smc_set_boot_reason(Xbox360SMCState *s, BOOT_REASON reason);
void xbox360_smc_set_error(Xbox360SMCState *s, SMC_ERROR_CODE error);
void xbox360_smc_clear_errors(Xbox360SMCState *s);
void xbox360_smc_set_temperature(Xbox360SMCState *s, int32_t cpu_temp, int32_t gpu_temp);
void xbox360_smc_set_fan_speed(Xbox360SMCState *s, uint8_t speed);
void xbox360_smc_set_voltages(Xbox360SMCState *s, int32_t v12, int32_t v5, int32_t v3v3);
void xbox360_smc_set_rtc(Xbox360SMCState *s, uint8_t hours, uint8_t minutes, uint8_t seconds);
void xbox360_smc_set_date(Xbox360SMCState *s, uint8_t day, uint8_t month, uint16_t year);
void xbox360_smc_dump_registers(Xbox360SMCState *s);
void xbox360_smc_print_state(Xbox360SMCState *s);
uint32_t xbox360_smc_execute_command(Xbox360SMCState *s, SMC_COMMAND cmd, const uint8_t *params, uint32_t param_len, uint8_t *response, uint32_t *resp_len);
void xbox360_smc_set_power_callback(Xbox360SMCState *s, void (*callback)(void *, POWER_STATE), void *opaque);
void xbox360_smc_set_reset_callback(Xbox360SMCState *s, void (*callback)(void *), void *opaque);

#endif
