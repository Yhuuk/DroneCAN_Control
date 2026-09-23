#include "app_settings.h"

#include "fram.h"
#include "throttle_control.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

/*
 * FRAM地址布局：
 * 0x0000~0x003F：配置记录A
 * 0x0040~0x007F：配置记录B
 * 0x0080~0x07FF：保留给后续用户数据。
 */
#define APP_SETTINGS_RECORD_MAGIC              0x31474643UL /* "CFG1" */
#define APP_SETTINGS_FORMAT_VERSION            1U
#define APP_SETTINGS_PAYLOAD_LENGTH            4U
#define APP_SETTINGS_COMMIT_MARKER              0xA55AC33CUL

/**
 *以下是每个字段的偏移
 记录空间的格式: 固定标识，格式版本，载荷长度，保存序号，用户数据空间(LIM,STEP)，CRC32校验，有效标记提交
 */
#define APP_SETTINGS_MAGIC_OFFSET               0U
#define APP_SETTINGS_VERSION_OFFSET             4U
#define APP_SETTINGS_PAYLOAD_LENGTH_OFFSET      6U
#define APP_SETTINGS_SEQUENCE_OFFSET            8U
#define APP_SETTINGS_PAYLOAD_OFFSET             12U
#define APP_SETTINGS_LIMIT_OFFSET               12U
#define APP_SETTINGS_STEP_OFFSET                14U
#define APP_SETTINGS_CRC_OFFSET                 56U
#define APP_SETTINGS_COMMIT_OFFSET              60U

#define APP_SETTINGS_DEFAULT_STEP               100U
#define APP_SETTINGS_RETRY_DELAY_TICKS           1000U

#define APP_SETTINGS_SLOT_NONE                  0U
#define APP_SETTINGS_SLOT_A                     1U
#define APP_SETTINGS_SLOT_B                     2U

_Static_assert((APP_SETTINGS_COMMIT_OFFSET + 4U) ==
                   APP_SETTINGS_RECORD_SIZE_BYTES,
               "commit marker must end the 64-byte record");
_Static_assert((APP_SETTINGS_PAYLOAD_OFFSET + APP_SETTINGS_PAYLOAD_LENGTH) <=
                   APP_SETTINGS_CRC_OFFSET,
               "settings payload overlaps CRC field");
_Static_assert((APP_SETTINGS_RECORD_B_ADDRESS +
                APP_SETTINGS_RECORD_SIZE_BYTES) <= FRAM_CAPACITY_BYTES,
               "settings records exceed FRAM capacity");

typedef struct
{
    bool valid;
    uint32_t sequence;
    AppSettings_t settings;
} AppSettingsDecodedRecord_t;

/**
 * g_current_settings 表示 最新希望保存的内存值，
 * g_saved_settings 表示 上次已确认写入的值
 * g_save_pending 表示RAM中的配置是否等待保存，修改参数时置true
 * g_save_deadline_tick 下一次允许尝试自动保存的FreeRTOS时刻，每次修改设为"现在+5秒"，保存失败为"现在+1秒"重试
 * 
 */
static AppSettings_t g_current_settings;
static AppSettings_t g_saved_settings;
static bool g_active_record_valid;
static uint8_t g_active_slot;
static uint32_t g_active_sequence;
static bool g_save_pending;
static uint32_t g_save_deadline_tick;

volatile uint8_t g_app_settings_loaded_slot;
volatile uint32_t g_app_settings_sequence;
volatile uint32_t g_app_settings_load_error_count;
volatile uint32_t g_app_settings_save_count;
volatile uint32_t g_app_settings_save_error_count;
volatile uint32_t g_app_settings_crc_error_count;

static void AppSettings_SetDefaults(AppSettings_t *settings)
{
    settings->throttle_limit = THROTTLE_CONTROL_DEFAULT_LIMIT;
    settings->throttle_step = APP_SETTINGS_DEFAULT_STEP;
}

static bool AppSettings_IsStepValid(uint16_t step)
{
    return (step == 1U) || (step == 10U) ||
           (step == 100U) || (step == 1000U);
}

static bool AppSettings_AreValuesValid(const AppSettings_t *settings)
{
    return (settings != NULL) &&
           (settings->throttle_limit <= THROTTLE_CONTROL_RAW_COMMAND_MAX) &&
           AppSettings_IsStepValid(settings->throttle_step);
}

static bool AppSettings_AreEqual(const AppSettings_t *left,
                                 const AppSettings_t *right)
{
    return (left->throttle_limit == right->throttle_limit) &&
           (left->throttle_step == right->throttle_step);
}

static void AppSettings_PutU16(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8U);
}

static void AppSettings_PutU32(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8U);
    destination[2] = (uint8_t)(value >> 16U);
    destination[3] = (uint8_t)(value >> 24U);
}

static uint16_t AppSettings_GetU16(const uint8_t *source)
{
    return (uint16_t)((uint16_t)source[0] |
                      ((uint16_t)source[1] << 8U));
}

static uint32_t AppSettings_GetU32(const uint8_t *source)
{
    return (uint32_t)source[0] |
           ((uint32_t)source[1] << 8U) |
           ((uint32_t)source[2] << 16U) |
           ((uint32_t)source[3] << 24U);
}

/**
 * @brief 计算CRC-32/ISO-HDLC（也称CRC-32/IEEE）。
 *
 * 固定参数：width=32、poly=0x04C11DB7、init=0xFFFFFFFF、RefIn=true、
 * RefOut=true、xorout=0xFFFFFFFF。反射实现使用等价多项式0xEDB88320；
 * 标准测试串"123456789"的结果应为0xCBF43926。
 */
static uint32_t AppSettings_Crc32(const uint8_t *data, uint16_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;

    for (uint16_t index = 0U; index < length; ++index)
    {
        crc ^= data[index];
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 1UL) != 0UL)
            {
                /**
                 * 该位为1，就进行(crc >> 1U) ^ 0xEDB88320UL计算
                 */
                crc = (crc >> 1U) ^ 0xEDB88320UL;
            }
            else
            {
                /**
                 * 该位为0，就向右移一位
                 */
                crc >>= 1U;
            }
        }
    }

    return crc ^ 0xFFFFFFFFUL;
}

static uint16_t AppSettings_GetSlotAddress(uint8_t slot)
{
    return (slot == APP_SETTINGS_SLOT_B)
               ? APP_SETTINGS_RECORD_B_ADDRESS
               : APP_SETTINGS_RECORD_A_ADDRESS;
}

static bool AppSettings_SequenceIsNewer(uint32_t candidate,
                                        uint32_t reference)
{
    /* 有符号差值能够在32位序号自然回绕后继续判断新旧。 */
    return (int32_t)(candidate - reference) > 0;
}

static void AppSettings_EncodeRecord(uint8_t *record,
                                     const AppSettings_t *settings,
                                     uint32_t sequence)
{
    const uint16_t crc_length =
        APP_SETTINGS_PAYLOAD_OFFSET + APP_SETTINGS_PAYLOAD_LENGTH;
    uint32_t crc;

    memset(record, 0, APP_SETTINGS_RECORD_SIZE_BYTES);
    AppSettings_PutU32(&record[APP_SETTINGS_MAGIC_OFFSET],
                       APP_SETTINGS_RECORD_MAGIC);
    AppSettings_PutU16(&record[APP_SETTINGS_VERSION_OFFSET],
                       APP_SETTINGS_FORMAT_VERSION);
    AppSettings_PutU16(&record[APP_SETTINGS_PAYLOAD_LENGTH_OFFSET],
                       APP_SETTINGS_PAYLOAD_LENGTH);
    AppSettings_PutU32(&record[APP_SETTINGS_SEQUENCE_OFFSET], sequence);
    AppSettings_PutU16(&record[APP_SETTINGS_LIMIT_OFFSET],
                       settings->throttle_limit);
    AppSettings_PutU16(&record[APP_SETTINGS_STEP_OFFSET],
                       settings->throttle_step);

    crc = AppSettings_Crc32(record, crc_length);
    AppSettings_PutU32(&record[APP_SETTINGS_CRC_OFFSET], crc);

    /* 提交标记必须由保存流程最后单独写入，此处保持无效值0。 */
    AppSettings_PutU32(&record[APP_SETTINGS_COMMIT_OFFSET], 0UL);
}

static bool AppSettings_DecodeRecord(
    const uint8_t *record,
    AppSettingsDecodedRecord_t *decoded)
{
    const uint16_t payload_length =
        AppSettings_GetU16(&record[APP_SETTINGS_PAYLOAD_LENGTH_OFFSET]);
    uint16_t crc_length;
    uint32_t stored_crc;
    uint32_t calculated_crc;

    decoded->valid = false;

    if ((AppSettings_GetU32(&record[APP_SETTINGS_COMMIT_OFFSET]) !=
         APP_SETTINGS_COMMIT_MARKER) ||
        (AppSettings_GetU32(&record[APP_SETTINGS_MAGIC_OFFSET]) !=
         APP_SETTINGS_RECORD_MAGIC) ||
        (AppSettings_GetU16(&record[APP_SETTINGS_VERSION_OFFSET]) !=
         APP_SETTINGS_FORMAT_VERSION) ||
        (payload_length != APP_SETTINGS_PAYLOAD_LENGTH))
    {
        return false;
    }

    crc_length = (uint16_t)(APP_SETTINGS_PAYLOAD_OFFSET + payload_length);
    stored_crc = AppSettings_GetU32(&record[APP_SETTINGS_CRC_OFFSET]);
    calculated_crc = AppSettings_Crc32(record, crc_length);
    if (stored_crc != calculated_crc)
    {
        ++g_app_settings_crc_error_count;
        return false;
    }

    decoded->sequence =
        AppSettings_GetU32(&record[APP_SETTINGS_SEQUENCE_OFFSET]);
    decoded->settings.throttle_limit =
        AppSettings_GetU16(&record[APP_SETTINGS_LIMIT_OFFSET]);
    decoded->settings.throttle_step =
        AppSettings_GetU16(&record[APP_SETTINGS_STEP_OFFSET]);

    if (!AppSettings_AreValuesValid(&decoded->settings))
    {
        return false;
    }

    decoded->valid = true;
    return true;
}

static HAL_StatusTypeDef AppSettings_ReadSlot(
    uint8_t slot,
    AppSettingsDecodedRecord_t *decoded)
{
    uint8_t record[APP_SETTINGS_RECORD_SIZE_BYTES];

    if (FRAM_Read(AppSettings_GetSlotAddress(slot),
                  record,
                  (uint16_t)sizeof(record)) != HAL_OK)
    {
        decoded->valid = false;
        ++g_app_settings_load_error_count;
        return HAL_ERROR;
    }

    (void)AppSettings_DecodeRecord(record, decoded);
    return HAL_OK;
}

static HAL_StatusTypeDef AppSettings_WriteNewRecord(
    const AppSettings_t *settings)
{
    uint8_t record[APP_SETTINGS_RECORD_SIZE_BYTES];
    uint8_t verify_record[APP_SETTINGS_RECORD_SIZE_BYTES];
    uint8_t marker_bytes[4];
    uint8_t target_slot;
    uint16_t target_address;
    uint32_t next_sequence;
    AppSettingsDecodedRecord_t verified;

    if ((g_fram_initialized == 0U) ||
        !AppSettings_AreValuesValid(settings))
    {
        return HAL_ERROR;
    }

    target_slot = !g_active_record_valid
                      ? APP_SETTINGS_SLOT_A
                      : ((g_active_slot == APP_SETTINGS_SLOT_A)
                             ? APP_SETTINGS_SLOT_B
                             : APP_SETTINGS_SLOT_A);
    target_address = AppSettings_GetSlotAddress(target_slot);
    next_sequence = g_active_record_valid ? (g_active_sequence + 1UL) : 1UL;

    AppSettings_EncodeRecord(record, settings, next_sequence);

    /*
     * 先清除目标槽位旧的提交标记，再写正文和CRC，最后才写有效标记。
     * 任意一步掉电都只会使目标槽无效，另一槽的上一版配置仍然保留。
     */
    memset(marker_bytes, 0, sizeof(marker_bytes));
    if (FRAM_Write((uint16_t)(target_address + APP_SETTINGS_COMMIT_OFFSET),
                   marker_bytes,
                   (uint16_t)sizeof(marker_bytes)) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (FRAM_Write(target_address,
                   record,
                   APP_SETTINGS_COMMIT_OFFSET) != HAL_OK)
    {
        return HAL_ERROR;
    }

    AppSettings_PutU32(marker_bytes, APP_SETTINGS_COMMIT_MARKER);
    if (FRAM_Write((uint16_t)(target_address + APP_SETTINGS_COMMIT_OFFSET),
                   marker_bytes,
                   (uint16_t)sizeof(marker_bytes)) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 写后回读并重新执行CRC/语义检查，成功后才切换当前活动槽。 */
    if ((FRAM_Read(target_address,
                   verify_record,
                   (uint16_t)sizeof(verify_record)) != HAL_OK) ||
        !AppSettings_DecodeRecord(verify_record, &verified) ||
        (verified.sequence != next_sequence) ||
        !AppSettings_AreEqual(&verified.settings, settings))
    {
        return HAL_ERROR;
    }

    g_active_record_valid = true;
    g_active_slot = target_slot;
    g_active_sequence = next_sequence;
    g_saved_settings = *settings;
    g_current_settings = *settings;
    g_app_settings_loaded_slot = target_slot;
    g_app_settings_sequence = next_sequence;
    ++g_app_settings_save_count;
    return HAL_OK;
}

static HAL_StatusTypeDef AppSettings_AttemptSave(uint32_t now_tick)
{
    HAL_StatusTypeDef status;

    if (!g_save_pending)
    {
        return HAL_OK;
    }

    if (g_active_record_valid &&
        AppSettings_AreEqual(&g_current_settings, &g_saved_settings))
    {
        g_save_pending = false;
        return HAL_OK;
    }

    status = AppSettings_WriteNewRecord(&g_current_settings);
    if (status == HAL_OK)
    {
        g_save_pending = false;
    }
    else
    {
        ++g_app_settings_save_error_count;
        g_save_deadline_tick = now_tick + APP_SETTINGS_RETRY_DELAY_TICKS;
    }

    return status;
}

HAL_StatusTypeDef AppSettings_Init(void)
{
    static const uint8_t crc_test_data[] = "123456789";
    AppSettingsDecodedRecord_t record_a = {0};
    AppSettingsDecodedRecord_t record_b = {0};

    AppSettings_SetDefaults(&g_current_settings);
    g_saved_settings = g_current_settings;
    g_active_record_valid = false;
    g_active_slot = APP_SETTINGS_SLOT_NONE;
    g_active_sequence = 0UL;
    g_save_pending = false;
    g_save_deadline_tick = 0UL;

    g_app_settings_loaded_slot = APP_SETTINGS_SLOT_NONE;
    g_app_settings_sequence = 0UL;
    g_app_settings_load_error_count = 0UL;
    g_app_settings_save_count = 0UL;
    g_app_settings_save_error_count = 0UL;
    g_app_settings_crc_error_count = 0UL;

    /* 防止CRC参数被将来误改；标准测试结果不匹配时拒绝使用记录。
       sizeof(crc_test_data) - 1U，减去的是C字符串结尾的`\0`
    */
    if (AppSettings_Crc32(crc_test_data,
                          (uint16_t)(sizeof(crc_test_data) - 1U)) !=
        0xCBF43926UL)
    {
        ++g_app_settings_load_error_count;
        return HAL_ERROR;
    }

    if (g_fram_initialized == 0U)
    {
        ++g_app_settings_load_error_count;
        return HAL_ERROR;
    }

    (void)AppSettings_ReadSlot(APP_SETTINGS_SLOT_A, &record_a);
    (void)AppSettings_ReadSlot(APP_SETTINGS_SLOT_B, &record_b);

    if (record_a.valid && record_b.valid)
    {
        if (AppSettings_SequenceIsNewer(record_b.sequence,
                                        record_a.sequence))
        {
            g_active_slot = APP_SETTINGS_SLOT_B;
            g_active_sequence = record_b.sequence;
            g_current_settings = record_b.settings;
        }
        else
        {
            g_active_slot = APP_SETTINGS_SLOT_A;
            g_active_sequence = record_a.sequence;
            g_current_settings = record_a.settings;
        }
        g_active_record_valid = true;
    }
    else if (record_a.valid)
    {
        g_active_record_valid = true;
        g_active_slot = APP_SETTINGS_SLOT_A;
        g_active_sequence = record_a.sequence;
        g_current_settings = record_a.settings;
    }
    else if (record_b.valid)
    {
        g_active_record_valid = true;
        g_active_slot = APP_SETTINGS_SLOT_B;
        g_active_sequence = record_b.sequence;
        g_current_settings = record_b.settings;
    }
    else
    {
        /* 首次使用或两条记录都损坏：保留默认值并建立第一条有效记录。 */
        g_save_pending = true;
        if (AppSettings_AttemptSave(0UL) != HAL_OK)
        {
            return HAL_ERROR;
        }
        return HAL_OK;
    }

    g_saved_settings = g_current_settings;
    g_app_settings_loaded_slot = g_active_slot;
    g_app_settings_sequence = g_active_sequence;
    return HAL_OK;
}

void AppSettings_Get(AppSettings_t *settings)
{
    if (settings != NULL)
    {
        *settings = g_current_settings;
    }
}

void AppSettings_RequestDeferredSave(const AppSettings_t *settings,
                                     uint32_t now_tick,
                                     uint32_t delay_ticks)
{
    if (!AppSettings_AreValuesValid(settings))
    {
        return;
    }

    g_current_settings = *settings;
    g_save_pending = true;
    g_save_deadline_tick = now_tick + delay_ticks;
}

HAL_StatusTypeDef AppSettings_SaveNow(const AppSettings_t *settings,
                                      uint32_t now_tick)
{
    if (!AppSettings_AreValuesValid(settings))
    {
        ++g_app_settings_save_error_count;
        return HAL_ERROR;
    }

    g_current_settings = *settings;
    g_save_pending = true;
    return AppSettings_AttemptSave(now_tick);
}

HAL_StatusTypeDef AppSettings_ProcessDeferredSave(uint32_t now_tick)
{
    if (!g_save_pending ||
        ((int32_t)(now_tick - g_save_deadline_tick) < 0))
    {
        return HAL_OK;
    }

    return AppSettings_AttemptSave(now_tick);
}

uint32_t AppSettings_TicksUntilSave(uint32_t now_tick)
{
    const int32_t remaining =
        (int32_t)(g_save_deadline_tick - now_tick);

    if (!g_save_pending)
    {
        return UINT32_MAX;
    }

    return (remaining <= 0) ? 0U : (uint32_t)remaining;
}

/**
 * 返回true表示正在修改
 */
bool AppSettings_IsSavePending(void)
{
    return g_save_pending;
}
