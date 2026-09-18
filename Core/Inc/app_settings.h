#ifndef APP_SETTINGS_H
#define APP_SETTINGS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

/** 每条配置记录固定占用64字节，当前使用A/B两个槽位。 */
#define APP_SETTINGS_RECORD_SIZE_BYTES 64U
#define APP_SETTINGS_RECORD_A_ADDRESS  0x0000U
#define APP_SETTINGS_RECORD_B_ADDRESS  0x0040U

/** 油门参数的掉电保存内容；后续用户配置统一扩展该结构和记录版本。 */
typedef struct
{
    uint16_t throttle_limit;
    uint16_t throttle_step;
} AppSettings_t;

/**
 * @brief 从FRAM的A/B记录加载配置；无有效记录时采用默认值并尝试建立A记录。
 * @note  当前在FreeRTOS启动前调用，不会与运行期UI并发。
 */
HAL_StatusTypeDef AppSettings_Init(void);

/** @brief 取得当前配置快照；即使FRAM故障也会返回经过校验的安全默认值。 */
void AppSettings_Get(AppSettings_t *settings);

/**
 * @brief 请求延迟保存，重复调用会从最后一次变化重新计时。
 * @param now_tick 当前FreeRTOS Tick。
 * @param delay_ticks 停止变化多少Tick后保存。
 */
void AppSettings_RequestDeferredSave(const AppSettings_t *settings,
                                     uint32_t now_tick,
                                     uint32_t delay_ticks);

/**
 * @brief 立即保存配置，供用户确认退出编辑或离开页面时使用。
 * @note 失败时保留待保存状态，后续服务函数会自动重试。
 */
HAL_StatusTypeDef AppSettings_SaveNow(const AppSettings_t *settings,
                                      uint32_t now_tick);

/** @brief 到达延迟截止时间时执行一次保存；未到期时不访问FRAM。 */
HAL_StatusTypeDef AppSettings_ProcessDeferredSave(uint32_t now_tick);

/**
 * @brief 返回距离下一次自动保存还剩多少Tick。
 * @return 无待保存数据时返回UINT32_MAX；已经到期时返回0。
 */
uint32_t AppSettings_TicksUntilSave(uint32_t now_tick);

/** @brief true表示RAM中的最新配置尚未成功写入FRAM。 */
bool AppSettings_IsSavePending(void);

/* 调试器观察量。 */
extern volatile uint8_t g_app_settings_loaded_slot;
extern volatile uint32_t g_app_settings_sequence;
extern volatile uint32_t g_app_settings_load_error_count;
extern volatile uint32_t g_app_settings_save_count;
extern volatile uint32_t g_app_settings_save_error_count;
extern volatile uint32_t g_app_settings_crc_error_count;

#ifdef __cplusplus
}
#endif

#endif /* APP_SETTINGS_H */
