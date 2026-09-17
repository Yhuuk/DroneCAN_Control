#ifndef THROTTLE_CONTROL_H
#define THROTTLE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/** RawCommand正向油门的协议范围。 */
#define THROTTLE_CONTROL_RAW_COMMAND_MAX 8191U

/**
 * 默认只允许很低的调试上限，并且上电不选择任何电机。
 * 用户确认通道后才可能产生非零电机输出，降低误操作风险。
 */
#define THROTTLE_CONTROL_DEFAULT_LIMIT       100U
#define THROTTLE_CONTROL_DEFAULT_MOTOR_MASK  0x00U

/**
 * @brief 油门控制层对UiTask和CanTask发布的一致状态。
 *
 * raw_command已经完成锁定判断和摇杆正半轴映射：锁定、摇杆为0或负值时
 * 均为0；解锁且摇杆为正值时范围为1~limit。motor_mask的bit0~bit7
 * 分别对应1~8路电机，由CanTask据此生成完整8路RawCommand数组。
 */
typedef struct
{
    bool unlocked;
    uint16_t raw_command;
    uint16_t limit;
    uint8_t motor_mask;
} ThrottleControlSnapshot_t;

/** @brief 初始化为锁定状态；应在创建任务之前调用一次。 */
void ThrottleControl_Init(void);

/**
 * @brief 尝试解锁油门。
 * @return true表示已解锁或原本已经解锁；false表示摇杆快照无效，或者
 *         当前油门归一化值大于0，为防止电机突然启动而拒绝解锁。
 */
bool ThrottleControl_TryUnlock(void);

/** @brief 无条件锁定油门；后续快照中的RawCommand立即归零。 */
void ThrottleControl_Lock(void);

/** @brief 查询当前全局油门解锁状态。 */
bool ThrottleControl_IsUnlocked(void);

/**
 * @brief 原子更新油门通道掩码和RawCommand上限。
 * @param motor_mask bit0~bit7分别表示1~8路电机是否接收油门。
 * @param limit 允许的最大油门，范围0~8191；超出时限制为8191。
 *
 * UiTask是配置的唯一写入者；CanTask和UiTask通过快照读取同一份配置。
 */
void ThrottleControl_SetConfiguration(uint8_t motor_mask, uint16_t limit);

/**
 * @brief 读取最新摇杆并生成当前油门输出快照。
 * @param[out] snapshot 接收解锁状态、掩码、上限和0~limit RawCommand值。
 * @return true表示摇杆快照有效；false时raw_command仍被安全地置为0。
 */
bool ThrottleControl_GetSnapshot(ThrottleControlSnapshot_t *snapshot);

/* 调试统计量，可在Watch窗口观察解锁和锁定是否按预期发生。 */
extern volatile uint32_t g_throttle_unlock_count;
extern volatile uint32_t g_throttle_unlock_rejected_count;
extern volatile uint32_t g_throttle_lock_count;

#ifdef __cplusplus
}
#endif

#endif /* THROTTLE_CONTROL_H */
