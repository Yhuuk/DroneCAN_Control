#ifndef THROTTLE_CONTROL_H
#define THROTTLE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/** 当前联调阶段RawCommand允许的最大正向油门值。 */
#define THROTTLE_CONTROL_RAW_COMMAND_MAX 100U

/**
 * @brief 油门控制层对UiTask和CanTask发布的一致状态。
 *
 * raw_command已经完成锁定判断和摇杆正半轴映射：锁定、摇杆为0或负值时
 * 均为0；解锁且摇杆为正值时范围为1~100。
 */
typedef struct
{
    bool unlocked;
    uint16_t raw_command;
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
 * @brief 读取最新摇杆并生成当前油门输出快照。
 * @param[out] snapshot 接收解锁状态和0~100 RawCommand值。
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
