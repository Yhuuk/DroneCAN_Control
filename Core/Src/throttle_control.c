#include "throttle_control.h"

#include "joystick.h"

#include <stddef.h>

/** joystick模块归一化后的正半轴最大值。 */
#define THROTTLE_CONTROL_JOYSTICK_POSITIVE_MAX 1000

/*
 * InputTask写、CanTask和UiTask读。使用对齐的32位变量可在STM32L431上单次读写，
 * volatile防止编译器缓存跨任务状态；当前只有一位状态，不需要互斥锁。
 */
static volatile uint32_t g_throttle_unlocked_word;

volatile uint32_t g_throttle_unlock_count;
volatile uint32_t g_throttle_unlock_rejected_count;
volatile uint32_t g_throttle_lock_count;

void ThrottleControl_Init(void)
{
    g_throttle_unlocked_word = 0U;
    g_throttle_unlock_count = 0U;
    g_throttle_unlock_rejected_count = 0U;
    g_throttle_lock_count = 0U;
}

bool ThrottleControl_TryUnlock(void)
{
    JoystickNormalizedValues_t joystick;

    if (g_throttle_unlocked_word != 0U)
    {
        return true;
    }

    /*
     * 解锁只能发生在油门归一化值<=0时。这样即使用户在正油门位置长按
     * 开关，CanTask也仍然只会广播0，不会在解锁瞬间启动全部电机。
     */
    if (!Joystick_GetLatestNormalized(&joystick) ||
        (joystick.throttle_normalized > 0))
    {
        ++g_throttle_unlock_rejected_count;
        return false;
    }

    g_throttle_unlocked_word = 1U;
    ++g_throttle_unlock_count;
    return true;
}

void ThrottleControl_Lock(void)
{
    if (g_throttle_unlocked_word != 0U)
    {
        g_throttle_unlocked_word = 0U;
        ++g_throttle_lock_count;
    }
}

bool ThrottleControl_IsUnlocked(void)
{
    return g_throttle_unlocked_word != 0U;
}

bool ThrottleControl_GetSnapshot(ThrottleControlSnapshot_t *snapshot)
{
    JoystickNormalizedValues_t joystick;
    uint32_t mapped_command;

    if (snapshot == NULL)
    {
        return false;
    }

    snapshot->unlocked = ThrottleControl_IsUnlocked();
    snapshot->raw_command = 0U;

    if (!Joystick_GetLatestNormalized(&joystick))
    {
        return false;
    }

    if (!snapshot->unlocked || (joystick.throttle_normalized <= 0))
    {
        return true;
    }

    /*
     * 把摇杆正半轴1~1000线性映射为RawCommand 1~100。加入半个除数完成
     * 四舍五入；极小正值若舍入为0则提升为1，保证“大于0即为正油门”。
     */
    mapped_command =
        (((uint32_t)joystick.throttle_normalized *
          THROTTLE_CONTROL_RAW_COMMAND_MAX) +
         (THROTTLE_CONTROL_JOYSTICK_POSITIVE_MAX / 2U)) /
        THROTTLE_CONTROL_JOYSTICK_POSITIVE_MAX;

    if (mapped_command == 0U)
    {
        mapped_command = 1U;
    }
    else if (mapped_command > THROTTLE_CONTROL_RAW_COMMAND_MAX)
    {
        mapped_command = THROTTLE_CONTROL_RAW_COMMAND_MAX;
    }

    snapshot->raw_command = (uint16_t)mapped_command;
    return true;
}
