#include "throttle_control.h"

#include "joystick.h"

#include <stddef.h>

/** joystick模块归一化后的正半轴最大值。 */
#define THROTTLE_CONTROL_JOYSTICK_POSITIVE_MAX 1000

/* 低13位保存0~8191上限，高8位保存电机掩码。一次32位写入同时发布二者。 */
#define THROTTLE_CONTROL_LIMIT_MASK        0x00001FFFUL
#define THROTTLE_CONTROL_MOTOR_MASK_SHIFT 16U

/*
 * 解锁字由InputTask写，配置字由UiTask写，CanTask/UiTask读取。两个变量均为
 * 对齐32位值，STM32L431可单次完成读写；配置字把LIM和MASK一起发布，避免
 * CanTask读到一半新、一半旧的组合。volatile用于禁止跨任务缓存。
 */
static volatile uint32_t g_throttle_unlocked_word;
static volatile uint32_t g_throttle_configuration_word;

volatile uint32_t g_throttle_unlock_count;
volatile uint32_t g_throttle_unlock_rejected_count;
volatile uint32_t g_throttle_lock_count;

void ThrottleControl_Init(void)
{
    g_throttle_unlocked_word = 0U;
    g_throttle_configuration_word =
        ((uint32_t)THROTTLE_CONTROL_DEFAULT_MOTOR_MASK <<
         THROTTLE_CONTROL_MOTOR_MASK_SHIFT) |
        (uint32_t)THROTTLE_CONTROL_DEFAULT_LIMIT;
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

void ThrottleControl_SetConfiguration(uint8_t motor_mask, uint16_t limit)
{
    if (limit > THROTTLE_CONTROL_RAW_COMMAND_MAX)
    {
        limit = THROTTLE_CONTROL_RAW_COMMAND_MAX;
    }

    g_throttle_configuration_word =
        ((uint32_t)motor_mask << THROTTLE_CONTROL_MOTOR_MASK_SHIFT) |
        ((uint32_t)limit & THROTTLE_CONTROL_LIMIT_MASK);
}

bool ThrottleControl_GetSnapshot(ThrottleControlSnapshot_t *snapshot)
{
    JoystickNormalizedValues_t joystick;
    uint32_t configuration_word;
    uint32_t mapped_command;

    if (snapshot == NULL)
    {
        return false;
    }

    configuration_word = g_throttle_configuration_word;
    snapshot->limit =
        (uint16_t)(configuration_word & THROTTLE_CONTROL_LIMIT_MASK);
    snapshot->motor_mask =
        (uint8_t)(configuration_word >> THROTTLE_CONTROL_MOTOR_MASK_SHIFT);
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
     * 把摇杆正半轴1~1000线性映射为RawCommand 1~limit。加入半个除数完成
     * 四舍五入；limit为0时明确保持0，不执行“最小提升为1”。
     */
    if (snapshot->limit == 0U)
    {
        return true;
    }

    mapped_command =
        (((uint32_t)joystick.throttle_normalized *
          snapshot->limit) +
         (THROTTLE_CONTROL_JOYSTICK_POSITIVE_MAX / 2U)) /
        THROTTLE_CONTROL_JOYSTICK_POSITIVE_MAX;

    if (mapped_command == 0U)
    {
        mapped_command = 1U;
    }
    else if (mapped_command > snapshot->limit)
    {
        mapped_command = snapshot->limit;
    }

    /* 锁定可能与本次计算并发发生；发布快照前再检查一次，避免沿用非零值。 */
    if (!ThrottleControl_IsUnlocked())
    {
        snapshot->unlocked = false;
        snapshot->raw_command = 0U;
    }
    else
    {
        snapshot->raw_command = (uint16_t)mapped_command;
    }
    return true;
}
