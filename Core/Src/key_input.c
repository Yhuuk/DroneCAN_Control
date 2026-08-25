#include "key_input.h"

#include "main.h"

#include <stdbool.h>
#include <stddef.h>

/**
 * InputTask每10 ms扫描一次。连续3次读到相同的新电平后才确认状态改变，
 * 因而按下和释放的消抖时间约为30 ms。
 */
#define KEY_DEBOUNCE_SCAN_COUNT 3U

/** @brief 一个物理按键固定不变的GPIO描述。 */
typedef struct
{
    GPIO_TypeDef *gpio_port;
    uint16_t gpio_pin;
} KeyHardware_t;

/** @brief 一个物理按键在运行期间不断变化的状态机数据。 
 * 
 * debounce_count 是记录按键在消抖状态下连续扫描到相同电平的次数。它用于判断按键是否已经稳定按下或松开。
*/
typedef struct
{
    KeyState_t state;
    uint8_t debounce_count;
} KeyRuntime_t;

/*
 * 所有按键均由CubeMX配置为上拉输入：
 *   GPIO_PIN_SET   表示按键松开；
 *   GPIO_PIN_RESET 表示按键按下。
 *
 * 数组顺序必须与KeyId_t完全一致，便于使用key_id直接索引。
 */
static const KeyHardware_t g_key_hardware[KEY_ID_COUNT] =
{
    [KEY_ID_UP]      = {UP_GPIO_Port,      UP_Pin},
    [KEY_ID_DOWN]    = {DOWN_GPIO_Port,    DOWN_Pin},
    [KEY_ID_CONFIRM] = {Confrim_GPIO_Port, Confrim_Pin},
    [KEY_ID_BACK]    = {Back_GPIO_Port,    Back_Pin},
    [KEY_ID_SWITCH]  = {Switch_GPIO_Port,  Switch_Pin}
};

static KeyRuntime_t g_key_runtime[KEY_ID_COUNT];

/** @brief 将上拉输入的GPIO电平转换为更直观的“是否按下”。 */
static bool KeyInput_ReadPressed(KeyId_t key_id)
{
    return HAL_GPIO_ReadPin(g_key_hardware[key_id].gpio_port,
                            g_key_hardware[key_id].gpio_pin) == GPIO_PIN_RESET;
}

/** @brief 在事件数组仍有空间时追加一条事件。
 * 
 * KeyInput_AppendEvent 是 KeyInput_Scan的辅助函数，用于在扫描过程中记录按键事件。它会检查事件数组是否有足够空间，如果有，则将新的按键事件追加到数组中，并更新事件计数。
 */
static void KeyInput_AppendEvent(KeyEvent_t *events,
                                 uint8_t max_events,
                                 uint8_t *event_count,
                                 KeyId_t key_id,
                                 KeyEventType_t event_type)
{
    if ((events == NULL) || (event_count == NULL) ||
        (*event_count >= max_events))
    {
        return;
    }

    events[*event_count].key_id = key_id;
    events[*event_count].event_type = event_type;
    (*event_count)++;
}

void KeyInput_Init(void)
{
    for (uint8_t index = 0U; index < (uint8_t)KEY_ID_COUNT; ++index)
    {
        const KeyId_t key_id = (KeyId_t)index;

        /*
         * 上电时已经按住的按键直接作为“稳定按下”处理，但不产生事件。
         * 它必须经过一次可靠释放后才能再次触发，避免上电误操作。
         */
        g_key_runtime[index].state = KeyInput_ReadPressed(key_id)
                                         ? KEY_STATE_PRESSED
                                         : KEY_STATE_RELEASED;
        g_key_runtime[index].debounce_count = 0U;
    }
}

uint8_t KeyInput_Scan(KeyEvent_t *events, uint8_t max_events)
{
    uint8_t event_count = 0U;

    for (uint8_t index = 0U; index < (uint8_t)KEY_ID_COUNT; ++index)
    {
        const KeyId_t key_id = (KeyId_t)index;

        // 取出当前按键的运行时状态机数据，是地址
        KeyRuntime_t *const runtime = &g_key_runtime[index];  //取地址
        const bool pressed = KeyInput_ReadPressed(key_id);

        switch (runtime->state)
        {
            case KEY_STATE_RELEASED:
                if (pressed)
                {
                    /*
                     * 只看到一次低电平还不能判定按下，先进入按下消抖。
                     * 计数从0开始，后续连续3次扫描仍为低电平才确认。
                     */
                    runtime->state = KEY_STATE_DEBOUNCE_PRESS;
                    runtime->debounce_count = 0U;
                }
                break;

            case KEY_STATE_DEBOUNCE_PRESS:
                if (!pressed)
                {
                    /*
                     * 消抖期间电平重新变高，说明刚才可能只是机械抖动，
                     * 取消本次按下候选并恢复稳定松开状态。
                     */
                    runtime->state = KEY_STATE_RELEASED;
                    runtime->debounce_count = 0U;
                }
                else
                {
                    runtime->debounce_count++;

                    if (runtime->debounce_count >= KEY_DEBOUNCE_SCAN_COUNT)
                    {
                        runtime->state = KEY_STATE_PRESSED;
                        runtime->debounce_count = 0U;

                        /*
                         * 只在状态从“按下消抖”进入“稳定按下”时产生一次
                         * PRESSED事件。持续按住不会重复产生方向命令。
                         */
                        KeyInput_AppendEvent(events,
                                             max_events,
                                             &event_count,
                                             key_id,
                                             KEY_EVENT_PRESSED);
                    }
                }
                break;

            case KEY_STATE_PRESSED:
                if (!pressed)
                {
                    /* 第一次看到高电平，进入释放消抖而不是立即判定松开。 */
                    runtime->state = KEY_STATE_DEBOUNCE_RELEASE;
                    runtime->debounce_count = 0U;
                }
                break;

            case KEY_STATE_DEBOUNCE_RELEASE:
                if (pressed)
                {
                    /*
                     * 释放消抖期间又读到低电平，说明按键仍然处于按下
                     * 状态，高电平只是释放边沿的抖动。
                     */
                    runtime->state = KEY_STATE_PRESSED;
                    runtime->debounce_count = 0U;
                }
                else
                {
                    runtime->debounce_count++;

                    if (runtime->debounce_count >= KEY_DEBOUNCE_SCAN_COUNT)
                    {
                        runtime->state = KEY_STATE_RELEASED;
                        runtime->debounce_count = 0U;

                        KeyInput_AppendEvent(events,
                                             max_events,
                                             &event_count,
                                             key_id,
                                             KEY_EVENT_RELEASED);
                    }
                }
                break;

            default:
                /* 状态变量异常时恢复到安全的松开状态。 */
                runtime->state = KEY_STATE_RELEASED;
                runtime->debounce_count = 0U;
                break;
        }
    }

    return event_count;
}

KeyState_t KeyInput_GetState(KeyId_t key_id)
{
    if ((uint32_t)key_id >= (uint32_t)KEY_ID_COUNT)
    {
        return KEY_STATE_RELEASED;
    }

    return g_key_runtime[(uint8_t)key_id].state;
}
