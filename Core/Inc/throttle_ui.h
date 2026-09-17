#ifndef THROTTLE_UI_H
#define THROTTLE_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/** 油门页面可获得焦点的三个顶部控件和8个电机通道。 */
typedef enum
{
    THROTTLE_UI_FOCUS_ALL = 0,
    THROTTLE_UI_FOCUS_LIMIT,
    THROTTLE_UI_FOCUS_STEP,
    THROTTLE_UI_FOCUS_MOTOR_1,
    THROTTLE_UI_FOCUS_MOTOR_2,
    THROTTLE_UI_FOCUS_MOTOR_3,
    THROTTLE_UI_FOCUS_MOTOR_4,
    THROTTLE_UI_FOCUS_MOTOR_5,
    THROTTLE_UI_FOCUS_MOTOR_6,
    THROTTLE_UI_FOCUS_MOTOR_7,
    THROTTLE_UI_FOCUS_MOTOR_8
} ThrottleUiFocus_t;

/**
 * @brief 油门页面完整显示状态。
 *
 * motor_mask、limit和raw_command来自ThrottleControl共享快照；focus、step和
 * edit_mode只属于UiTask页面状态机。绘图层不直接读取ADC、按键或CAN。
 */
typedef struct
{
    bool throttle_unlocked;
    bool edit_mode;
    uint8_t motor_mask;
    uint16_t limit;
    uint16_t step;
    uint16_t raw_command;
    uint8_t throttle_percent;
    ThrottleUiFocus_t focus;
} ThrottleUiView_t;

/** @brief 按用户提供的 240x120 画布绘制完整油门页面。 */
void ThrottleUI_Draw(const ThrottleUiView_t *view);

/**
 * @brief 比较前后页面状态，并只重画发生变化的区域。
 *
 * 焦点、选中通道、LIM、Step、锁和MASK通过本接口局部刷新。连续变化的
 * 进度条与油门数值由下面两个专用接口按各自周期刷新。
 */
void ThrottleUI_Update(const ThrottleUiView_t *previous,
                       const ThrottleUiView_t *current);

/** @brief 只重画底部油门进度条，用于25 Hz连续显示。 */
void ThrottleUI_UpdateThrottleBar(const ThrottleUiView_t *view);

/** @brief 只重画底部0~8191精确油门数值，用于20 Hz连续显示。 */
void ThrottleUI_UpdateThrottleValue(const ThrottleUiView_t *view);

#ifdef __cplusplus
}
#endif

#endif /* THROTTLE_UI_H */
