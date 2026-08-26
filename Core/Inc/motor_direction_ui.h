#ifndef MOTOR_DIRECTION_UI_H
#define MOTOR_DIRECTION_UI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 界面左上角输出状态的显示值。
 *
 * 该枚举目前只决定界面显示“OFF”还是“ON”，不会直接启停电机。后续应由
 * UI状态机根据系统真实状态调用本模块，避免显示状态与控制状态脱节。
 */
typedef enum
{
    MOTOR_DIRECTION_UI_POWER_OFF = 0,
    MOTOR_DIRECTION_UI_POWER_ON
} MotorDirectionUiPowerState_t;

/** @brief 当前焦点所在的业务层级。 */
typedef enum
{
    /** 焦点包围左上角OFF/ON滑动开关。 */
    MOTOR_DIRECTION_UI_FOCUS_SWITCH = 0,

    /** 焦点包围当前选中的1~8通道编号。 */
    MOTOR_DIRECTION_UI_FOCUS_MOTOR,

    /** 焦点包围当前通道下面选中的NOR或REV三角形。 */
    MOTOR_DIRECTION_UI_FOCUS_DIRECTION
} MotorDirectionUiFocus_t;

/** @brief 方向选择焦点当前指向的明确方向。 */
typedef enum
{
    MOTOR_DIRECTION_UI_NORMAL = 0,
    MOTOR_DIRECTION_UI_REVERSED
} MotorDirectionUiDirection_t;

/**
 * @brief 绘制方向页面所需的完整视图状态。
 *
 * UiTask持有并修改该结构体；绘图模块只读取它，不执行按键业务，也不会
 * 发送CAN消息。这样可以保证界面绘制和控制逻辑彼此独立。
 */
typedef struct
{
    MotorDirectionUiPowerState_t power_state;
    MotorDirectionUiFocus_t focus;
    uint8_t selected_motor;
    MotorDirectionUiDirection_t selected_direction;
} MotorDirectionUiView_t;

/**
 * @brief 绘制完整的8路电机方向选择页面。
 *
 * 屏幕控制器当前按120x240竖屏显存访问，本页面按照参考图使用240x120
 * 逻辑坐标，并在输出像素时执行90度软件旋转。因此无需修改已经验证可用的
 * SH8501初始化表和MADCTL设置。
 *
 * @param view 页面状态地址；NULL或非法字段会按安全的OFF初始状态显示。
 */
void MotorDirectionUI_Draw(const MotorDirectionUiView_t *view);

/**
 * @brief 根据前后状态只刷新发生变化的焦点和开关区域。
 *
 * 初次进入页面应调用MotorDirectionUI_Draw()绘制完整页面；后续按键操作
 * 调用本函数，可避免每移动一次焦点都重新传输整屏28,800个像素。
 *
 * @param previous_view 修改前的页面状态。
 * @param current_view  修改后的页面状态。
 */
void MotorDirectionUI_Update(const MotorDirectionUiView_t *previous_view,
                             const MotorDirectionUiView_t *current_view);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DIRECTION_UI_H */
