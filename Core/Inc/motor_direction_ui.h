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

/**
 * @brief 绘制完整的8路电机方向选择页面。
 *
 * 屏幕控制器当前按120x240竖屏显存访问，本页面按照参考图使用240x120
 * 逻辑坐标，并在输出像素时执行90度软件旋转。因此无需修改已经验证可用的
 * SH8501初始化表和MADCTL设置。
 *
 * @param power_state    左上角显示的OFF/ON状态。
 * @param selected_motor 当前选中的电机编号，合法范围1~8；其它值不画选框。
 */
void MotorDirectionUI_Draw(MotorDirectionUiPowerState_t power_state,
                           uint8_t selected_motor);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DIRECTION_UI_H */
