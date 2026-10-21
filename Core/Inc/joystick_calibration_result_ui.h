#ifndef JOYSTICK_CALIBRATION_RESULT_UI_H
#define JOYSTICK_CALIBRATION_RESULT_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "joystick.h"

#include <stdbool.h>

typedef enum
{
    JOYSTICK_CALIBRATION_RESULT_FOCUS_CANCEL = 0,
    JOYSTICK_CALIBRATION_RESULT_FOCUS_SAVE
} JoystickCalibrationResultFocus_t;

/**
 * @brief 校准确认页视图。
 *
 * x_axis对应DMA[0]/方向轴，y_axis对应DMA[1]/油门轴。saved只表示本次
 * 用户候选值已经通过FRAM写后回读校验并同步到运行RAM。
 */
typedef struct
{
    JoystickAxisCalibration_t x_axis;
    JoystickAxisCalibration_t y_axis;
    JoystickCalibrationResultFocus_t focus;
    bool saved;
} JoystickCalibrationResultUiView_t;

/** @brief 按240x120参考图完整绘制校准结果确认页。 */
void JoystickCalibrationResultUI_Draw(
    const JoystickCalibrationResultUiView_t *view);

/** @brief 仅刷新焦点及SAVE颜色所在的底部区域。 */
void JoystickCalibrationResultUI_Update(
    const JoystickCalibrationResultUiView_t *previous,
    const JoystickCalibrationResultUiView_t *current);

#ifdef __cplusplus
}
#endif

#endif /* JOYSTICK_CALIBRATION_RESULT_UI_H */
