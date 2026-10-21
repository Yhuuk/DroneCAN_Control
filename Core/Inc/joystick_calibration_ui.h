#ifndef JOYSTICK_CALIBRATION_UI_H
#define JOYSTICK_CALIBRATION_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    JOYSTICK_CALIBRATION_UI_STEP_CENTER = 1,
    JOYSTICK_CALIBRATION_UI_STEP_UP = 2,
    JOYSTICK_CALIBRATION_UI_STEP_DOWN = 3,
    JOYSTICK_CALIBRATION_UI_STEP_LEFT = 4,
    JOYSTICK_CALIBRATION_UI_STEP_RIGHT = 5
} JoystickCalibrationUiStep_t;

typedef enum
{
    JOYSTICK_CALIBRATION_UI_FOCUS_ESC = 0,
    JOYSTICK_CALIBRATION_UI_FOCUS_OK,
    JOYSTICK_CALIBRATION_UI_FOCUS_NEXT
} JoystickCalibrationUiFocus_t;

typedef enum
{
    JOYSTICK_CALIBRATION_UI_READY = 0,
    JOYSTICK_CALIBRATION_UI_SAMPLING,
    JOYSTICK_CALIBRATION_UI_SUCCEEDED,
    JOYSTICK_CALIBRATION_UI_FAILED,
    /** 用户在本步骤成功前选择NEXT时显示的提示状态。 */
    JOYSTICK_CALIBRATION_UI_INCOMPLETE
} JoystickCalibrationUiState_t;

/** @brief 当前校准步骤需要显示的全部状态。 */
typedef struct
{
    uint16_t raw_x;
    uint16_t raw_y;
    JoystickCalibrationUiStep_t step;
    JoystickCalibrationUiFocus_t focus;
    JoystickCalibrationUiState_t state;
} JoystickCalibrationUiView_t;

/** @brief 完整绘制240x120当前校准步骤页面。 */
void JoystickCalibrationUI_Draw(const JoystickCalibrationUiView_t *view);

/** @brief 根据原始值、状态或焦点变化只重画对应局部区域。 */
void JoystickCalibrationUI_Update(
    const JoystickCalibrationUiView_t *previous,
    const JoystickCalibrationUiView_t *current);

#ifdef __cplusplus
}
#endif

#endif /* JOYSTICK_CALIBRATION_UI_H */
