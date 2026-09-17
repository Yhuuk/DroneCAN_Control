#ifndef THROTTLE_UI_ASSET_H
#define THROTTLE_UI_ASSET_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 原稿8个电机图标的灰度像素。
 *
 * 每个图标固定26x26。0表示黑色前景，255表示原稿浅色背景；绘图层可在
 * 不改变齿轮和数字轮廓的前提下，把背景替换成通道选中色。
 * 
 * THROTTLE_UI_MOTOR_ICON_PIXELS 是单个油门电机图标的像素数量
 */
#define THROTTLE_UI_MOTOR_ICON_COUNT   8U
#define THROTTLE_UI_MOTOR_ICON_WIDTH  26U
#define THROTTLE_UI_MOTOR_ICON_HEIGHT 26U
#define THROTTLE_UI_MOTOR_ICON_PIXELS \
    (THROTTLE_UI_MOTOR_ICON_WIDTH * THROTTLE_UI_MOTOR_ICON_HEIGHT)

extern const uint8_t
    g_throttle_ui_motor_icon_gray[THROTTLE_UI_MOTOR_ICON_COUNT]
                                    [THROTTLE_UI_MOTOR_ICON_PIXELS];

#ifdef __cplusplus
}
#endif

#endif /* THROTTLE_UI_ASSET_H */
