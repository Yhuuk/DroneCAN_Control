#ifndef BRIGHTNESS_UI_H
#define BRIGHTNESS_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief 临时亮度测试页面需要显示的全部状态。 */
typedef struct
{
    /** SH8501A WRDISBV(0x51)使用的原始亮度值，范围0~255。 */
    uint8_t brightness;
} BrightnessUiView_t;

/** @brief 绘制完整的240x120横屏亮度测试页面。 */
void BrightnessUI_Draw(const BrightnessUiView_t *view);

/**
 * @brief 亮度变化后只刷新百分比、原始值和进度条区域。
 * @param previous 变化前视图；NULL时退回完整页面刷新。
 * @param current  变化后视图；NULL时使用安全默认亮度。
 */
void BrightnessUI_Update(const BrightnessUiView_t *previous,
                         const BrightnessUiView_t *current);

#ifdef __cplusplus
}
#endif

#endif /* BRIGHTNESS_UI_H */
