#ifndef THROTTLE_UI_H
#define THROTTLE_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 临时油门调试页所需的数据。
 *
 * 该结构体只包含显示数据，不直接依赖 ADC 或摇杆驱动，保持 UI 与采集层解耦。
 */
typedef struct
{
    uint16_t throttle_raw;
    uint16_t direction_raw;
} ThrottleUiView_t;

/** @brief 绘制完整的临时油门/方向 ADC 调试页面。 */
void ThrottleUI_Draw(const ThrottleUiView_t *view);

/**
 * @brief 比较前后数据，并仅刷新发生变化的数值区域。
 *
 * @param previous 上一次已经显示的数据。
 * @param current  当前准备显示的数据。
 */
void ThrottleUI_UpdateValues(const ThrottleUiView_t *previous,
                             const ThrottleUiView_t *current);

#ifdef __cplusplus
}
#endif

#endif /* THROTTLE_UI_H */
