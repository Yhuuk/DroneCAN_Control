#ifndef SETTINGS_UI_H
#define SETTINGS_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief 240x120设置页同一时刻固定显示三个条目。 */
#define SETTINGS_UI_VISIBLE_ITEM_COUNT 3U

/** @brief 设置页面五个入口的显示顺序与参考SVG从上到下完全一致。 */
typedef enum
{
    SETTINGS_UI_FOCUS_SYSTEM = 0,
    SETTINGS_UI_FOCUS_JOYSTICK_CALIBRATION,
    SETTINGS_UI_FOCUS_NODE_ID,
    SETTINGS_UI_FOCUS_BRIGHTNESS,
    SETTINGS_UI_FOCUS_SOUND,
    SETTINGS_UI_FOCUS_COUNT
} SettingsUiFocus_t;

/**
 * @brief 设置页显示状态。
 *
 * focus表示当前选中的条目；first_visible_item独立记录当前窗口最上方
 * 的条目。两者分开后，向上移动可先在现有三行内移动焦点，到达第一行
 * 后再次向上时才滚动列表。
 */
typedef struct
{
    SettingsUiFocus_t focus;
    uint8_t first_visible_item;
} SettingsUiView_t;

/** @brief 绘制完整的240x120横屏设置页面。 */
void SettingsUI_Draw(const SettingsUiView_t *view);

/**
 * @brief 焦点变化时只恢复旧行并重画新行，避免重复发送完整画面。
 * @param previous 焦点移动前的设置页视图。
 * @param current 焦点移动后的设置页视图。
 */
void SettingsUI_UpdateFocus(const SettingsUiView_t *previous,
                            const SettingsUiView_t *current);

#ifdef __cplusplus
}
#endif

#endif /* SETTINGS_UI_H */
