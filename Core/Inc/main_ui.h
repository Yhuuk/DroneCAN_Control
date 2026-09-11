#ifndef MAIN_UI_H
#define MAIN_UI_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 主页面四个功能入口。 */
typedef enum
{
    MAIN_UI_FOCUS_DIRECTION = 0,
    MAIN_UI_FOCUS_THROTTLE,
    MAIN_UI_FOCUS_SETTINGS,
    MAIN_UI_FOCUS_STATUS
} MainUiFocus_t;

/**
 * @brief 绘制主页面所需的显示状态。
 *
 * focus字段已经由UiTask的UP/DOWN按键和页面路由维护；Node ID、CAN状态、
 * 油门锁及百分比仍通过本结构体统一传给绘图层。后续接入真实业务状态时
 * 无需修改底层绘图接口。
 */
typedef struct
{
    /** 本控制器的DroneCAN Source Node ID，有效范围0~127。 */
    uint8_t node_id;

    /** true显示绿色CAN状态点；false显示灰色状态点。 */
    bool can_online;

    /** true显示开锁图标；false显示闭锁图标。 */
    bool throttle_unlocked;

    /** 主页面左下角显示的油门百分比，绘图时限制在0~100。 */
    uint8_t throttle_percent;

    /** 当前焦点所在入口；开机默认使用MAIN_UI_FOCUS_DIRECTION。 */
    MainUiFocus_t focus;
} MainUiView_t;

/**
 * @brief 绘制完整的240x120横屏主页面。
 *
 * 页面按照240x120逻辑坐标生成，并旋转写入当前120x240物理显存，不修改
 * 已经验证可用的SH8501初始化方向配置。
 *
 * @param view 主页面状态；NULL或字段非法时使用安全默认值。
 */
void MainUI_Draw(const MainUiView_t *view);

/**
 * @brief 仅更新主页面的旧、新焦点框区域。
 *
 * 用当前视图恢复旧焦点区域，再绘制新焦点区域，避免按键选择入口时刷新
 * 完整屏幕。若任一参数为NULL，则退回完整页面绘制。
 *
 * @param previous_view 焦点移动前的主页面状态。
 * @param current_view  焦点移动后的主页面状态。
 */
void MainUI_UpdateFocus(const MainUiView_t *previous_view,
                        const MainUiView_t *current_view);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_UI_H */
