#include "settings_ui.h"

#include "lcd_init.h"
#include "settings_ui_asset.h"
#include "spi1_bus.h"

#include <stdbool.h>
#include <stddef.h>

/* 参考SVG使用240x120逻辑横屏；LCD控制器实际显存方向为120x240。 */
#define SETTINGS_UI_LOGICAL_WIDTH          240U
#define SETTINGS_UI_LOGICAL_HEIGHT         120U

/*
 * 放大后的列表每页固定显示三行，每行占40像素。24像素文字和图标在行内
 * 垂直居中，既保留参考图的疏朗比例，也不会在120像素高度内发生裁切。
 */
#define SETTINGS_UI_VISIBLE_ROW_COUNT         SETTINGS_UI_VISIBLE_ITEM_COUNT
#define SETTINGS_UI_ROW_HEIGHT               40U

#define SETTINGS_UI_COLOR_BACKGROUND       BLACK
#define SETTINGS_UI_COLOR_FOREGROUND       WHITE
/* 与主页面焦点颜色一致，只在当前条目的圆角边框中使用。 */
#define SETTINGS_UI_COLOR_FOCUS            YELLOW
#define SETTINGS_UI_FOCUS_RADIUS             5U
#define SETTINGS_UI_FOCUS_BORDER_WIDTH       2U
#define SETTINGS_UI_FOCUS_HORIZONTAL_MARGIN  6U
#define SETTINGS_UI_FOCUS_VERTICAL_INSET     2U

typedef struct
{
    uint16_t left;
    uint16_t top;
    uint16_t right;
    uint16_t bottom;
} SettingsUiRectangle_t;

typedef struct
{
    uint16_t atlas_offset;
    uint16_t width;
    uint16_t height;
    uint16_t packed_row_bytes;
} SettingsUiItemAsset_t;

/*
 * 图集中的五个条目均由原SVG按2倍尺寸直接重新栅格化，而不是把12像素
 * 点阵简单复制成2x2方块。这样能保持24像素文字和图标边缘的清晰度。
 */
static const SettingsUiItemAsset_t
    g_settings_ui_item_assets[SETTINGS_UI_FOCUS_COUNT] = {
        [SETTINGS_UI_FOCUS_SYSTEM] = {0U, 116U, 24U, 15U},
        [SETTINGS_UI_FOCUS_JOYSTICK_CALIBRATION] =
            {360U, 92U, 22U, 12U},
        [SETTINGS_UI_FOCUS_NODE_ID] = {624U, 88U, 18U, 11U},
        [SETTINGS_UI_FOCUS_BRIGHTNESS] = {822U, 80U, 22U, 10U},
        [SETTINGS_UI_FOCUS_SOUND] = {1042U, 80U, 22U, 10U}
};

static const SettingsUiView_t g_settings_ui_safe_default = {
    .focus = SETTINGS_UI_FOCUS_SYSTEM,
    .first_visible_item = 0U
};

/* UiTask是唯一绘图任务，单条物理扫描行缓冲区不需要额外互斥。 */
static uint8_t g_settings_ui_line_buffer[LCD_W * 2U];

_Static_assert(SETTINGS_UI_ASSET_ITEM_COUNT == SETTINGS_UI_FOCUS_COUNT,
               "SettingsUI item count must match focus count");
_Static_assert((SETTINGS_UI_VISIBLE_ROW_COUNT * SETTINGS_UI_ROW_HEIGHT) ==
                   SETTINGS_UI_LOGICAL_HEIGHT,
               "SettingsUI rows must fill the 120-pixel logical screen");

#if (LCD_W != SETTINGS_UI_LOGICAL_HEIGHT) || \
    (LCD_H != SETTINGS_UI_LOGICAL_WIDTH)
#error "SettingsUI expects a 120x240 physical framebuffer"
#endif

static bool SettingsUI_PointInRoundedRectangle(
    uint16_t x,
    uint16_t y,
    const SettingsUiRectangle_t *rectangle,
    uint16_t radius)
{
    uint16_t corner_x;
    uint16_t corner_y;
    int32_t dx;
    int32_t dy;

    if ((rectangle == NULL) ||
        (x < rectangle->left) || (x > rectangle->right) ||
        (y < rectangle->top) || (y > rectangle->bottom))
    {
        return false;
    }

    if ((x >= (uint16_t)(rectangle->left + radius)) &&
        (x <= (uint16_t)(rectangle->right - radius)))
    {
        return true;
    }
    if ((y >= (uint16_t)(rectangle->top + radius)) &&
        (y <= (uint16_t)(rectangle->bottom - radius)))
    {
        return true;
    }

    corner_x = (x < (uint16_t)(rectangle->left + radius))
                   ? (uint16_t)(rectangle->left + radius)
                   : (uint16_t)(rectangle->right - radius);
    corner_y = (y < (uint16_t)(rectangle->top + radius))
                   ? (uint16_t)(rectangle->top + radius)
                   : (uint16_t)(rectangle->bottom - radius);
    dx = (int32_t)x - (int32_t)corner_x;
    dy = (int32_t)y - (int32_t)corner_y;

    return ((dx * dx) + (dy * dy)) <=
           ((int32_t)radius * (int32_t)radius);
}

static bool SettingsUI_PointOnFocusBorder(
    uint16_t x,
    uint16_t y,
    const SettingsUiRectangle_t *outer)
{
    SettingsUiRectangle_t inner;

    if (!SettingsUI_PointInRoundedRectangle(
            x, y, outer, SETTINGS_UI_FOCUS_RADIUS))
    {
        return false;
    }

    inner.left = (uint16_t)(outer->left + SETTINGS_UI_FOCUS_BORDER_WIDTH);
    inner.top = (uint16_t)(outer->top + SETTINGS_UI_FOCUS_BORDER_WIDTH);
    inner.right = (uint16_t)(outer->right - SETTINGS_UI_FOCUS_BORDER_WIDTH);
    inner.bottom = (uint16_t)(outer->bottom - SETTINGS_UI_FOCUS_BORDER_WIDTH);

    return !SettingsUI_PointInRoundedRectangle(
        x,
        y,
        &inner,
        SETTINGS_UI_FOCUS_RADIUS - SETTINGS_UI_FOCUS_BORDER_WIDTH);
}

static void SettingsUI_NormalizeView(SettingsUiView_t *destination,
                                     const SettingsUiView_t *source)
{
    const uint8_t maximum_window_start =
        (uint8_t)(SETTINGS_UI_FOCUS_COUNT - SETTINGS_UI_VISIBLE_ROW_COUNT);

    *destination = (source != NULL) ? *source : g_settings_ui_safe_default;
    if ((uint32_t)destination->focus >= (uint32_t)SETTINGS_UI_FOCUS_COUNT)
    {
        destination->focus = SETTINGS_UI_FOCUS_SYSTEM;
        destination->first_visible_item = 0U;
        return;
    }

    if (destination->first_visible_item > maximum_window_start)
    {
        destination->first_visible_item = 0U;
    }

    /* 修复外部传入的不一致状态，同时保留合法的“焦点与窗口分离”状态。 */
    if ((uint8_t)destination->focus < destination->first_visible_item)
    {
        destination->first_visible_item = (uint8_t)destination->focus;
    }
    else if ((uint8_t)destination->focus >=
             (uint8_t)(destination->first_visible_item +
                       SETTINGS_UI_VISIBLE_ROW_COUNT))
    {
        destination->first_visible_item =
            (uint8_t)((uint8_t)destination->focus -
                      (SETTINGS_UI_VISIBLE_ROW_COUNT - 1U));
    }
}

/**
 * @brief 读取三行滚动窗口的第一个条目。
 *
 * 窗口起点由按键处理器独立维护，不能再仅根据焦点编号反推。否则从
 * 亮度向上回到Node ID时，窗口会过早跳回0、1、2，而不是继续显示
 * 摇杆校准、Node ID、亮度。
 */
static uint8_t SettingsUI_GetWindowStart(const SettingsUiView_t *view)
{
    return view->first_visible_item;
}

/** @brief 计算某个可见条目在240x120逻辑画布中的实际内容矩形。 */
static bool SettingsUI_GetItemRectangle(const SettingsUiView_t *view,
                                        uint8_t item_index,
                                        SettingsUiRectangle_t *rectangle)
{
    const uint8_t window_start = SettingsUI_GetWindowStart(view);
    uint8_t row;
    const SettingsUiItemAsset_t *asset;

    if ((rectangle == NULL) ||
        (item_index < window_start) ||
        (item_index >= (uint8_t)(window_start +
                                 SETTINGS_UI_VISIBLE_ROW_COUNT)) ||
        (item_index >= (uint8_t)SETTINGS_UI_FOCUS_COUNT))
    {
        return false;
    }

    row = (uint8_t)(item_index - window_start);
    asset = &g_settings_ui_item_assets[item_index];

    rectangle->left =
        (uint16_t)((SETTINGS_UI_LOGICAL_WIDTH - asset->width) / 2U);
    rectangle->top =
        (uint16_t)(((uint16_t)row * SETTINGS_UI_ROW_HEIGHT) +
                   ((SETTINGS_UI_ROW_HEIGHT - asset->height) / 2U));
    rectangle->right =
        (uint16_t)(rectangle->left + asset->width - 1U);
    rectangle->bottom =
        (uint16_t)(rectangle->top + asset->height - 1U);

    return true;
}

/** @brief 计算当前焦点框；横向随条目宽度变化，纵向占据所在的40像素行。 */
static void SettingsUI_GetFocusRectangle(const SettingsUiView_t *view,
                                         SettingsUiRectangle_t *rectangle)
{
    SettingsUiRectangle_t item_rectangle;
    const uint8_t window_start = SettingsUI_GetWindowStart(view);
    const uint8_t row = (uint8_t)view->focus - window_start;

    if (!SettingsUI_GetItemRectangle(
            view, (uint8_t)view->focus, &item_rectangle))
    {
        /* NormalizeView正常调用时不会进入；保留安全回退避免未初始化坐标。 */
        item_rectangle.left = 0U;
        item_rectangle.right = SETTINGS_UI_LOGICAL_WIDTH - 1U;
    }

    rectangle->left =
        (item_rectangle.left > SETTINGS_UI_FOCUS_HORIZONTAL_MARGIN)
            ? (uint16_t)(item_rectangle.left -
                         SETTINGS_UI_FOCUS_HORIZONTAL_MARGIN)
            : 0U;
    rectangle->right =
        (uint16_t)(item_rectangle.right +
                   SETTINGS_UI_FOCUS_HORIZONTAL_MARGIN);
    if (rectangle->right >= SETTINGS_UI_LOGICAL_WIDTH)
    {
        rectangle->right = SETTINGS_UI_LOGICAL_WIDTH - 1U;
    }

    rectangle->top =
        (uint16_t)(((uint16_t)row * SETTINGS_UI_ROW_HEIGHT) +
                   SETTINGS_UI_FOCUS_VERTICAL_INSET);
    rectangle->bottom =
        (uint16_t)((((uint16_t)row + 1U) * SETTINGS_UI_ROW_HEIGHT) -
                   SETTINGS_UI_FOCUS_VERTICAL_INSET - 1U);
}

/** @brief 从Flash图集中读取滚动窗口某个逻辑像素的固定内容。 */
static uint16_t SettingsUI_GetStaticLogicalPixel(
    const SettingsUiView_t *view,
    uint16_t logical_x,
    uint16_t logical_y)
{
    const uint8_t window_start = SettingsUI_GetWindowStart(view);
    const uint8_t row = (uint8_t)(logical_y / SETTINGS_UI_ROW_HEIGHT);
    const uint8_t item_index = (uint8_t)(window_start + row);
    SettingsUiRectangle_t rectangle;
    const SettingsUiItemAsset_t *asset;
    uint16_t source_x;
    uint16_t source_y;
    uint32_t byte_index;
    uint8_t mask;

    if ((row >= SETTINGS_UI_VISIBLE_ROW_COUNT) ||
        (item_index >= (uint8_t)SETTINGS_UI_FOCUS_COUNT) ||
        !SettingsUI_GetItemRectangle(view, item_index, &rectangle) ||
        (logical_x < rectangle.left) || (logical_x > rectangle.right) ||
        (logical_y < rectangle.top) || (logical_y > rectangle.bottom))
    {
        return SETTINGS_UI_COLOR_BACKGROUND;
    }

    asset = &g_settings_ui_item_assets[item_index];
    source_x = (uint16_t)(logical_x - rectangle.left);
    source_y = (uint16_t)(logical_y - rectangle.top);
    byte_index = (uint32_t)asset->atlas_offset +
                 ((uint32_t)source_y * asset->packed_row_bytes) +
                 ((uint32_t)source_x >> 3U);
    mask = (uint8_t)(1U << (source_x & 7U));

    return ((g_settings_ui_item_atlas_1bpp[byte_index] & mask) != 0U)
               ? SETTINGS_UI_COLOR_FOREGROUND
               : SETTINGS_UI_COLOR_BACKGROUND;
}

/**
 * @brief 重画一个设置页逻辑区域。
 *
 * 三个可见条目直接从MCU Flash图集读取；运行时只叠加当前焦点圆角框。
 * 逻辑区域旋转90度后，以整条物理扫描行发送给OLED。
 */
static void SettingsUI_DrawLogicalRegion(const SettingsUiView_t *view,
                                         uint16_t logical_left,
                                         uint16_t logical_top,
                                         uint16_t logical_right,
                                         uint16_t logical_bottom)
{
    SettingsUiView_t normalized;
    SettingsUiRectangle_t focus_bounds;
    uint16_t physical_left;
    uint16_t physical_right;
    uint16_t physical_top;
    uint16_t physical_bottom;

    if ((logical_left > logical_right) || (logical_top > logical_bottom) ||
        (logical_left >= SETTINGS_UI_LOGICAL_WIDTH) ||
        (logical_top >= SETTINGS_UI_LOGICAL_HEIGHT))
    {
        return;
    }

    SettingsUI_NormalizeView(&normalized, view);
    SettingsUI_GetFocusRectangle(&normalized, &focus_bounds);

    if (logical_right >= SETTINGS_UI_LOGICAL_WIDTH)
    {
        logical_right = SETTINGS_UI_LOGICAL_WIDTH - 1U;
    }
    if (logical_bottom >= SETTINGS_UI_LOGICAL_HEIGHT)
    {
        logical_bottom = SETTINGS_UI_LOGICAL_HEIGHT - 1U;
    }

    physical_left = (SETTINGS_UI_LOGICAL_HEIGHT - 1U) - logical_bottom;
    physical_right = (SETTINGS_UI_LOGICAL_HEIGHT - 1U) - logical_top;
    physical_top = logical_left;
    physical_bottom = logical_right;

    /* SH8501物理列窗口需要4像素对齐；扩展像素也从当前列表状态恢复。 */
    physical_left &= (uint16_t)~3U;
    physical_right |= 3U;
    if (physical_right >= LCD_W)
    {
        physical_right = LCD_W - 1U;
    }

    /* 地址窗口和后续全部像素构成一个完整SPI事务。 */
    if (SPI1_Bus_Acquire() != HAL_OK)
    {
        Error_Handler();
        return;
    }

    LCD_Address_Set(physical_left,
                    physical_top,
                    physical_right,
                    physical_bottom);

    for (uint16_t physical_y = physical_top;
         physical_y <= physical_bottom;
         ++physical_y)
    {
        uint16_t buffer_index = 0U;

        for (uint16_t physical_x = physical_left;
             physical_x <= physical_right;
             ++physical_x)
        {
            const uint16_t logical_x = physical_y;
            const uint16_t logical_y =
                (SETTINGS_UI_LOGICAL_HEIGHT - 1U) - physical_x;
            uint16_t color = SettingsUI_GetStaticLogicalPixel(
                &normalized,
                logical_x,
                logical_y);

            if (SettingsUI_PointOnFocusBorder(
                    logical_x, logical_y, &focus_bounds))
            {
                color = SETTINGS_UI_COLOR_FOCUS;
            }

            g_settings_ui_line_buffer[buffer_index++] =
                (uint8_t)(color >> 8U);
            g_settings_ui_line_buffer[buffer_index++] = (uint8_t)color;
        }

        LCD_WriteDataBuffer(g_settings_ui_line_buffer, buffer_index);
    }

    SPI1_Bus_Release();
}

void SettingsUI_Draw(const SettingsUiView_t *view)
{
    SettingsUI_DrawLogicalRegion(view,
                                 0U,
                                 0U,
                                 SETTINGS_UI_LOGICAL_WIDTH - 1U,
                                 SETTINGS_UI_LOGICAL_HEIGHT - 1U);
}

void SettingsUI_UpdateFocus(const SettingsUiView_t *previous,
                            const SettingsUiView_t *current)
{
    SettingsUiView_t previous_normalized;
    SettingsUiView_t current_normalized;
    SettingsUiRectangle_t bounds;

    if ((previous == NULL) || (current == NULL))
    {
        SettingsUI_Draw(current);
        return;
    }

    SettingsUI_NormalizeView(&previous_normalized, previous);
    SettingsUI_NormalizeView(&current_normalized, current);
    if (previous_normalized.focus == current_normalized.focus)
    {
        return;
    }

    /*
     * 跨过第三行时，可见的三个条目都会改变位置，必须重画完整列表。
     * 同一窗口内移动只恢复旧焦点行并重画新焦点行，避免不必要的全屏传输。
     */
    if (SettingsUI_GetWindowStart(&previous_normalized) !=
        SettingsUI_GetWindowStart(&current_normalized))
    {
        SettingsUI_Draw(&current_normalized);
        return;
    }

    SettingsUI_GetFocusRectangle(&previous_normalized, &bounds);
    SettingsUI_DrawLogicalRegion(&current_normalized,
                                 bounds.left,
                                 bounds.top,
                                 bounds.right,
                                 bounds.bottom);

    SettingsUI_GetFocusRectangle(&current_normalized, &bounds);
    SettingsUI_DrawLogicalRegion(&current_normalized,
                                 bounds.left,
                                 bounds.top,
                                 bounds.right,
                                 bounds.bottom);
}

