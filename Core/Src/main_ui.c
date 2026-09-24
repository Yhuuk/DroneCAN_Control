#include "main_ui.h"

#include "lcd_init.h"
#include "spi1_bus.h"
#include "ui_static_asset.h"

#include <stddef.h>
#include <string.h>

/* 主页面使用与设计稿一致的240x120横屏逻辑坐标。 */
#define MAIN_UI_LOGICAL_WIDTH          240U
#define MAIN_UI_LOGICAL_HEIGHT         120U
#define MAIN_UI_FONT_WIDTH              12U
#define MAIN_UI_FONT_HEIGHT             24U
#define MAIN_UI_FONT_BYTES_PER_ROW       2U

/*
 * "ID:"固定标签已固化在Flash底图；Node ID数值仍是动态元素，可独立定位。
 * 当前数值X=38，保持优化前已经确认的页面布局不变。
 */
#define MAIN_UI_NODE_VALUE_X            38U
#define MAIN_UI_NODE_VALUE_Y             0U

/* 锁图标以原图中心为基准放大约1.5倍，便于观察油门锁定状态。 */
#define MAIN_UI_LOCK_ICON_X             186U
#define MAIN_UI_LOCK_ICON_Y               0U
#define MAIN_UI_LOCK_ICON_WIDTH          18U
#define MAIN_UI_LOCK_ICON_HEIGHT         23U
#define MAIN_UI_UNLOCKED_ICON_WIDTH      27U

/* RGB565配色由参考图主色转换而来。 */
#define MAIN_UI_COLOR_BACKGROUND       0x1657U /* #15CAB8 */
#define MAIN_UI_COLOR_FOREGROUND       BLACK
#define MAIN_UI_COLOR_CAN_ONLINE       0x2FC2U /* #29fd16 */
#define MAIN_UI_COLOR_CAN_OFFLINE      0x8410U
#define MAIN_UI_COLOR_THROTTLE         0xFA8BU /* #06ea11 */ /*0xFA8BU*/
#define MAIN_UI_COLOR_FOCUS            YELLOW

/* 主页面底部油门条的固定几何参数。 */
#define MAIN_UI_BAR_LEFT                61U
#define MAIN_UI_BAR_TOP                105U
#define MAIN_UI_BAR_RIGHT              123U
#define MAIN_UI_BAR_BOTTOM             112U

/**
 * @brief 本次刷新需要合成的动态图层。
 *
 * 优化三“动态元素专用局部刷新”：焦点、锁和油门条各自只运行自己的
 * 像素生成逻辑。局部更新不再进入完整页面的Node ID、CAN点等判断链。
 */
typedef enum
{
    MAIN_UI_RENDER_FULL = 0,
    MAIN_UI_RENDER_FOCUS,
    MAIN_UI_RENDER_LOCK,
    MAIN_UI_RENDER_THROTTLE_BAR
} MainUiRenderMode_t;

/*
 * 12x24字高与23像素高的锁图标最接近。字库仍由lcd_draw.c唯一提供定义，
 * 本文件只声明并读取点阵，避免在多个源文件中重复定义字库数据。
 */
extern const unsigned char ascii_2412[][48];

/*
 * 只缓存一条物理扫描行，避免为完整240x120 RGB565画面分配57,600字节。
 * MainUI只由UiTask调用，因此无需为该缓冲区增加互斥锁。
 */
static uint8_t g_main_ui_physical_row_buffer[LCD_W * 2U];

/*
 * 锁体保持18x23不变。开锁点阵为了容纳向右伸出的锁梁，每行使用4字节
 * 保存27个有效像素；关锁点阵每行仍使用3字节保存18个有效像素。
 */
static const uint8_t g_main_ui_unlocked_icon[92U] = {
    /* 锁梁左触点右移11像素，正好落在关锁时的右触点位置。 */
    0x00U, 0x00U, 0x7EU, 0x00U,
    0x00U, 0x00U, 0xFFU, 0x00U,
    0x00U, 0x80U, 0xC3U, 0x01U,
    0x00U, 0xC0U, 0x81U, 0x03U,
    0x00U, 0xC0U, 0x00U, 0x03U,
    0x00U, 0xE0U, 0x00U, 0x07U,
    0x00U, 0xE0U, 0x00U, 0x07U,
    0x00U, 0xE0U, 0x00U, 0x07U,
    0x00U, 0xE0U, 0x00U, 0x07U,
    0xFCU, 0xFFU, 0x00U, 0x00U,
    0xFFU, 0xFFU, 0x03U, 0x00U,
    0x03U, 0x00U, 0x03U, 0x00U,
    0x03U, 0x00U, 0x03U, 0x00U,
    0x03U, 0x03U, 0x03U, 0x00U,
    0x83U, 0x07U, 0x03U, 0x00U,
    0x83U, 0x07U, 0x03U, 0x00U,
    0x03U, 0x03U, 0x03U, 0x00U,
    0x03U, 0x03U, 0x03U, 0x00U,
    0x83U, 0x07U, 0x03U, 0x00U,
    0x03U, 0x03U, 0x03U, 0x00U,
    0x03U, 0x00U, 0x03U, 0x00U,
    0xFFU, 0xFFU, 0x03U, 0x00U,
    0xFEU, 0xFFU, 0x01U, 0x00U
};

static const uint8_t g_main_ui_locked_icon[69U] = {
    0xC0U, 0x0FU, 0x00U,
    0xE0U, 0x1FU, 0x00U,
    0x70U, 0x38U, 0x00U,
    0x38U, 0x70U, 0x00U,
    0x18U, 0x60U, 0x00U,
    0x1CU, 0xE0U, 0x00U,
    0x1CU, 0xE0U, 0x00U,
    0x1CU, 0xE0U, 0x00U,
    0x1CU, 0xE0U, 0x00U,
    0xFEU, 0xFFU, 0x01U,
    0xFFU, 0xFFU, 0x03U,
    0x03U, 0x00U, 0x03U,
    0x03U, 0x00U, 0x03U,
    0x03U, 0x03U, 0x03U,
    0x83U, 0x07U, 0x03U,
    0x83U, 0x07U, 0x03U,
    0x83U, 0x07U, 0x03U,
    0x03U, 0x03U, 0x03U,
    0x83U, 0x07U, 0x03U,
    0x83U, 0x07U, 0x03U,
    0x03U, 0x00U, 0x03U,
    0xFFU, 0xFFU, 0x03U,
    0xFEU, 0xFFU, 0x01U
};

static const MainUiView_t g_main_ui_safe_default_view = {
    .node_id = 126U,
    .can_online = false,
    .throttle_unlocked = false,
    .throttle_percent = 20U,
    .focus = MAIN_UI_FOCUS_DIRECTION
};

#if (LCD_W != 120U) || (LCD_H != 240U)
#error "MainUI software rotation expects a 120x240 LCD framebuffer"
#endif

static bool MainUI_PointInRectangle(uint16_t x,
                                    uint16_t y,
                                    uint16_t left,
                                    uint16_t top,
                                    uint16_t right,
                                    uint16_t bottom)
{
    return (x >= left) && (x <= right) &&
           (y >= top) && (y <= bottom);
}

static bool MainUI_PointInCircle(uint16_t x,
                                 uint16_t y,
                                 uint16_t center_x,
                                 uint16_t center_y,
                                 uint16_t radius)
{
    const int32_t dx = (int32_t)x - (int32_t)center_x;
    const int32_t dy = (int32_t)y - (int32_t)center_y;

    return ((dx * dx) + (dy * dy)) <=
           ((int32_t)radius * (int32_t)radius);
}

/** @brief 判断像素是否在圆角矩形内部，用于生成入口焦点轮廓。 */
static bool MainUI_PointInRoundedRectangle(uint16_t x,
                                           uint16_t y,
                                           uint16_t left,
                                           uint16_t top,
                                           uint16_t right,
                                           uint16_t bottom,
                                           uint16_t radius)
{
    if (!MainUI_PointInRectangle(x, y, left, top, right, bottom))
    {
        return false;
    }

    if ((x >= (left + radius)) && (x <= (right - radius)))
    {
        return true;
    }
    if ((y >= (top + radius)) && (y <= (bottom - radius)))
    {
        return true;
    }

    return MainUI_PointInCircle(x, y, left + radius, top + radius, radius) ||
           MainUI_PointInCircle(x, y, right - radius, top + radius, radius) ||
           MainUI_PointInCircle(x, y, left + radius, bottom - radius, radius) ||
           MainUI_PointInCircle(x, y, right - radius, bottom - radius, radius);
}

static bool MainUI_PointOnRoundedRectangleBorder(uint16_t x,
                                                 uint16_t y,
                                                 uint16_t left,
                                                 uint16_t top,
                                                 uint16_t right,
                                                 uint16_t bottom,
                                                 uint16_t radius)
{
    if (!MainUI_PointInRoundedRectangle(
            x, y, left, top, right, bottom, radius))
    {
        return false;
    }

    if ((right - left < 3U) || (bottom - top < 3U) || (radius < 2U))
    {
        return true;
    }

    return !MainUI_PointInRoundedRectangle(
        x, y, left + 3U, top + 3U, right - 3U, bottom - 3U, radius - 3U);
}

/**
 * @brief 查询12x24 ASCII字符串在当前坐标是否有前景像素。
 *
 * ascii_2412的每个字符包含48字节：24行，每行12个像素占2字节。
 * 每个字节均按低位在左的顺序保存，因此先根据Y定位行，再根据X选择
 * 该行的第1或第2字节以及对应bit。
 */
static bool MainUI_TextPixel(uint16_t x,
                             uint16_t y,
                             uint16_t text_x,
                             uint16_t text_y,
                             const char *text)
{
    uint16_t character_index;
    uint16_t local_x;
    uint16_t local_y;
    uint16_t glyph_byte_index;
    uint8_t character;
    size_t text_length;

    if ((text == NULL) || (x < text_x) || (y < text_y))
    {
        return false;
    }

    local_x = x - text_x;
    local_y = y - text_y;
    if (local_y >= MAIN_UI_FONT_HEIGHT)
    {
        return false;
    }

    character_index = local_x / MAIN_UI_FONT_WIDTH;
    text_length = strlen(text);
    if ((size_t)character_index >= text_length)
    {
        return false;
    }

    local_x %= MAIN_UI_FONT_WIDTH;
    glyph_byte_index = (local_y * MAIN_UI_FONT_BYTES_PER_ROW) +
                       (local_x / 8U);
    character = (uint8_t)text[character_index];
    if ((character < (uint8_t)' ') || (character > (uint8_t)'~'))
    {
        return false;
    }

    return (ascii_2412[character - (uint8_t)' '][glyph_byte_index] &
            (uint8_t)(1U << (local_x % 8U))) != 0U;
}

/** @brief 向字符串尾部追加一个0~255的十进制数，返回新的尾指针。 */
static char *MainUI_AppendUint8(char *destination, uint8_t value)
{
    if (value >= 100U)
    {
        *destination++ = (char)('0' + (value / 100U));
        value %= 100U;
        *destination++ = (char)('0' + (value / 10U));
        *destination++ = (char)('0' + (value % 10U));
    }
    else if (value >= 10U)
    {
        *destination++ = (char)('0' + (value / 10U));
        *destination++ = (char)('0' + (value % 10U));
    }
    else
    {
        *destination++ = (char)('0' + value);
    }

    return destination;
}

/** @brief 将Node ID数值转换为不带标签的十进制字符串。 */
static void MainUI_FormatNodeValueText(const MainUiView_t *view,
                                       char node_value_text[4])
{
    char *write_pointer;

    write_pointer = MainUI_AppendUint8(node_value_text, view->node_id);
    *write_pointer = '\0';
}

/**
 * @brief 查询一个逻辑坐标是否命中1-bit图标点阵。
 * @param x 当前主页面的逻辑X坐标。
 * @param y 当前主页面的逻辑Y坐标。
 * @param left 图标左上角X坐标。
 * @param top 图标左上角Y坐标。
 * @param width 图标有效宽度，单位为像素。
 * @param height 图标有效高度，单位为像素。
 * @param bytes_per_row 点阵每行占用的字节数。
 * @param bitmap 图标点阵首地址；每个字节的bit0对应靠左像素。
 * @return true表示该位置应绘制图标前景色，false表示保持背景色。
 */
static bool MainUI_BitmapPixel(uint16_t x,
                               uint16_t y,
                               uint16_t left,
                               uint16_t top,
                               uint16_t width,
                               uint16_t height,
                               uint16_t bytes_per_row,
                               const uint8_t *bitmap)
{
    uint16_t local_x;
    uint16_t local_y;
    uint32_t byte_index;

    if ((bitmap == NULL) ||
        (x < left) || (y < top) ||
        (x >= (left + width)) || (y >= (top + height)))
    {
        return false;
    }

    local_x = (uint16_t)(x - left);
    local_y = (uint16_t)(y - top);
    byte_index = ((uint32_t)local_y * bytes_per_row) +
                 ((uint32_t)local_x / 8U);

    return (bitmap[byte_index] &
            (uint8_t)(1U << (local_x % 8U))) != 0U;
}

static bool MainUI_LockIconPixel(uint16_t x,
                                 uint16_t y,
                                 bool unlocked)
{
    if (unlocked)
    {
        return MainUI_BitmapPixel(x,
                                  y,
                                  MAIN_UI_LOCK_ICON_X,
                                  MAIN_UI_LOCK_ICON_Y,
                                  MAIN_UI_UNLOCKED_ICON_WIDTH,
                                  MAIN_UI_LOCK_ICON_HEIGHT,
                                  4U,
                                  g_main_ui_unlocked_icon);
    }

    return MainUI_BitmapPixel(x,
                              y,
                              MAIN_UI_LOCK_ICON_X,
                              MAIN_UI_LOCK_ICON_Y,
                              MAIN_UI_LOCK_ICON_WIDTH,
                              MAIN_UI_LOCK_ICON_HEIGHT,
                              3U,
                              g_main_ui_locked_icon);
}

/**
 * @brief 获取一个主页面入口对应的焦点框逻辑坐标。
 * @return true表示入口有效且已经写入坐标，false表示入口枚举非法。
 */
static bool MainUI_GetFocusBounds(MainUiFocus_t focus,
                                  uint16_t *left,
                                  uint16_t *top,
                                  uint16_t *right,
                                  uint16_t *bottom)
{
    if ((left == NULL) || (top == NULL) ||
        (right == NULL) || (bottom == NULL))
    {
        return false;
    }

    switch (focus)
    {
        case MAIN_UI_FOCUS_DIRECTION:
            *left = 5U; *top = 35U; *right = 53U; *bottom = 83U;
            break;
        case MAIN_UI_FOCUS_THROTTLE:
            *left = 66U; *top = 39U; *right = 114U; *bottom = 81U;
            break;
        case MAIN_UI_FOCUS_SETTINGS:
            *left = 126U; *top = 38U; *right = 174U; *bottom = 82U;
            break;
        case MAIN_UI_FOCUS_STATUS:
            *left = 186U; *top = 39U; *right = 233U; *bottom = 82U;
            break;
        default:
            return false;
    }

    return true;
}

static bool MainUI_FocusPixel(uint16_t x,
                              uint16_t y,
                              MainUiFocus_t focus)
{
    uint16_t left;
    uint16_t top;
    uint16_t right;
    uint16_t bottom;

    if (!MainUI_GetFocusBounds(focus, &left, &top, &right, &bottom))
    {
        return false;
    }

    return MainUI_PointOnRoundedRectangleBorder(
        x, y, left, top, right, bottom, 6U);
}

/**
 * @brief 从MCU内部Flash读取主页面静态底图像素。
 *
 * 优化二“MCU内部Flash静态底图”：固定标签、四个入口图标、THR文字和
 * 进度条外框已经在编译前转换为4位调色板位图。资源按120x240物理扫描
 * 顺序存放，每字节保存两个像素，因此完整底图只占14,400字节Flash，
 * 不占RAM，也不访问与OLED共用SPI1的外部FRAM。
 */
static uint16_t MainUI_GetStaticPhysicalPixel(uint32_t packed_row_index,
                                              uint16_t physical_x)
{
    const uint32_t packed_index =
        packed_row_index + ((uint32_t)physical_x >> 1U);
    const uint8_t packed = g_main_ui_static_4bpp[packed_index];
    const uint8_t palette_index = ((physical_x & 1U) == 0U)
                                      ? (uint8_t)(packed >> 4U)
                                      : (uint8_t)(packed & 0x0FU);

    return g_main_ui_static_palette[palette_index];
}

/** @brief 在静态底图上仅合成当前油门条的动态填充部分。 */
static uint16_t MainUI_OverlayThrottleBar(uint16_t x,
                                          uint16_t y,
                                          const MainUiView_t *view,
                                          uint16_t color)
{
    const uint16_t inner_width =
        MAIN_UI_BAR_RIGHT - MAIN_UI_BAR_LEFT - 1U;
    const uint16_t fill_width =
        (uint16_t)(((uint32_t)inner_width * view->throttle_percent) / 100U);

    if ((fill_width > 0U) &&
        (x > MAIN_UI_BAR_LEFT) &&
        (x <= (MAIN_UI_BAR_LEFT + fill_width)) &&
        (y > MAIN_UI_BAR_TOP) &&
        (y < MAIN_UI_BAR_BOTTOM))
    {
        return MAIN_UI_COLOR_THROTTLE;
    }

    return color;
}

/**
 * @brief 生成一个主页面像素，并按刷新模式只合成必要的动态元素。
 *
 * 优化一“区域判断优化”：完整刷新先按Y坐标分为顶部状态栏、中央入口区和
 * 底部油门区；每个像素不会再检查四个图标、全部文字和全部状态。优化三
 * 的局部模式则进一步只执行焦点、锁或油门条中的一个分支。
 */
static uint16_t MainUI_GetRenderedPixel(uint16_t x,
                                        uint16_t y,
                                        const MainUiView_t *view,
                                        const char *node_value_text,
                                        uint16_t static_color,
                                        MainUiRenderMode_t render_mode)
{
    uint16_t color = static_color;

    if ((render_mode == MAIN_UI_RENDER_FULL) ||
        (render_mode == MAIN_UI_RENDER_FOCUS))
    {
        if ((y >= 35U) && (y <= 83U) &&
            MainUI_FocusPixel(x, y, view->focus))
        {
            color = MAIN_UI_COLOR_FOCUS;

            /* 原页面中固定图标位于焦点框之上，保持完全相同的叠放顺序。 */
            if (static_color != MAIN_UI_COLOR_BACKGROUND)
            {
                color = static_color;
            }
        }
    }

    if ((render_mode == MAIN_UI_RENDER_FULL) && (y < 24U))
    {
        /* ID:和CAN已经在底图中；这里只绘制会变化的Node ID数值。 */
        if ((x >= MAIN_UI_NODE_VALUE_X) &&
            MainUI_TextPixel(x,
                             y,
                             MAIN_UI_NODE_VALUE_X,
                             MAIN_UI_NODE_VALUE_Y,
                             node_value_text))
        {
            color = MAIN_UI_COLOR_FOREGROUND;
        }
    }

    if (((render_mode == MAIN_UI_RENDER_FULL) ||
         (render_mode == MAIN_UI_RENDER_LOCK)) &&
        (y < MAIN_UI_LOCK_ICON_HEIGHT) &&
        MainUI_LockIconPixel(x, y, view->throttle_unlocked))
    {
        color = MAIN_UI_COLOR_FOREGROUND;
    }

    if ((render_mode == MAIN_UI_RENDER_FULL) &&
        (y <= 18U) &&
        MainUI_PointInCircle(x, y, 148U, 12U, 6U))
    {
        color = view->can_online
                    ? MAIN_UI_COLOR_CAN_ONLINE
                    : MAIN_UI_COLOR_CAN_OFFLINE;
    }

    if (((render_mode == MAIN_UI_RENDER_FULL) ||
         (render_mode == MAIN_UI_RENDER_THROTTLE_BAR)) &&
        (y >= MAIN_UI_BAR_TOP) && (y <= MAIN_UI_BAR_BOTTOM))
    {
        color = MainUI_OverlayThrottleBar(x, y, view, color);
    }

    return color;
}

static void MainUI_ValidateView(const MainUiView_t *source,
                                MainUiView_t *destination)
{
    if (destination == NULL)
    {
        return;
    }

    *destination = (source != NULL) ? *source : g_main_ui_safe_default_view;

    if (destination->node_id > 127U)
    {
        destination->node_id = g_main_ui_safe_default_view.node_id;
    }
    if (destination->throttle_percent > 100U)
    {
        destination->throttle_percent = 100U;
    }
    if ((destination->focus != MAIN_UI_FOCUS_DIRECTION) &&
        (destination->focus != MAIN_UI_FOCUS_THROTTLE) &&
        (destination->focus != MAIN_UI_FOCUS_SETTINGS) &&
        (destination->focus != MAIN_UI_FOCUS_STATUS))
    {
        destination->focus = MAIN_UI_FOCUS_DIRECTION;
    }
}

/**
 * @brief 重画一个包含端点的主页面逻辑矩形区域。
 *
 * 240x120逻辑画布旋转后写入120x240物理显存。SH8501的物理列窗口按
 * 4像素边界向外对齐，扩展出来的少量像素也使用当前视图重新生成，
 * 因此不会留下旧焦点边缘。
 */
static void MainUI_DrawLogicalRegion(const MainUiView_t *view,
                                     uint16_t logical_left,
                                     uint16_t logical_top,
                                     uint16_t logical_right,
                                     uint16_t logical_bottom,
                                     MainUiRenderMode_t render_mode)
{
    MainUiView_t validated_view;
    char node_value_text[4];
    uint16_t physical_left;
    uint16_t physical_right;
    uint16_t physical_top;
    uint16_t physical_bottom;

    if ((logical_left > logical_right) ||
        (logical_top > logical_bottom) ||
        (logical_left >= MAIN_UI_LOGICAL_WIDTH) ||
        (logical_top >= MAIN_UI_LOGICAL_HEIGHT))
    {
        return;
    }

    if (logical_right >= MAIN_UI_LOGICAL_WIDTH)
    {
        logical_right = MAIN_UI_LOGICAL_WIDTH - 1U;
    }
    if (logical_bottom >= MAIN_UI_LOGICAL_HEIGHT)
    {
        logical_bottom = MAIN_UI_LOGICAL_HEIGHT - 1U;
    }

    MainUI_ValidateView(view, &validated_view);
    MainUI_FormatNodeValueText(&validated_view, node_value_text);

    /* logical_x=physical_y，logical_y=119-physical_x。 */
    physical_left = (MAIN_UI_LOGICAL_HEIGHT - 1U) - logical_bottom;
    physical_right = (MAIN_UI_LOGICAL_HEIGHT - 1U) - logical_top;
    physical_top = logical_left;
    physical_bottom = logical_right;

    physical_left &= (uint16_t)~3U;
    physical_right |= 3U;
    if (physical_right >= LCD_W)
    {
        physical_right = LCD_W - 1U;
    }

    /* 地址窗口与其后全部像素必须连续，禁止FRAM事务插入其中。 */
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
        const uint32_t packed_row_index =
            (uint32_t)physical_y * UI_STATIC_ASSET_PACKED_ROW_BYTES;

        for (uint16_t physical_x = physical_left;
             physical_x <= physical_right;
             ++physical_x)
        {
            const uint16_t logical_x = physical_y;
            const uint16_t logical_y =
                (MAIN_UI_LOGICAL_HEIGHT - 1U) - physical_x;
            const uint16_t static_color = MainUI_GetStaticPhysicalPixel(
                packed_row_index, physical_x);
            const uint16_t color = MainUI_GetRenderedPixel(
                logical_x,
                logical_y,
                &validated_view,
                node_value_text,
                static_color,
                render_mode);

            g_main_ui_physical_row_buffer[buffer_index++] =
                (uint8_t)(color >> 8);
            g_main_ui_physical_row_buffer[buffer_index++] = (uint8_t)color;
        }

        LCD_WriteDataBuffer(g_main_ui_physical_row_buffer, buffer_index);
    }

    SPI1_Bus_Release();
}

void MainUI_Draw(const MainUiView_t *view)
{
    MainUI_DrawLogicalRegion(view,
                             0U,
                             0U,
                             MAIN_UI_LOGICAL_WIDTH - 1U,
                             MAIN_UI_LOGICAL_HEIGHT - 1U,
                             MAIN_UI_RENDER_FULL);
}

void MainUI_UpdateFocus(const MainUiView_t *previous_view,
                        const MainUiView_t *current_view)
{
    MainUiView_t previous;
    MainUiView_t current;
    uint16_t left;
    uint16_t top;
    uint16_t right;
    uint16_t bottom;

    if ((previous_view == NULL) || (current_view == NULL))
    {
        MainUI_Draw(current_view);
        return;
    }

    MainUI_ValidateView(previous_view, &previous);
    MainUI_ValidateView(current_view, &current);
    if (previous.focus == current.focus)
    {
        return;
    }

    /* 使用新视图重画旧焦点区域，先恢复被旧边框覆盖的背景和图标。 */
    if (MainUI_GetFocusBounds(previous.focus,
                              &left,
                              &top,
                              &right,
                              &bottom))
    {
        MainUI_DrawLogicalRegion(&current,
                                 left,
                                 top,
                                 right,
                                 bottom,
                                 MAIN_UI_RENDER_FOCUS);
    }

    /* 再只重画新焦点区域，不发送与焦点无关的顶部和底部画面。 */
    if (MainUI_GetFocusBounds(current.focus,
                              &left,
                              &top,
                              &right,
                              &bottom))
    {
        MainUI_DrawLogicalRegion(&current,
                                 left,
                                 top,
                                 right,
                                 bottom,
                                 MAIN_UI_RENDER_FOCUS);
    }
}

void MainUI_UpdateThrottleStatus(const MainUiView_t *previous_view,
                                 const MainUiView_t *current_view)
{
    MainUiView_t previous;
    MainUiView_t current;

    if ((previous_view == NULL) || (current_view == NULL))
    {
        MainUI_Draw(current_view);
        return;
    }

    MainUI_ValidateView(previous_view, &previous);
    MainUI_ValidateView(current_view, &current);

    if (previous.throttle_unlocked != current.throttle_unlocked)
    {
        /*
         * 开锁图标比闭锁图标更宽，因此按两种图标的并集重画，确保从
         * 开锁切回闭锁时不会残留右侧锁梁像素。
         */
        MainUI_DrawLogicalRegion(
            &current,
            MAIN_UI_LOCK_ICON_X,
            MAIN_UI_LOCK_ICON_Y,
            MAIN_UI_LOCK_ICON_X + MAIN_UI_UNLOCKED_ICON_WIDTH - 1U,
            MAIN_UI_LOCK_ICON_Y + MAIN_UI_LOCK_ICON_HEIGHT - 1U,
            MAIN_UI_RENDER_LOCK);
    }

    if (previous.throttle_percent != current.throttle_percent)
    {
        /*
         * THR标签和白色边框已在Flash底图中，只刷新会变化的条形内部。
         * 窗口按4列对齐后扩出的像素也由同一底图正确恢复。
         */
        MainUI_DrawLogicalRegion(&current,
                                 MAIN_UI_BAR_LEFT,
                                 MAIN_UI_BAR_TOP,
                                 MAIN_UI_BAR_RIGHT,
                                 MAIN_UI_BAR_BOTTOM,
                                 MAIN_UI_RENDER_THROTTLE_BAR);
    }
}
