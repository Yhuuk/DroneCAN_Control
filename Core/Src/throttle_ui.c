#include "throttle_ui.h"

#include "lcd_init.h"

#include <stdbool.h>
#include <stddef.h>

/* 普通标签使用6x12字库，ADC数值单独放大一级为8x16。 */
extern const unsigned char ascii_1206[][12];
extern const unsigned char ascii_1608[][16];

/* LCD 物理安装方向为 120x240，UI 使用更直观的横屏逻辑坐标 240x120。 */
#define THROTTLE_UI_LOGICAL_WIDTH       240U
#define THROTTLE_UI_LOGICAL_HEIGHT      120U

#define THROTTLE_UI_FONT_WIDTH          6U
#define THROTTLE_UI_FONT_HEIGHT         12U
#define THROTTLE_UI_VALUE_FONT_WIDTH    8U
#define THROTTLE_UI_VALUE_FONT_HEIGHT   16U

#define THROTTLE_UI_COLOR_BACKGROUND    0x1D79U
#define THROTTLE_UI_COLOR_BORDER        0xFFFFU
#define THROTTLE_UI_COLOR_TITLE         0xFFFFU
#define THROTTLE_UI_COLOR_LABEL         0xBDF7U
#define THROTTLE_UI_COLOR_VALUE         0x07E0U
#define THROTTLE_UI_COLOR_HINT          0x7BEFU
#define THROTTLE_UI_COLOR_DIVIDER       0x4A69U

/* 两个固定宽度数值的逻辑坐标，用于后续局部刷新。 */
#define THROTTLE_UI_LEFT_VALUE_X        58U
#define THROTTLE_UI_RIGHT_VALUE_X       178U
#define THROTTLE_UI_RAW_LABEL_Y         61U
#define THROTTLE_UI_VALUE_Y             59U

/* 每个RGB565像素按屏幕要求存成“高字节、低字节”。 */
static uint8_t g_throttle_ui_line_buffer[LCD_W * 2U];

static bool ThrottleUI_TextPixel(uint16_t x,
                                 uint16_t y,
                                 uint16_t text_x,
                                 uint16_t text_y,
                                 const char *text,
                                 uint16_t text_length)
{
    uint16_t character_index;
    uint16_t local_x;
    uint16_t local_y;
    unsigned char character;

    if ((text == NULL) || (text_length == 0U) ||
        (x < text_x) || (y < text_y))
    {
        return false;
    }

    local_y = (uint16_t)(y - text_y);
    if (local_y >= THROTTLE_UI_FONT_HEIGHT)
    {
        return false;
    }

    character_index = (uint16_t)((x - text_x) / THROTTLE_UI_FONT_WIDTH);
    if (character_index >= text_length)
    {
        return false;
    }

    local_x = (uint16_t)((x - text_x) % THROTTLE_UI_FONT_WIDTH);
    character = (unsigned char)text[character_index];

    if ((character == '\0') || (character < ' ') || (character > '~'))
    {
        return false;
    }

    return ((ascii_1206[character - ' '][local_y] >> local_x) & 0x01U) != 0U;
}

/**
 * @brief 判断像素是否属于8x16的大号ADC数值。
 *
 * 只给两个四位原始值使用更大的字库，页面标题、标签和返回提示仍保持
 * 6x12，既增强读数可见性，也不会挤压左右两栏的固定布局。
 */
static bool ThrottleUI_ValueTextPixel(uint16_t x,
                                      uint16_t y,
                                      uint16_t text_x,
                                      uint16_t text_y,
                                      const char text[5])
{
    uint16_t character_index;
    uint16_t local_x;
    uint16_t local_y;
    unsigned char character;

    if ((text == NULL) || (x < text_x) || (y < text_y))
    {
        return false;
    }

    local_x = (uint16_t)(x - text_x);
    local_y = (uint16_t)(y - text_y);
    if (local_y >= THROTTLE_UI_VALUE_FONT_HEIGHT)
    {
        return false;
    }

    character_index =
        (uint16_t)(local_x / THROTTLE_UI_VALUE_FONT_WIDTH);
    if (character_index >= 4U)
    {
        return false;
    }

    local_x = (uint16_t)(local_x % THROTTLE_UI_VALUE_FONT_WIDTH);
    character = (unsigned char)text[character_index];
    if ((character < ' ') || (character > '~'))
    {
        return false;
    }

    return ((ascii_1608[character - ' '][local_y] >> local_x) & 0x01U) != 0U;
}

/**
 * @brief 将 12 位 ADC 数值格式化成固定四位十进制字符。
 *
 * 固定宽度可确保局部刷新时新数字完全覆盖旧数字，例如 4095 -> 0007。
 */
static void ThrottleUI_FormatAdcValue(uint16_t value, char text[5])
{
    if (value > 4095U)
    {
        value = 4095U;
    }

    text[0] = (char)('0' + ((value / 1000U) % 10U));
    text[1] = (char)('0' + ((value / 100U) % 10U));
    text[2] = (char)('0' + ((value / 10U) % 10U));
    text[3] = (char)('0' + (value % 10U));
    text[4] = '\0';
}

static uint16_t ThrottleUI_GetLogicalPixel(uint16_t x,
                                           uint16_t y,
                                           const char throttle_text[5],
                                           const char direction_text[5])
{
    uint16_t color = THROTTLE_UI_COLOR_BACKGROUND;

    /* 外框与两轴分区线。 */
    if ((x == 0U) || (x == (THROTTLE_UI_LOGICAL_WIDTH - 1U)) ||
        (y == 0U) || (y == (THROTTLE_UI_LOGICAL_HEIGHT - 1U)))
    {
        color = THROTTLE_UI_COLOR_BORDER;
    }
    if ((y == 27U) || (y == 95U) ||
        (((x == 119U) || (x == 120U)) && (y >= 28U) && (y < 95U)))
    {
        color = THROTTLE_UI_COLOR_DIVIDER;
    }

    if (ThrottleUI_TextPixel(x, y, 93U, 8U, "ADC DEBUG", 9U))
    {
        color = THROTTLE_UI_COLOR_TITLE;
    }
    else if (ThrottleUI_TextPixel(x, y, 36U, 38U, "THROTTLE", 8U) ||
             ThrottleUI_TextPixel(x, y, 153U, 38U, "DIRECTION", 9U) ||
             ThrottleUI_TextPixel(x,
                                  y,
                                  30U,
                                  THROTTLE_UI_RAW_LABEL_Y,
                                  "RAW:",
                                  4U) ||
             ThrottleUI_TextPixel(x,
                                  y,
                                  150U,
                                  THROTTLE_UI_RAW_LABEL_Y,
                                  "RAW:",
                                  4U))
    {
        color = THROTTLE_UI_COLOR_LABEL;
    }
    else if (ThrottleUI_ValueTextPixel(x,
                                       y,
                                       THROTTLE_UI_LEFT_VALUE_X,
                                       THROTTLE_UI_VALUE_Y,
                                       throttle_text) ||
             ThrottleUI_ValueTextPixel(x,
                                       y,
                                       THROTTLE_UI_RIGHT_VALUE_X,
                                       THROTTLE_UI_VALUE_Y,
                                       direction_text))
    {
        color = THROTTLE_UI_COLOR_VALUE;
    }
    else if (ThrottleUI_TextPixel(x,
                                  y,
                                  84U,
                                  81U,
                                  "12BIT 0-4095",
                                  12U) ||
             ThrottleUI_TextPixel(x,
                                  y,
                                  84U,
                                  103U,
                                  "BACK: RETURN",
                                  12U))
    {
        color = THROTTLE_UI_COLOR_HINT;
    }

    return color;
}

/**
 * @brief 把逻辑坐标区域旋转为 LCD 物理坐标并连续发送。
 *
 * 使用与主页面、转向页面一致的横屏旋转关系：
 * logical(x,y) -> physical(119-y,x)。此前油门页采用了相反方向的映射，
 * 导致整页相对其它页面倒置180°。
 * 物理列边界按控制器要求扩展到 4 像素对齐，但每个扩展像素仍重新计算颜色，
 * 不会把局部区域外的内容错误覆盖成纯色。
 */
static void ThrottleUI_DrawLogicalRegion(const ThrottleUiView_t *view,
                                         uint16_t x1,
                                         uint16_t y1,
                                         uint16_t x2,
                                         uint16_t y2)
{
    char throttle_text[5];
    char direction_text[5];
    uint16_t physical_left;
    uint16_t physical_right;
    uint16_t physical_top;
    uint16_t physical_bottom;
    uint16_t physical_y;

    if ((view == NULL) || (x1 > x2) || (y1 > y2) ||
        (x2 >= THROTTLE_UI_LOGICAL_WIDTH) ||
        (y2 >= THROTTLE_UI_LOGICAL_HEIGHT))
    {
        return;
    }

    ThrottleUI_FormatAdcValue(view->throttle_raw, throttle_text);
    ThrottleUI_FormatAdcValue(view->direction_raw, direction_text);


    /**
     * 这是一个旋转坐标变换，控制显示器的显示方向
     */
    physical_left =
        (uint16_t)((THROTTLE_UI_LOGICAL_HEIGHT - 1U) - y2);
    physical_right =
        (uint16_t)((THROTTLE_UI_LOGICAL_HEIGHT - 1U) - y1);
    physical_top = x1;
    physical_bottom = x2;

    physical_left = (uint16_t)(physical_left & (uint16_t)~0x0003U);
    physical_right = (uint16_t)(physical_right | 0x0003U);
    if (physical_right >= LCD_W)
    {
        physical_right = (uint16_t)(LCD_W - 1U);
    }

    LCD_Address_Set(physical_left,
                    physical_top,
                    physical_right,
                    physical_bottom);

    for (physical_y = physical_top; physical_y <= physical_bottom; ++physical_y)
    {
        uint16_t buffer_index = 0U;
        uint16_t physical_x;

        for (physical_x = physical_left; physical_x <= physical_right; ++physical_x)
        {
            uint16_t logical_x = physical_y;
            uint16_t logical_y =
                (uint16_t)((THROTTLE_UI_LOGICAL_HEIGHT - 1U) - physical_x);
            uint16_t color = ThrottleUI_GetLogicalPixel(logical_x,
                                                        logical_y,
                                                        throttle_text,
                                                        direction_text);

            g_throttle_ui_line_buffer[buffer_index++] =
                (uint8_t)(color >> 8U);
            g_throttle_ui_line_buffer[buffer_index++] =
                (uint8_t)(color & 0x00FFU);
        }

        LCD_WriteDataBuffer(g_throttle_ui_line_buffer, buffer_index);
    }
}

void ThrottleUI_Draw(const ThrottleUiView_t *view)
{
    if (view == NULL)
    {
        return;
    }

    ThrottleUI_DrawLogicalRegion(view,
                                 0U,
                                 0U,
                                 (uint16_t)(THROTTLE_UI_LOGICAL_WIDTH - 1U),
                                 (uint16_t)(THROTTLE_UI_LOGICAL_HEIGHT - 1U));
}

void ThrottleUI_UpdateValues(const ThrottleUiView_t *previous,
                             const ThrottleUiView_t *current)
{
    if (current == NULL)
    {
        return;
    }

    if (previous == NULL)
    {
        ThrottleUI_Draw(current);
        return;
    }

    if (previous->throttle_raw != current->throttle_raw)
    {
        ThrottleUI_DrawLogicalRegion(current,
                                     56U,
                                     57U,
                                     91U,
                                     76U);
    }

    if (previous->direction_raw != current->direction_raw)
    {
        ThrottleUI_DrawLogicalRegion(current,
                                     176U,
                                     57U,
                                     211U,
                                     76U);
    }
}
