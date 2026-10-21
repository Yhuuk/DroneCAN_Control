#include "joystick_calibration_result_ui.h"

#include "lcd_init.h"
#include "spi1_bus.h"

#include <stddef.h>
#include <stdint.h>

#define RESULT_UI_WIDTH                 240U
#define RESULT_UI_HEIGHT                120U
#define RESULT_UI_FONT_WIDTH              6U
#define RESULT_UI_FONT_HEIGHT            12U

/* 坐标按用户提供的240x120 SVG布置：左右双栏、20像素行距。 */
#define RESULT_UI_LEFT_LABEL_X            7U
#define RESULT_UI_LEFT_VALUE_X           77U
#define RESULT_UI_RIGHT_LABEL_X         133U
#define RESULT_UI_RIGHT_VALUE_X         203U
#define RESULT_UI_ROW_MIN_Y              24U
#define RESULT_UI_ROW_CENTER_Y           44U
#define RESULT_UI_ROW_MAX_Y              64U
#define RESULT_UI_ROW_DEAD_Y             84U
#define RESULT_UI_BUTTON_Y              104U

#define RESULT_UI_COLOR_BACKGROUND      BLACK
#define RESULT_UI_COLOR_TEXT            WHITE
#define RESULT_UI_COLOR_AXIS            0x3F3FU /* 参考图#39E7FF。 */
#define RESULT_UI_COLOR_LABEL           0xF401U /* 参考图#F4800C。 */
#define RESULT_UI_COLOR_DIVIDER         0x26DEU /* 参考图#22D8F3。 */
#define RESULT_UI_COLOR_CANCEL          0xE147U /* 参考图#E42B3D。 */
#define RESULT_UI_COLOR_SAVE            0xFF21U /* 参考图#FEE50A。 */
#define RESULT_UI_COLOR_SAVED           0x4741U /* 参考图#45EA0D。 */
#define RESULT_UI_COLOR_FOCUS           0x3F3FU

typedef struct
{
    uint16_t left;
    uint16_t top;
    uint16_t right;
    uint16_t bottom;
} ResultUiRectangle_t;

extern const unsigned char ascii_1206[][12];

static uint8_t g_result_ui_line_buffer[LCD_W * 2U];

/* Microsoft YaHei 12 px生成的“轴”4位覆盖率字形，12x12、两个像素/字节。 */
static const uint8_t g_result_ui_axis_glyph_4bpp[72U] = {
    0x00U, 0xC1U, 0x00U, 0x03U, 0xA0U, 0x00U,
    0x02U, 0xB0U, 0x00U, 0x03U, 0xA0U, 0x00U,
    0x8EU, 0xDCU, 0x8CU, 0xDDU, 0xECU, 0xE4U,
    0x0CU, 0x10U, 0x0CU, 0x13U, 0xA0U, 0x94U,
    0x58U, 0x76U, 0x0CU, 0x13U, 0xA0U, 0x94U,
    0x7CU, 0xDDU, 0x6CU, 0xCDU, 0xECU, 0xE4U,
    0x00U, 0x76U, 0x0CU, 0x13U, 0xA0U, 0x94U,
    0xADU, 0xEDU, 0x8CU, 0x13U, 0xA0U, 0x94U,
    0x00U, 0x76U, 0x0CU, 0x13U, 0xA0U, 0x94U,
    0x00U, 0x76U, 0x0CU, 0xDCU, 0xCCU, 0xE4U,
    0x00U, 0x76U, 0x0CU, 0x10U, 0x00U, 0x94U,
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U
};

static const JoystickCalibrationResultUiView_t g_result_ui_safe_default = {
    .x_axis = {.minimum = 0U, .center = 2048U, .maximum = 4095U,
               .dead_zone = JOYSTICK_DEFAULT_DIRECTION_DEAD_ZONE},
    .y_axis = {.minimum = 0U, .center = 2048U, .maximum = 4095U,
               .dead_zone = JOYSTICK_DEFAULT_THROTTLE_DEAD_ZONE},
    .focus = JOYSTICK_CALIBRATION_RESULT_FOCUS_SAVE,
    .saved = false
};

static bool ResultUI_TextPixel(uint16_t x,
                               uint16_t y,
                               uint16_t text_x,
                               uint16_t text_y,
                               const char *text)
{
    uint16_t length = 0U;
    uint16_t local_x;
    uint16_t local_y;
    uint16_t character_index;
    uint8_t character;

    if ((text == NULL) || (x < text_x) || (y < text_y))
    {
        return false;
    }

    local_x = (uint16_t)(x - text_x);
    local_y = (uint16_t)(y - text_y);
    if (local_y >= RESULT_UI_FONT_HEIGHT)
    {
        return false;
    }

    while (text[length] != '\0')
    {
        ++length;
    }

    character_index = (uint16_t)(local_x / RESULT_UI_FONT_WIDTH);
    if (character_index >= length)
    {
        return false;
    }

    local_x %= RESULT_UI_FONT_WIDTH;
    character = (uint8_t)text[character_index];
    if ((character < (uint8_t)' ') || (character > (uint8_t)'~'))
    {
        return false;
    }

    return ((ascii_1206[character - (uint8_t)' '][local_y] >> local_x) &
            0x01U) != 0U;
}

static void ResultUI_FormatValue(uint16_t value, char output[5])
{
    output[0] = (value >= 1000U) ? (char)('0' + (value / 1000U)) : ' ';
    output[1] = (value >= 100U)
                    ? (char)('0' + ((value / 100U) % 10U))
                    : ' ';
    output[2] = (value >= 10U)
                    ? (char)('0' + ((value / 10U) % 10U))
                    : ' ';
    output[3] = (char)('0' + (value % 10U));
    output[4] = '\0';
}

static uint8_t ResultUI_GetAxisGlyphCoverage(uint16_t x, uint16_t y)
{
    const uint8_t packed =
        g_result_ui_axis_glyph_4bpp[((uint32_t)y * 6U) + (x / 2U)];
    return ((x & 1U) == 0U) ? (uint8_t)(packed >> 4U)
                            : (uint8_t)(packed & 0x0FU);
}

static uint16_t ResultUI_ApplyCoverage(uint16_t foreground,
                                       uint8_t coverage)
{
    const uint16_t red =
        (uint16_t)((((foreground >> 11U) & 0x1FU) * coverage) / 15U);
    const uint16_t green =
        (uint16_t)((((foreground >> 5U) & 0x3FU) * coverage) / 15U);
    const uint16_t blue =
        (uint16_t)(((foreground & 0x1FU) * coverage) / 15U);
    return (uint16_t)((red << 11U) | (green << 5U) | blue);
}

static bool ResultUI_PointInRoundedRectangle(
    uint16_t x,
    uint16_t y,
    const ResultUiRectangle_t *rectangle,
    uint16_t radius)
{
    uint16_t corner_x;
    uint16_t corner_y;
    int32_t dx;
    int32_t dy;

    if ((x < rectangle->left) || (x > rectangle->right) ||
        (y < rectangle->top) || (y > rectangle->bottom))
    {
        return false;
    }

    if (((x >= (uint16_t)(rectangle->left + radius)) &&
         (x <= (uint16_t)(rectangle->right - radius))) ||
        ((y >= (uint16_t)(rectangle->top + radius)) &&
         (y <= (uint16_t)(rectangle->bottom - radius))))
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

static bool ResultUI_PointOnFocusBorder(
    uint16_t x,
    uint16_t y,
    JoystickCalibrationResultFocus_t focus)
{
    ResultUiRectangle_t outer;
    ResultUiRectangle_t inner;

    if (focus == JOYSTICK_CALIBRATION_RESULT_FOCUS_CANCEL)
    {
        outer.left = 7U;
        outer.right = 59U;
    }
    else
    {
        outer.left = 191U;
        outer.right = 230U;
    }
    outer.top = 102U;
    outer.bottom = 118U;

    if (!ResultUI_PointInRoundedRectangle(x, y, &outer, 3U))
    {
        return false;
    }

    inner.left = (uint16_t)(outer.left + 1U);
    inner.top = (uint16_t)(outer.top + 1U);
    inner.right = (uint16_t)(outer.right - 1U);
    inner.bottom = (uint16_t)(outer.bottom - 1U);
    return !ResultUI_PointInRoundedRectangle(x, y, &inner, 2U);
}

static uint16_t ResultUI_GetLogicalPixel(
    uint16_t x,
    uint16_t y,
    const JoystickCalibrationResultUiView_t *view,
    const char x_values[4][5],
    const char y_values[4][5])
{
    static const char *const labels[4] = {"MIN", "CEN", "MAX", "DEAD"};
    static const uint16_t row_y[4] = {
        RESULT_UI_ROW_MIN_Y,
        RESULT_UI_ROW_CENTER_Y,
        RESULT_UI_ROW_MAX_Y,
        RESULT_UI_ROW_DEAD_Y
    };
    uint16_t color = RESULT_UI_COLOR_BACKGROUND;
    uint8_t coverage;

    /* 标题：ASCII轴名和12x12中文“轴”组成参考图的“X轴/Y轴”。 */
    if (ResultUI_TextPixel(x, y, 44U, 6U, "X") ||
        ResultUI_TextPixel(x, y, 170U, 6U, "Y"))
    {
        color = RESULT_UI_COLOR_AXIS;
    }
    if ((x >= 51U) && (x < 63U) && (y >= 6U) && (y < 18U))
    {
        coverage = ResultUI_GetAxisGlyphCoverage(
            (uint16_t)(x - 51U), (uint16_t)(y - 6U));
        if (coverage != 0U)
        {
            color = ResultUI_ApplyCoverage(RESULT_UI_COLOR_AXIS, coverage);
        }
    }
    if ((x >= 177U) && (x < 189U) && (y >= 6U) && (y < 18U))
    {
        coverage = ResultUI_GetAxisGlyphCoverage(
            (uint16_t)(x - 177U), (uint16_t)(y - 6U));
        if (coverage != 0U)
        {
            color = ResultUI_ApplyCoverage(RESULT_UI_COLOR_AXIS, coverage);
        }
    }

    for (uint8_t row = 0U; row < 4U; ++row)
    {
        if (ResultUI_TextPixel(x, y,
                               RESULT_UI_LEFT_LABEL_X, row_y[row],
                               labels[row]) ||
            ResultUI_TextPixel(x, y,
                               RESULT_UI_RIGHT_LABEL_X, row_y[row],
                               labels[row]))
        {
            color = RESULT_UI_COLOR_LABEL;
        }
        if (ResultUI_TextPixel(x, y,
                               RESULT_UI_LEFT_VALUE_X, row_y[row],
                               x_values[row]) ||
            ResultUI_TextPixel(x, y,
                               RESULT_UI_RIGHT_VALUE_X, row_y[row],
                               y_values[row]))
        {
            color = RESULT_UI_COLOR_TEXT;
        }
    }

    /* 参考图中部和按钮区的青色分割线。 */
    if (((x == 119U) && (y <= 100U)) || (y == 100U))
    {
        color = RESULT_UI_COLOR_DIVIDER;
    }

    if (ResultUI_TextPixel(x, y, 13U, RESULT_UI_BUTTON_Y, "CANCEL"))
    {
        color = RESULT_UI_COLOR_CANCEL;
    }
    if (ResultUI_TextPixel(x, y, 198U, RESULT_UI_BUTTON_Y, "SAVE"))
    {
        color = view->saved ? RESULT_UI_COLOR_SAVED
                            : RESULT_UI_COLOR_SAVE;
    }

    /* 焦点采用与分割线同色的1像素圆角框，不改变参考图按钮自身颜色。 */
    if (ResultUI_PointOnFocusBorder(x, y, view->focus))
    {
        color = RESULT_UI_COLOR_FOCUS;
    }

    return color;
}

static void ResultUI_DrawLogicalRegion(
    const JoystickCalibrationResultUiView_t *view,
    uint16_t logical_left,
    uint16_t logical_top,
    uint16_t logical_right,
    uint16_t logical_bottom)
{
    const JoystickCalibrationResultUiView_t *const active_view =
        (view != NULL) ? view : &g_result_ui_safe_default;
    char x_values[4][5];
    char y_values[4][5];
    uint16_t physical_left;
    uint16_t physical_right;
    uint16_t physical_top;
    uint16_t physical_bottom;

    if ((logical_left > logical_right) || (logical_top > logical_bottom) ||
        (logical_left >= RESULT_UI_WIDTH) ||
        (logical_top >= RESULT_UI_HEIGHT))
    {
        return;
    }
    if (logical_right >= RESULT_UI_WIDTH)
    {
        logical_right = RESULT_UI_WIDTH - 1U;
    }
    if (logical_bottom >= RESULT_UI_HEIGHT)
    {
        logical_bottom = RESULT_UI_HEIGHT - 1U;
    }

    ResultUI_FormatValue(active_view->x_axis.minimum, x_values[0]);
    ResultUI_FormatValue(active_view->x_axis.center, x_values[1]);
    ResultUI_FormatValue(active_view->x_axis.maximum, x_values[2]);
    ResultUI_FormatValue(active_view->x_axis.dead_zone, x_values[3]);
    ResultUI_FormatValue(active_view->y_axis.minimum, y_values[0]);
    ResultUI_FormatValue(active_view->y_axis.center, y_values[1]);
    ResultUI_FormatValue(active_view->y_axis.maximum, y_values[2]);
    ResultUI_FormatValue(active_view->y_axis.dead_zone, y_values[3]);

    physical_left = (RESULT_UI_HEIGHT - 1U) - logical_bottom;
    physical_right = (RESULT_UI_HEIGHT - 1U) - logical_top;
    physical_top = logical_left;
    physical_bottom = logical_right;
    physical_left &= (uint16_t)~3U;
    physical_right |= 3U;
    if (physical_right >= LCD_W)
    {
        physical_right = LCD_W - 1U;
    }

    if (SPI1_Bus_Acquire() != HAL_OK)
    {
        Error_Handler();
        return;
    }

    LCD_Address_Set(physical_left, physical_top,
                    physical_right, physical_bottom);
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
                (RESULT_UI_HEIGHT - 1U) - physical_x;
            const uint16_t pixel = ResultUI_GetLogicalPixel(
                logical_x, logical_y, active_view, x_values, y_values);

            g_result_ui_line_buffer[buffer_index++] =
                (uint8_t)(pixel >> 8U);
            g_result_ui_line_buffer[buffer_index++] = (uint8_t)pixel;
        }
        LCD_WriteDataBuffer(g_result_ui_line_buffer, buffer_index);
    }

    SPI1_Bus_Release();
}

void JoystickCalibrationResultUI_Draw(
    const JoystickCalibrationResultUiView_t *view)
{
    ResultUI_DrawLogicalRegion(view,
                               0U, 0U,
                               RESULT_UI_WIDTH - 1U,
                               RESULT_UI_HEIGHT - 1U);
}

void JoystickCalibrationResultUI_Update(
    const JoystickCalibrationResultUiView_t *previous,
    const JoystickCalibrationResultUiView_t *current)
{
    if ((previous == NULL) || (current == NULL))
    {
        JoystickCalibrationResultUI_Draw(current);
        return;
    }

    if ((previous->focus != current->focus) ||
        (previous->saved != current->saved))
    {
        ResultUI_DrawLogicalRegion(current, 0U, 101U, 239U, 119U);
    }
}
