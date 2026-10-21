#include "joystick_calibration_ui.h"

#include "joystick_calibration_ui_asset.h"
#include "lcd_init.h"
#include "spi1_bus.h"

#include <stdbool.h>
#include <stddef.h>

#define CAL_UI_LOGICAL_WIDTH             240U
#define CAL_UI_LOGICAL_HEIGHT            120U
#define CAL_UI_SMALL_FONT_WIDTH            6U
#define CAL_UI_SMALL_FONT_HEIGHT          12U
#define CAL_UI_ERROR_FONT_WIDTH            8U
#define CAL_UI_ERROR_FONT_HEIGHT          16U

/* 下面坐标直接对应用户提供的240x120参考PNG。 */
#define CAL_UI_HEADER_X                    8U
#define CAL_UI_HEADER_Y                   13U
#define CAL_UI_RAW_TEXT_X                154U
#define CAL_UI_RAW_TEXT_Y                 13U
#define CAL_UI_PROMPT_X                   83U
#define CAL_UI_PROMPT_Y                   32U
#define CAL_UI_CROSS_X                   112U
#define CAL_UI_CROSS_Y                    52U
#define CAL_UI_STABLE_X                   90U
#define CAL_UI_STABLE_Y                   75U
#define CAL_UI_ERROR_X                    72U
#define CAL_UI_ERROR_Y                    73U
#define CAL_UI_INCOMPLETE_X               66U
#define CAL_UI_INCOMPLETE_Y               75U
#define CAL_UI_UP_PROMPT_X                 71U
#define CAL_UI_UP_PROMPT_Y                 32U
#define CAL_UI_UP_ARROW_X                 111U
#define CAL_UI_UP_ARROW_Y                  55U
#define CAL_UI_VALID_TRAVEL_X              96U
#define CAL_UI_VALID_TRAVEL_Y              77U
#define CAL_UI_ESC_TEXT_X                 10U
#define CAL_UI_OK_TEXT_X                 114U
#define CAL_UI_NEXT_TEXT_X               207U
#define CAL_UI_BUTTON_TEXT_Y             102U

#define CAL_UI_COLOR_BACKGROUND          BLACK
#define CAL_UI_COLOR_TEXT                WHITE
#define CAL_UI_COLOR_CROSS               0x1F7EU /* 接近参考图#19F1F8。 */
#define CAL_UI_COLOR_ARROW               0xFE60U /* 参考图黄色箭头。 */
#define CAL_UI_COLOR_SUCCESS             0x0E48U /* 接近参考图#0BC940。 */
#define CAL_UI_COLOR_ERROR               RED
#define CAL_UI_COLOR_WARNING             0xFD20U /* 琥珀色：未完成提示。 */
#define CAL_UI_COLOR_FOCUS               YELLOW
#define CAL_UI_FOCUS_BORDER_WIDTH          2U
#define CAL_UI_FOCUS_RADIUS                4U

typedef struct
{
    uint16_t left;
    uint16_t top;
    uint16_t right;
    uint16_t bottom;
} CalibrationUiRectangle_t;

extern const unsigned char ascii_1206[][12];
extern const unsigned char ascii_1608[][16];

static uint8_t g_calibration_ui_line_buffer[LCD_W * 2U];

static const JoystickCalibrationUiView_t g_calibration_ui_safe_default = {
    .raw_x = 0U,
    .raw_y = 0U,
    .step = JOYSTICK_CALIBRATION_UI_STEP_CENTER,
    .focus = JOYSTICK_CALIBRATION_UI_FOCUS_OK,
    .state = JOYSTICK_CALIBRATION_UI_READY
};

static bool CalibrationUI_PointInRoundedRectangle(
    uint16_t x,
    uint16_t y,
    const CalibrationUiRectangle_t *rectangle,
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

static bool CalibrationUI_PointOnFocusBorder(
    uint16_t x,
    uint16_t y,
    const CalibrationUiRectangle_t *outer)
{
    CalibrationUiRectangle_t inner;

    if (!CalibrationUI_PointInRoundedRectangle(
            x, y, outer, CAL_UI_FOCUS_RADIUS))
    {
        return false;
    }

    inner.left = (uint16_t)(outer->left + CAL_UI_FOCUS_BORDER_WIDTH);
    inner.top = (uint16_t)(outer->top + CAL_UI_FOCUS_BORDER_WIDTH);
    inner.right = (uint16_t)(outer->right - CAL_UI_FOCUS_BORDER_WIDTH);
    inner.bottom = (uint16_t)(outer->bottom - CAL_UI_FOCUS_BORDER_WIDTH);
    return !CalibrationUI_PointInRoundedRectangle(
        x,
        y,
        &inner,
        CAL_UI_FOCUS_RADIUS - CAL_UI_FOCUS_BORDER_WIDTH);
}

static void CalibrationUI_GetFocusRectangle(
    JoystickCalibrationUiFocus_t focus,
    CalibrationUiRectangle_t *rectangle)
{
    if (focus == JOYSTICK_CALIBRATION_UI_FOCUS_ESC)
    {
        rectangle->left = 5U;
        rectangle->right = 39U;
    }
    else if (focus == JOYSTICK_CALIBRATION_UI_FOCUS_OK)
    {
        rectangle->left = 105U;
        rectangle->right = 134U;
    }
    else
    {
        rectangle->left = 195U;
        rectangle->right = 236U;
    }
    rectangle->top = 98U;
    rectangle->bottom = 118U;
}

static bool CalibrationUI_SmallTextPixel(uint16_t x,
                                         uint16_t y,
                                         uint16_t text_x,
                                         uint16_t text_y,
                                         const char *text)
{
    uint16_t local_x;
    uint16_t local_y;
    uint16_t character_index;
    uint16_t text_length = 0U;
    uint8_t character;

    if ((text == NULL) || (x < text_x) || (y < text_y))
    {
        return false;
    }

    local_x = (uint16_t)(x - text_x);
    local_y = (uint16_t)(y - text_y);
    if (local_y >= CAL_UI_SMALL_FONT_HEIGHT)
    {
        return false;
    }

    while (text[text_length] != '\0')
    {
        ++text_length;
    }
    character_index = (uint16_t)(local_x / CAL_UI_SMALL_FONT_WIDTH);
    if (character_index >= text_length)
    {
        return false;
    }

    local_x %= CAL_UI_SMALL_FONT_WIDTH;
    character = (uint8_t)text[character_index];
    if ((character < (uint8_t)' ') || (character > (uint8_t)'~'))
    {
        return false;
    }

    return ((ascii_1206[character - (uint8_t)' '][local_y] >> local_x) &
            0x01U) != 0U;
}

static bool CalibrationUI_ErrorTextPixel(uint16_t x,
                                         uint16_t y,
                                         uint16_t text_x,
                                         uint16_t text_y,
                                         const char *text)
{
    uint16_t local_x;
    uint16_t local_y;
    uint16_t character_index;
    uint16_t text_length = 0U;
    uint8_t character;

    if ((text == NULL) || (x < text_x) || (y < text_y))
    {
        return false;
    }

    local_x = (uint16_t)(x - text_x);
    local_y = (uint16_t)(y - text_y);
    if (local_y >= CAL_UI_ERROR_FONT_HEIGHT)
    {
        return false;
    }

    while (text[text_length] != '\0')
    {
        ++text_length;
    }
    character_index = (uint16_t)(local_x / CAL_UI_ERROR_FONT_WIDTH);
    if (character_index >= text_length)
    {
        return false;
    }

    local_x %= CAL_UI_ERROR_FONT_WIDTH;
    character = (uint8_t)text[character_index];
    if ((character < (uint8_t)' ') || (character > (uint8_t)'~'))
    {
        return false;
    }

    return ((ascii_1608[character - (uint8_t)' '][local_y] >> local_x) &
            0x01U) != 0U;
}

static void CalibrationUI_FormatAdcValue(uint16_t value, char output[4])
{
    output[0] = (value >= 1000U) ? (char)('0' + (value / 1000U)) : ' ';
    output[1] = (value >= 100U)
                    ? (char)('0' + ((value / 100U) % 10U))
                    : ' ';
    output[2] = (value >= 10U)
                    ? (char)('0' + ((value / 10U) % 10U))
                    : ' ';
    output[3] = (char)('0' + (value % 10U));
}

static void CalibrationUI_FormatRawText(
    const JoystickCalibrationUiView_t *view,
    char text[14])
{
    char x_value[4];
    char y_value[4];

    CalibrationUI_FormatAdcValue(view->raw_x, x_value);
    CalibrationUI_FormatAdcValue(view->raw_y, y_value);
    text[0] = 'X';
    text[1] = ':';
    text[2] = x_value[0];
    text[3] = x_value[1];
    text[4] = x_value[2];
    text[5] = x_value[3];
    text[6] = ' ';
    text[7] = 'Y';
    text[8] = ':';
    text[9] = y_value[0];
    text[10] = y_value[1];
    text[11] = y_value[2];
    text[12] = y_value[3];
    text[13] = '\0';
}

static uint8_t CalibrationUI_GetCoverage(const uint8_t *asset,
                                         uint16_t row_bytes,
                                         uint16_t x,
                                         uint16_t y)
{
    const uint8_t packed = asset[((uint32_t)y * row_bytes) + (x / 2U)];
    return ((x & 1U) == 0U) ? (uint8_t)(packed >> 4U)
                            : (uint8_t)(packed & 0x0FU);
}

static bool CalibrationUI_IsDirectionStep(JoystickCalibrationUiStep_t step)
{
    return (step >= JOYSTICK_CALIBRATION_UI_STEP_UP) &&
           (step <= JOYSTICK_CALIBRATION_UI_STEP_RIGHT);
}

static const char *CalibrationUI_GetHeaderText(
    JoystickCalibrationUiStep_t step)
{
    switch (step)
    {
        case JOYSTICK_CALIBRATION_UI_STEP_UP:
            return "CAL:2/5";
        case JOYSTICK_CALIBRATION_UI_STEP_DOWN:
            return "CAL:3/5";
        case JOYSTICK_CALIBRATION_UI_STEP_LEFT:
            return "CAL:4/5";
        case JOYSTICK_CALIBRATION_UI_STEP_RIGHT:
            return "CAL:5/5";
        case JOYSTICK_CALIBRATION_UI_STEP_CENTER:
        default:
            return "CAL:1/5";
    }
}

/** @brief 用第二个16x16字形替换“向上拨至极限”中的“上”。 */
static uint8_t CalibrationUI_GetDirectionPromptCoverage(
    JoystickCalibrationUiStep_t step,
    uint16_t x,
    uint16_t y)
{
    const uint8_t *direction_glyph = NULL;

    if (step == JOYSTICK_CALIBRATION_UI_STEP_DOWN)
    {
        direction_glyph = g_joystick_calibration_down_glyph_4bpp;
    }
    else if (step == JOYSTICK_CALIBRATION_UI_STEP_LEFT)
    {
        direction_glyph = g_joystick_calibration_left_glyph_4bpp;
    }
    else if (step == JOYSTICK_CALIBRATION_UI_STEP_RIGHT)
    {
        direction_glyph = g_joystick_calibration_right_glyph_4bpp;
    }

    if ((direction_glyph != NULL) && (x >= 16U) && (x < 32U))
    {
        return CalibrationUI_GetCoverage(
            direction_glyph,
            JOYSTICK_CALIBRATION_DIRECTION_GLYPH_ROW_BYTES,
            (uint16_t)(x - 16U),
            y);
    }

    return CalibrationUI_GetCoverage(
        g_joystick_calibration_up_prompt_4bpp,
        JOYSTICK_CALIBRATION_UP_PROMPT_ROW_BYTES,
        x,
        y);
}

/**
 * @brief 从唯一的向上箭头资源按步骤反向取样，实现0/90/180/270度旋转。
 */
static uint8_t CalibrationUI_GetDirectionArrowCoverage(
    JoystickCalibrationUiStep_t step,
    uint16_t x,
    uint16_t y)
{
    uint16_t source_x = x;
    uint16_t source_y = y;
    const uint16_t maximum_coordinate =
        JOYSTICK_CALIBRATION_UP_ARROW_WIDTH - 1U;

    if (step == JOYSTICK_CALIBRATION_UI_STEP_DOWN)
    {
        source_x = (uint16_t)(maximum_coordinate - x);
        source_y = (uint16_t)(maximum_coordinate - y);
    }
    else if (step == JOYSTICK_CALIBRATION_UI_STEP_LEFT)
    {
        /* 向上箭头顺时针旋转270度。 */
        source_x = (uint16_t)(maximum_coordinate - y);
        source_y = x;
    }
    else if (step == JOYSTICK_CALIBRATION_UI_STEP_RIGHT)
    {
        /* 向上箭头顺时针旋转90度。 */
        source_x = y;
        source_y = (uint16_t)(maximum_coordinate - x);
    }

    return CalibrationUI_GetCoverage(
        g_joystick_calibration_up_arrow_4bpp,
        JOYSTICK_CALIBRATION_UP_ARROW_ROW_BYTES,
        source_x,
        source_y);
}

/** @brief 读取逐行、MSB优先的1位透明字模。 */
static bool CalibrationUI_GetMaskPixel(const uint8_t *asset,
                                       uint16_t row_bytes,
                                       uint16_t x,
                                       uint16_t y)
{
    const uint8_t packed = asset[((uint32_t)y * row_bytes) + (x / 8U)];
    return ((packed >> (7U - (x & 7U))) & 0x01U) != 0U;
}

/** @brief 在黑色背景上按0~15覆盖率混合一个RGB565前景色。 */
static uint16_t CalibrationUI_ApplyCoverage(uint16_t foreground,
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

static uint16_t CalibrationUI_GetLogicalPixel(
    uint16_t x,
    uint16_t y,
    const JoystickCalibrationUiView_t *view,
    const char raw_text[14])
{
    CalibrationUiRectangle_t focus_rectangle;
    const bool is_direction_step =
        CalibrationUI_IsDirectionStep(view->step);
    const bool highlights_y =
        (view->step == JOYSTICK_CALIBRATION_UI_STEP_UP) ||
        (view->step == JOYSTICK_CALIBRATION_UI_STEP_DOWN);
    const bool highlights_x =
        (view->step == JOYSTICK_CALIBRATION_UI_STEP_LEFT) ||
        (view->step == JOYSTICK_CALIBRATION_UI_STEP_RIGHT);
    const bool show_incomplete =
        (view->state == JOYSTICK_CALIBRATION_UI_INCOMPLETE) ||
        (view->state == JOYSTICK_CALIBRATION_UI_READY) ||
        (is_direction_step &&
         (view->state == JOYSTICK_CALIBRATION_UI_SAMPLING));
    const char *const header_text =
        CalibrationUI_GetHeaderText(view->step);
    uint16_t color = CAL_UI_COLOR_BACKGROUND;
    uint8_t coverage;

    if (CalibrationUI_SmallTextPixel(
            x, y, CAL_UI_HEADER_X, CAL_UI_HEADER_Y, header_text) ||
        CalibrationUI_SmallTextPixel(
            x, y, CAL_UI_RAW_TEXT_X, CAL_UI_RAW_TEXT_Y, raw_text) ||
        CalibrationUI_SmallTextPixel(
            x, y, CAL_UI_ESC_TEXT_X, CAL_UI_BUTTON_TEXT_Y, "ESC") ||
        CalibrationUI_SmallTextPixel(
            x, y, CAL_UI_OK_TEXT_X, CAL_UI_BUTTON_TEXT_Y, "OK") ||
        CalibrationUI_SmallTextPixel(
            x, y, CAL_UI_NEXT_TEXT_X, CAL_UI_BUTTON_TEXT_Y, "NEXT"))
    {
        color = CAL_UI_COLOR_TEXT;
    }

    /* CAL:2/5、CAL:3/5校准Y轴：仅把顶部“Y:数值”标成结果绿色。 */
    if (highlights_y &&
        CalibrationUI_SmallTextPixel(
            x,
            y,
            (uint16_t)(CAL_UI_RAW_TEXT_X +
                       (7U * CAL_UI_SMALL_FONT_WIDTH)),
            CAL_UI_RAW_TEXT_Y,
            &raw_text[7]))
    {
        color = CAL_UI_COLOR_SUCCESS;
    }
    else if (highlights_x)
    {
        /* 先把整段设绿，再把Y字段恢复白色，最终只有X字段高亮。 */
        if (CalibrationUI_SmallTextPixel(
                x, y, CAL_UI_RAW_TEXT_X, CAL_UI_RAW_TEXT_Y, raw_text))
        {
            color = CAL_UI_COLOR_SUCCESS;
        }
        if (CalibrationUI_SmallTextPixel(
                x,
                y,
                (uint16_t)(CAL_UI_RAW_TEXT_X +
                           (7U * CAL_UI_SMALL_FONT_WIDTH)),
                CAL_UI_RAW_TEXT_Y,
                &raw_text[7]))
        {
            color = CAL_UI_COLOR_TEXT;
        }
    }

    if (!is_direction_step &&
        (x >= CAL_UI_PROMPT_X) &&
        (x < (uint16_t)(CAL_UI_PROMPT_X +
                        JOYSTICK_CALIBRATION_PROMPT_WIDTH)) &&
        (y >= CAL_UI_PROMPT_Y) &&
        (y < (uint16_t)(CAL_UI_PROMPT_Y +
                        JOYSTICK_CALIBRATION_PROMPT_HEIGHT)))
    {
        coverage = CalibrationUI_GetCoverage(
            g_joystick_calibration_prompt_4bpp,
            JOYSTICK_CALIBRATION_PROMPT_ROW_BYTES,
            (uint16_t)(x - CAL_UI_PROMPT_X),
            (uint16_t)(y - CAL_UI_PROMPT_Y));
        if (coverage != 0U)
        {
            color = CalibrationUI_ApplyCoverage(
                CAL_UI_COLOR_TEXT, coverage);
        }
    }
    else if (is_direction_step &&
             (x >= CAL_UI_UP_PROMPT_X) &&
             (x < (uint16_t)(CAL_UI_UP_PROMPT_X +
                             JOYSTICK_CALIBRATION_UP_PROMPT_WIDTH)) &&
             (y >= CAL_UI_UP_PROMPT_Y) &&
             (y < (uint16_t)(CAL_UI_UP_PROMPT_Y +
                             JOYSTICK_CALIBRATION_UP_PROMPT_HEIGHT)))
    {
        coverage = CalibrationUI_GetDirectionPromptCoverage(
            view->step,
            (uint16_t)(x - CAL_UI_UP_PROMPT_X),
            (uint16_t)(y - CAL_UI_UP_PROMPT_Y));
        if (coverage != 0U)
        {
            color = CalibrationUI_ApplyCoverage(
                CAL_UI_COLOR_TEXT, coverage);
        }
    }
    if ((view->state == JOYSTICK_CALIBRATION_UI_FAILED) &&
        CalibrationUI_ErrorTextPixel(
            x, y, CAL_UI_ERROR_X, CAL_UI_ERROR_Y, "Try it again"))
    {
        color = CAL_UI_COLOR_ERROR;
    }

    if (!is_direction_step &&
        (x >= CAL_UI_CROSS_X) &&
        (x < (uint16_t)(CAL_UI_CROSS_X + JOYSTICK_CALIBRATION_CROSS_WIDTH)) &&
        (y >= CAL_UI_CROSS_Y) &&
        (y < (uint16_t)(CAL_UI_CROSS_Y + JOYSTICK_CALIBRATION_CROSS_HEIGHT)))
    {
        coverage = CalibrationUI_GetCoverage(
            g_joystick_calibration_cross_4bpp,
            JOYSTICK_CALIBRATION_CROSS_ROW_BYTES,
            (uint16_t)(x - CAL_UI_CROSS_X),
            (uint16_t)(y - CAL_UI_CROSS_Y));
        if (coverage != 0U)
        {
            color = CalibrationUI_ApplyCoverage(
                CAL_UI_COLOR_CROSS, coverage);
        }
    }
    else if (is_direction_step &&
             (x >= CAL_UI_UP_ARROW_X) &&
             (x < (uint16_t)(CAL_UI_UP_ARROW_X +
                             JOYSTICK_CALIBRATION_UP_ARROW_WIDTH)) &&
             (y >= CAL_UI_UP_ARROW_Y) &&
             (y < (uint16_t)(CAL_UI_UP_ARROW_Y +
                             JOYSTICK_CALIBRATION_UP_ARROW_HEIGHT)))
    {
        coverage = CalibrationUI_GetDirectionArrowCoverage(
            view->step,
            (uint16_t)(x - CAL_UI_UP_ARROW_X),
            (uint16_t)(y - CAL_UI_UP_ARROW_Y));
        if (coverage != 0U)
        {
            color = CalibrationUI_ApplyCoverage(
                CAL_UI_COLOR_ARROW, coverage);
        }
    }

    if (!is_direction_step &&
        (view->state == JOYSTICK_CALIBRATION_UI_SUCCEEDED) &&
        (x >= CAL_UI_STABLE_X) &&
        (x < (uint16_t)(CAL_UI_STABLE_X +
                        JOYSTICK_CALIBRATION_STABLE_WIDTH)) &&
        (y >= CAL_UI_STABLE_Y) &&
        (y < (uint16_t)(CAL_UI_STABLE_Y +
                        JOYSTICK_CALIBRATION_STABLE_HEIGHT)))
    {
        coverage = CalibrationUI_GetCoverage(
            g_joystick_calibration_stable_4bpp,
            JOYSTICK_CALIBRATION_STABLE_ROW_BYTES,
            (uint16_t)(x - CAL_UI_STABLE_X),
            (uint16_t)(y - CAL_UI_STABLE_Y));
        if (coverage != 0U)
        {
            color = CalibrationUI_ApplyCoverage(
                CAL_UI_COLOR_SUCCESS, coverage);
        }
    }
    else if (is_direction_step &&
             (view->state == JOYSTICK_CALIBRATION_UI_SUCCEEDED) &&
             (x >= CAL_UI_VALID_TRAVEL_X) &&
             (x < (uint16_t)(CAL_UI_VALID_TRAVEL_X +
                             JOYSTICK_CALIBRATION_VALID_TRAVEL_WIDTH)) &&
             (y >= CAL_UI_VALID_TRAVEL_Y) &&
             (y < (uint16_t)(CAL_UI_VALID_TRAVEL_Y +
                             JOYSTICK_CALIBRATION_VALID_TRAVEL_HEIGHT)))
    {
        coverage = CalibrationUI_GetCoverage(
            g_joystick_calibration_valid_travel_4bpp,
            JOYSTICK_CALIBRATION_VALID_TRAVEL_ROW_BYTES,
            (uint16_t)(x - CAL_UI_VALID_TRAVEL_X),
            (uint16_t)(y - CAL_UI_VALID_TRAVEL_Y));
        if (coverage != 0U)
        {
            color = CalibrationUI_ApplyCoverage(
                CAL_UI_COLOR_SUCCESS, coverage);
        }
    }
    else if (show_incomplete &&
             (x >= CAL_UI_INCOMPLETE_X) &&
             (x < (uint16_t)(CAL_UI_INCOMPLETE_X +
                             JOYSTICK_CALIBRATION_INCOMPLETE_WIDTH)) &&
             (y >= CAL_UI_INCOMPLETE_Y) &&
             (y < (uint16_t)(CAL_UI_INCOMPLETE_Y +
                             JOYSTICK_CALIBRATION_INCOMPLETE_HEIGHT)) &&
             CalibrationUI_GetMaskPixel(
                 g_joystick_calibration_incomplete_1bpp,
                 JOYSTICK_CALIBRATION_INCOMPLETE_ROW_BYTES,
                 (uint16_t)(x - CAL_UI_INCOMPLETE_X),
                 (uint16_t)(y - CAL_UI_INCOMPLETE_Y)))
    {
        color = CAL_UI_COLOR_WARNING;
    }

    CalibrationUI_GetFocusRectangle(view->focus, &focus_rectangle);
    if (CalibrationUI_PointOnFocusBorder(x, y, &focus_rectangle))
    {
        color = CAL_UI_COLOR_FOCUS;
    }

    return color;
}

static void CalibrationUI_DrawLogicalRegion(
    const JoystickCalibrationUiView_t *view,
    uint16_t logical_left,
    uint16_t logical_top,
    uint16_t logical_right,
    uint16_t logical_bottom)
{
    const JoystickCalibrationUiView_t *const active_view =
        (view != NULL) ? view : &g_calibration_ui_safe_default;
    char raw_text[14];
    uint16_t physical_left;
    uint16_t physical_right;
    uint16_t physical_top;
    uint16_t physical_bottom;

    if ((logical_left > logical_right) || (logical_top > logical_bottom) ||
        (logical_left >= CAL_UI_LOGICAL_WIDTH) ||
        (logical_top >= CAL_UI_LOGICAL_HEIGHT))
    {
        return;
    }

    if (logical_right >= CAL_UI_LOGICAL_WIDTH)
    {
        logical_right = CAL_UI_LOGICAL_WIDTH - 1U;
    }
    if (logical_bottom >= CAL_UI_LOGICAL_HEIGHT)
    {
        logical_bottom = CAL_UI_LOGICAL_HEIGHT - 1U;
    }

    CalibrationUI_FormatRawText(active_view, raw_text);
    physical_left = (CAL_UI_LOGICAL_HEIGHT - 1U) - logical_bottom;
    physical_right = (CAL_UI_LOGICAL_HEIGHT - 1U) - logical_top;
    physical_top = logical_left;
    physical_bottom = logical_right;

    /* SH8501A物理列窗口按4像素对齐，扩展部分仍由本页面正确重建。 */
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
                (CAL_UI_LOGICAL_HEIGHT - 1U) - physical_x;
            const uint16_t pixel = CalibrationUI_GetLogicalPixel(
                logical_x, logical_y, active_view, raw_text);

            g_calibration_ui_line_buffer[buffer_index++] =
                (uint8_t)(pixel >> 8U);
            g_calibration_ui_line_buffer[buffer_index++] = (uint8_t)pixel;
        }

        LCD_WriteDataBuffer(g_calibration_ui_line_buffer, buffer_index);
    }

    SPI1_Bus_Release();
}

void JoystickCalibrationUI_Draw(const JoystickCalibrationUiView_t *view)
{
    CalibrationUI_DrawLogicalRegion(view,
                                    0U,
                                    0U,
                                    CAL_UI_LOGICAL_WIDTH - 1U,
                                    CAL_UI_LOGICAL_HEIGHT - 1U);
}

void JoystickCalibrationUI_Update(
    const JoystickCalibrationUiView_t *previous,
    const JoystickCalibrationUiView_t *current)
{
    if ((previous == NULL) || (current == NULL))
    {
        JoystickCalibrationUI_Draw(current);
        return;
    }

    if (previous->step != current->step)
    {
        /* 步骤切换会改变标题、说明、图标和结果文字，必须完整重建。 */
        JoystickCalibrationUI_Draw(current);
        return;
    }

    if ((previous->raw_x != current->raw_x) ||
        (previous->raw_y != current->raw_y))
    {
        CalibrationUI_DrawLogicalRegion(current, 150U, 12U, 239U, 26U);
    }

    if (previous->state != current->state)
    {
        /*
         * 固定说明、十字及三种互斥状态文字共用该区域；用统一范围
         * 恢复旧状态，避免中英文长度不同导致残留像素。
         */
        CalibrationUI_DrawLogicalRegion(current, 55U, 30U, 185U, 90U);
    }

    if (previous->focus != current->focus)
    {
        /* 同时恢复旧框并绘制新框，避免三按钮周围留下黄色像素。 */
        CalibrationUI_DrawLogicalRegion(current, 0U, 97U, 239U, 119U);
    }
}
