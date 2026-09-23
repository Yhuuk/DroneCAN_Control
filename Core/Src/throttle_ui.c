#include "throttle_ui.h"

#include "lcd_init.h"
#include "throttle_ui_asset.h"
#include "spi1_bus.h"
#include "ui_static_asset.h"

#include <stddef.h>
#include <string.h>

/* 用户画布与其它页面统一使用240x120横屏逻辑坐标。 */
#define THROTTLE_UI_LOGICAL_WIDTH   240U
#define THROTTLE_UI_LOGICAL_HEIGHT  120U

#define THROTTLE_UI_SMALL_FONT_WIDTH   6U
#define THROTTLE_UI_SMALL_FONT_HEIGHT 12U
#define THROTTLE_UI_MEDIUM_FONT_WIDTH  8U
#define THROTTLE_UI_MEDIUM_FONT_HEIGHT 16U
#define THROTTLE_UI_LARGE_FONT_WIDTH  12U
#define THROTTLE_UI_LARGE_FONT_HEIGHT 24U

#define THROTTLE_UI_LOCK_X              211U
#define THROTTLE_UI_LOCK_Y                0U
#define THROTTLE_UI_LOCKED_WIDTH         18U
#define THROTTLE_UI_UNLOCKED_WIDTH       27U
#define THROTTLE_UI_LOCK_HEIGHT          23U

/*
 * 电机焦点框左右及上方扩大到5像素；下方保持4像素，避免第二排边框
 * 与Y=96开始的THR区域相碰。较大圆角让边框转折更平滑。
 */
#define THROTTLE_UI_MOTOR_FOCUS_PADDING         5U
#define THROTTLE_UI_MOTOR_FOCUS_BOTTOM_PADDING  4U
#define THROTTLE_UI_MOTOR_FOCUS_RADIUS          7U
#define THROTTLE_UI_MOTOR_FOCUS_BORDER_WIDTH    4U
#define THROTTLE_UI_ALL_FOCUS_RADIUS             6U
#define THROTTLE_UI_ALL_FOCUS_BORDER_WIDTH       3U
#define THROTTLE_UI_TOP_FOCUS_BORDER_WIDTH      2U

#define THROTTLE_UI_COLOR_BACKGROUND  0x0F9EU /* SVG #0BF1F5 */
#define THROTTLE_UI_COLOR_FOREGROUND  BLACK   /** #f800ba */
#define THROTTLE_UI_COLOR_ALL_OFF     BLACK
#define THROTTLE_UI_COLOR_ALL_ON      RED
#define THROTTLE_UI_COLOR_PARAMETER   BLACK
#define THROTTLE_UI_COLOR_TOP_FOCUS   YELLOW  /* 与主界面保持一致的导航焦点 */
#define THROTTLE_UI_COLOR_MOTOR_FOCUS 0xF817  /** #f800ba ，区别于琥珀色选中背景 */
#define THROTTLE_UI_COLOR_EDIT_FOCUS  WHITE
#define THROTTLE_UI_COLOR_THROTTLE_FILL  0xFA8BU

/*
 * 底部连续油门信息的两个独立刷新区域。进度条与精确数值分开后，UiTask
 * 可以使用不同刷新频率，也不会为了更新其中一个而重复发送整个THR区域。
 */
#define THROTTLE_UI_BAR_LEFT          61U
#define THROTTLE_UI_BAR_TOP          105U
#define THROTTLE_UI_BAR_RIGHT        123U
#define THROTTLE_UI_BAR_BOTTOM       112U
#define THROTTLE_UI_RAW_VALUE_LEFT   128U
#define THROTTLE_UI_RAW_VALUE_TOP    101U
#define THROTTLE_UI_RAW_VALUE_RIGHT  159U
#define THROTTLE_UI_RAW_VALUE_BOTTOM 116U

/**
 * @brief 油门页面区域刷新所需的动态图层集合。
 *
 * 优化三“动态元素专用局部刷新”：不同调用点明确选择顶部参数、电机、
 * 进度条、精确数值等绘制器，避免一个很小的窗口仍执行整页判断链。
 */
typedef enum
{
    THROTTLE_UI_RENDER_FULL = 0,
    THROTTLE_UI_RENDER_TOP,
    THROTTLE_UI_RENDER_MOTOR,
    THROTTLE_UI_RENDER_LOCK,
    THROTTLE_UI_RENDER_BAR,
    THROTTLE_UI_RENDER_RAW_VALUE,
    THROTTLE_UI_RENDER_MASK
} ThrottleUiRenderMode_t;

#if defined(__GNUC__)
#define THROTTLE_UI_MAYBE_UNUSED __attribute__((unused))
#else
#define THROTTLE_UI_MAYBE_UNUSED
#endif

extern const unsigned char ascii_1206[][12];
extern const unsigned char ascii_1608[][16];
extern const unsigned char ascii_2412[][48];

static const uint16_t g_motor_icon_x[8U] =
    {24U, 79U, 133U, 188U, 24U, 79U, 134U, 188U};
static const uint16_t g_motor_icon_y[8U] =
    {29U, 29U, 29U, 29U, 66U, 66U, 66U, 66U};

/* 与主页面使用完全相同的1.5倍锁图标点阵。 */
static const uint8_t g_throttle_ui_unlocked_icon[92U] = {
    0x00U, 0x00U, 0x7EU, 0x00U, 0x00U, 0x00U, 0xFFU, 0x00U,
    0x00U, 0x80U, 0xC3U, 0x01U, 0x00U, 0xC0U, 0x81U, 0x03U,
    0x00U, 0xC0U, 0x00U, 0x03U, 0x00U, 0xE0U, 0x00U, 0x07U,
    0x00U, 0xE0U, 0x00U, 0x07U, 0x00U, 0xE0U, 0x00U, 0x07U,
    0x00U, 0xE0U, 0x00U, 0x07U, 0xFCU, 0xFFU, 0x00U, 0x00U,
    0xFFU, 0xFFU, 0x03U, 0x00U, 0x03U, 0x00U, 0x03U, 0x00U,
    0x03U, 0x00U, 0x03U, 0x00U, 0x03U, 0x03U, 0x03U, 0x00U,
    0x83U, 0x07U, 0x03U, 0x00U, 0x83U, 0x07U, 0x03U, 0x00U,
    0x03U, 0x03U, 0x03U, 0x00U, 0x03U, 0x03U, 0x03U, 0x00U,
    0x83U, 0x07U, 0x03U, 0x00U, 0x03U, 0x03U, 0x03U, 0x00U,
    0x03U, 0x00U, 0x03U, 0x00U, 0xFFU, 0xFFU, 0x03U, 0x00U,
    0xFEU, 0xFFU, 0x01U, 0x00U
};

static const uint8_t g_throttle_ui_locked_icon[69U] = {
    0xC0U, 0x0FU, 0x00U, 0xE0U, 0x1FU, 0x00U, 0x70U, 0x38U, 0x00U,
    0x38U, 0x70U, 0x00U, 0x18U, 0x60U, 0x00U, 0x1CU, 0xE0U, 0x00U,
    0x1CU, 0xE0U, 0x00U, 0x1CU, 0xE0U, 0x00U, 0x1CU, 0xE0U, 0x00U,
    0xFEU, 0xFFU, 0x01U, 0xFFU, 0xFFU, 0x03U, 0x03U, 0x00U, 0x03U,
    0x03U, 0x00U, 0x03U, 0x03U, 0x03U, 0x03U, 0x83U, 0x07U, 0x03U,
    0x83U, 0x07U, 0x03U, 0x83U, 0x07U, 0x03U, 0x03U, 0x03U, 0x03U,
    0x83U, 0x07U, 0x03U, 0x83U, 0x07U, 0x03U, 0x03U, 0x00U, 0x03U,
    0xFFU, 0xFFU, 0x03U, 0xFEU, 0xFFU, 0x01U
};

static uint8_t g_throttle_ui_line_buffer[LCD_W * 2U];

#if (LCD_W != 120U) || (LCD_H != 240U)
#error "ThrottleUI software rotation expects a 120x240 LCD framebuffer"
#endif

static bool ThrottleUI_PointInRectangle(uint16_t x,
                                        uint16_t y,
                                        uint16_t left,
                                        uint16_t top,
                                        uint16_t right,
                                        uint16_t bottom)
{
    return (x >= left) && (x <= right) && (y >= top) && (y <= bottom);
}

static bool ThrottleUI_PointInCircle(uint16_t x,
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

static bool ThrottleUI_PointInRoundedRectangle(uint16_t x,
                                               uint16_t y,
                                               uint16_t left,
                                               uint16_t top,
                                               uint16_t right,
                                               uint16_t bottom,
                                               uint16_t radius)
{
    if (!ThrottleUI_PointInRectangle(x, y, left, top, right, bottom))
    {
        return false;
    }

    if (((x >= (left + radius)) && (x <= (right - radius))) ||
        ((y >= (top + radius)) && (y <= (bottom - radius))))
    {
        return true;
    }

    return ThrottleUI_PointInCircle(x, y, left + radius, top + radius, radius) ||
           ThrottleUI_PointInCircle(x, y, right - radius, top + radius, radius) ||
           ThrottleUI_PointInCircle(x, y, left + radius, bottom - radius, radius) ||
           ThrottleUI_PointInCircle(x, y, right - radius, bottom - radius, radius);
}

/** @brief 按给定线宽生成圆角边框像素。 */
static bool ThrottleUI_PointOnRoundedBorder(uint16_t x,
                                            uint16_t y,
                                            uint16_t left,
                                            uint16_t top,
                                            uint16_t right,
                                            uint16_t bottom,
                                            uint16_t radius,
                                            uint16_t border_width)
{
    if (border_width == 0U)
    {
        return false;
    }

    if (!ThrottleUI_PointInRoundedRectangle(
            x, y, left, top, right, bottom, radius))
    {
        return false;
    }

    if ((right - left < (2U * border_width)) ||
        (bottom - top < (2U * border_width)) ||
        (radius < border_width))
    {
        return true;
    }

    return !ThrottleUI_PointInRoundedRectangle(
        x, y,
        left + border_width,
        top + border_width,
        right - border_width,
        bottom - border_width,
        radius - border_width);
}

static bool THROTTLE_UI_MAYBE_UNUSED
ThrottleUI_SmallTextPixel(uint16_t x,
                          uint16_t y,
                          uint16_t text_x,
                          uint16_t text_y,
                          const char *text)
{
    uint16_t local_x;
    uint16_t local_y;
    uint16_t character_index;
    uint8_t character;

    if ((text == NULL) || (x < text_x) || (y < text_y))
    {
        return false;
    }

    local_x = x - text_x;
    local_y = y - text_y;
    if (local_y >= THROTTLE_UI_SMALL_FONT_HEIGHT)
    {
        return false;
    }

    character_index = local_x / THROTTLE_UI_SMALL_FONT_WIDTH;
    if ((size_t)character_index >= strlen(text))
    {
        return false;
    }

    local_x %= THROTTLE_UI_SMALL_FONT_WIDTH;
    character = (uint8_t)text[character_index];
    if ((character < (uint8_t)' ') || (character > (uint8_t)'~'))
    {
        return false;
    }

    return ((ascii_1206[character - (uint8_t)' '][local_y] >> local_x) &
            0x01U) != 0U;
}

/**
 * @brief 判断指定像素是否属于8x16 ASCII字符串的前景。
 *
 * @note 保留独立的6x12解析函数，便于较小区域后续继续使用；当前油门页的
 *       参数标签和值临时统一使用本函数，以便直接对比两种字号的实际效果。
 */
static bool ThrottleUI_MediumTextPixel(uint16_t x,
                                       uint16_t y,
                                       uint16_t text_x,
                                       uint16_t text_y,
                                       const char *text)
{
    uint16_t local_x;
    uint16_t local_y;
    uint16_t character_index;
    uint8_t character;

    if ((text == NULL) || (x < text_x) || (y < text_y))
    {
        return false;
    }

    local_x = x - text_x;
    local_y = y - text_y;
    if (local_y >= THROTTLE_UI_MEDIUM_FONT_HEIGHT)
    {
        return false;
    }

    character_index = local_x / THROTTLE_UI_MEDIUM_FONT_WIDTH;
    if ((size_t)character_index >= strlen(text))
    {
        return false;
    }

    local_x %= THROTTLE_UI_MEDIUM_FONT_WIDTH;
    character = (uint8_t)text[character_index];
    if ((character < (uint8_t)' ') || (character > (uint8_t)'~'))
    {
        return false;
    }

    return ((ascii_1608[character - (uint8_t)' '][local_y] >> local_x) &
            0x01U) != 0U;
}

static bool ThrottleUI_LargeTextPixel(uint16_t x,
                                      uint16_t y,
                                      uint16_t text_x,
                                      uint16_t text_y,
                                      const char *text)
{
    uint16_t local_x;
    uint16_t local_y;
    uint16_t character_index;
    uint16_t byte_index;
    uint8_t character;

    if ((text == NULL) || (x < text_x) || (y < text_y))
    {
        return false;
    }

    local_x = x - text_x;
    local_y = y - text_y;
    if (local_y >= THROTTLE_UI_LARGE_FONT_HEIGHT)
    {
        return false;
    }

    character_index = local_x / THROTTLE_UI_LARGE_FONT_WIDTH;
    if ((size_t)character_index >= strlen(text))
    {
        return false;
    }

    local_x %= THROTTLE_UI_LARGE_FONT_WIDTH;
    byte_index = (local_y * 2U) + (local_x / 8U);
    character = (uint8_t)text[character_index];
    if ((character < (uint8_t)' ') || (character > (uint8_t)'~'))
    {
        return false;
    }

    return (ascii_2412[character - (uint8_t)' '][byte_index] &
            (uint8_t)(1U << (local_x % 8U))) != 0U;
}

/**
 * @brief 在12x24字模基础上向右扩展1像素，生成更醒目的粗体效果。
 *
 * 仅ALL状态标签使用本函数；THR等其它12x24文字仍保持原始字重。
 */
static bool ThrottleUI_LargeBoldTextPixel(uint16_t x,
                                          uint16_t y,
                                          uint16_t text_x,
                                          uint16_t text_y,
                                          const char *text)
{
    if (ThrottleUI_LargeTextPixel(x, y, text_x, text_y, text))
    {
        return true;
    }

    return (x > text_x) &&
           ThrottleUI_LargeTextPixel(x - 1U, y, text_x, text_y, text);
}

static bool ThrottleUI_BitmapPixel(uint16_t x,
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

    if ((bitmap == NULL) || (x < left) || (y < top))
    {
        return false;
    }

    local_x = x - left;
    local_y = y - top;
    if ((local_x >= width) || (local_y >= height))
    {
        return false;
    }

    return (bitmap[(local_y * bytes_per_row) + (local_x / 8U)] &
            (uint8_t)(1U << (local_x % 8U))) != 0U;
}

static void ThrottleUI_FormatUint16(uint16_t value, char text[5])
{
    uint16_t divisor = 1000U;
    uint8_t index = 0U;
    bool started = false;

    while (divisor > 0U)
    {
        const uint8_t digit = (uint8_t)(value / divisor);

        if ((digit != 0U) || started || (divisor == 1U))
        {
            text[index++] = (char)('0' + digit);
            started = true;
        }
        value %= divisor;
        divisor /= 10U;
    }
    text[index] = '\0';
}

static void ThrottleUI_FormatMask(uint8_t mask, char text[3])
{
    static const char hex_digits[] = "0123456789ABCDEF";

    text[0] = hex_digits[(mask >> 4U) & 0x0FU];
    text[1] = hex_digits[mask & 0x0FU];
    text[2] = '\0';
}

/**
 * @brief 返回无符号十进制数实际需要显示的字符数量。
 *
 * 数值0也需要一个字符，因此本函数最小返回1。LIM和STEP的焦点框使用
 * 此结果计算右边界，使不同位数始终保持与三位数状态相同的右侧留白。
 */
static uint8_t ThrottleUI_GetUint16DigitCount(uint16_t value)
{
    uint8_t digit_count = 1U;

    while (value >= 10U)
    {
        value /= 10U;
        ++digit_count;
    }

    return digit_count;
}

static void ThrottleUI_ValidateView(const ThrottleUiView_t *source,
                                    ThrottleUiView_t *destination)
{
    if (destination == NULL)
    {
        return;
    }

    if (source == NULL)
    {
        memset(destination, 0, sizeof(*destination));
        destination->limit = 100U;
        destination->step = 100U;
        destination->focus = THROTTLE_UI_FOCUS_ALL;
        return;
    }

    *destination = *source;
    if (destination->limit > 8191U)
    {
        destination->limit = 8191U;
    }
    if (destination->raw_command > destination->limit)
    {
        destination->raw_command = destination->limit;
    }
    if (destination->throttle_percent > 100U)
    {
        destination->throttle_percent = 100U;
    }
    if ((destination->step != 1U) && (destination->step != 10U) &&
        (destination->step != 100U) && (destination->step != 1000U))
    {
        destination->step = 100U;
    }
    if (destination->focus > THROTTLE_UI_FOCUS_MOTOR_8)
    {
        destination->focus = THROTTLE_UI_FOCUS_ALL;
    }
    if ((destination->focus != THROTTLE_UI_FOCUS_LIMIT) &&
        (destination->focus != THROTTLE_UI_FOCUS_STEP))
    {
        destination->edit_mode = false;
    }
}

static bool ThrottleUI_GetFocusBounds(ThrottleUiFocus_t focus,
                                      const ThrottleUiView_t *view,
                                      uint16_t *left,
                                      uint16_t *top,
                                      uint16_t *right,
                                      uint16_t *bottom)
{
    uint8_t motor_index;

    if ((left == NULL) || (top == NULL) ||
        (right == NULL) || (bottom == NULL))
    {
        return false;
    }

    switch (focus)
    {
        case THROTTLE_UI_FOCUS_ALL:
            *left = 0U; *top = 0U; *right = 42U; *bottom = 26U;
            return true;
        case THROTTLE_UI_FOCUS_LIMIT:
            *left = 40U;
            *top = 3U;
            *right = (uint16_t)(82U +
                ((uint16_t)ThrottleUI_GetUint16DigitCount(
                    (view != NULL) ? view->limit : 100U) *
                 THROTTLE_UI_MEDIUM_FONT_WIDTH));
            *bottom = 22U;
            return true;
        case THROTTLE_UI_FOCUS_STEP:
            *left = 110U;
            *top = 3U;
            *right = (uint16_t)(166U +
                ((uint16_t)ThrottleUI_GetUint16DigitCount(
                    (view != NULL) ? view->step : 100U) *
                 THROTTLE_UI_MEDIUM_FONT_WIDTH));
            *bottom = 22U;
            return true;
        default:
            break;
    }

    if ((focus < THROTTLE_UI_FOCUS_MOTOR_1) ||
        (focus > THROTTLE_UI_FOCUS_MOTOR_8))
    {
        return false;
    }

    motor_index = (uint8_t)(focus - THROTTLE_UI_FOCUS_MOTOR_1);
    *left = g_motor_icon_x[motor_index] -
            THROTTLE_UI_MOTOR_FOCUS_PADDING;
    *top = g_motor_icon_y[motor_index] -
           THROTTLE_UI_MOTOR_FOCUS_PADDING;
    *right = g_motor_icon_x[motor_index] +
             THROTTLE_UI_MOTOR_ICON_WIDTH +
             THROTTLE_UI_MOTOR_FOCUS_PADDING - 1U;
    *bottom = g_motor_icon_y[motor_index] +
              THROTTLE_UI_MOTOR_ICON_HEIGHT +
              THROTTLE_UI_MOTOR_FOCUS_BOTTOM_PADDING - 1U;
    return true;
}

static bool ThrottleUI_FocusPixel(uint16_t x,
                                  uint16_t y,
                                  const ThrottleUiView_t *view)
{
    uint16_t left;
    uint16_t top;
    uint16_t right;
    uint16_t bottom;
    uint16_t radius = 4U;
    uint16_t border_width = THROTTLE_UI_TOP_FOCUS_BORDER_WIDTH;

    if (!ThrottleUI_GetFocusBounds(
            view->focus, view, &left, &top, &right, &bottom))
    {
        return false;
    }

    if (view->focus >= THROTTLE_UI_FOCUS_MOTOR_1)
    {
        radius = THROTTLE_UI_MOTOR_FOCUS_RADIUS;
        border_width = THROTTLE_UI_MOTOR_FOCUS_BORDER_WIDTH;
    }
    else if (view->focus == THROTTLE_UI_FOCUS_ALL)
    {
        radius = THROTTLE_UI_ALL_FOCUS_RADIUS;
        border_width = THROTTLE_UI_ALL_FOCUS_BORDER_WIDTH;
    }

    return ThrottleUI_PointOnRoundedBorder(
        x, y, left, top, right, bottom, radius, border_width);
}

/**
 * @brief 用坐标直接定位一个电机图标，避免每像素循环检查8路电机。
 *
 * 优化一“区域判断优化”：先用Y坐标确定上下排，再用互不重叠的X区间
 * 直接得到唯一通道。空隙像素立即返回，最坏也只是4次范围判断。
 */
static bool ThrottleUI_GetMotorIndexAtPixel(uint16_t x,
                                             uint16_t y,
                                             uint8_t *motor_index)
{
    uint8_t row_offset;

    if (motor_index == NULL)
    {
        return false;
    }

    if ((y >= 29U) && (y < (29U + THROTTLE_UI_MOTOR_ICON_HEIGHT)))
    {
        row_offset = 0U;
    }
    else if ((y >= 66U) &&
             (y < (66U + THROTTLE_UI_MOTOR_ICON_HEIGHT)))
    {
        row_offset = 4U;
    }
    else
    {
        return false;
    }

    if ((x >= 24U) && (x < (24U + THROTTLE_UI_MOTOR_ICON_WIDTH)))
    {
        *motor_index = row_offset;
    }
    else if ((x >= 79U) &&
             (x < (79U + THROTTLE_UI_MOTOR_ICON_WIDTH)))
    {
        *motor_index = (uint8_t)(row_offset + 1U);
    }
    else if ((x >= g_motor_icon_x[row_offset + 2U]) &&
             (x < (g_motor_icon_x[row_offset + 2U] +
                   THROTTLE_UI_MOTOR_ICON_WIDTH)))
    {
        *motor_index = (uint8_t)(row_offset + 2U);
    }
    else if ((x >= 188U) &&
             (x < (188U + THROTTLE_UI_MOTOR_ICON_WIDTH)))
    {
        *motor_index = (uint8_t)(row_offset + 3U);
    }
    else
    {
        return false;
    }

    return true;
}

/**
 * @brief 用原稿灰度轮廓和Flash查找表合成一个电机图标像素。
 *
 * 原SVG的0~255灰度仍然决定齿轮、数字和抗锯齿边缘；区别只是把原先每
 * 像素三次乘法/除法改为一次Flash查表。[0]是未选中底色，[1]是选中
 * 底色，因此页面外观和选择逻辑不变。
 */
static bool ThrottleUI_GetMotorIconPixel(uint16_t x,
                                         uint16_t y,
                                         const ThrottleUiView_t *view,
                                         uint16_t *color)
{
    uint8_t motor_index;
    uint16_t local_x;
    uint16_t local_y;
    uint8_t gray;
    uint8_t selection_index;

    if ((color == NULL) ||
        !ThrottleUI_GetMotorIndexAtPixel(x, y, &motor_index))
    {
        return false;
    }

    local_x = x - g_motor_icon_x[motor_index];
    local_y = y - g_motor_icon_y[motor_index];
    gray = g_throttle_ui_motor_icon_gray[motor_index]
             [(local_y * THROTTLE_UI_MOTOR_ICON_WIDTH) + local_x];
    selection_index =
        ((view->motor_mask & (uint8_t)(1UL << motor_index)) != 0U) ? 1U : 0U;
    *color = g_throttle_ui_motor_color_lut[selection_index][gray];
    return true;
}

static bool ThrottleUI_LockPixel(uint16_t x,
                                 uint16_t y,
                                 bool unlocked)
{
    if (unlocked)
    {
        return ThrottleUI_BitmapPixel(x, y,
                                      THROTTLE_UI_LOCK_X,
                                      THROTTLE_UI_LOCK_Y,
                                      THROTTLE_UI_UNLOCKED_WIDTH,
                                      THROTTLE_UI_LOCK_HEIGHT,
                                      4U,
                                      g_throttle_ui_unlocked_icon);
    }

    return ThrottleUI_BitmapPixel(x, y,
                                  THROTTLE_UI_LOCK_X,
                                  THROTTLE_UI_LOCK_Y,
                                  THROTTLE_UI_LOCKED_WIDTH,
                                  THROTTLE_UI_LOCK_HEIGHT,
                                  3U,
                                  g_throttle_ui_locked_icon);
}

/**
 * @brief 从MCU内部Flash中的4位调色板油门底图读取一个固定像素。
 *
 * 优化二“MCU内部Flash静态底图”：背景、LIM/STEP标签、±、THR、MASK
 * 标签和进度条白框已预生成。每两个物理像素占1字节，页面固定内容只占
 * 14,400字节Flash；运行时不访问外部FRAM，也不重复解析固定字体。
 */
static uint16_t ThrottleUI_GetStaticPhysicalPixel(uint32_t packed_row_index,
                                                  uint16_t physical_x)
{
    const uint32_t packed_index =
        packed_row_index + ((uint32_t)physical_x >> 1U);
    const uint8_t packed = g_throttle_ui_static_4bpp[packed_index];
    const uint8_t palette_index = ((physical_x & 1U) == 0U)
                                      ? (uint8_t)(packed >> 4U)
                                      : (uint8_t)(packed & 0x0FU);

    return g_throttle_ui_static_palette[palette_index];
}

/** @brief 在底图上合成顶部ALL、参数值、焦点和锁。 */
static uint16_t ThrottleUI_OverlayTop(uint16_t x,
                                      uint16_t y,
                                      const ThrottleUiView_t *view,
                                      const char *limit_text,
                                      const char *step_text,
                                      uint16_t static_color)
{
    uint16_t color = static_color;

    if (ThrottleUI_FocusPixel(x, y, view))
    {
        color = view->edit_mode
                    ? THROTTLE_UI_COLOR_EDIT_FOCUS
                    : ((view->focus >= THROTTLE_UI_FOCUS_MOTOR_1)
                           ? THROTTLE_UI_COLOR_MOTOR_FOCUS
                           : THROTTLE_UI_COLOR_TOP_FOCUS);

        /* 固定LIM/STEP标签及±在原实现中覆盖焦点边框，保持原叠放顺序。 */
        if (static_color != THROTTLE_UI_COLOR_BACKGROUND)
        {
            color = static_color;
        }
    }

    if ((x <= 38U) && ThrottleUI_LargeBoldTextPixel(x, y, 2U, 0U, "ALL"))
    {
        color = (view->motor_mask == 0xFFU)
                    ? THROTTLE_UI_COLOR_ALL_ON
                    : THROTTLE_UI_COLOR_ALL_OFF;
    }
    else if (((x >= 78U) && (x <= 109U) &&
              ThrottleUI_MediumTextPixel(x, y, 78U, 4U, limit_text)) ||
             ((x >= 163U) && (x <= 202U) &&
              ThrottleUI_MediumTextPixel(x, y, 163U, 4U, step_text)))
    {
        color = THROTTLE_UI_COLOR_PARAMETER;
    }

    if ((x >= THROTTLE_UI_LOCK_X) &&
        ThrottleUI_LockPixel(x, y, view->throttle_unlocked))
    {
        color = THROTTLE_UI_COLOR_FOREGROUND;
    }

    return color;
}

/** @brief 在底图上合成当前电机焦点及唯一命中的电机图标。 */
static uint16_t ThrottleUI_OverlayMotor(uint16_t x,
                                        uint16_t y,
                                        const ThrottleUiView_t *view,
                                        uint16_t color)
{
    uint16_t motor_color;

    if (ThrottleUI_FocusPixel(x, y, view))
    {
        color = (view->focus >= THROTTLE_UI_FOCUS_MOTOR_1)
                    ? THROTTLE_UI_COLOR_MOTOR_FOCUS
                    : THROTTLE_UI_COLOR_TOP_FOCUS;
    }

    if (ThrottleUI_GetMotorIconPixel(x, y, view, &motor_color))
    {
        color = motor_color;
    }

    return color;
}

/** @brief 在底图进度条白框内部合成当前油门百分比。 */
static uint16_t ThrottleUI_OverlayBar(uint16_t x,
                                      uint16_t y,
                                      const ThrottleUiView_t *view,
                                      uint16_t color)
{
    const uint16_t fill_width =
        (uint16_t)(((uint32_t)(THROTTLE_UI_BAR_RIGHT -
                               THROTTLE_UI_BAR_LEFT - 1U) *
                    view->throttle_percent) / 100U);

    if ((fill_width > 0U) && (x > THROTTLE_UI_BAR_LEFT) &&
        (x <= (THROTTLE_UI_BAR_LEFT + fill_width)) &&
        (y > THROTTLE_UI_BAR_TOP) && (y < THROTTLE_UI_BAR_BOTTOM))
    {
        return THROTTLE_UI_COLOR_THROTTLE_FILL;
    }

    return color;
}

/**
 * @brief 生成油门页面像素，并只运行刷新模式要求的动态绘制器。
 *
 * 优化一“区域判断优化”：完整刷新按顶部、两排电机、底部三个大区域
 * 分派；优化三使局部刷新进一步直接进入BAR/RAW/MASK等专用路径。
 */
static uint16_t ThrottleUI_GetRenderedPixel(uint16_t x,
                                            uint16_t y,
                                            const ThrottleUiView_t *view,
                                            const char *limit_text,
                                            const char *step_text,
                                            const char *raw_text,
                                            const char *mask_text,
                                            uint16_t static_color,
                                            ThrottleUiRenderMode_t render_mode)
{
    uint16_t color = static_color;

    if ((render_mode == THROTTLE_UI_RENDER_FULL) ||
        (render_mode == THROTTLE_UI_RENDER_TOP))
    {
        if (y <= 26U)
        {
            color = ThrottleUI_OverlayTop(
                x, y, view, limit_text, step_text, color);
        }
    }

    if ((render_mode == THROTTLE_UI_RENDER_FULL) ||
        (render_mode == THROTTLE_UI_RENDER_MOTOR))
    {
        if ((y >= 24U) && (y <= 95U))
        {
            color = ThrottleUI_OverlayMotor(x, y, view, color);
        }
    }

    if ((render_mode == THROTTLE_UI_RENDER_LOCK) &&
        ThrottleUI_LockPixel(x, y, view->throttle_unlocked))
    {
        color = THROTTLE_UI_COLOR_FOREGROUND;
    }

    if (((render_mode == THROTTLE_UI_RENDER_FULL) ||
         (render_mode == THROTTLE_UI_RENDER_BAR)) &&
        (y >= THROTTLE_UI_BAR_TOP) && (y <= THROTTLE_UI_BAR_BOTTOM))
    {
        color = ThrottleUI_OverlayBar(x, y, view, color);
    }

    if (((render_mode == THROTTLE_UI_RENDER_FULL) ||
         (render_mode == THROTTLE_UI_RENDER_RAW_VALUE)) &&
        (x >= THROTTLE_UI_RAW_VALUE_LEFT) &&
        (x <= THROTTLE_UI_RAW_VALUE_RIGHT) &&
        ThrottleUI_MediumTextPixel(
            x, y, THROTTLE_UI_RAW_VALUE_LEFT, THROTTLE_UI_RAW_VALUE_TOP,
            raw_text))
    {
        color = THROTTLE_UI_COLOR_FOREGROUND;
    }

    if (((render_mode == THROTTLE_UI_RENDER_FULL) ||
         (render_mode == THROTTLE_UI_RENDER_MASK)) &&
        (x >= 215U) &&
        ThrottleUI_MediumTextPixel(x, y, 215U, 101U, mask_text))
    {
        color = THROTTLE_UI_COLOR_FOREGROUND;
    }

    return color;
}

static void ThrottleUI_DrawLogicalRegion(const ThrottleUiView_t *view,
                                         uint16_t logical_left,
                                         uint16_t logical_top,
                                         uint16_t logical_right,
                                         uint16_t logical_bottom,
                                         ThrottleUiRenderMode_t render_mode)
{
    ThrottleUiView_t validated;
    char limit_text[5];
    char step_text[5];
    char raw_text[5];
    char mask_text[3];
    uint16_t physical_left;
    uint16_t physical_right;
    uint16_t physical_top;
    uint16_t physical_bottom;

    if ((logical_left > logical_right) || (logical_top > logical_bottom) ||
        (logical_left >= THROTTLE_UI_LOGICAL_WIDTH) ||
        (logical_top >= THROTTLE_UI_LOGICAL_HEIGHT))
    {
        return;
    }

    if (logical_right >= THROTTLE_UI_LOGICAL_WIDTH)
    {
        logical_right = THROTTLE_UI_LOGICAL_WIDTH - 1U;
    }
    if (logical_bottom >= THROTTLE_UI_LOGICAL_HEIGHT)
    {
        logical_bottom = THROTTLE_UI_LOGICAL_HEIGHT - 1U;
    }

    ThrottleUI_ValidateView(view, &validated);
    ThrottleUI_FormatUint16(validated.limit, limit_text);
    ThrottleUI_FormatUint16(validated.step, step_text);
    ThrottleUI_FormatUint16(validated.raw_command, raw_text);
    ThrottleUI_FormatMask(validated.motor_mask, mask_text);

    physical_left =
        (THROTTLE_UI_LOGICAL_HEIGHT - 1U) - logical_bottom;
    physical_right =
        (THROTTLE_UI_LOGICAL_HEIGHT - 1U) - logical_top;
    physical_top = logical_left;
    physical_bottom = logical_right;

    /* SH8501列窗口按4像素对齐；扩展像素仍由完整页面生成器正确恢复。 */
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
                (THROTTLE_UI_LOGICAL_HEIGHT - 1U) - physical_x;
            const uint16_t static_color =
                ThrottleUI_GetStaticPhysicalPixel(
                    packed_row_index, physical_x);
            const uint16_t pixel = ThrottleUI_GetRenderedPixel(
                logical_x,
                logical_y,
                &validated,
                limit_text,
                step_text,
                raw_text,
                mask_text,
                static_color,
                render_mode);

            g_throttle_ui_line_buffer[buffer_index++] =
                (uint8_t)(pixel >> 8U);
            g_throttle_ui_line_buffer[buffer_index++] =
                (uint8_t)(pixel & 0x00FFU);
        }

        LCD_WriteDataBuffer(g_throttle_ui_line_buffer, buffer_index);
    }

    SPI1_Bus_Release();
}

void ThrottleUI_Draw(const ThrottleUiView_t *view)
{
    ThrottleUI_DrawLogicalRegion(view,
                                 0U,
                                 0U,
                                 THROTTLE_UI_LOGICAL_WIDTH - 1U,
                                 THROTTLE_UI_LOGICAL_HEIGHT - 1U,
                                 THROTTLE_UI_RENDER_FULL);
}

/** @brief 根据焦点所在层选择顶部或电机专用绘制器。 */
static ThrottleUiRenderMode_t ThrottleUI_GetFocusRenderMode(
    ThrottleUiFocus_t focus)
{
    return (focus >= THROTTLE_UI_FOCUS_MOTOR_1)
               ? THROTTLE_UI_RENDER_MOTOR
               : THROTTLE_UI_RENDER_TOP;
}

void ThrottleUI_Update(const ThrottleUiView_t *previous_view,
                       const ThrottleUiView_t *current_view)
{
    ThrottleUiView_t previous;
    ThrottleUiView_t current;
    uint16_t left;
    uint16_t top;
    uint16_t right;
    uint16_t bottom;

    if ((previous_view == NULL) || (current_view == NULL))
    {
        ThrottleUI_Draw(current_view);
        return;
    }

    ThrottleUI_ValidateView(previous_view, &previous);
    ThrottleUI_ValidateView(current_view, &current);

    if ((previous.focus != current.focus) ||
        (previous.edit_mode != current.edit_mode))
    {
        if (ThrottleUI_GetFocusBounds(
                previous.focus, &previous,
                &left, &top, &right, &bottom))
        {
            ThrottleUI_DrawLogicalRegion(
                &current,
                left,
                top,
                right,
                bottom,
                ThrottleUI_GetFocusRenderMode(previous.focus));
        }
        if ((previous.focus != current.focus) &&
            ThrottleUI_GetFocusBounds(
                current.focus, &current,
                &left, &top, &right, &bottom))
        {
            ThrottleUI_DrawLogicalRegion(
                &current,
                left,
                top,
                right,
                bottom,
                ThrottleUI_GetFocusRenderMode(current.focus));
        }
    }

    if (previous.motor_mask != current.motor_mask)
    {
        const uint8_t changed_mask =
            (uint8_t)(previous.motor_mask ^ current.motor_mask);

        /* ALL颜色和MASK文本始终跟随8位掩码。 */
        ThrottleUI_DrawLogicalRegion(&current,
                                     0U,
                                     0U,
                                     42U,
                                     26U,
                                     THROTTLE_UI_RENDER_TOP);
        ThrottleUI_DrawLogicalRegion(&current,
                                     168U,
                                     98U,
                                     234U,
                                     118U,
                                     THROTTLE_UI_RENDER_MASK);

        for (uint8_t index = 0U; index < 8U; ++index)
        {
            if ((changed_mask & (uint8_t)(1UL << index)) != 0U)
            {
                ThrottleUI_DrawLogicalRegion(
                    &current,
                    g_motor_icon_x[index] -
                        THROTTLE_UI_MOTOR_FOCUS_PADDING,
                    g_motor_icon_y[index] -
                        THROTTLE_UI_MOTOR_FOCUS_PADDING,
                        g_motor_icon_x[index] +
                            THROTTLE_UI_MOTOR_ICON_WIDTH +
                            THROTTLE_UI_MOTOR_FOCUS_PADDING - 1U,
                        g_motor_icon_y[index] +
                            THROTTLE_UI_MOTOR_ICON_HEIGHT +
                            THROTTLE_UI_MOTOR_FOCUS_BOTTOM_PADDING - 1U,
                        THROTTLE_UI_RENDER_MOTOR);
            }
        }
    }

    if (previous.limit != current.limit)
    {
        uint16_t previous_right;
        uint16_t current_right;

        (void)ThrottleUI_GetFocusBounds(
            THROTTLE_UI_FOCUS_LIMIT, &previous,
            &left, &top, &previous_right, &bottom);
        (void)ThrottleUI_GetFocusBounds(
            THROTTLE_UI_FOCUS_LIMIT, &current,
            &left, &top, &current_right, &bottom);

        /* 使用新旧右边界的较大值，同时清除缩短数值留下的数字和边框。 */
        right = (previous_right > current_right)
                    ? previous_right
                    : current_right;
        ThrottleUI_DrawLogicalRegion(&current,
                                     left,
                                     top,
                                     right,
                                     bottom,
                                     THROTTLE_UI_RENDER_TOP);
    }

    if (previous.step != current.step)
    {
        uint16_t previous_right;
        uint16_t current_right;

        (void)ThrottleUI_GetFocusBounds(
            THROTTLE_UI_FOCUS_STEP, &previous,
            &left, &top, &previous_right, &bottom);
        (void)ThrottleUI_GetFocusBounds(
            THROTTLE_UI_FOCUS_STEP, &current,
            &left, &top, &current_right, &bottom);

        /* STEP从1000减小时，同一刷新区域会把旧的第四位及旧边框擦除。 */
        right = (previous_right > current_right)
                    ? previous_right
                    : current_right;
        ThrottleUI_DrawLogicalRegion(&current,
                                     left,
                                     top,
                                     right,
                                     bottom,
                                     THROTTLE_UI_RENDER_TOP);
    }

    if (previous.throttle_unlocked != current.throttle_unlocked)
    {
        ThrottleUI_DrawLogicalRegion(&current,
                                     209U,
                                     0U,
                                     239U,
                                     26U,
                                     THROTTLE_UI_RENDER_LOCK);
    }

}

void ThrottleUI_UpdateThrottleBar(const ThrottleUiView_t *view)
{
    /*
     * 只重画边框及其内部填充；THR标签和右侧数字都属于固定/独立区域，
     * 因此25 Hz更新不会重复发送它们的像素。
     */
    ThrottleUI_DrawLogicalRegion(view,
                                 THROTTLE_UI_BAR_LEFT,
                                 THROTTLE_UI_BAR_TOP,
                                 THROTTLE_UI_BAR_RIGHT,
                                 THROTTLE_UI_BAR_BOTTOM,
                                 THROTTLE_UI_RENDER_BAR);
}

void ThrottleUI_UpdateThrottleValue(const ThrottleUiView_t *view)
{
    /*
     * 固定覆盖8x16字模的最大4位宽度。数值由四位变成三位时，区域生成器
     * 会同时恢复多余字符位置的页面背景，不会留下旧数字尾迹。
     */
    ThrottleUI_DrawLogicalRegion(view,
                                 THROTTLE_UI_RAW_VALUE_LEFT,
                                 THROTTLE_UI_RAW_VALUE_TOP,
                                 THROTTLE_UI_RAW_VALUE_RIGHT,
                                 THROTTLE_UI_RAW_VALUE_BOTTOM,
                                 THROTTLE_UI_RENDER_RAW_VALUE);
}
