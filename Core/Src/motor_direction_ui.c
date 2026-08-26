#include "motor_direction_ui.h"

#include "lcd_init.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/*
 * 参考图的逻辑尺寸是240x120，而面板显存仍按120x240访问。这里明确区分
 * “界面逻辑坐标”和“LCD物理显存坐标”，防止后续页面布局混用两套坐标。
 */
#define UI_LOGICAL_WIDTH             240U
#define UI_LOGICAL_HEIGHT            120U
#define UI_MOTOR_COUNT                 8U

/* 6x12 ASCII字模的固定尺寸，适合120像素高的小屏界面。 */
#define UI_FONT_WIDTH                  6U
#define UI_FONT_HEIGHT                12U

/* 页面分区和主要图形尺寸，数值均使用240x120逻辑坐标。 */
#define UI_HEADER_BOTTOM_Y            31U
#define UI_MOTOR_NUMBER_Y             38U
#define UI_MOTOR_FIRST_CENTER_X       17U
#define UI_MOTOR_COLUMN_PITCH         29U
#define UI_MAIN_UP_TRIANGLE_TOP_Y      58U
#define UI_MAIN_DOWN_TRIANGLE_TOP_Y    93U
#define UI_MAIN_TRIANGLE_HEIGHT        17U
#define UI_MAIN_TRIANGLE_HALF_WIDTH    10U

/* 页面使用的RGB565颜色。 */
#define UI_COLOR_BACKGROUND          BLACK
/*
 * 外框使用低饱和度蓝灰色，只承担页面分区，不与红色OFF状态或黄色焦点
 * 争夺视觉注意力。焦点框使用黄色，能与青色编号和蓝灰色外框清晰区分。
 */
#define UI_COLOR_HEADER_BORDER       GRAYBLUE
#define UI_COLOR_BODY_BORDER         GRAYBLUE
/*
 * 三种焦点使用不同但不过度刺眼的颜色：暖黄提示总开关，柔白突出通道
 * 数字，青绿突出方向三角形。颜色之外还用尺寸和位置编码，避免只依赖
 * 色觉来分辨焦点层级。
 */
#define UI_COLOR_FOCUS_SWITCH        0xFDC0U
#define UI_COLOR_FOCUS_MOTOR         WHITE
#define UI_COLOR_FOCUS_DIRECTION     0x07F5U
#define UI_COLOR_STATUS_BACKGROUND   LGRAY
#define UI_COLOR_STATUS_OFF          RED
#define UI_COLOR_STATUS_ON           GREEN
#define UI_COLOR_NUMBER              CYAN
#define UI_COLOR_UP                   MAGENTA
#define UI_COLOR_DOWN                 0xD81FU
#define UI_COLOR_SEPARATOR            GRAYBLUE

/*
 * lcd_font.h中的字库由lcd_draw.c定义。此处只声明已经存在的6x12 ASCII
 * 字模，避免再次包含该数据定义头文件而产生重复符号。
 */
extern const unsigned char ascii_1206[][12];

/*
 * 只缓存一条物理扫描行：120个RGB565像素共240字节。完整屏幕需要57,600
 * 字节，超过本MCU可自由使用的RAM，因此不建立整屏帧缓冲区。
 * MotorDirectionUI只由UiTask调用，所以该静态缓冲区不需要互斥锁。
 */
static uint8_t g_ui_physical_row_buffer[LCD_W * 2U];

static const MotorDirectionUiView_t g_safe_default_view = {
    .power_state = MOTOR_DIRECTION_UI_POWER_OFF,
    .focus = MOTOR_DIRECTION_UI_FOCUS_SWITCH,
    .selected_motor = 1U,
    .selected_direction = MOTOR_DIRECTION_UI_NORMAL
};

#if (LCD_W != 120U) || (LCD_H != 240U)
#error "MotorDirectionUI software rotation expects a 120x240 LCD framebuffer"
#endif

/** @brief 判断点是否位于包含端点的矩形内部。 */
static bool MotorDirectionUI_PointInRectangle(uint16_t x,
                                              uint16_t y,
                                              uint16_t left,
                                              uint16_t top,
                                              uint16_t right,
                                              uint16_t bottom)
{
    return (x >= left) && (x <= right) &&
           (y >= top) && (y <= bottom);
}

/** @brief 判断点是否位于1像素矩形边框上。 */
static bool MotorDirectionUI_PointOnRectangleBorder(uint16_t x,
                                                    uint16_t y,
                                                    uint16_t left,
                                                    uint16_t top,
                                                    uint16_t right,
                                                    uint16_t bottom)
{
    if (!MotorDirectionUI_PointInRectangle(x, y, left, top, right, bottom))
    {
        return false;
    }

    return (x == left) || (x == right) || (y == top) || (y == bottom);
}

/** @brief 判断点是否位于实心圆内，用于OFF/ON状态灯。 */
static bool MotorDirectionUI_PointInCircle(uint16_t x,
                                           uint16_t y,
                                           uint16_t center_x,
                                           uint16_t center_y,
                                           uint16_t radius)
{
    const int32_t dx = (int32_t)x - (int32_t)center_x;
    const int32_t dy = (int32_t)y - (int32_t)center_y;
    const int32_t radius_squared = (int32_t)radius * (int32_t)radius;

    return ((dx * dx) + (dy * dy)) <= radius_squared;
}

/**
 * @brief 判断点是否位于水平胶囊形区域内。
 *
 * 胶囊由中间矩形和左右两个半圆组成，用于绘制具有明确左右位置的
 * OFF/ON滑动开关轨道。
 */
static bool MotorDirectionUI_PointInCapsule(uint16_t x,
                                            uint16_t y,
                                            uint16_t left,
                                            uint16_t top,
                                            uint16_t right,
                                            uint16_t bottom)
{
    uint16_t radius;
    uint16_t center_y;

    if ((right <= left) || (bottom <= top))
    {
        return false;
    }

    radius = (bottom - top) / 2U;
    center_y = top + radius;

    if (MotorDirectionUI_PointInRectangle(
            x, y, left + radius, top, right - radius, bottom))
    {
        return true;
    }

    return MotorDirectionUI_PointInCircle(
               x, y, left + radius, center_y, radius) ||
           MotorDirectionUI_PointInCircle(
               x, y, right - radius, center_y, radius);
}

/** @brief 判断点是否位于1像素胶囊形边框上。 */
static bool MotorDirectionUI_PointOnCapsuleBorder(uint16_t x,
                                                  uint16_t y,
                                                  uint16_t left,
                                                  uint16_t top,
                                                  uint16_t right,
                                                  uint16_t bottom)
{
    if (!MotorDirectionUI_PointInCapsule(x, y, left, top, right, bottom))
    {
        return false;
    }

    /* 尺寸过小时不存在有效内层，此时整个胶囊都视为边框。 */
    if (((right - left) < 3U) || ((bottom - top) < 3U))
    {
        return true;
    }

    return !MotorDirectionUI_PointInCapsule(
        x, y, left + 1U, top + 1U, right - 1U, bottom - 1U);
}

/**
 * @brief 判断点是否位于实心三角形内。
 *
 * upward=true时尖端朝上；false时尖端朝下。三角形逐行扩张或收缩，
 * 不需要浮点运算，也不需要在RAM中保存整幅图像。
 */
static bool MotorDirectionUI_PointInTriangle(uint16_t x,
                                             uint16_t y,
                                             uint16_t center_x,
                                             uint16_t top,
                                             uint16_t height,
                                             uint16_t half_width,
                                             bool upward)
{
    uint16_t row;
    uint16_t row_half_width;
    int32_t dx;

    if ((height < 2U) || (y < top) || (y >= (top + height)))
    {
        return false;
    }

    row = y - top;
    if (!upward)
    {
        row = (height - 1U) - row;
    }

    row_half_width = (uint16_t)(((uint32_t)row * half_width) /
                                (uint32_t)(height - 1U));
    dx = (int32_t)x - (int32_t)center_x;

    return (dx >= -(int32_t)row_half_width) &&
           (dx <= (int32_t)row_half_width);
}

/** @brief 查询6x12 ASCII字符串在指定逻辑坐标是否有前景像素。 */
static bool MotorDirectionUI_TextPixel(uint16_t x,
                                       uint16_t y,
                                       uint16_t text_x,
                                       uint16_t text_y,
                                       const char *text)
{
    uint16_t character_index;
    uint16_t local_x;
    uint16_t local_y;
    uint8_t character;
    size_t text_length;

    if ((text == NULL) || (x < text_x) || (y < text_y))
    {
        return false;
    }

    local_x = x - text_x;
    local_y = y - text_y;
    if (local_y >= UI_FONT_HEIGHT)
    {
        return false;
    }

    character_index = local_x / UI_FONT_WIDTH;
    text_length = strlen(text);
    if ((size_t)character_index >= text_length)
    {
        return false;
    }

    local_x %= UI_FONT_WIDTH;
    character = (uint8_t)text[character_index];
    if ((character < (uint8_t)' ') || (character > (uint8_t)'~'))
    {
        return false;
    }

    return (ascii_1206[character - (uint8_t)' '][local_y] &
            (uint8_t)(1U << local_x)) != 0U;
}

/** @brief 生成参考页面在一个240x120逻辑坐标处的RGB565颜色。 */
static uint16_t MotorDirectionUI_GetLogicalPixel(
    uint16_t x,
    uint16_t y,
    const MotorDirectionUiView_t *view)
{
    uint16_t color = UI_COLOR_BACKGROUND;
    uint8_t motor_index;

    /* 顶部与主体使用统一蓝灰色边框，形成分区但不抢占状态色和焦点色。 */
    if (MotorDirectionUI_PointOnRectangleBorder(
            x, y, 0U, 0U, UI_LOGICAL_WIDTH - 1U, UI_HEADER_BOTTOM_Y))
    {
        color = UI_COLOR_HEADER_BORDER;
    }

    if (MotorDirectionUI_PointOnRectangleBorder(
            x, y, 0U, UI_HEADER_BOTTOM_Y, UI_LOGICAL_WIDTH - 1U,
            UI_LOGICAL_HEIGHT - 1U))
    {
        color = UI_COLOR_BODY_BORDER;
    }

    /*
     * 左上角使用胶囊形滑动开关：灰色外层形成轨道边缘，浅灰内层形成
     * 轨道底色。OFF时圆点在左、文字在右；ON时二者交换位置。
     */
    if (MotorDirectionUI_PointInCapsule(x, y, 2U, 3U, 43U, 19U))
    {
        color = UI_COLOR_SEPARATOR;
    }

    if (MotorDirectionUI_PointInCapsule(x, y, 3U, 4U, 42U, 18U))
    {
        color = UI_COLOR_STATUS_BACKGROUND;
    }

    if (MotorDirectionUI_TextPixel(
            x, y,
            (view->power_state == MOTOR_DIRECTION_UI_POWER_ON) ? 5U : 22U,
            5U,
            (view->power_state == MOTOR_DIRECTION_UI_POWER_ON) ? "ON" : "OFF"))
    {
        color = (view->power_state == MOTOR_DIRECTION_UI_POWER_ON)
                    ? UI_COLOR_STATUS_ON
                    : UI_COLOR_STATUS_OFF;
    }

    if (MotorDirectionUI_PointInCircle(
            x, y,
            (view->power_state == MOTOR_DIRECTION_UI_POWER_ON) ? 34U : 11U,
            11U,
            7U))
    {
        color = (view->power_state == MOTOR_DIRECTION_UI_POWER_ON)
                    ? UI_COLOR_STATUS_ON
                    : UI_COLOR_STATUS_OFF;
    }

    /*
     * 开关焦点使用45x21胶囊形轮廓，大小覆盖整个滑动开关，同时与页面
     * 外框保留间隔。它只表示当前操作目标，不改变OFF/ON状态颜色。
     */
    if ((view->focus == MOTOR_DIRECTION_UI_FOCUS_SWITCH) &&
        MotorDirectionUI_PointOnCapsuleBorder(x, y, 1U, 1U, 45U, 21U))
    {
        color = UI_COLOR_FOCUS_SWITCH;
    }

    /* 顶部图例：上三角=NOR，下三角=REV，中间竖线用于视觉分隔。 */
    if (MotorDirectionUI_PointInTriangle(
            x, y, 95U, 6U, 16U, 10U, true))
    {
        color = UI_COLOR_UP;
    }

    if (MotorDirectionUI_TextPixel(x, y, 109U, 5U, "NOR"))
    {
        color = UI_COLOR_UP;
    }

    if (MotorDirectionUI_PointInRectangle(x, y, 154U, 3U, 155U, 27U))
    {
        color = UI_COLOR_SEPARATOR;
    }

    if (MotorDirectionUI_PointInTriangle(
            x, y, 174U, 6U, 16U, 10U, false))
    {
        color = UI_COLOR_DOWN;
    }

    if (MotorDirectionUI_TextPixel(x, y, 188U, 5U, "REV"))
    {
        color = UI_COLOR_DOWN;
    }

    for (motor_index = 0U; motor_index < UI_MOTOR_COUNT; ++motor_index)
    {
        const uint16_t center_x =
            UI_MOTOR_FIRST_CENTER_X +
            ((uint16_t)motor_index * UI_MOTOR_COLUMN_PITCH);
        const char motor_number[2] = {
            (char)((uint8_t)'1' + motor_index),
            '\0'
        };

        /* 通道焦点使用柔白色19x19方框，与青色编号和蓝灰外框区分。 */
        if ((view->focus == MOTOR_DIRECTION_UI_FOCUS_MOTOR) &&
            (view->selected_motor == (uint8_t)(motor_index + 1U)) &&
            MotorDirectionUI_PointOnRectangleBorder(
                x, y, center_x - 9U, 37U, center_x + 9U, 55U))
        {
            color = UI_COLOR_FOCUS_MOTOR;
        }

        if (MotorDirectionUI_TextPixel(
                x, y, center_x - 3U, UI_MOTOR_NUMBER_Y, motor_number))
        {
            color = UI_COLOR_NUMBER;
        }

        if (MotorDirectionUI_PointInTriangle(
                x, y, center_x, UI_MAIN_UP_TRIANGLE_TOP_Y,
                UI_MAIN_TRIANGLE_HEIGHT, UI_MAIN_TRIANGLE_HALF_WIDTH, true))
        {
            color = UI_COLOR_UP;
        }

        if (MotorDirectionUI_PointInTriangle(
                x, y, center_x, UI_MAIN_DOWN_TRIANGLE_TOP_Y,
                UI_MAIN_TRIANGLE_HEIGHT, UI_MAIN_TRIANGLE_HALF_WIDTH, false))
        {
            color = UI_COLOR_DOWN;
        }

        /*
         * 方向焦点是27x24矩形，比数字焦点明显更大，并包围所选通道的
         * NOR或REV三角形。UP固定选择上方NOR，DOWN固定选择下方REV。
         */
        if ((view->focus == MOTOR_DIRECTION_UI_FOCUS_DIRECTION) &&
            (view->selected_motor == (uint8_t)(motor_index + 1U)))
        {
            const uint16_t focus_top =
                (view->selected_direction == MOTOR_DIRECTION_UI_NORMAL)
                    ? 55U
                    : 90U;
            const uint16_t focus_bottom =
                (view->selected_direction == MOTOR_DIRECTION_UI_NORMAL)
                    ? 78U
                    : 113U;

            if (MotorDirectionUI_PointOnRectangleBorder(
                    x, y, center_x - 13U, focus_top,
                    center_x + 13U, focus_bottom))
            {
                color = UI_COLOR_FOCUS_DIRECTION;
            }
        }
    }

    return color;
}

/** @brief 复制并校验视图状态，非法字段回退到安全显示值。 */
static void MotorDirectionUI_ValidateView(
    const MotorDirectionUiView_t *source,
    MotorDirectionUiView_t *destination)
{
    if (destination == NULL)
    {
        return;
    }

    *destination = (source != NULL) ? *source : g_safe_default_view;

    if ((destination->power_state != MOTOR_DIRECTION_UI_POWER_OFF) &&
        (destination->power_state != MOTOR_DIRECTION_UI_POWER_ON))
    {
        destination->power_state = MOTOR_DIRECTION_UI_POWER_OFF;
    }
    if ((destination->focus != MOTOR_DIRECTION_UI_FOCUS_SWITCH) &&
        (destination->focus != MOTOR_DIRECTION_UI_FOCUS_MOTOR) &&
        (destination->focus != MOTOR_DIRECTION_UI_FOCUS_DIRECTION))
    {
        destination->focus = MOTOR_DIRECTION_UI_FOCUS_SWITCH;
    }
    if ((destination->selected_motor < 1U) ||
        (destination->selected_motor > UI_MOTOR_COUNT))
    {
        destination->selected_motor = 1U;
    }
    if ((destination->selected_direction != MOTOR_DIRECTION_UI_NORMAL) &&
        (destination->selected_direction != MOTOR_DIRECTION_UI_REVERSED))
    {
        destination->selected_direction = MOTOR_DIRECTION_UI_NORMAL;
    }
}

/**
 * @brief 绘制一个包含端点的240x120逻辑矩形区域。
 *
 * 逻辑画布旋转90度写入物理显存。每生成一条物理扫描行后，将该行全部
 * RGB565字节交给LCD_WriteDataBuffer()一次发送，避免逐像素调用HAL。
 */
static void MotorDirectionUI_DrawLogicalRegion(
    const MotorDirectionUiView_t *view,
    uint16_t logical_left,
    uint16_t logical_top,
    uint16_t logical_right,
    uint16_t logical_bottom)
{
    uint16_t physical_left;
    uint16_t physical_right;
    uint16_t physical_top;
    uint16_t physical_bottom;

    if ((view == NULL) ||
        (logical_left > logical_right) ||
        (logical_top > logical_bottom) ||
        (logical_left >= UI_LOGICAL_WIDTH) ||
        (logical_top >= UI_LOGICAL_HEIGHT))
    {
        return;
    }

    if (logical_right >= UI_LOGICAL_WIDTH)
    {
        logical_right = UI_LOGICAL_WIDTH - 1U;
    }
    if (logical_bottom >= UI_LOGICAL_HEIGHT)
    {
        logical_bottom = UI_LOGICAL_HEIGHT - 1U;
    }

    /* logical_x=physical_y，logical_y=119-physical_x。 */
    physical_left = (UI_LOGICAL_HEIGHT - 1U) - logical_bottom;
    physical_right = (UI_LOGICAL_HEIGHT - 1U) - logical_top;
    physical_top = logical_left;
    physical_bottom = logical_right;

    /*
     * SH8501显存列窗口按4像素边界处理。向外扩展局部区域不会改变业务
     * 内容，只会多重画边缘最多3列，并避免非对齐列地址造成显示异常。
     */
    physical_left &= (uint16_t)~3U;
    physical_right |= 3U;
    if (physical_right >= LCD_W)
    {
        physical_right = LCD_W - 1U;
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
                (UI_LOGICAL_HEIGHT - 1U) - physical_x;
            const uint16_t color = MotorDirectionUI_GetLogicalPixel(
                logical_x, logical_y, view);

            g_ui_physical_row_buffer[buffer_index++] =
                (uint8_t)(color >> 8);
            g_ui_physical_row_buffer[buffer_index++] = (uint8_t)color;
        }

        LCD_WriteDataBuffer(g_ui_physical_row_buffer, buffer_index);
    }
}

/** @brief 用当前视图内容重画某个旧/新焦点可能覆盖的最小区域。 */
static void MotorDirectionUI_DrawFocusRegion(
    const MotorDirectionUiView_t *current_view,
    MotorDirectionUiFocus_t focus,
    uint8_t motor,
    MotorDirectionUiDirection_t direction)
{
    if (focus == MOTOR_DIRECTION_UI_FOCUS_SWITCH)
    {
        MotorDirectionUI_DrawLogicalRegion(
            current_view, 0U, 0U, 46U, 22U);
    }
    else if (focus == MOTOR_DIRECTION_UI_FOCUS_MOTOR)
    {
        const uint16_t center_x = UI_MOTOR_FIRST_CENTER_X +
            ((uint16_t)(motor - 1U) * UI_MOTOR_COLUMN_PITCH);

        MotorDirectionUI_DrawLogicalRegion(
            current_view, center_x - 10U, 36U, center_x + 10U, 56U);
    }
    else if (focus == MOTOR_DIRECTION_UI_FOCUS_DIRECTION)
    {
        const uint16_t center_x = UI_MOTOR_FIRST_CENTER_X +
            ((uint16_t)(motor - 1U) * UI_MOTOR_COLUMN_PITCH);
        const uint16_t top =
            (direction == MOTOR_DIRECTION_UI_NORMAL) ? 54U : 89U;
        const uint16_t bottom =
            (direction == MOTOR_DIRECTION_UI_NORMAL) ? 79U : 114U;

        MotorDirectionUI_DrawLogicalRegion(
            current_view, center_x - 14U, top, center_x + 14U, bottom);
    }
    else
    {
        /* 状态已经过校验，正常路径不会到达这里。 */
    }
}

void MotorDirectionUI_Draw(const MotorDirectionUiView_t *view)
{
    MotorDirectionUiView_t validated_view;

    /* NULL和任何非法字段都回退到安全状态，绝不错误显示为已经ON。 */
    MotorDirectionUI_ValidateView(view, &validated_view);

    /*
     * 初次显示仍绘制完整页面，但底层已经改为每条扫描行批量发送，而不是
     * 每像素调用两次HAL_SPI_Transmit。
     */
    MotorDirectionUI_DrawLogicalRegion(
        &validated_view,
        0U,
        0U,
        UI_LOGICAL_WIDTH - 1U,
        UI_LOGICAL_HEIGHT - 1U);
}

void MotorDirectionUI_Update(const MotorDirectionUiView_t *previous_view,
                             const MotorDirectionUiView_t *current_view)
{
    MotorDirectionUiView_t previous;
    MotorDirectionUiView_t current;

    if ((previous_view == NULL) || (current_view == NULL))
    {
        MotorDirectionUI_Draw(current_view);
        return;
    }

    MotorDirectionUI_ValidateView(previous_view, &previous);
    MotorDirectionUI_ValidateView(current_view, &current);

    /* OFF/ON文字和滑块位置变化时必须重画完整开关区域。 */
    if (previous.power_state != current.power_state)
    {
        MotorDirectionUI_DrawLogicalRegion(
            &current, 0U, 0U, 46U, 22U);
    }

    if ((previous.focus != current.focus) ||
        (previous.selected_motor != current.selected_motor) ||
        (previous.selected_direction != current.selected_direction))
    {
        /* 先用新页面状态擦除旧焦点，再绘制新焦点。 */
        MotorDirectionUI_DrawFocusRegion(
            &current,
            previous.focus,
            previous.selected_motor,
            previous.selected_direction);
        MotorDirectionUI_DrawFocusRegion(
            &current,
            current.focus,
            current.selected_motor,
            current.selected_direction);
    }
}
