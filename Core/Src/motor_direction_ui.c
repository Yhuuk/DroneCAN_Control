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
#define UI_COLOR_FOCUS               YELLOW
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
    MotorDirectionUiPowerState_t power_state,
    uint8_t selected_motor)
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
            (power_state == MOTOR_DIRECTION_UI_POWER_ON) ? 5U : 22U,
            5U,
            (power_state == MOTOR_DIRECTION_UI_POWER_ON) ? "ON" : "OFF"))
    {
        color = (power_state == MOTOR_DIRECTION_UI_POWER_ON)
                    ? UI_COLOR_STATUS_ON
                    : UI_COLOR_STATUS_OFF;
    }

    if (MotorDirectionUI_PointInCircle(
            x, y,
            (power_state == MOTOR_DIRECTION_UI_POWER_ON) ? 34U : 11U,
            11U,
            7U))
    {
        color = (power_state == MOTOR_DIRECTION_UI_POWER_ON)
                    ? UI_COLOR_STATUS_ON
                    : UI_COLOR_STATUS_OFF;
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

        /* 当前选中的电机编号外侧显示黄色方框，与蓝灰色页面边框区分。 */
        if ((selected_motor == (uint8_t)(motor_index + 1U)) &&
            MotorDirectionUI_PointOnRectangleBorder(
                x, y, center_x - 9U, 37U, center_x + 9U, 55U))
        {
            color = UI_COLOR_FOCUS;
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
    }

    return color;
}

void MotorDirectionUI_Draw(MotorDirectionUiPowerState_t power_state,
                           uint8_t selected_motor)
{
    uint16_t physical_x;
    uint16_t physical_y;

    /* 非法状态按更安全的OFF显示，避免界面错误地提示输出已经启用。 */
    if ((power_state != MOTOR_DIRECTION_UI_POWER_OFF) &&
        (power_state != MOTOR_DIRECTION_UI_POWER_ON))
    {
        power_state = MOTOR_DIRECTION_UI_POWER_OFF;
    }

    /*
     * 一次性选择完整的120x240物理显存窗口，然后按控制器要求的行优先顺序
     * 连续写像素。相比逐点调用LCD_Address_Set，此方式只设置一次窗口。
     */
    LCD_Address_Set(0U, 0U, LCD_W - 1U, LCD_H - 1U);

    for (physical_y = 0U; physical_y < LCD_H; ++physical_y)
    {
        for (physical_x = 0U; physical_x < LCD_W; ++physical_x)
        {
            /*
             * 240x120逻辑画布顺时针旋转后写入120x240显存：
             * logical_x = physical_y
             * logical_y = 119 - physical_x
             */
            const uint16_t logical_x = physical_y;
            const uint16_t logical_y =
                (UI_LOGICAL_HEIGHT - 1U) - physical_x;

            LCD_WR_DATA(MotorDirectionUI_GetLogicalPixel(
                logical_x, logical_y, power_state, selected_motor));
        }
    }
}
