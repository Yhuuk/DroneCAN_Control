#include "brightness_ui.h"

#include "lcd_init.h"
#include "spi1_bus.h"

#include <stdbool.h>
#include <stddef.h>

/* 与现有三个页面一致：先在240x120逻辑画布生成，再旋转写入物理显存。 */
#define BRIGHTNESS_UI_LOGICAL_WIDTH    240U
#define BRIGHTNESS_UI_LOGICAL_HEIGHT   120U
#define BRIGHTNESS_UI_FONT_WIDTH         8U
#define BRIGHTNESS_UI_FONT_HEIGHT       16U
#define BRIGHTNESS_UI_LARGE_FONT_WIDTH  12U
#define BRIGHTNESS_UI_LARGE_FONT_HEIGHT 24U

#define BRIGHTNESS_UI_BAR_LEFT          20U
#define BRIGHTNESS_UI_BAR_TOP           70U
#define BRIGHTNESS_UI_BAR_RIGHT        219U
#define BRIGHTNESS_UI_BAR_BOTTOM        91U
#define BRIGHTNESS_UI_BAR_BORDER_WIDTH   2U

#define BRIGHTNESS_UI_COLOR_BACKGROUND 0x1657U /* 与主页面一致的青绿色。 */
#define BRIGHTNESS_UI_COLOR_TEXT       BLACK
#define BRIGHTNESS_UI_COLOR_VALUE      WHITE
#define BRIGHTNESS_UI_COLOR_BAR_BORDER WHITE
#define BRIGHTNESS_UI_COLOR_BAR_FILL   YELLOW

extern const unsigned char ascii_1608[][16];
extern const unsigned char ascii_2412[][48];

/* 页面只由UiTask绘制，单行缓冲区无需额外互斥。 */
static uint8_t g_brightness_ui_line_buffer[LCD_W * 2U];

static bool BrightnessUI_PointInRoundedRectangle(
    uint16_t x,
    uint16_t y,
    uint16_t left,
    uint16_t top,
    uint16_t right,
    uint16_t bottom,
    uint16_t radius)
{
    uint16_t corner_x;
    uint16_t corner_y;
    int32_t dx;
    int32_t dy;

    if ((x < left) || (x > right) || (y < top) || (y > bottom))
    {
        return false;
    }

    if ((x >= (uint16_t)(left + radius)) &&
        (x <= (uint16_t)(right - radius)))
    {
        return true;
    }
    if ((y >= (uint16_t)(top + radius)) &&
        (y <= (uint16_t)(bottom - radius)))
    {
        return true;
    }

    corner_x = (x < (uint16_t)(left + radius))
                   ? (uint16_t)(left + radius)
                   : (uint16_t)(right - radius);
    corner_y = (y < (uint16_t)(top + radius))
                   ? (uint16_t)(top + radius)
                   : (uint16_t)(bottom - radius);
    dx = (int32_t)x - (int32_t)corner_x;
    dy = (int32_t)y - (int32_t)corner_y;

    return ((dx * dx) + (dy * dy)) <=
           ((int32_t)radius * (int32_t)radius);
}

static bool BrightnessUI_PointOnRoundedRectangleBorder(
    uint16_t x,
    uint16_t y,
    uint16_t left,
    uint16_t top,
    uint16_t right,
    uint16_t bottom,
    uint16_t radius,
    uint16_t border_width)
{
    if (!BrightnessUI_PointInRoundedRectangle(
            x, y, left, top, right, bottom, radius))
    {
        return false;
    }

    if ((right <= (uint16_t)(left + (border_width * 2U))) ||
        (bottom <= (uint16_t)(top + (border_width * 2U))))
    {
        return true;
    }

    return !BrightnessUI_PointInRoundedRectangle(
        x,
        y,
        (uint16_t)(left + border_width),
        (uint16_t)(top + border_width),
        (uint16_t)(right - border_width),
        (uint16_t)(bottom - border_width),
        (radius > border_width) ? (uint16_t)(radius - border_width) : 0U);
}

static bool BrightnessUI_TextPixel(uint16_t x,
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

    local_x = x - text_x;
    local_y = y - text_y;
    if (local_y >= BRIGHTNESS_UI_FONT_HEIGHT)
    {
        return false;
    }

    while (text[text_length] != '\0')
    {
        ++text_length;
    }
    character_index = local_x / BRIGHTNESS_UI_FONT_WIDTH;
    if (character_index >= text_length)
    {
        return false;
    }

    local_x %= BRIGHTNESS_UI_FONT_WIDTH;
    character = (uint8_t)text[character_index];
    if ((character < (uint8_t)' ') || (character > (uint8_t)'~'))
    {
        return false;
    }

    return ((ascii_1608[character - (uint8_t)' '][local_y] >> local_x) &
            0x01U) != 0U;
}

static bool BrightnessUI_LargeTextPixel(uint16_t x,
                                        uint16_t y,
                                        uint16_t text_x,
                                        uint16_t text_y,
                                        const char *text)
{
    uint16_t local_x;
    uint16_t local_y;
    uint16_t character_index;
    uint16_t glyph_byte_index;
    uint16_t text_length = 0U;
    uint8_t character;

    if ((text == NULL) || (x < text_x) || (y < text_y))
    {
        return false;
    }

    local_x = x - text_x;
    local_y = y - text_y;
    if (local_y >= BRIGHTNESS_UI_LARGE_FONT_HEIGHT)
    {
        return false;
    }

    while (text[text_length] != '\0')
    {
        ++text_length;
    }
    character_index = local_x / BRIGHTNESS_UI_LARGE_FONT_WIDTH;
    if (character_index >= text_length)
    {
        return false;
    }

    local_x %= BRIGHTNESS_UI_LARGE_FONT_WIDTH;
    character = (uint8_t)text[character_index];
    if ((character < (uint8_t)' ') || (character > (uint8_t)'~'))
    {
        return false;
    }

    glyph_byte_index = (uint16_t)((local_y * 2U) + (local_x / 8U));
    return (ascii_2412[character - (uint8_t)' '][glyph_byte_index] &
            (uint8_t)(1U << (local_x % 8U))) != 0U;
}

static void BrightnessUI_FormatUint8(uint8_t value, char text[4])
{
    if (value >= 100U)
    {
        text[0] = (char)('0' + (value / 100U));
        text[1] = (char)('0' + ((value / 10U) % 10U));
        text[2] = (char)('0' + (value % 10U));
        text[3] = '\0';
    }
    else if (value >= 10U)
    {
        text[0] = (char)('0' + (value / 10U));
        text[1] = (char)('0' + (value % 10U));
        text[2] = '\0';
        text[3] = '\0';
    }
    else
    {
        text[0] = (char)('0' + value);
        text[1] = '\0';
        text[2] = '\0';
        text[3] = '\0';
    }
}

static void BrightnessUI_FormatPercent(uint8_t brightness, char text[5])
{
    const uint8_t percent =
        (uint8_t)((((uint32_t)brightness * 100U) + 127U) / 255U);
    char number[4];
    uint8_t index = 0U;

    BrightnessUI_FormatUint8(percent, number);
    while ((number[index] != '\0') && (index < 3U))
    {
        text[index] = number[index];
        ++index;
    }
    text[index++] = '%';
    text[index] = '\0';
}

static uint16_t BrightnessUI_TextWidth(const char *text,
                                       uint16_t character_width)
{
    uint16_t length = 0U;

    if (text != NULL)
    {
        while (text[length] != '\0')
        {
            ++length;
        }
    }
    return (uint16_t)(length * character_width);
}

static uint16_t BrightnessUI_GetLogicalPixel(
    uint16_t x,
    uint16_t y,
    const BrightnessUiView_t *view,
    const char *percent_text,
    const char *raw_text)
{
    uint16_t color = BRIGHTNESS_UI_COLOR_BACKGROUND;
    const uint16_t percent_x =
        (BRIGHTNESS_UI_LOGICAL_WIDTH -
         BrightnessUI_TextWidth(percent_text,
                                BRIGHTNESS_UI_LARGE_FONT_WIDTH)) /
        2U;
    const uint16_t raw_x =
        (BRIGHTNESS_UI_LOGICAL_WIDTH -
         BrightnessUI_TextWidth(raw_text, BRIGHTNESS_UI_FONT_WIDTH)) /
        2U;
    const uint16_t inner_width =
        BRIGHTNESS_UI_BAR_RIGHT - BRIGHTNESS_UI_BAR_LEFT -
        (BRIGHTNESS_UI_BAR_BORDER_WIDTH * 2U);
    const uint16_t fill_width =
        (uint16_t)((((uint32_t)inner_width * view->brightness) + 127U) /
                   255U);

    if (BrightnessUI_TextPixel(x, y, 80U, 4U, "BRIGHTNESS") ||
        BrightnessUI_TextPixel(x, y, 56U, 101U, "UP/DOWN     BACK"))
    {
        color = BRIGHTNESS_UI_COLOR_TEXT;
    }

    if (BrightnessUI_LargeTextPixel(
            x, y, percent_x, 28U, percent_text) ||
        BrightnessUI_TextPixel(x, y, raw_x, 52U, raw_text))
    {
        color = BRIGHTNESS_UI_COLOR_VALUE;
    }

    if (BrightnessUI_PointOnRoundedRectangleBorder(
            x,
            y,
            BRIGHTNESS_UI_BAR_LEFT,
            BRIGHTNESS_UI_BAR_TOP,
            BRIGHTNESS_UI_BAR_RIGHT,
            BRIGHTNESS_UI_BAR_BOTTOM,
            6U,
            BRIGHTNESS_UI_BAR_BORDER_WIDTH))
    {
        color = BRIGHTNESS_UI_COLOR_BAR_BORDER;
    }

    if ((fill_width > 0U) &&
        (x >= (uint16_t)(BRIGHTNESS_UI_BAR_LEFT +
                         BRIGHTNESS_UI_BAR_BORDER_WIDTH)) &&
        (x < (uint16_t)(BRIGHTNESS_UI_BAR_LEFT +
                        BRIGHTNESS_UI_BAR_BORDER_WIDTH + fill_width)) &&
        (y >= (uint16_t)(BRIGHTNESS_UI_BAR_TOP +
                         BRIGHTNESS_UI_BAR_BORDER_WIDTH)) &&
        (y <= (uint16_t)(BRIGHTNESS_UI_BAR_BOTTOM -
                         BRIGHTNESS_UI_BAR_BORDER_WIDTH)))
    {
        color = BRIGHTNESS_UI_COLOR_BAR_FILL;
    }

    return color;
}

static void BrightnessUI_DrawLogicalRegion(const BrightnessUiView_t *view,
                                           uint16_t logical_left,
                                           uint16_t logical_top,
                                           uint16_t logical_right,
                                           uint16_t logical_bottom)
{
    static const BrightnessUiView_t safe_default = {.brightness = 0xFFU};
    const BrightnessUiView_t *const active_view =
        (view != NULL) ? view : &safe_default;
    char percent_text[5];
    char brightness_number[4];
    char raw_text[8] = {'V', 'A', 'L', ':', '0', '\0', '\0', '\0'};
    uint16_t physical_left;
    uint16_t physical_right;
    uint16_t physical_top;
    uint16_t physical_bottom;

    if ((logical_left > logical_right) || (logical_top > logical_bottom) ||
        (logical_left >= BRIGHTNESS_UI_LOGICAL_WIDTH) ||
        (logical_top >= BRIGHTNESS_UI_LOGICAL_HEIGHT))
    {
        return;
    }

    if (logical_right >= BRIGHTNESS_UI_LOGICAL_WIDTH)
    {
        logical_right = BRIGHTNESS_UI_LOGICAL_WIDTH - 1U;
    }
    if (logical_bottom >= BRIGHTNESS_UI_LOGICAL_HEIGHT)
    {
        logical_bottom = BRIGHTNESS_UI_LOGICAL_HEIGHT - 1U;
    }

    BrightnessUI_FormatPercent(active_view->brightness, percent_text);
    BrightnessUI_FormatUint8(active_view->brightness, brightness_number);
    for (uint8_t index = 0U;
         (index < 3U) && (brightness_number[index] != '\0');
         ++index)
    {
        raw_text[4U + index] = brightness_number[index];
        raw_text[5U + index] = '\0';
    }

    physical_left = (BRIGHTNESS_UI_LOGICAL_HEIGHT - 1U) - logical_bottom;
    physical_right = (BRIGHTNESS_UI_LOGICAL_HEIGHT - 1U) - logical_top;
    physical_top = logical_left;
    physical_bottom = logical_right;

    /* SH8501物理列窗口按4像素对齐，扩展像素仍由页面生成器正确恢复。 */
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
                (BRIGHTNESS_UI_LOGICAL_HEIGHT - 1U) - physical_x;
            const uint16_t pixel = BrightnessUI_GetLogicalPixel(
                logical_x,
                logical_y,
                active_view,
                percent_text,
                raw_text);

            g_brightness_ui_line_buffer[buffer_index++] =
                (uint8_t)(pixel >> 8U);
            g_brightness_ui_line_buffer[buffer_index++] = (uint8_t)pixel;
        }

        LCD_WriteDataBuffer(g_brightness_ui_line_buffer, buffer_index);
    }

    SPI1_Bus_Release();
}

void BrightnessUI_Draw(const BrightnessUiView_t *view)
{
    BrightnessUI_DrawLogicalRegion(view,
                                   0U,
                                   0U,
                                   BRIGHTNESS_UI_LOGICAL_WIDTH - 1U,
                                   BRIGHTNESS_UI_LOGICAL_HEIGHT - 1U);
}

void BrightnessUI_Update(const BrightnessUiView_t *previous,
                         const BrightnessUiView_t *current)
{
    if ((previous == NULL) || (current == NULL))
    {
        BrightnessUI_Draw(current);
        return;
    }

    if (previous->brightness == current->brightness)
    {
        return;
    }

    /*
     * 标题和底部按键提示都是固定内容。亮度变化只重画百分比、原始值和
     * 进度条，避免每次按UP/DOWN都发送完整的240x120画面。
     */
    BrightnessUI_DrawLogicalRegion(current,
                                   0U,
                                   26U,
                                   BRIGHTNESS_UI_LOGICAL_WIDTH - 1U,
                                   93U);
}
