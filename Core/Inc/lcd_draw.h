#ifndef LCD_DRAW_H
#define LCD_DRAW_H

#include "lcd_init.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 显示一个 ASCII 字符。
 * @param sizey 字高，仅支持 16、24、32；字符宽度为字高的一半。
 */
void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t ch,
                  uint16_t foreground, uint16_t background, uint8_t sizey);

/** @brief 显示以 '\0' 结尾的 ASCII 字符串，sizey 支持 16、24、32。 */
void LCD_ShowString(uint16_t x, uint16_t y, const char *text,
                    uint16_t foreground, uint16_t background, uint16_t sizey);

/** @brief 按固定 len 位显示无符号整数，前导零以空格代替。 */
void LCD_ShowNum(uint16_t x, uint16_t y, uint32_t number, uint8_t len,
                 uint16_t foreground, uint16_t background, uint8_t sizey);

/**
 * @brief 显示非负浮点数。
 * @param precision 小数位数。
 * @param len 不含小数点的总数字位数。
 */
void LCD_ShowFloatNum(uint16_t x, uint16_t y, float number,
                      uint8_t precision, uint8_t len,
                      uint16_t foreground, uint16_t background, uint8_t sizey);

/* 中文字模使用 UTF-8 三字节编码索引，字高支持 12/16/24/32。 */
void LCD_ShowChinese12x12(uint16_t x, uint16_t y, const char *text,
                          uint16_t foreground, uint16_t background, uint8_t sizey);
void LCD_ShowChinese16x16(uint16_t x, uint16_t y, const char *text,
                          uint16_t foreground, uint16_t background, uint8_t sizey);
void LCD_ShowChinese24x24(uint16_t x, uint16_t y, const char *text,
                          uint16_t foreground, uint16_t background, uint8_t sizey);
void LCD_ShowChinese32x32(uint16_t x, uint16_t y, const char *text,
                          uint16_t foreground, uint16_t background, uint8_t sizey);
void LCD_ShowChinese(uint16_t x, uint16_t y, const char *text,
                     uint16_t foreground, uint16_t background, uint8_t sizey);

/** @brief 显示 UTF-8 中英文混合字符串。 */
void LCD_ShowStr(uint16_t x, uint16_t y, const char *text,
                 uint16_t foreground, uint16_t background, uint8_t sizey);

/** @brief 将字符串按屏幕宽度居中显示；x 参数为兼容原示例而保留。 */
void LCD_StrCenter(uint16_t x, uint16_t y, const char *text,
                   uint16_t foreground, uint16_t background, uint8_t sizey);

#ifdef __cplusplus
}
#endif

#endif /* LCD_DRAW_H */
