#ifndef LCD_INIT_H
#define LCD_INIT_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 0：竖屏正向；1：竖屏旋转 180°。修改后需重新编译。 */
#define USE_HORIZONTAL 0U

/* 当前 0.95 英寸屏幕的逻辑分辨率。 */
#define LCD_W 120U
#define LCD_H 240U

/* 常用 RGB565 颜色。 */
#define WHITE       0xFFFFU
#define BLACK       0x0000U
#define BLUE        0x001FU
#define BRED        0xF81FU
#define GRED        0xFFE0U
#define GBLUE       0x07FFU
#define RED         0xF800U
#define MAGENTA     0xF81FU
#define GREEN       0x07E0U
#define CYAN        0x7FFFU
#define YELLOW      0xFFE0U
#define BROWN       0xBC40U
#define BRRED       0xFC07U
#define GRAY        0x8430U
#define DARKBLUE    0x01CFU
#define LIGHTBLUE   0x7D7CU
#define GRAYBLUE    0x5458U
#define LIGHTGREEN  0x841FU
#define LGRAY       0xC618U
#define LGRAYBLUE   0xA651U
#define LBBLUE      0x2B12U

/** 初始化屏幕控制器。调用前必须完成 MX_GPIO_Init() 和 MX_SPI1_Init()。 */
void LCD_Init(void);

/** 设置包含端点的显示窗口，并进入显存写入模式。 */
void LCD_Address_Set(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye);

/** 用 RGB565 颜色填充 [xs, xe) x [ys, ye) 区域。 */
void LCD_Fill(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye, uint16_t color);

/* 以下为 lcd_draw.c 使用的底层写接口，应用层通常无需直接调用。 */
void LCD_WR_REG(uint8_t reg);
void LCD_WR_DATA8(uint8_t data);
void LCD_WR_DATA(uint16_t data);

#ifdef __cplusplus
}
#endif

#endif /* LCD_INIT_H */
