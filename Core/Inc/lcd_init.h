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

/**
 * @brief 设置SH8501A的全局显示亮度。
 *
 * 该接口发送WRDISBV(0x51)及一个8位DBV参数。0x00表示最低亮度，
 * 0xFF表示最高亮度；具体亮度与DBV之间的非线性关系由屏幕模组决定。
 * 函数内部会获取SPI1总线互斥锁，调用者不能在已经持有该锁时调用。
 *
 * @param brightness 0x00~0xFF全局亮度控制值。
 * @retval HAL_OK 指令已经发送；HAL_ERROR 无法获得SPI1总线。
 */
HAL_StatusTypeDef LCD_SetBrightness(uint8_t brightness);

/** 设置包含端点的显示窗口，并进入显存写入模式。 */
void LCD_Address_Set(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye);

/** 用 RGB565 颜色填充 [xs, xe) x [ys, ye) 区域。 */
void LCD_Fill(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye, uint16_t color);

/* 以下为 lcd_draw.c 使用的底层写接口，应用层通常无需直接调用。 */
void LCD_WR_REG(uint8_t reg);
void LCD_WR_DATA8(uint8_t data);
void LCD_WR_DATA(uint16_t data);

/**
 * @brief 在一次CS低电平事务中连续写入一批显存数据。
 *
 * 数据必须已经按照屏幕要求排列，例如RGB565像素采用高字节在前。函数
 * 当前使用阻塞式硬件SPI，但一次HAL调用可以发送整批数据，适合扫描行或
 * 图像块刷新，避免逐字节调用HAL带来的巨大软件开销。
 *
 * @param data   待发送数据地址；NULL时不发送。
 * @param length 数据字节数；0时不发送。
 */
void LCD_WriteDataBuffer(const uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* LCD_INIT_H */
