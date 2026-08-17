#include "lcd_draw.h"
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-braces"
#endif
#include "lcd_font.h"
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
#include <string.h>



/**
 * @brief       显示单个字符
 * @param       x:字符显示位置列起始坐标
 * @param       y:字符显示位置行起始坐标
 * @param       num:显示字符的ASCII码
 * @param       fc:字符颜色
 * @param       bc:字符背景颜色
 * @param       sizey:字符大小
 * @retval      无
 */
void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    uint8_t temp, sizex, t, m = 0;
    uint16_t i, TypefaceNum; // 一个字符所占字节大小

    if ((num < (uint8_t)' ') || (num > (uint8_t)'~') ||
        ((sizey != 16U) && (sizey != 24U) && (sizey != 32U)))
    {
        return;
    }

    sizex = sizey / 2;
    TypefaceNum = (sizex / 8 + ((sizex % 8) ? 1 : 0)) * sizey;
    num = num - ' ';                                     // 得到偏移后的值
    LCD_Address_Set(x, y, x + sizex - 1, y + sizey - 1); // 设置显示窗口
    for (i = 0; i < TypefaceNum; i++)
    {
        if (sizey == 16)
            temp = ascii_1608[num][i]; // 调用8x16字体
        else if (sizey == 24)
            temp = ascii_2412[num][i]; // 调用12x24字体
        else if (sizey == 32)
            temp = ascii_3216[num][i]; // 调用16x32字体
        else
            return;
        for (t = 0; t < 8; t++)
        {
                if (temp & (0x01 << t))
                    LCD_WR_DATA(fc);
                else
                    LCD_WR_DATA(bc);
                m++;
                if (m % sizex == 0)
                {
                    m = 0;
                    break;
                }
        }
    }
}

/**
 * @brief       显示字符串
 * @param       x:字符串显示位置列起始坐标
 * @param       y:字符串显示位置行起始坐标
 * @param       *s:显示的字符串内容
 * @param       fc:字符颜色
 * @param       bc:字符背景颜色
 * @param       sizey:字符大小
 * @retval      无
 */
void LCD_ShowString(uint16_t x, uint16_t y,const char *s, uint16_t fc, uint16_t bc, uint16_t sizey)
{
    while ((*s <= '~') && (*s >= ' ')) // 判断是不是非法字符
    {
        if (x > (LCD_W - 1) || y > (LCD_H - 1))
            return;
        LCD_ShowChar(x, y, *s, fc, bc, sizey);
        x += sizey / 2;
        s++;
    }
}

/**
 * @brief       幂运算(内部调用)
 * @param       m:底数
 * @param       n:指数
 * @retval      result:m的n次幂
 */
static uint32_t LCD_PowU8(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--)
    {
        result *= m;
    }
    return result;
}

/**
 * @brief       显示数字
 * @param       x:数字显示位置列起始坐标
 * @param       y:数字显示位置行起始坐标
 * @param       num:显示的数字(0~4294967295)
 * @param       len:显示数字的位数
 * @param       fc:字符颜色
 * @param       bc:字符背景颜色
 * @param       sizey:字符大小
 * @retval      无
 */
void LCD_ShowNum(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    uint8_t t, temp, enshow = 0;
    uint8_t sizex = sizey / 2;
    for (t = 0; t < len; t++)
    {
        temp = (num / LCD_PowU8(10U, (uint8_t)(len - t - 1U))) % 10U;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                LCD_ShowChar(x + t * sizex, y, ' ', fc, bc, sizey);
                continue;
            }
            else
            {
                enshow = 1;
            }
        }
        LCD_ShowChar(x + t * sizex, y, temp + '0', fc, bc, sizey);
    }
}

/**
 * @brief       显示浮点数
 * @param       x:数字显示位置列起始坐标
 * @param       y:数字显示位置行起始坐标
 * @param       num:显示的浮点数
 * @param       pre:显示浮点数精度
 * @param       len:显示浮点数的位数(不包含小数点)
 * @param       fc:字符颜色
 * @param       bc:字符背景颜色
 * @param       sizey:字符大小
 * @retval      无
 */
void LCD_ShowFloatNum(uint16_t x, uint16_t y, float num, uint8_t pre, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    uint32_t i, temp, num1;
    uint8_t sizex = sizey / 2;
    num1 = (uint32_t)(num * (float)LCD_PowU8(10U, pre));
    for (i = 0; i < len; i++)
    {
        temp = (num1 / LCD_PowU8(10U, (uint8_t)(len - i - 1U))) % 10U;
        if (i == (len - pre))
        {
            LCD_ShowChar(x + (len - pre) * sizex, y, '.', fc, bc, sizey);
            i++;
            len += 1;
        }
        LCD_ShowChar(x + i * sizex, y, temp + '0', fc, bc, sizey);
    }
}

/**
 * @brief       显示12x12汉字
 * @param       x:汉字显示位置列起始坐标
 * @param       y:汉字显示位置行起始坐标
 * @param       *s:显示中文字符起始地址
 * @param       fc:字符颜色
 * @param       bc:字符背景颜色
 * @param       sizey:字符大小
 * @retval      无
 */
void LCD_ShowChinese12x12(uint16_t x, uint16_t y,const char *s, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    uint8_t i, j, m = 0;
    uint16_t k, HZnum;    // 汉字数目
    uint16_t TypefaceNum; // 一个字符所占字节大小

    TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
    HZnum = sizeof(tfont12) / sizeof(typFONT_GB12); // 统计汉字数目
    for (k = 0; k < HZnum; k++)
    {
        if ((tfont12[k].Index[0] == (uint8_t)s[0]) &&
            (tfont12[k].Index[1] == (uint8_t)s[1]) &&
            (tfont12[k].Index[2] == (uint8_t)s[2]))
        {
            LCD_Address_Set(x, y, x + sizey - 1, y + sizey - 1);
            for (i = 0; i < TypefaceNum; i++)
            {
                for (j = 0; j < 8; j++)
                {
                        if (tfont12[k].Msk[i] & (0x01 << j))
                        {
                            LCD_WR_DATA(fc);
                        }
                        else
                        {
                            LCD_WR_DATA(bc);
                        }
                        m++;
                        if (m % sizey == 0)
                        {
                            m = 0;
                            break;
                        }
                }
            }
            return; // 找到并显示后立即退出，避免继续遍历字库。
        }
    }
}

/**
 * @brief       显示16x16汉字
 * @param       x:汉字显示位置列起始坐标
 * @param       y:汉字显示位置行起始坐标
 * @param       *s:显示中文字符起始地址
 * @param       fc:字符颜色
 * @param       bc:字符背景颜色
 * @param       sizey:字符大小
 * @retval      无
 */
void LCD_ShowChinese16x16(uint16_t x, uint16_t y,const char *s, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    uint8_t i, j, m = 0;
    uint16_t k, HZnum;    // 汉字数目
    uint16_t TypefaceNum; // 一个字符所占字节大小

    TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
    HZnum = sizeof(tfont16) / sizeof(typFONT_GB16); // 统计汉字数目
    for (k = 0; k < HZnum; k++)
    {
        if ((tfont16[k].Index[0] == (uint8_t)s[0]) &&
            (tfont16[k].Index[1] == (uint8_t)s[1]) &&
            (tfont16[k].Index[2] == (uint8_t)s[2]))
        {
            LCD_Address_Set(x, y, x + sizey - 1, y + sizey - 1);
            for (i = 0; i < TypefaceNum; i++)
            {
                for (j = 0; j < 8; j++)
                {
                        if (tfont16[k].Msk[i] & (0x01 << j))
                        {
                            LCD_WR_DATA(fc);
                        }
                        else
                        {
                            LCD_WR_DATA(bc);
                        }
                        m++;
                        if (m % sizey == 0)
                        {
                            m = 0;
                            break;
                        }
                    
                }
            }
            return; // 找到并显示后立即退出，避免继续遍历字库。
        }
    }
}

/**
 * @brief       显示24x24汉字
 * @param       x:汉字显示位置列起始坐标
 * @param       y:汉字显示位置行起始坐标
 * @param       *s:显示中文字符起始地址
 * @param       fc:字符颜色
 * @param       bc:字符背景颜色
 * @param       sizey:字符大小
 * @retval      无
 */
void LCD_ShowChinese24x24(uint16_t x, uint16_t y,const char *s, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    uint8_t i, j, m = 0;
    uint16_t k, HZnum;    // 汉字数目
    uint16_t TypefaceNum; // 一个字符所占字节大小

    TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
    HZnum = sizeof(tfont24) / sizeof(typFONT_GB24); // 统计汉字数目
    for (k = 0; k < HZnum; k++)
    {
        if ((tfont24[k].Index[0] == (uint8_t)s[0]) &&
            (tfont24[k].Index[1] == (uint8_t)s[1]) &&
            (tfont24[k].Index[2] == (uint8_t)s[2]))
        {
            LCD_Address_Set(x, y, x + sizey - 1, y + sizey - 1);
            for (i = 0; i < TypefaceNum; i++)
            {
                for (j = 0; j < 8; j++)
                {
                  
                        if (tfont24[k].Msk[i] & (0x01 << j))
                        {
                            LCD_WR_DATA(fc);
                        }
                        else
                        {
                            LCD_WR_DATA(bc);
                        }
                        m++;
                        if (m % sizey == 0)
                        {
                            m = 0;
                            break;
                        }
                }
            }
            return; // 找到并显示后立即退出，避免继续遍历字库。
        }
    }
}

/**
 * @brief       显示32x32汉字
 * @param       x:汉字显示位置列起始坐标
 * @param       y:汉字显示位置行起始坐标
 * @param       *s:显示中文字符起始地址
 * @param       fc:字符颜色
 * @param       bc:字符背景颜色
 * @param       sizey:字符大小
 * @retval      无
 */
void LCD_ShowChinese32x32(uint16_t x, uint16_t y,const char *s, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    uint8_t i, j, m = 0;
    uint16_t k, HZnum;    // 汉字数目
    uint16_t TypefaceNum; // 一个字符所占字节大小

    TypefaceNum = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
    HZnum = sizeof(tfont32) / sizeof(typFONT_GB32); // 统计汉字数目
    for (k = 0; k < HZnum; k++)
    {
        if ((tfont32[k].Index[0] == (uint8_t)s[0]) &&
            (tfont32[k].Index[1] == (uint8_t)s[1]) &&
            (tfont32[k].Index[2] == (uint8_t)s[2]))
        {
            LCD_Address_Set(x, y, x + sizey - 1, y + sizey - 1);
            for (i = 0; i < TypefaceNum; i++)
            {
                for (j = 0; j < 8; j++)
                {
                        if (tfont32[k].Msk[i] & (0x01 << j))
                        {
                            LCD_WR_DATA(fc);
                        }
                        else
                        {
                            LCD_WR_DATA(bc);
                        }
                        m++;
                        if (m % sizey == 0)
                        {
                            m = 0;
                            break;
                        }

                }
            }
            return; // 找到并显示后立即退出，避免继续遍历字库。
        }
    }
}

/**
 * @brief       显示汉字串
 * @param       x:汉字显示位置列起始坐标
 * @param       y:汉字显示位置行起始坐标
 * @param       *s:显示中文字符
 * @param       fc:字符颜色
 * @param       bc:字符背景颜色
 * @param       sizey:字符大小
 * @retval      无
 */
void LCD_ShowChinese(uint16_t x, uint16_t y,const char *s, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    while (*s != 0)
    {
        if (sizey == 12)
            LCD_ShowChinese12x12(x, y, s, fc, bc, sizey);
        else if (sizey == 16)
            LCD_ShowChinese16x16(x, y, s, fc, bc, sizey);
        else if (sizey == 24)
            LCD_ShowChinese24x24(x, y, s, fc, bc, sizey);
        else if (sizey == 32)
            LCD_ShowChinese32x32(x, y, s, fc, bc, sizey);
        else
            return;
        s += 3; /* 常用汉字在 UTF-8 中占 3 字节。 */
        x += sizey;
    }
}

/**
 * @brief       中英字符混显
 * @param       x:显示位置列起始坐标
 * @param       y:显示位置行起始坐标
 * @param       *s:显示字符起始地址
 * @param       fc:字符颜色
 * @param       bc:字符背景颜色
 * @param       sizey:字符大小
 * @retval      无
 */
void LCD_ShowStr(uint16_t x, uint16_t y,const char *s, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    uint16_t x0 = x;
    uint8_t bHz = 0; // 字符或者中文
    while (*s != 0)  // 数据未结束
    {
        if (!bHz) // 英文
        {
            if (x > (LCD_W - sizey / 2) || y > (LCD_H - sizey))
            {
                return;
            }
            if (*s > 0x80)
            {
                bHz = 1; // 中文
            }
            else // 字符
            {
                if (*s == 0x0D) // 换行符号
                {
                    y += sizey;
                    x = x0;
                    s++;
                }
                else
                {
                    LCD_ShowChar(x, y, *s, fc, bc, sizey);
                    x += sizey / 2; // 字符,为全字的一半
                }
                s++;
            }
        }
        else // 中文
        {
            if (x > (LCD_W - sizey) || y > (LCD_H - sizey))
            {
                return;
            }
            bHz = 0;
            if (sizey == 12)
                LCD_ShowChinese12x12(x, y, s, fc, bc, sizey);
            else if (sizey == 16)
                LCD_ShowChinese16x16(x, y, s, fc, bc, sizey);
            else if (sizey == 24)
                LCD_ShowChinese24x24(x, y, s, fc, bc, sizey);
            else
                LCD_ShowChinese32x32(x, y, s, fc, bc, sizey);
            s += 3; /* 常用汉字在 UTF-8 中占 3 字节。 */
            x += sizey;
        }
    }
}

/**
 * @brief       字符居中显示
 * @param       x:此输入参数无效
 * @param       y:显示位置行起始坐标
 * @param       *s:显示字符起始地址
 * @param       fc:字符颜色
 * @param       bc:字符背景颜色
 * @param       sizey:字符大小
 * @retval      无
 */
void LCD_StrCenter(uint16_t x, uint16_t y,const char *s, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    uint16_t len = strlen((const char *)s);
    uint16_t x1 = (LCD_W - len * 8) / 2;
    LCD_ShowStr(x1, y, s, fc, bc, sizey);
}

/**
 * @brief       图片显示函数
 * @param       x:图片显示位置列起始坐标
 * @param       y:图片显示位置行起始坐标
 * @param       width:图片宽度
 * @param       height:图片高度
 * @param       pic:图片取模数组
 * @retval      无
 */
void LCD_ShowPicture(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t pic[])
{
    uint8_t picH, picL;
    uint16_t i, j;
    uint32_t k = 0;
    LCD_Address_Set(x, y, x + width - 1, y + height - 1);
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            picH = pic[k * 2];
            picL = pic[k * 2 + 1];
            LCD_WR_DATA(picH << 8 | picL);
            k++;
        }
    }
}
