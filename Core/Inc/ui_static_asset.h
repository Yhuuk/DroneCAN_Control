#ifndef UI_STATIC_ASSET_H
#define UI_STATIC_ASSET_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 240x120逻辑画布旋转为120x240物理扫描顺序后，每两个像素打包成1字节。
 * 数组声明为const，链接器会把它放入MCU内部Flash，不占用外部FRAM，
 * 也不会在运行时占用28,800字节RAM。
 */
#define UI_STATIC_ASSET_PHYSICAL_WIDTH       120U
#define UI_STATIC_ASSET_PHYSICAL_HEIGHT      240U
#define UI_STATIC_ASSET_PACKED_ROW_BYTES      60U
#define UI_STATIC_ASSET_PACKED_SIZE        14400U

extern const uint8_t g_main_ui_static_4bpp[UI_STATIC_ASSET_PACKED_SIZE];
extern const uint16_t g_main_ui_static_palette[16U];
extern const uint8_t g_throttle_ui_static_4bpp[UI_STATIC_ASSET_PACKED_SIZE];
extern const uint16_t g_throttle_ui_static_palette[16U];

/* [0]未选中背景、[1]选中背景；索引为SVG图标的0~255灰度值。 */
extern const uint16_t g_throttle_ui_motor_color_lut[2U][256U];

#ifdef __cplusplus
}
#endif

#endif /* UI_STATIC_ASSET_H */
