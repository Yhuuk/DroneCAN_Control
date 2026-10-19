#ifndef SETTINGS_UI_ASSET_H
#define SETTINGS_UI_ASSET_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * 五个条目均由用户提供的SVG以2倍尺寸重新栅格化，文字行高为24像素。
 * 每个条目独立压缩为1 bpp并顺序放入同一个图集。图集声明为const，
 * 因而由链接器放入MCU内部Flash，不占用运行时RAM或外部FRAM。
 */
#define SETTINGS_UI_ASSET_ITEM_COUNT          5U
#define SETTINGS_UI_ASSET_ATLAS_SIZE       1262U

extern const uint8_t
    g_settings_ui_item_atlas_1bpp[SETTINGS_UI_ASSET_ATLAS_SIZE];

#ifdef __cplusplus
}
#endif

#endif /* SETTINGS_UI_ASSET_H */
