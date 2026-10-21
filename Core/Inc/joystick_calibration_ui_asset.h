#ifndef JOYSTICK_CALIBRATION_UI_ASSET_H
#define JOYSTICK_CALIBRATION_UI_ASSET_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 参考PNG直接提取的4位覆盖率图，两个像素打包为一个字节并存入MCU Flash。 */
#define JOYSTICK_CALIBRATION_PROMPT_WIDTH          80U
#define JOYSTICK_CALIBRATION_PROMPT_HEIGHT         16U
#define JOYSTICK_CALIBRATION_PROMPT_ROW_BYTES      40U
#define JOYSTICK_CALIBRATION_STABLE_WIDTH          60U
#define JOYSTICK_CALIBRATION_STABLE_HEIGHT         12U
#define JOYSTICK_CALIBRATION_STABLE_ROW_BYTES      30U
#define JOYSTICK_CALIBRATION_CROSS_WIDTH           16U
#define JOYSTICK_CALIBRATION_CROSS_HEIGHT          16U
#define JOYSTICK_CALIBRATION_CROSS_ROW_BYTES        8U
#define JOYSTICK_CALIBRATION_INCOMPLETE_WIDTH      108U
#define JOYSTICK_CALIBRATION_INCOMPLETE_HEIGHT      12U
#define JOYSTICK_CALIBRATION_INCOMPLETE_ROW_BYTES   14U
#define JOYSTICK_CALIBRATION_UP_PROMPT_WIDTH         95U
#define JOYSTICK_CALIBRATION_UP_PROMPT_HEIGHT        16U
#define JOYSTICK_CALIBRATION_UP_PROMPT_ROW_BYTES     48U
#define JOYSTICK_CALIBRATION_UP_ARROW_WIDTH          17U
#define JOYSTICK_CALIBRATION_UP_ARROW_HEIGHT         17U
#define JOYSTICK_CALIBRATION_UP_ARROW_ROW_BYTES       9U
#define JOYSTICK_CALIBRATION_VALID_TRAVEL_WIDTH      48U
#define JOYSTICK_CALIBRATION_VALID_TRAVEL_HEIGHT     12U
#define JOYSTICK_CALIBRATION_VALID_TRAVEL_ROW_BYTES  24U
#define JOYSTICK_CALIBRATION_DIRECTION_GLYPH_WIDTH   16U
#define JOYSTICK_CALIBRATION_DIRECTION_GLYPH_HEIGHT  16U
#define JOYSTICK_CALIBRATION_DIRECTION_GLYPH_ROW_BYTES 8U

extern const uint8_t g_joystick_calibration_prompt_4bpp[640U];
extern const uint8_t g_joystick_calibration_stable_4bpp[360U];
extern const uint8_t g_joystick_calibration_cross_4bpp[128U];

/* “请完成当前步骤校准”采用1位透明字模；1表示绘制提示前景色。 */
extern const uint8_t g_joystick_calibration_incomplete_1bpp[168U];
extern const uint8_t g_joystick_calibration_up_prompt_4bpp[768U];
extern const uint8_t g_joystick_calibration_up_arrow_4bpp[153U];
extern const uint8_t g_joystick_calibration_valid_travel_4bpp[288U];
extern const uint8_t g_joystick_calibration_down_glyph_4bpp[128U];
extern const uint8_t g_joystick_calibration_left_glyph_4bpp[128U];
extern const uint8_t g_joystick_calibration_right_glyph_4bpp[128U];

#ifdef __cplusplus
}
#endif

#endif /* JOYSTICK_CALIBRATION_UI_ASSET_H */
