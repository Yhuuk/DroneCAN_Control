#ifndef JOYSTICK_H
#define JOYSTICK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 摇杆两个模拟轴的一次一致性快照。
 *
 * 当前硬件连接与 ADC1 扫描顺序如下：
 * - direction_raw：PA0 / ADC1_IN5 / Rank 1，对应方向轴；
 * - throttle_raw：PA1 / ADC1_IN6 / Rank 2，对应油门轴。
 *
 * ADC 为 12 位，因此正常原始值范围是 0~4095。这里暂不做中心点、死区、
 * 方向翻转或量程标定，便于先直接观察摇杆和硬件 ADC 的真实输出。
 */
typedef struct
{
    uint16_t throttle_raw;
    uint16_t direction_raw;
} JoystickRawValues_t;

/**
 * @brief 校准 ADC1，并启动两个摇杆通道的循环 DMA 采样。
 *
 * CubeMX 已负责 ADC/GPIO/DMA 的底层配置，本函数负责应用层启动顺序：
 * ADC 自校准 -> 启动循环 DMA -> 关闭 DMA 半传输/传输完成中断。
 * DMA 错误中断保持开启，用于发现真正的采集故障。
 *
 * @retval HAL_OK    初始化成功，或模块此前已经成功启动。
 * @retval HAL_ERROR ADC/DMA 句柄异常，或启动失败。
 * @retval 其它值    HAL 校准或启动函数返回的具体错误状态。
 */
HAL_StatusTypeDef Joystick_Init(void);

/**
 * @brief 从 DMA 循环缓冲区发布一份稳定快照。
 *
 * 该函数应由 InputTask 每 10 ms 调用一次。它不等待 ADC，不操作 LCD，
 * 只复制两个半字并通过一次对齐的 32 位写操作发布快照，因此执行时间很短。
 */
void Joystick_Process(void);

/**
 * @brief 获取 InputTask 最近一次发布的摇杆原始值。
 *
 * @param[out] values 用于接收两个 ADC 原始值的结构体指针。
 *
 * @retval true  已经初始化且成功取得快照。
 * @retval false 参数为空，或 ADC/DMA 尚未成功启动。
 */
bool Joystick_GetLatestRaw(JoystickRawValues_t *values);

/* 调试统计量，可在调试器 Watch 窗口中直接观察。 */
extern volatile uint32_t g_joystick_snapshot_count;
extern volatile uint32_t g_joystick_dma_error_count;

#ifdef __cplusplus
}
#endif

#endif /* JOYSTICK_H */
