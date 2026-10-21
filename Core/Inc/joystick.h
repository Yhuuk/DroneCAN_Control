#ifndef JOYSTICK_H
#define JOYSTICK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

/** ADC1一次扫描包含两个摇杆通道，DMA下标只能是0或1。 */
#define JOYSTICK_ADC_CHANNEL_COUNT                 2U

/** 当前PCB/CubeMX的默认映射：PA0/Rank 1为方向，PA1/Rank 2为油门。 */
#define JOYSTICK_DEFAULT_DIRECTION_ADC_INDEX       0U
#define JOYSTICK_DEFAULT_THROTTLE_ADC_INDEX        1U

/** 尚未完成用户校准时使用的出厂参数，与现有实测值保持一致。 */
#define JOYSTICK_DEFAULT_THROTTLE_MINIMUM          250U
#define JOYSTICK_DEFAULT_THROTTLE_CENTER           1991U
#define JOYSTICK_DEFAULT_THROTTLE_MAXIMUM          3650U
#define JOYSTICK_DEFAULT_THROTTLE_DEAD_ZONE        40U
#define JOYSTICK_DEFAULT_DIRECTION_MINIMUM         350U
#define JOYSTICK_DEFAULT_DIRECTION_CENTER          2067U
#define JOYSTICK_DEFAULT_DIRECTION_MAXIMUM         3660U
#define JOYSTICK_DEFAULT_DIRECTION_DEAD_ZONE       30U

/**
 * @brief 单个摇杆轴的数值校准参数。
 *
 * minimum/maximum始终表示ADC数值大小关系，必须满足
 * minimum < center < maximum。物理正方向是否对应ADC递增由独立的
 * inverted标志描述，避免把“数值最小”与“物理负方向”混为一谈。
 */
typedef struct
{
    uint16_t minimum;
    uint16_t center;
    uint16_t maximum;
    uint16_t dead_zone;
} JoystickAxisCalibration_t;

/**
 * @brief 两个物理轴完整的持久化校准配置。
 *
 * ADC下标对应ADC扫描/DMA结果数组的位置，而不是MCU ADC通道号：
 * 下标0是Rank 1，1是Rank 2。两个轴必须使用不同下标。
 *
 * calibration_valid只表示用户是否完成过完整校准；它与FRAM记录的CRC、
 * 提交标记是否有效是两层不同概念。false时仍保存一套可工作的默认参数。
 * 
 * throttle_adc_index = 1，direction_adc_index = 0.这个是代表对应的ADC通道在DMA结果数组中的下标。
 * 
 * throttle_inverted = false，direction_inverted = false。这个是代表对应的ADC通道的物理方向是否与ADC数值递增方向一致。为了解决物理硬件连接是否方向
 */
typedef struct
{
    JoystickAxisCalibration_t throttle;
    JoystickAxisCalibration_t direction;
    uint8_t throttle_adc_index;
    uint8_t direction_adc_index;
    bool throttle_inverted;
    bool direction_inverted;
    bool calibration_valid;
} JoystickCalibrationSettings_t;

/**
 * @brief 摇杆两个模拟轴的一次一致性快照。
 *
 * 当前硬件连接与 ADC1 扫描顺序如下：
 * - direction_raw：PA0 / ADC1_IN5 / Rank 1，对应方向轴；
 * - throttle_raw：PA1 / ADC1_IN6 / Rank 2，对应油门轴。
 *
 * ADC 为 12 位，因此正常原始值范围是 0~4095。原始值接口继续保留，
 * 便于以后检查ADC、供电和摇杆硬件。
 */
typedef struct
{
    uint16_t throttle_raw;
    uint16_t direction_raw;
} JoystickRawValues_t;

/**
 * @brief 应用机械中心、死区和两侧量程校准后的摇杆快照。
 *
 * 两个轴均输出-1000~+1000：有效最小端为-1000，中心死区内为0，
 * 有效最大端为+1000。当前只完成双向归一化，不在这里把油门负半轴钳位为0。
 */
typedef struct
{
    int16_t throttle_normalized;
    int16_t direction_normalized;
} JoystickNormalizedValues_t;

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
 * 读取两个DMA半字，完成整数校准计算，并分别通过对齐的32位写操作发布
 * 原始快照与归一化快照，因此执行时间很短。
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

/**
 * @brief 获取InputTask最近一次发布的校准值。
 *
 * @param[out] values 用于接收两个-1000~+1000归一化值的结构体指针。
 *
 * @retval true  已经初始化且成功取得快照。
 * @retval false 参数为空，或ADC/DMA尚未成功启动。
 */
bool Joystick_GetLatestNormalized(JoystickNormalizedValues_t *values);

/**
 * @brief 校验并立即替换应用程序正在使用的摇杆校准参数。
 *
 * 复制过程在很短的临界区内完成，避免InputTask恰好读取到一半旧值、一半
 * 新值。该函数只更新RAM，不负责写FRAM；调用方必须先确认持久化成功。
 *
 * @retval true  参数完整合法，已立即生效。
 * @retval false 参数为空、轴范围/死区非法或两个DMA下标冲突，未作修改。
 */
bool Joystick_ApplyCalibrationSettings(
    const JoystickCalibrationSettings_t *settings);

/** @brief 取得当前实际参与归一化运算的RAM校准参数快照。 */
void Joystick_GetCalibrationSettings(
    JoystickCalibrationSettings_t *settings);

/* 调试统计量，可在调试器 Watch 窗口中直接观察。 */
extern volatile uint32_t g_joystick_snapshot_count;
extern volatile uint32_t g_joystick_dma_error_count;

#ifdef __cplusplus
}
#endif

#endif /* JOYSTICK_H */
