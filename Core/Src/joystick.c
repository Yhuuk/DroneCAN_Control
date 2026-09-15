#include "joystick.h"

#include "adc.h"

/**
 * ADC1 的扫描 Rank 与 DMA 数组下标严格一一对应。
 * 如果以后在 CubeMX 中调整 ADC Rank，必须同步修改这里的下标定义。
 */
enum
{
    /* PA0 / ADC1_IN5 / Rank 1：方向摇杆。 */
    JOYSTICK_DMA_DIRECTION_INDEX = 0U,

    /* PA1 / ADC1_IN6 / Rank 2：油门摇杆。 */
    JOYSTICK_DMA_THROTTLE_INDEX,
    JOYSTICK_DMA_CHANNEL_COUNT
};

/** 校准后的每个轴统一输出到-1000~+1000。 */
#define JOYSTICK_NORMALIZED_LIMIT 1000

/**
 * @brief 单轴校准参数。
 *
 * min/max采用略微向机械行程内部收缩后的有效端点，使摇杆即使受温度、
 * 电源和机械误差影响，也能可靠到达-1000和+1000。
 */
typedef struct
{
    uint16_t minimum;
    uint16_t center;
    uint16_t maximum;
    uint16_t dead_zone;
} JoystickAxisCalibration_t;

/**
 * .minimum = 250U 这些都是给结构体成员赋初始值，叫做 指定成员初始化器
 */
/* PA1油门轴：实测约240 / 1991 / 3660，中心死区取±40。 */
static const JoystickAxisCalibration_t g_throttle_calibration = {
    .minimum = 250U,
    .center = 1991U,
    .maximum = 3650U,
    .dead_zone = 40U
};

/* PA0方向轴：实测约343 / 2067 / 3670，中心死区取±30。 */
static const JoystickAxisCalibration_t g_direction_calibration = {
    .minimum = 350U,
    .center = 2067U,
    .maximum = 3660U,
    .dead_zone = 30U
};

/*
 * DMA 以 half-word 写入此数组：Rank 1 写 [0]，Rank 2 写 [1]，随后循环覆盖。
 * volatile 用于明确告诉编译器：数组内容会被 DMA 外设异步修改。
 */
static volatile uint16_t g_joystick_dma_values[JOYSTICK_DMA_CHANNEL_COUNT];

/*
 * 将两个 16 位 ADC 值打包在一个对齐的 32 位变量中发布。
 * STM32L431 对齐的 32 位读写是单次完成的，这样 UiTask 不会读到一半为旧值、
 * 一半为新值的结构体。低 16 位为油门，高 16 位为方向。
 */
static volatile uint32_t g_joystick_published_raw_values;

/*
 * 归一化值也打包成一个32位快照：低16位为油门，高16位为方向。
 * int16_t按原始补码位型打包，读取时再恢复成有符号数。
 */
static volatile uint32_t g_joystick_published_normalized_values;
static volatile bool g_joystick_started;

volatile uint32_t g_joystick_snapshot_count;
volatile uint32_t g_joystick_dma_error_count;

/**
 * @brief 使用中心两侧不同的实际行程，把一个ADC原始值分段映射为±1000。
 *
 * 死区下边界到minimum独立映射为0~-1000，死区上边界到maximum独立
 * 映射为0~+1000，因此即使中心两侧机械行程不对称，两端输出仍然一致。
 * 全程使用32位整数并加入四舍五入，不需要浮点运算。
 */
static int16_t Joystick_NormalizeAxis(
    uint16_t raw_value,
    const JoystickAxisCalibration_t *calibration)
{
    const uint16_t lower_dead_zone =
        (uint16_t)(calibration->center - calibration->dead_zone);
    const uint16_t upper_dead_zone =
        (uint16_t)(calibration->center + calibration->dead_zone);
    uint32_t distance;
    uint32_t span;
    uint32_t magnitude;

    if (raw_value <= calibration->minimum)
    {
        return (int16_t)-JOYSTICK_NORMALIZED_LIMIT;
    }

    if (raw_value >= calibration->maximum)
    {
        return (int16_t)JOYSTICK_NORMALIZED_LIMIT;
    }

    /**
     * (span / 2U) 是整数除法中的四舍五入
     */
    if (raw_value < lower_dead_zone)
    {
        distance = (uint32_t)lower_dead_zone - raw_value;
        span = (uint32_t)lower_dead_zone - calibration->minimum;
        magnitude = ((distance * JOYSTICK_NORMALIZED_LIMIT) +
                     (span / 2U)) /
                    span;
        return (int16_t)-(int16_t)magnitude;
    }

    if (raw_value > upper_dead_zone)
    {
        distance = (uint32_t)raw_value - upper_dead_zone;
        span = (uint32_t)calibration->maximum - upper_dead_zone;
        magnitude = ((distance * JOYSTICK_NORMALIZED_LIMIT) +
                     (span / 2U)) /
                    span;
        return (int16_t)magnitude;
    }

    /* center-dead_zone到center+dead_zone之间统一视为松手中心。 */
    return 0;
}

HAL_StatusTypeDef Joystick_Init(void)
{
    HAL_StatusTypeDef status;

    /* 允许重复调用，但绝不能在 DMA 已运行时再次校准或重启 ADC。 */
    if (g_joystick_started)
    {
        return HAL_OK;
    }

    if (hadc1.DMA_Handle == NULL)
    {
        return HAL_ERROR;
    }

    g_joystick_dma_values[JOYSTICK_DMA_DIRECTION_INDEX] = 0U;
    g_joystick_dma_values[JOYSTICK_DMA_THROTTLE_INDEX] = 0U;
    g_joystick_published_raw_values = 0U;
    g_joystick_published_normalized_values = 0U;
    g_joystick_snapshot_count = 0U;
    g_joystick_dma_error_count = 0U;

    /* 单端输入模式下，启动采样前执行一次硬件自校准以减小 ADC 偏移误差。 */
    status = HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * ADC1 已由 CubeMX 配置为：连续扫描两个 Rank + DMA Circular。
     * 因此启动一次后，DMA 会持续把最新的两个通道结果写入上面的数组。
     * 
     * HAL_ADC_Start_DMA() 内部会先启动 ADC1，再启动 DMA。
     */
    status = HAL_ADC_Start_DMA(&hadc1,
                               (uint32_t *)(void *)g_joystick_dma_values,
                               JOYSTICK_DMA_CHANNEL_COUNT);
    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * 本应用只需 InputTask 每 10 ms 读取一次最新快照，不需要每完成 1/2 个或
     * 2 个转换就打断 CPU。因此关闭 HT/TC，仅保留 HAL 已开启的 DMA 错误中断。
     * 
     * DMA的HT/TC中断是干什么的，它是默认启动的吗
     */
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT | DMA_IT_TC);

    g_joystick_started = true;
    return HAL_OK;
}

void Joystick_Process(void)
{
    uint16_t throttle;
    uint16_t direction;
    int16_t normalized_throttle;
    int16_t normalized_direction;

    if (!g_joystick_started)
    {
        return;
    }

    /*
     * DMA 可能恰好在两次读取之间进入下一轮；这对 100 Hz 人机输入没有实际
     * 影响。对外发布时仍使用一次 32 位写入，保证消费者取得成对的一致快照。
     * 
     * normalized_direction 和 normalized_throttle先转换成无符号16位整数uint16_t
     */
    throttle = g_joystick_dma_values[JOYSTICK_DMA_THROTTLE_INDEX];
    direction = g_joystick_dma_values[JOYSTICK_DMA_DIRECTION_INDEX];

    normalized_throttle =
        Joystick_NormalizeAxis(throttle, &g_throttle_calibration);
    normalized_direction =
        Joystick_NormalizeAxis(direction, &g_direction_calibration);

    g_joystick_published_raw_values = ((uint32_t)direction << 16U) |
                                      (uint32_t)throttle;
    g_joystick_published_normalized_values =
        ((uint32_t)(uint16_t)normalized_direction << 16U) |
        (uint32_t)(uint16_t)normalized_throttle;
    ++g_joystick_snapshot_count;
}

bool Joystick_GetLatestRaw(JoystickRawValues_t *values)
{
    uint32_t packed_values;

    if ((values == NULL) || !g_joystick_started)
    {
        return false;
    }

    /* 只读取一次发布变量，确保两个成员来自同一份 InputTask 快照。 */
    packed_values = g_joystick_published_raw_values;
    values->throttle_raw = (uint16_t)(packed_values & 0xFFFFU);
    values->direction_raw = (uint16_t)(packed_values >> 16U);
    return true;
}

bool Joystick_GetLatestNormalized(JoystickNormalizedValues_t *values)
{
    uint32_t packed_values;

    if ((values == NULL) || !g_joystick_started)
    {
        return false;
    }

    /* 一次32位读取取得同一轮InputTask发布的两个有符号归一化值。 */
    packed_values = g_joystick_published_normalized_values;
    values->throttle_normalized =
        (int16_t)(uint16_t)(packed_values & 0xFFFFU);
    values->direction_normalized =
        (int16_t)(uint16_t)(packed_values >> 16U);
    return true;
}

/**
 * @brief HAL ADC 错误回调。
 *
 * HT/TC 中断已经关闭，但 DMA 传输错误仍会通过 HAL 进入此回调。回调中只
 * 记录计数，不执行显示、延时或重启操作，避免在中断上下文做耗时工作。
 */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        ++g_joystick_dma_error_count;
    }
}
