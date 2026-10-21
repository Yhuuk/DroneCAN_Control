#ifndef JOYSTICK_CALIBRATION_H
#define JOYSTICK_CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/*
 * 中心点采样参数集中放在这里，后续可依据量产统计微调，而不把魔法数字
 * 散落到UiTask中。InputTask每10 ms发布一次快照，100组约等于1秒数据。
 */
#define JOYSTICK_CENTER_SAMPLE_COUNT             100U
#define JOYSTICK_CENTER_ADC_MAXIMUM             4095U
#define JOYSTICK_CENTER_REASONABLE_MINIMUM      1024U
#define JOYSTICK_CENTER_REASONABLE_MAXIMUM      3071U
#define JOYSTICK_CENTER_MAX_PEAK_TO_PEAK          24U

/*
 * 极限位置同样统计100组（约1秒）。峰峰值阈值沿用中心采样；有效行程
 * 至少覆盖12位ADC满量程的25%，防止只推一小段就误判为机械极限。
 * 
 * JOYSTICK_EXTREME_MAX_PEAK_TO_PEAK 表示这个表示采集极限值时的这一组数据中的峰峰值，超过这个值就认为摇杆在移动或者接触不稳定。
 * JOYSTICK_EXTREME_MINIMUM_TRAVEL 表示摇杆四个方向的极限值到摇杆中心值的最小距离。防止用户校准只是小推，而没有到达摇杆的物理极限
 */
#define JOYSTICK_EXTREME_SAMPLE_COUNT            100U
#define JOYSTICK_EXTREME_MAX_PEAK_TO_PEAK         24U
#define JOYSTICK_EXTREME_MINIMUM_TRAVEL         1024U

/** @brief 第一步中心点采样状态。 */
typedef enum
{
    JOYSTICK_CENTER_CALIBRATION_READY = 0,
    JOYSTICK_CENTER_CALIBRATION_SAMPLING,
    JOYSTICK_CENTER_CALIBRATION_SUCCEEDED,
    JOYSTICK_CENTER_CALIBRATION_FAILED
} JoystickCenterCalibrationState_t;

/**
 * @brief 非阻塞中心点统计会话。
 *
 * 所有成员只保存在RAM。candidate_x/candidate_y只有candidate_valid为true
 * 时才代表本次成功结果；它们不会写入FRAM，也不会覆盖joystick.c当前
 * 正式校准参数。snapshot_sequence用于避免UiTask重复统计同一份快照。
 * 
 * sum_x 和sum_y是100组ADC的采集累加
 * candidate_valid: 中心候选值是否有效
 */
typedef struct
{
    JoystickCenterCalibrationState_t state;
    uint16_t sample_count;
    uint16_t minimum_x;
    uint16_t maximum_x;
    uint16_t minimum_y;
    uint16_t maximum_y;
    uint32_t sum_x;
    uint32_t sum_y;
    uint32_t last_snapshot_sequence;
    uint16_t candidate_x;
    uint16_t candidate_y;
    bool candidate_valid;
} JoystickCenterCalibration_t;

/** @brief 极限采样所使用的ADC轴。 */
typedef enum
{
    JOYSTICK_CALIBRATION_AXIS_X = 0,
    JOYSTICK_CALIBRATION_AXIS_Y
} JoystickCalibrationAxis_t;

/** @brief 当前方向相对于中心值应位于最小端还是最大端。 */
typedef enum
{
    JOYSTICK_CALIBRATION_EXTREME_MINIMUM = 0,
    JOYSTICK_CALIBRATION_EXTREME_MAXIMUM
} JoystickCalibrationExtremeDirection_t;

/** @brief 四个极限结果在RAM数组中的固定槽位。 */
typedef enum
{
    JOYSTICK_CALIBRATION_SLOT_UP = 0,
    JOYSTICK_CALIBRATION_SLOT_DOWN,
    JOYSTICK_CALIBRATION_SLOT_LEFT,
    JOYSTICK_CALIBRATION_SLOT_RIGHT,
    JOYSTICK_CALIBRATION_SLOT_COUNT
} JoystickCalibrationExtremeSlot_t;

typedef enum
{
    JOYSTICK_EXTREME_CALIBRATION_READY = 0,
    JOYSTICK_EXTREME_CALIBRATION_SAMPLING,
    JOYSTICK_EXTREME_CALIBRATION_SUCCEEDED,
    JOYSTICK_EXTREME_CALIBRATION_FAILED
} JoystickExtremeCalibrationState_t;

/**
 * @brief 一个方向极限的参数化非阻塞采样器及RAM候选结果。
 *
 * axis和direction决定从raw_x/raw_y中选择哪一路及向中心哪一侧判断。
 * candidate_value只在candidate_valid为true时有效。四个方向分别使用
 * 一个实例，因此页面前进时不会覆盖前面步骤的候选值。
 */
typedef struct
{
    JoystickExtremeCalibrationState_t state;
    JoystickCalibrationAxis_t axis;
    JoystickCalibrationExtremeDirection_t direction;
    uint16_t sample_count;
    uint16_t minimum_value;
    uint16_t maximum_value;
    uint32_t sum_value;
    uint32_t last_snapshot_sequence;
    uint16_t center_value;
    uint16_t candidate_value;
    bool candidate_valid;
} JoystickExtremeCalibration_t;

/** @brief 清空临时结果并回到尚未采样状态。 */
void JoystickCenterCalibration_Init(JoystickCenterCalibration_t *calibration);

/**
 * @brief 开始一次新的100组中心采样。
 * @param snapshot_sequence 启动时最新快照序号；该快照不会被重复计入。
 * @return false表示参数为空或已有一次采样正在进行。
 */
bool JoystickCenterCalibration_Start(
    JoystickCenterCalibration_t *calibration,
    uint32_t snapshot_sequence);

/**
 * @brief 向非阻塞统计器提交一份最新ADC快照。
 *
 * 相同snapshot_sequence会被忽略；收满100组后同时检查两轴ADC合法范围、
 * 峰峰值稳定性和平均中心合理区间，然后转为SUCCEEDED或FAILED。
 *
 * @return true表示本次调用使采样完成（成功或失败），UI应立即刷新状态。
 */
bool JoystickCenterCalibration_Process(
    JoystickCenterCalibration_t *calibration,
    uint32_t snapshot_sequence,
    uint16_t raw_x,
    uint16_t raw_y);

/** @brief 清空一个方向的极限临时候选值。 */
void JoystickExtremeCalibration_Init(
    JoystickExtremeCalibration_t *calibration);

/**
 * @brief 以第一步有效中心为基准，按指定轴和极值方向开始采样。
 * @return false表示参数无效、第一步无有效中心或已有采样正在进行。
 */
bool JoystickExtremeCalibration_Start(
    JoystickExtremeCalibration_t *calibration,
    uint32_t snapshot_sequence,
    JoystickCalibrationAxis_t axis,
    JoystickCalibrationExtremeDirection_t direction,
    uint16_t center_value,
    bool center_valid);

/**
 * @brief 提交X/Y快照；内部按axis取值并检查稳定性及正确方向行程。
 * @return true表示本次调用使采样完成（成功或失败）。
 */
bool JoystickExtremeCalibration_Process(
    JoystickExtremeCalibration_t *calibration,
    uint32_t snapshot_sequence,
    uint16_t raw_x,
    uint16_t raw_y);

#ifdef __cplusplus
}
#endif

#endif /* JOYSTICK_CALIBRATION_H */
