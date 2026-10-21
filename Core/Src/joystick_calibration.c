#include "joystick_calibration.h"

#include <stddef.h>

void JoystickCenterCalibration_Init(JoystickCenterCalibration_t *calibration)
{
    if (calibration == NULL)
    {
        return;
    }

    calibration->state = JOYSTICK_CENTER_CALIBRATION_READY;
    calibration->sample_count = 0U;
    calibration->minimum_x = JOYSTICK_CENTER_ADC_MAXIMUM;
    calibration->maximum_x = 0U;
    calibration->minimum_y = JOYSTICK_CENTER_ADC_MAXIMUM;
    calibration->maximum_y = 0U;
    calibration->sum_x = 0U;
    calibration->sum_y = 0U;
    calibration->last_snapshot_sequence = 0U;
    calibration->candidate_x = 0U;
    calibration->candidate_y = 0U;
    calibration->candidate_valid = false;
}

bool JoystickCenterCalibration_Start(
    JoystickCenterCalibration_t *calibration,
    uint32_t snapshot_sequence)
{
    if ((calibration == NULL) ||
        (calibration->state == JOYSTICK_CENTER_CALIBRATION_SAMPLING))
    {
        return false;
    }

    calibration->state = JOYSTICK_CENTER_CALIBRATION_SAMPLING;
    calibration->sample_count = 0U;
    calibration->minimum_x = JOYSTICK_CENTER_ADC_MAXIMUM;
    calibration->maximum_x = 0U;
    calibration->minimum_y = JOYSTICK_CENTER_ADC_MAXIMUM;
    calibration->maximum_y = 0U;
    calibration->sum_x = 0U;
    calibration->sum_y = 0U;
    calibration->last_snapshot_sequence = snapshot_sequence;
    calibration->candidate_x = 0U;
    calibration->candidate_y = 0U;
    calibration->candidate_valid = false;
    return true;
}

bool JoystickCenterCalibration_Process(
    JoystickCenterCalibration_t *calibration,
    uint32_t snapshot_sequence,
    uint16_t raw_x,
    uint16_t raw_y)
{
    uint16_t average_x;
    uint16_t average_y;
    bool stable;
    bool center_reasonable;

    if ((calibration == NULL) ||
        (calibration->state != JOYSTICK_CENTER_CALIBRATION_SAMPLING) ||
        (snapshot_sequence == calibration->last_snapshot_sequence))
    {
        return false;
    }

    calibration->last_snapshot_sequence = snapshot_sequence;

    /* 12位ADC不可能产生大于4095的合法结果；异常时立即结束本轮。 */
    if ((raw_x > JOYSTICK_CENTER_ADC_MAXIMUM) ||
        (raw_y > JOYSTICK_CENTER_ADC_MAXIMUM))
    {
        calibration->state = JOYSTICK_CENTER_CALIBRATION_FAILED;
        calibration->candidate_valid = false;
        return true;
    }

    if (raw_x < calibration->minimum_x)
    {
        calibration->minimum_x = raw_x;
    }
    if (raw_x > calibration->maximum_x)
    {
        calibration->maximum_x = raw_x;
    }
    if (raw_y < calibration->minimum_y)
    {
        calibration->minimum_y = raw_y;
    }
    if (raw_y > calibration->maximum_y)
    {
        calibration->maximum_y = raw_y;
    }

    calibration->sum_x += raw_x;
    calibration->sum_y += raw_y;
    ++calibration->sample_count;

    if (calibration->sample_count < JOYSTICK_CENTER_SAMPLE_COUNT)
    {
        return false;
    }

    /* 加半个除数后做整数除法，得到四舍五入的100组平均值。 */
    average_x = (uint16_t)((calibration->sum_x +
                            (JOYSTICK_CENTER_SAMPLE_COUNT / 2U)) /
                           JOYSTICK_CENTER_SAMPLE_COUNT);
    average_y = (uint16_t)((calibration->sum_y +
                            (JOYSTICK_CENTER_SAMPLE_COUNT / 2U)) /
                           JOYSTICK_CENTER_SAMPLE_COUNT);

    /*
     * 实测松手噪声约为±5 ADC计数，峰峰值阈值24留有温漂和电源噪声余量；
     * 任一轴超过阈值都说明摇杆在移动或接触不稳定。
     */
    stable =
        ((uint16_t)(calibration->maximum_x - calibration->minimum_x) <=
         JOYSTICK_CENTER_MAX_PEAK_TO_PEAK) &&
        ((uint16_t)(calibration->maximum_y - calibration->minimum_y) <=
         JOYSTICK_CENTER_MAX_PEAK_TO_PEAK);

    /*
     * 中心只允许落在12位ADC中间50%区域，防止摇杆稳定停在任一机械极限
     * 时被误当成中心；范围仍覆盖当前两轴约1990/2067的实测中心。
     */
    center_reasonable =
        (average_x >= JOYSTICK_CENTER_REASONABLE_MINIMUM) &&
        (average_x <= JOYSTICK_CENTER_REASONABLE_MAXIMUM) &&
        (average_y >= JOYSTICK_CENTER_REASONABLE_MINIMUM) &&
        (average_y <= JOYSTICK_CENTER_REASONABLE_MAXIMUM);

    if (stable && center_reasonable)
    {
        calibration->candidate_x = average_x;
        calibration->candidate_y = average_y;
        calibration->candidate_valid = true;
        calibration->state = JOYSTICK_CENTER_CALIBRATION_SUCCEEDED;
    }
    else
    {
        calibration->candidate_x = 0U;
        calibration->candidate_y = 0U;
        calibration->candidate_valid = false;
        calibration->state = JOYSTICK_CENTER_CALIBRATION_FAILED;
    }

    return true;
}

void JoystickExtremeCalibration_Init(
    JoystickExtremeCalibration_t *calibration)
{
    if (calibration == NULL)
    {
        return;
    }

    calibration->state = JOYSTICK_EXTREME_CALIBRATION_READY;
    calibration->axis = JOYSTICK_CALIBRATION_AXIS_X;
    calibration->direction = JOYSTICK_CALIBRATION_EXTREME_MINIMUM;
    calibration->sample_count = 0U;
    calibration->minimum_value = JOYSTICK_CENTER_ADC_MAXIMUM;
    calibration->maximum_value = 0U;
    calibration->sum_value = 0U;
    calibration->last_snapshot_sequence = 0U;
    calibration->center_value = 0U;
    calibration->candidate_value = 0U;
    calibration->candidate_valid = false;
}

bool JoystickExtremeCalibration_Start(
    JoystickExtremeCalibration_t *calibration,
    uint32_t snapshot_sequence,
    JoystickCalibrationAxis_t axis,
    JoystickCalibrationExtremeDirection_t direction,
    uint16_t center_value,
    bool center_valid)
{
    if ((calibration == NULL) || !center_valid ||
        (center_value > JOYSTICK_CENTER_ADC_MAXIMUM) ||
        ((axis != JOYSTICK_CALIBRATION_AXIS_X) &&
         (axis != JOYSTICK_CALIBRATION_AXIS_Y)) ||
        ((direction != JOYSTICK_CALIBRATION_EXTREME_MINIMUM) &&
         (direction != JOYSTICK_CALIBRATION_EXTREME_MAXIMUM)) ||
        (calibration->state == JOYSTICK_EXTREME_CALIBRATION_SAMPLING))
    {
        return false;
    }

    calibration->state = JOYSTICK_EXTREME_CALIBRATION_SAMPLING;
    calibration->axis = axis;
    calibration->direction = direction;
    calibration->sample_count = 0U;
    calibration->minimum_value = JOYSTICK_CENTER_ADC_MAXIMUM;
    calibration->maximum_value = 0U;
    calibration->sum_value = 0U;
    calibration->last_snapshot_sequence = snapshot_sequence;
    calibration->center_value = center_value;
    calibration->candidate_value = 0U;
    calibration->candidate_valid = false;
    return true;
}

bool JoystickExtremeCalibration_Process(
    JoystickExtremeCalibration_t *calibration,
    uint32_t snapshot_sequence,
    uint16_t raw_x,
    uint16_t raw_y)
{
    uint16_t raw_value;
    uint16_t average_value;
    bool stable;
    bool travel_valid;

    if ((calibration == NULL) ||
        (calibration->state != JOYSTICK_EXTREME_CALIBRATION_SAMPLING) ||
        (snapshot_sequence == calibration->last_snapshot_sequence))
    {
        return false;
    }

    calibration->last_snapshot_sequence = snapshot_sequence;
    raw_value = (calibration->axis == JOYSTICK_CALIBRATION_AXIS_X)
                    ? raw_x
                    : raw_y;
    if (raw_value > JOYSTICK_CENTER_ADC_MAXIMUM)
    {
        calibration->state = JOYSTICK_EXTREME_CALIBRATION_FAILED;
        calibration->candidate_valid = false;
        return true;
    }

    if (raw_value < calibration->minimum_value)
    {
        calibration->minimum_value = raw_value;
    }
    if (raw_value > calibration->maximum_value)
    {
        calibration->maximum_value = raw_value;
    }
    calibration->sum_value += raw_value;
    ++calibration->sample_count;

    if (calibration->sample_count < JOYSTICK_EXTREME_SAMPLE_COUNT)
    {
        return false;
    }

    /**
     * JOYSTICK_EXTREME_SAMPLE_COUNT / 2U 就是为了在整数除法时实现四舍五入。
     */
    average_value = (uint16_t)((calibration->sum_value +
                                (JOYSTICK_EXTREME_SAMPLE_COUNT / 2U)) /
                               JOYSTICK_EXTREME_SAMPLE_COUNT);
    stable =
        ((uint16_t)(calibration->maximum_value -
                    calibration->minimum_value) <=
         JOYSTICK_EXTREME_MAX_PEAK_TO_PEAK);

    if (calibration->direction ==
        JOYSTICK_CALIBRATION_EXTREME_MAXIMUM)
    {
        travel_valid =
            (average_value > calibration->center_value) &&
            ((uint16_t)(average_value - calibration->center_value) >=
             JOYSTICK_EXTREME_MINIMUM_TRAVEL);
    }
    else
    {
        travel_valid =
            (average_value < calibration->center_value) &&
            ((uint16_t)(calibration->center_value - average_value) >=
             JOYSTICK_EXTREME_MINIMUM_TRAVEL);
    }

    if (stable && travel_valid)
    {
        calibration->candidate_value = average_value;
        calibration->candidate_valid = true;
        calibration->state = JOYSTICK_EXTREME_CALIBRATION_SUCCEEDED;
    }
    else
    {
        calibration->candidate_value = 0U;
        calibration->candidate_valid = false;
        calibration->state = JOYSTICK_EXTREME_CALIBRATION_FAILED;
    }

    return true;
}
