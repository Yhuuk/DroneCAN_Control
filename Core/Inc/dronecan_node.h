#ifndef DRONECAN_NODE_H
#define DRONECAN_NODE_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/** 本控制器当前实际管理的ESC通道数量。 */
#define DRONECAN_ESC_CHANNEL_COUNT       8U

/**
 * RawCommand在DSDL中虽然使用有符号int14，但本控制器当前只使用正向油门。
 * 因此传输边界只接受0~8191：0表示停转，8191表示协议允许的最大正向命令。
 */
#define DRONECAN_ESC_RAW_COMMAND_MIN     0U
#define DRONECAN_ESC_RAW_COMMAND_MAX     8191U

/**
 * @brief 一次完整的8路ESC油门发布快照。
 *
 * motor[0]对应第1路电机，motor[7]对应第8路电机。这里故意不保存
 * motor_mask：标准uavcan.equipment.esc.RawCommand没有掩码字段，后续业务层
 * 应先依据本地掩码生成完整8路快照，再把该结构体交给CanTask发布。
 *
 * 使用uint16_t可以在应用层类型上排除负油门；发布函数还会逐项检查上限，
 * 防止超出DSDL有符号int14的正向范围。
 */
typedef struct
{
    uint16_t motor[DRONECAN_ESC_CHANNEL_COUNT];
} DroneCANThrottleCommand_t;

/** DirectionQuery响应中与控制器业务有关的全部字段。 */
typedef struct
{
    uint8_t protocol_version;
    uint8_t status;
    uint16_t request_id;
    uint8_t active_source_node_id;
    uint16_t active_request_id;
    uint8_t query_motor_mask;
    uint8_t valid_mask;
    uint8_t reversed_mask;
    uint8_t timeout_mask;
    uint8_t crc_error_mask;
    uint8_t unsupported_mask;
    uint8_t protocol_error_mask;
    uint8_t maintenance_error;
} DroneCANDirectionQueryResponse_t;

void DroneCAN_Node_Init(void);

/**
 * @brief 将一份完整的8路正向油门快照编码并加入libcanard发送队列。
 *
 * @param command 8路油门快照；每一路都必须处于0~8191。
 * @return 大于0：成功加入libcanard的软件CAN帧数量；
 *         0或负数：没有入队，错误值沿用libcanard约定。
 *
 * @note 本函数只完成DSDL编码和libcanard入队，不会直接调用HAL发送，也不会
 *       自动周期发布。为保持CanardInstance单一所有权，只应由CanTask调用；
 *       入队后的帧由CanTask持续调用DroneCAN_ProcessTx()送入CAN硬件邮箱。
 */
int16_t DroneCAN_PublishRawCommand(
    const DroneCANThrottleCommand_t *command);

int16_t DroneCAN_SetMotorDirection(
    uint8_t motor_mask,
    uint8_t operation);

/** @brief 将motor_mask选中的电机明确设置为Normal方向。 */
int16_t DroneCAN_SetMotorsNormal(uint8_t motor_mask);

/** @brief 将motor_mask选中的电机明确设置为Reversed方向。 */
int16_t DroneCAN_SetMotorsReversed(uint8_t motor_mask);

int16_t DroneCAN_Motor1_SetNormal(void);
int16_t DroneCAN_Motor1_SetReversed(void);

/**
 * @brief 发送DirectionQuery服务请求。
 * @param operation DSDL中的START_QUERY或GET_RESULT。
 * @param motor_mask START时为待查通道；GET_RESULT时必须为0。
 * @param request_id 应用事务ID；同一次START/GET轮询必须保持不变。
 */
int16_t DroneCAN_SendDirectionQuery(uint8_t operation,
                                    uint8_t motor_mask,
                                    uint16_t request_id);

/** 取走最近一条DirectionQuery响应；没有新响应时返回false。 */
bool DroneCAN_TakeDirectionQueryResponse(
    DroneCANDirectionQueryResponse_t *response);

HAL_StatusTypeDef DroneCAN_ProcessTx(void);
HAL_StatusTypeDef DroneCAN_ProcessRx(void);



#endif // DRONECAN_NODE_H
