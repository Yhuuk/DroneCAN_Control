#ifndef DRONECAN_NODE_H
#define DRONECAN_NODE_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

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
