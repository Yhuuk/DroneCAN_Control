#ifndef DRONECAN_NODE_H
#define DRONECAN_NODE_H

#include <stdint.h>
#include "main.h"

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

HAL_StatusTypeDef DroneCAN_ProcessTx(void);



#endif // DRONECAN_NODE_H
