#ifndef CAN_PORT_H
#define CAN_PORT_H

#include "main.h"
#include "can.h"

/** @brief 从bxCAN FIFO读取的一帧29位扩展数据帧。 */
typedef struct
{
    uint32_t extended_id;
    uint8_t data[8];
    uint8_t data_length;
} CAN_PortRxFrame_t;

// Function prototypes
HAL_StatusTypeDef CAN_Port_Init(void);
HAL_StatusTypeDef CAN_Port_SendExtendedMessage(uint32_t extended_id, uint8_t const *data, uint8_t data_length);
/** @return HAL_OK读取一帧；HAL_BUSY表示FIFO为空；其它值表示硬件错误。 */
HAL_StatusTypeDef CAN_Port_TryReceive(CAN_PortRxFrame_t *frame);
// void CAN_Port_Transmit(CAN_HandleTypeDef *hcan, CanTxMsgTypeDef *TxMessage);
// void CAN_Port_Receive(CAN_HandleTypeDef *hcan, CanRxMsgTypeDef *RxMessage);

#endif // CAN_PORT_H
