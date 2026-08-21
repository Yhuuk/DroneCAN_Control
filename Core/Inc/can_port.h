#ifndef CAN_PORT_H
#define CAN_PORT_H

#include "main.h"
#include "can.h"

// Function prototypes
HAL_StatusTypeDef CAN_Port_Init(void);
HAL_StatusTypeDef CAN_Port_SendExtendedMessage(uint32_t extended_id, uint8_t const *data, uint8_t data_length);
// void CAN_Port_Transmit(CAN_HandleTypeDef *hcan, CanTxMsgTypeDef *TxMessage);
// void CAN_Port_Receive(CAN_HandleTypeDef *hcan, CanRxMsgTypeDef *RxMessage);

#endif // CAN_PORT_H