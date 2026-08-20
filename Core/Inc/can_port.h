#ifndef CAN_PORT_H
#define CAN_PORT_H

#include "main.h"
#include "can.h"

// Function prototypes
HAL_StatusTypeDef CAN_Port_Init(void);
// void CAN_Port_Transmit(CAN_HandleTypeDef *hcan, CanTxMsgTypeDef *TxMessage);
// void CAN_Port_Receive(CAN_HandleTypeDef *hcan, CanRxMsgTypeDef *RxMessage);

#endif // CAN_PORT_H