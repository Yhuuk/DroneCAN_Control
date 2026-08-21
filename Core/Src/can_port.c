#include "can_port.h"

HAL_StatusTypeDef CAN_Port_Init(void) {

    CAN_FilterTypeDef filterConfig = {0};
    HAL_StatusTypeDef status;

    // Configure the CAN filter
    //当前为CAN发送端，过滤器掩码设置为0，接收所有ID的报文
    filterConfig.FilterBank = 0; // Use filter bank 0
    filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    filterConfig.FilterIdHigh = 0x0000;
    filterConfig.FilterIdLow = 0x0000;
    filterConfig.FilterMaskIdHigh = 0x0000;
    filterConfig.FilterMaskIdLow = 0x0000;
    filterConfig.FilterFIFOAssignment = CAN_FilterFIFO0;
    filterConfig.FilterActivation = CAN_FILTER_ENABLE;   //激活
    filterConfig.SlaveStartFilterBank = 14;

    status = HAL_CAN_ConfigFilter(&hcan1, &filterConfig);
    if(status != HAL_OK) {
        // Handle error
        return status;

    }

    //开始启动 CAN1
    status = HAL_CAN_Start(&hcan1);
    if(status != HAL_OK) {
        // Handle error
        return status;
    }

    return HAL_OK;
}

HAL_StatusTypeDef CAN_Port_SendExtendedMessage(uint32_t extended_id, uint8_t const *data, uint8_t data_length) {
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox;

    //CANARD_CAN_EXT_ID_MASK = 0x1FFFFFFF,这里不依赖<canard.h>，直接使用数值
    if((data == NULL) ||
       (data_length > 8U) ||
       (extended_id > 0x1FFFFFFFU)) {
        return HAL_ERROR; // Invalid parameters
       
    }

    if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
        return HAL_BUSY; // No free mailbox
    }

    tx_header.ExtId = extended_id;
    tx_header.IDE = CAN_ID_EXT; // Extended ID
    tx_header.RTR = CAN_RTR_DATA; // Data frame
    tx_header.DLC = data_length;
    tx_header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &tx_mailbox);


}