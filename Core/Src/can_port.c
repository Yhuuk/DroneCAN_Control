#include "can_port.h"
#include "dronecan_config.h"
#include "dronecan_dshot.DirectionQuery.h"

HAL_StatusTypeDef CAN_Port_Init(void) {

    CAN_FilterTypeDef filterConfig = {0};
    HAL_StatusTypeDef status;

    uint32_t dronecan_response_id;
    uint32_t dronecan_response_mask;
    uint32_t bxcan_filter_id;
    uint32_t bxcan_filter_mask;

    /*
     * DirectionQuery服务响应的29位DroneCAN ID：
     * bit23..16=服务ID 200；bit15=0表示Response；bit14..8=目的节点126；
     * bit7=1表示Service；bit6..0=响应节点42。priority bit28..24不比较。
     */
    dronecan_response_id =
        ((uint32_t)DRONECAN_DSHOT_DIRECTIONQUERY_ID << 16U) |
        ((uint32_t)DRONECAN_CONTROLLER_NODE_ID << 8U) |
        (1UL << 7U) |
        (uint32_t)DRONECAN_DSHOT_NODE_ID;

    dronecan_response_mask =
        (0xFFUL << 16U) |
        (1UL << 15U) |
        (0x7FUL << 8U) |
        (1UL << 7U) |
        0x7FUL;

    /* bxCAN过滤寄存器使用ExtID左移3位的布局，并比较IDE=1、RTR=0。 */
    bxcan_filter_id = (dronecan_response_id << 3U) | CAN_ID_EXT;
    bxcan_filter_mask =
        (dronecan_response_mask << 3U) | CAN_ID_EXT | (1UL << 1U);

    // Configure the CAN filter
    filterConfig.FilterBank = 0; // Use filter bank 0
    filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    filterConfig.FilterIdHigh = (bxcan_filter_id >> 16U) & 0xFFFFU;
    filterConfig.FilterIdLow = bxcan_filter_id & 0xFFFFU;
    filterConfig.FilterMaskIdHigh =
        (bxcan_filter_mask >> 16U) & 0xFFFFU;
    filterConfig.FilterMaskIdLow = bxcan_filter_mask & 0xFFFFU;
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

HAL_StatusTypeDef CAN_Port_TryReceive(CAN_PortRxFrame_t *frame)
{
    CAN_RxHeaderTypeDef rx_header = {0};
    uint8_t rx_data[8] = {0};

    if (frame == NULL)
    {
        return HAL_ERROR;
    }

    if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0U)
    {
        return HAL_BUSY;
    }

    if (HAL_CAN_GetRxMessage(
            &hcan1, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if ((rx_header.IDE != CAN_ID_EXT) ||
        (rx_header.RTR != CAN_RTR_DATA) ||
        (rx_header.DLC > 8U))
    {
        return HAL_ERROR;
    }

    frame->extended_id = rx_header.ExtId;
    frame->data_length = (uint8_t)rx_header.DLC;
    for (uint8_t index = 0U; index < frame->data_length; ++index)
    {
        frame->data[index] = rx_data[index];
    }

    return HAL_OK;
}
