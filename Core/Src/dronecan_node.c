#include "dronecan_node.h"
#include "can_port.h"
#include "dronecan_config.h"
#include <dronecan_dshot.DirectionCommand.h>
#include <dronecan_dshot.DirectionQuery.h>
#include <uavcan.equipment.esc.RawCommand.h>
#include <canard.h>

#define DRONECAN_MEMORY_POOL_SIZE      2048U  //内存池大小
#define DRONECAN_RX_FRAME_BUDGET          8U

static CanardInstance canard_instance;

//使用uint32_t数组保证内存池至少4字节对齐,512 × 4 = 2048字节。
static uint32_t memory_pool[DRONECAN_MEMORY_POOL_SIZE / sizeof(uint32_t)];

//DroneCAN Transfer-ID范围为0～31,ibcanard成功入队后会自动递增。
static uint8_t g_direction_transfer_id = 0U; 
static uint8_t g_direction_query_transfer_id = 0U;

//这个是 DirectionCommand自定义应用层的请求编号
static uint16_t g_direction_request_id = 1U;
static DroneCANDirectionQueryResponse_t g_direction_query_response;
static bool g_direction_query_response_available;
static uint64_t g_last_rx_cleanup_usec;

volatile uint32_t g_dronecan_rx_frame_count;
volatile uint32_t g_dronecan_rx_error_count;
volatile uint32_t g_direction_query_response_count;

static void DroneCAN_OnTransferReceived(CanardInstance *ins,
                                        CanardRxTransfer *transfer);
static bool DroneCAN_ShouldAcceptTransfer(
    const CanardInstance *ins,
    uint64_t *out_data_type_signature,
    uint16_t data_type_id,
    CanardTransferType transfer_type,
    uint8_t source_node_id);


void DroneCAN_Node_Init(void){
    canardInit(&canard_instance, 
        memory_pool, 
        sizeof(memory_pool), 
        DroneCAN_OnTransferReceived,
        DroneCAN_ShouldAcceptTransfer,
        NULL);      /*用户上下文 */

    canardSetLocalNodeID(&canard_instance, DRONECAN_CONTROLLER_NODE_ID);
    g_direction_query_response_available = false;
    g_last_rx_cleanup_usec = 0ULL;

}


int16_t DroneCAN_SetMotorDirection(
    uint8_t motor_mask,
    uint8_t operation){

        struct dronecan_dshot_DirectionCommand direction_command = {0};
        uint8_t payload_buffer[DRONECAN_DSHOT_DIRECTIONCOMMAND_MAX_SIZE] = {0};

        CanardTxTransfer transfer = {0};
        uint32_t playload_length;
        int16_t result;

        if(motor_mask == 0U){
            return -CANARD_ERROR_INVALID_ARGUMENT;
        }

        if((operation != DRONECAN_DSHOT_DIRECTIONCOMMAND_OPERATION_SET_NORMAL) &&
           (operation != DRONECAN_DSHOT_DIRECTIONCOMMAND_OPERATION_SET_REVERSED)){
            return -CANARD_ERROR_INVALID_ARGUMENT;

        }


        direction_command.protocol_version = DRONECAN_DSHOT_DIRECTIONCOMMAND_PROTOCOL_VERSION;
        direction_command.operation = operation;
        direction_command.motor_mask = motor_mask;
        direction_command.request_id = g_direction_request_id;
        direction_command.confirmation = DRONECAN_DSHOT_DIRECTIONCOMMAND_CONFIRMATION_VALUE;

        //dronecan_dshot_DirectionCommand_encode是返回编码后的字节长度,如果返回0表示编码失败。
        playload_length = dronecan_dshot_DirectionCommand_encode(&direction_command, payload_buffer);

        if(playload_length != DRONECAN_DSHOT_DIRECTIONCOMMAND_MAX_SIZE){
            return -CANARD_ERROR_INTERNAL;
        }

        /**
         * direction_command 到 transfer就是把转向DSDL7字节消息编码成了一个libcanard传输对象,这个传输对象包含了消息的类型ID、优先级、传输ID、负载数据等信息。
         * 
         */
        canardInitTxTransfer(&transfer);
        
        transfer.transfer_type = CanardTransferTypeBroadcast;
        transfer.data_type_signature = DRONECAN_DSHOT_DIRECTIONCOMMAND_SIGNATURE;
        transfer.data_type_id = DRONECAN_DSHOT_DIRECTIONCOMMAND_ID;
        transfer.inout_transfer_id = &g_direction_transfer_id;   //g_direction_transfer_id会在ibcanard成功入队后自动递增。
        transfer.priority = CANARD_TRANSFER_PRIORITY_MEDIUM;
        transfer.payload = payload_buffer;
        transfer.payload_len = (uint16_t)playload_length;

        /**
         * 
         * 这是只是加入libcanard的发送队列,并没有真正发送出去,还没有调用HAL_CAN_AddTxMessage()。
         * 
         * canardBroadcastObj 是 ibcanard的广播发送函数,它会将传输对象入队到发送队列中,然后等待CAN总线空闲时发送出去。
         */
        //
         result = canardBroadcastObj(&canard_instance, &transfer);


         /**
          * result > 0：成功生成并加入了对应数量的CAN帧
          * 
          * result < 0：libcanard错误
          * 
          * result == 1 代表成功入队了一个CAN帧,因为这个消息的负载只有7字节,所以只需要一个CAN帧就能发送出去。
          */
         if(result > 0){
            g_direction_request_id ++;
         }

         return result;
}


int16_t DroneCAN_Motor1_SetNormal(void){
    return DroneCAN_SetMotorsNormal(0x01U);
}

static bool DroneCAN_ShouldAcceptTransfer(
    const CanardInstance *ins,
    uint64_t *out_data_type_signature,
    uint16_t data_type_id,
    CanardTransferType transfer_type,
    uint8_t source_node_id)
{
    (void)ins;

    if ((out_data_type_signature != NULL) &&
        (transfer_type == CanardTransferTypeResponse) &&
        (data_type_id == DRONECAN_DSHOT_DIRECTIONQUERY_ID) &&
        (source_node_id == DRONECAN_DSHOT_NODE_ID))
    {
        *out_data_type_signature = DRONECAN_DSHOT_DIRECTIONQUERY_SIGNATURE;
        return true;
    }

    return false;
}

static void DroneCAN_OnTransferReceived(CanardInstance *ins,
                                        CanardRxTransfer *transfer)
{
    struct dronecan_dshot_DirectionQueryResponse decoded = {0};

    (void)ins;

    if ((transfer == NULL) ||
        (transfer->transfer_type != CanardTransferTypeResponse) ||
        (transfer->data_type_id != DRONECAN_DSHOT_DIRECTIONQUERY_ID) ||
        (transfer->source_node_id != DRONECAN_DSHOT_NODE_ID))
    {
        return;
    }

    /* 生成代码约定：decode返回false表示成功，true表示Payload非法。 */
    if (dronecan_dshot_DirectionQueryResponse_decode(transfer, &decoded))
    {
        g_dronecan_rx_error_count++;
        return;
    }

    g_direction_query_response.protocol_version = decoded.protocol_version;
    g_direction_query_response.status = decoded.status;
    g_direction_query_response.request_id = decoded.request_id;
    g_direction_query_response.active_source_node_id =
        decoded.active_source_node_id;
    g_direction_query_response.active_request_id = decoded.active_request_id;
    g_direction_query_response.query_motor_mask = decoded.query_motor_mask;
    g_direction_query_response.valid_mask = decoded.valid_mask;
    g_direction_query_response.reversed_mask = decoded.reversed_mask;
    g_direction_query_response.timeout_mask = decoded.timeout_mask;
    g_direction_query_response.crc_error_mask = decoded.crc_error_mask;
    g_direction_query_response.unsupported_mask = decoded.unsupported_mask;
    g_direction_query_response.protocol_error_mask = decoded.protocol_error_mask;
    g_direction_query_response.maintenance_error = decoded.maintenance_error;
    g_direction_query_response_available = true;
    g_direction_query_response_count++;
}

int16_t DroneCAN_SendDirectionQuery(uint8_t operation,
                                    uint8_t motor_mask,
                                    uint16_t request_id)
{
    struct dronecan_dshot_DirectionQueryRequest request = {0};
    uint8_t payload[DRONECAN_DSHOT_DIRECTIONQUERY_REQUEST_MAX_SIZE] = {0};
    uint32_t payload_length;

    if ((operation !=
         DRONECAN_DSHOT_DIRECTIONQUERY_REQUEST_OPERATION_START_QUERY) &&
        (operation !=
         DRONECAN_DSHOT_DIRECTIONQUERY_REQUEST_OPERATION_GET_RESULT))
    {
        return -CANARD_ERROR_INVALID_ARGUMENT;
    }

    if ((operation ==
         DRONECAN_DSHOT_DIRECTIONQUERY_REQUEST_OPERATION_START_QUERY) &&
        (motor_mask == 0U))
    {
        return -CANARD_ERROR_INVALID_ARGUMENT;
    }

    request.protocol_version =
        DRONECAN_DSHOT_DIRECTIONQUERY_REQUEST_PROTOCOL_VERSION;
    request.operation = operation;
    request.motor_mask =
        (operation == DRONECAN_DSHOT_DIRECTIONQUERY_REQUEST_OPERATION_START_QUERY)
            ? motor_mask
            : 0U;
    request.request_id = request_id;
    request.confirmation =
        (operation == DRONECAN_DSHOT_DIRECTIONQUERY_REQUEST_OPERATION_START_QUERY)
            ? DRONECAN_DSHOT_DIRECTIONQUERY_REQUEST_CONFIRMATION_VALUE
            : 0U;

    payload_length =
        dronecan_dshot_DirectionQueryRequest_encode(&request, payload);
    if (payload_length != DRONECAN_DSHOT_DIRECTIONQUERY_REQUEST_MAX_SIZE)
    {
        return -CANARD_ERROR_INTERNAL;
    }

    return canardRequestOrRespond(
        &canard_instance,
        DRONECAN_DSHOT_NODE_ID,
        DRONECAN_DSHOT_DIRECTIONQUERY_SIGNATURE,
        DRONECAN_DSHOT_DIRECTIONQUERY_ID,
        &g_direction_query_transfer_id,
        CANARD_TRANSFER_PRIORITY_MEDIUM,
        CanardRequest,
        payload,
        (uint16_t)payload_length);
}

bool DroneCAN_TakeDirectionQueryResponse(
    DroneCANDirectionQueryResponse_t *response)
{
    if ((response == NULL) || (!g_direction_query_response_available))
    {
        return false;
    }

    *response = g_direction_query_response;
    g_direction_query_response_available = false;
    return true;
}

HAL_StatusTypeDef DroneCAN_ProcessRx(void)
{
    CAN_PortRxFrame_t port_frame;
    const uint64_t now_usec = (uint64_t)HAL_GetTick() * 1000ULL;

    for (uint8_t index = 0U; index < DRONECAN_RX_FRAME_BUDGET; ++index)
    {
        CanardCANFrame canard_frame = {0};
        HAL_StatusTypeDef status = CAN_Port_TryReceive(&port_frame);

        if (status == HAL_BUSY)
        {
            break;
        }
        if (status != HAL_OK)
        {
            g_dronecan_rx_error_count++;
            return status;
        }

        canard_frame.id = port_frame.extended_id | CANARD_CAN_FRAME_EFF;
        canard_frame.data_len = port_frame.data_length;
        for (uint8_t byte = 0U; byte < port_frame.data_length; ++byte)
        {
            canard_frame.data[byte] = port_frame.data[byte];
        }

        if (canardHandleRxFrame(
                &canard_instance, &canard_frame, now_usec) < 0)
        {
            g_dronecan_rx_error_count++;
        }
        g_dronecan_rx_frame_count++;
    }

    /* 每秒释放一次未完成多帧传输留下的超时接收状态。 */
    if ((now_usec - g_last_rx_cleanup_usec) >= 1000000ULL)
    {
        canardCleanupStaleTransfers(&canard_instance, now_usec);
        g_last_rx_cleanup_usec = now_usec;
    }

    return HAL_OK;
}

int16_t DroneCAN_Motor1_SetReversed(void){
    return DroneCAN_SetMotorsReversed(0x01U);
}

int16_t DroneCAN_SetMotorsNormal(uint8_t motor_mask){
    return DroneCAN_SetMotorDirection(
        motor_mask,
        DRONECAN_DSHOT_DIRECTIONCOMMAND_OPERATION_SET_NORMAL);
}

int16_t DroneCAN_SetMotorsReversed(uint8_t motor_mask){
    return DroneCAN_SetMotorDirection(
        motor_mask,
        DRONECAN_DSHOT_DIRECTIONCOMMAND_OPERATION_SET_REVERSED);
}


HAL_StatusTypeDef DroneCAN_ProcessTx(void){

    CanardCANFrame *tx_frame;

    HAL_StatusTypeDef status;

    //canardPeekTxQueue 是ibcanard的函数,它会返回发送队列中优先级最高的CAN帧,如果发送队列为空则返回NULL。
    /**
     * canardPeekTxQueue 
     */
    tx_frame = canardPeekTxQueue(&canard_instance);

    /**
     * 如果没有要发送的CAN帧,直接返回HAL_OK,表示没有错误,只是没有要发送的CAN帧。
     */
    if(tx_frame == NULL){

        return HAL_OK;    //没有要发送的CAN帧,直接返回

    }

    if((tx_frame->id & CANARD_CAN_FRAME_EFF) == 0U){
        /**
         * DroneCAN 帧必须是29位扩展帧,而不是11位标准帧,所以如果tx_frame->id的最高位没有设置,说明这个CAN帧不是扩展帧,libcanard不支持发送标准帧。
         */
        return HAL_ERROR; 

    }

    if((tx_frame->id & CANARD_CAN_FRAME_RTR) != 0U){
        /**
         * DroneCAN 帧必须是数据帧,而不是远程帧,所以如果tx_frame->id的第二高位设置了,说明这个CAN帧是远程帧,libcanard不支持发送远程帧。
         */
        return HAL_ERROR; 

    }

    //CAN_Port_SendExtendedMessage 中调用了 HAL_CAN_AddTxMessage() 发送CAN帧,如果发送成功则返回HAL_OK,否则返回HAL_ERROR或HAL_BUSY。
    status = CAN_Port_SendExtendedMessage(tx_frame->id & CANARD_CAN_EXT_ID_MASK, tx_frame->data, tx_frame->data_len);

    if(status == HAL_OK){

        /**
         * canardPopTxQueue 是ibcanard的函数,它会从发送队列中移除优先级最高的CAN帧,因为这个CAN帧已经成功发送出去,所以需要从发送队列中移除。
         * 
         * HAL_OK = 帧已经复制进CAN硬件发送邮箱
         */
        canardPopTxQueue(&canard_instance);  //发送成功,从发送队列中移除这个CAN帧。

    }


    return status;



}
