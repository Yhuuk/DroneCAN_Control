#ifndef APP_MESSAGES_H
#define APP_MESSAGES_H

#include <stdint.h>

/** @brief 由应用输入层提交给CanTask的命令种类。 */
typedef enum
{
    CAN_COMMAND_SET_DIRECTION = 1,

    /** 启动一次motor_mask指定通道的方向查询服务事务。 */
    CAN_COMMAND_START_DIRECTION_QUERY
} CanCommandType_t;

/**
 * @brief 应用层使用的电机目标方向。
 *
 * UI/业务层只表达Normal/Reversed语义，不接触DSDL结构体、Transfer-ID
 * 或libcanard。CanTask收到该值后，再调用dronecan_node中的封装函数。
 */
typedef enum
{
    MOTOR_DIRECTION_NORMAL = 1,
    MOTOR_DIRECTION_REVERSED = 2
} MotorDirection_t;

/**
 * @brief UI/业务层通过FreeRTOS消息队列发送给CanTask的一条命令。
 *
 * 消息队列会复制该结构体的全部内容，因此发送方可以使用局部变量，
 * 不需要在两个任务之间共享结构体地址。
 */
typedef struct
{
    CanCommandType_t command_type;

    /**
     * UiTask为每次提交分配的本地关联号。
     *
     * 该值不会放入DroneCAN报文，只用于CanTask把处理结果准确返回给
     * 发起本次操作的UiTask，避免把旧结果误认为当前操作的结果。
     */
    uint16_t request_token;

    /** bit0选择电机1，bit1选择电机2，依此类推；不能为0。 */
    uint8_t motor_mask;

    /** 目标方向，而不是“切换方向”；接收方将设置为明确状态。 */
    MotorDirection_t direction;
} CanCommand_t;

/** @brief CanTask对一条应用命令的本地处理结果。 */
typedef enum
{
    /** 参数有效，并已成功加入libcanard软件发送队列。 */
    CAN_COMMAND_RESULT_ACCEPTED = 0,

    /** 参数无效，或libcanard未能把消息加入软件发送队列。 */
    CAN_COMMAND_RESULT_REJECTED
} CanCommandResultStatus_t;

/**
 * @brief CanTask返回给UiTask的一条命令处理结果。
 *
 * ACCEPTED表示方向消息已被libcanard受理，随后由CanTask搬入CAN硬件
 * 邮箱；它不是转接板执行完成的远端应答。当前DirectionCommand是广播
 * 消息，没有与之配套的响应DSDL，因此两者必须明确区分。
 */
typedef struct
{
    /** 原样返回CanCommand_t.request_token，用于匹配本次UI操作。 */
    uint16_t request_token;

    CanCommandType_t command_type;
    uint8_t motor_mask;
    MotorDirection_t direction;
    CanCommandResultStatus_t status;

    /**
     * DroneCAN封装函数的原始返回值：大于0表示入队帧数，0或负数
     * 表示未入队。参数在到达DroneCAN层之前被拒绝时该值为0。
     */
    int16_t transport_result;
} CanCommandResult_t;

/** @brief CanTask通知UiTask的方向查询阶段。 */
typedef enum
{
    /** START_QUERY已经得到ACCEPTED/IN_PROGRESS响应，查询仍在运行。 */
    DIRECTION_QUERY_EVENT_ACTIVE = 0,

    /** 收到并校验通过的STATUS_COMPLETE最终结果。 */
    DIRECTION_QUERY_EVENT_COMPLETE,

    /** 查询被远端拒绝、响应非法、发送失败或总等待超时。 */
    DIRECTION_QUERY_EVENT_FAILED
} DirectionQueryEventType_t;

/* 不与DSDL的1~9状态码冲突的本地失败原因。 */
#define DIRECTION_QUERY_LOCAL_STATUS_TX_ERROR          0xFDU
#define DIRECTION_QUERY_LOCAL_STATUS_INVALID_RESPONSE  0xFEU
#define DIRECTION_QUERY_LOCAL_STATUS_TIMEOUT           0xFFU

/**
 * @brief CanTask交给UiTask的一条方向查询进度或最终结果。
 *
 * response_status保存DirectionQuery DSDL的STATUS_*数值；0xFD~0xFF用于
 * 本地错误。只有event_type为COMPLETE时，UiTask才把方向位图用于显示。
 */
typedef struct
{
    uint16_t request_token;
    uint16_t request_id;
    DirectionQueryEventType_t event_type;
    uint8_t response_status;
    uint8_t query_motor_mask;
    uint8_t valid_mask;
    uint8_t reversed_mask;
    uint8_t timeout_mask;
    uint8_t crc_error_mask;
    uint8_t unsupported_mask;
    uint8_t protocol_error_mask;
    uint8_t maintenance_error;
} DirectionQueryEvent_t;




#endif /* APP_MESSAGES_H */
