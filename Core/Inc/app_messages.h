#ifndef APP_MESSAGES_H
#define APP_MESSAGES_H

#include <stdint.h>

/** @brief 由应用输入层提交给CanTask的命令种类。 */
typedef enum
{
    CAN_COMMAND_SET_DIRECTION = 1
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

    /** bit0选择电机1，bit1选择电机2，依此类推；不能为0。 */
    uint8_t motor_mask;

    /** 目标方向，而不是“切换方向”；接收方将设置为明确状态。 */
    MotorDirection_t direction;
} CanCommand_t;




#endif /* APP_MESSAGES_H */
