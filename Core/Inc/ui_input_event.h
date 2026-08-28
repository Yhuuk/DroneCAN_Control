#ifndef UI_INPUT_EVENT_H
#define UI_INPUT_EVENT_H

#include "app_messages.h"
#include "key_input.h"

/**
 * @brief UI层真正关心的按键动作。
 *
 * 按下、松开和消抖均由key_input模块处理。UI只接收已经确认完成的短按
 * 或长按动作，因此UiTask不需要再次读取GPIO，也不需要自己计算按住时间。
 */
typedef enum
{
    UI_INPUT_ACTION_SHORT_PRESS = 0,
    UI_INPUT_ACTION_LONG_PRESS
} UiInputAction_t;

/**
 * @brief InputTask通过UiEventQueue交给UiTask的一条输入事件。
 *
 * 结构体只保存“哪个按键”和“什么动作”，不保存指针。FreeRTOS消息队列
 * 会复制整个结构体，所以InputTask中的局部变量离开作用域后仍然安全。
 */
typedef struct
{
    /** 产生动作的物理按键，例如UP、DOWN、CONFIRM或BACK。 */
    KeyId_t key_id;

    /** 已经由按键层判定完成的短按或长按。 */
    UiInputAction_t action;
} UiInputEvent_t;

/**
 * @brief UiEventQueue中消息的种类。
 *
 * 同一个队列既可接收InputTask产生的按键动作，也可接收CanTask返回的
 * 命令处理结果。所有UI业务仍由UiTask串行处理，因此不需要共享状态或
 * 额外互斥锁。
 * 
 * UiEventMessageType_t UI事件消息类型
 */
typedef enum
{
    UI_EVENT_MESSAGE_INPUT = 0,
    UI_EVENT_MESSAGE_CAN_COMMAND_RESULT
} UiEventMessageType_t;

/** @brief 交给UiTask处理的一条带类型消息。
 * 
 * 
 * union 是联合体，两个数据类型共享一个内存地址，一次只能一种类型
 */
typedef struct
{
    UiEventMessageType_t message_type;

    union
    {
        UiInputEvent_t input;
        CanCommandResult_t can_command_result;
    } data;
} UiEventMessage_t;

#endif /* UI_INPUT_EVENT_H */
