#ifndef KEY_INPUT_H
#define KEY_INPUT_H

#include <stdint.h>

/**
 * @brief 控制器面板上的物理按键编号。
 *
 * 该编号只描述“是哪一个按键”，不包含按键对应的业务功能。
 * 例如当前UP用于设置1号电机Normal，但以后进入OLED菜单后，UP也可以
 * 用于移动光标；这种业务映射应放在InputTask或UI状态机中完成。
 */
typedef enum
{
    KEY_ID_UP = 0,
    KEY_ID_DOWN,
    KEY_ID_CONFIRM,
    KEY_ID_BACK,
    KEY_ID_SWITCH,
    KEY_ID_COUNT
} KeyId_t;

/**
 * @brief 单个按键的消抖状态。
 *
 * 机械按键在按下和松开瞬间都会产生数毫秒的高低电平抖动，因此不能
 * 读取到一次低电平就立即判定按下，也不能读取到一次高电平就立即判定
 * 松开。下面两个DEBOUNCE状态用于确认电平是否已经连续稳定。
 */
typedef enum
{
    /** 按键已经稳定松开，等待检测新的按下动作。 */
    KEY_STATE_RELEASED = 0,

    /**
     * 第一次检测到按下电平，但还不能确认是真正按下；只有连续多个
     * 扫描周期都保持按下，才会进入KEY_STATE_PRESSED并产生一次按下事件。
     */
    KEY_STATE_DEBOUNCE_PRESS,

    /**
     * 按键已经稳定按下。停留在此状态期间不会重复产生按下事件，
     * 因此用户一直按住按键也只会提交一次方向命令。
     */
    KEY_STATE_PRESSED,

    /**
     * 第一次检测到松开电平，但还不能确认是真正松开；只有连续多个
     * 扫描周期都保持松开，才会回到KEY_STATE_RELEASED并产生松开事件。
     */
    KEY_STATE_DEBOUNCE_RELEASE
} KeyState_t;

/** @brief 状态机对外产生的离散按键事件。 */
typedef enum
{
    KEY_EVENT_PRESSED = 0,
    KEY_EVENT_RELEASED
} KeyEventType_t;

/** @brief 一条按键事件，包含按键编号和动作类型。 */
typedef struct
{
    KeyId_t key_id;
    KeyEventType_t event_type;
} KeyEvent_t;

/**
 * @brief 初始化5个按键的消抖状态机。
 *
 * 如果某个按键在系统启动时已经被按住，该按键会初始化为PRESSED，
 * 但不会产生按下事件；必须先松开并再次按下后才会产生事件。这样可以
 * 防止上电时按键卡住或用户按住按键导致意外发送方向修改命令。
 */
void KeyInput_Init(void);

/**
 * @brief 扫描全部5个按键并推进各自的消抖状态机。
 *
 * InputTask应每10 ms调用一次。本函数最多为每个按键产生一条事件，
 * 因此调用方通常提供KEY_ID_COUNT个KeyEvent_t的数组。
 *
 * @param[out] events      用于保存本次扫描产生的事件。
 * @param[in]  max_events  events数组最多能够保存的事件数量。
 * @return 实际写入events数组的事件数量。
 */
uint8_t KeyInput_Scan(KeyEvent_t *events, uint8_t max_events);

/**
 * @brief 获取指定按键当前所处的内部状态，主要用于调试器观察。
 * @param key_id 按键编号。
 * @return 有效按键返回对应状态；无效编号返回KEY_STATE_RELEASED。
 */
KeyState_t KeyInput_GetState(KeyId_t key_id);

#endif /* KEY_INPUT_H */
