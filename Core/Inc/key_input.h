#ifndef KEY_INPUT_H
#define KEY_INPUT_H

#include <stdint.h>

/*
 * InputTask必须按照该周期调用KeyInput_Scan()，因为消抖和长按时间均由
 * 扫描次数换算。后续若修改扫描周期，应同时检查按键手感和任务周期。
 */
#define KEY_INPUT_SCAN_PERIOD_MS       10U
#define KEY_INPUT_DEBOUNCE_TIME_MS     30U
#define KEY_INPUT_LONG_PRESS_TIME_MS  800U

/**
 * @brief 控制器面板上的物理按键编号。
 *
 * 该编号只描述“是哪一个按键”，不包含按键对应的业务功能。
 * 例如UP在不同页面可以表示移动光标或选择NOR；这种业务映射应由
 * UiTask状态机完成，而不是写进GPIO扫描和消抖模块。
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

/*
 * 一个按键稳定释放时可能同时产生SHORT_PRESS和RELEASED两条事件，因此
 * 最坏情况下5个按键同一扫描周期释放，需要容纳KEY_ID_COUNT * 2条事件。
 */
#define KEY_INPUT_MAX_EVENTS_PER_SCAN (KEY_ID_COUNT * 2U)

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
     * 因此用户一直按住按键也只会产生一次稳定按下事件。
     */
    KEY_STATE_PRESSED,

    /**
     * 第一次检测到松开电平，但还不能确认是真正松开；只有连续多个
     * 扫描周期都保持松开，才会回到KEY_STATE_RELEASED并产生松开事件。
     */
    KEY_STATE_DEBOUNCE_RELEASE
} KeyState_t;

/**
 * @brief 状态机对外产生的离散按键事件。
 *
 * PRESSED/RELEASED描述消抖后的物理边沿；SHORT_PRESS/LONG_PRESS描述用户
 * 动作。UI业务应优先使用SHORT_PRESS和LONG_PRESS，而不是自行计算时间。
 */
typedef enum
{
    /** 按键经过按下消抖，刚进入稳定按下状态。 */
    KEY_EVENT_PRESSED = 0,

    /** 按键经过释放消抖，刚回到稳定松开状态。 */
    KEY_EVENT_RELEASED,

    /** 稳定按下不足800 ms后松开；在确认释放时产生一次。 */
    KEY_EVENT_SHORT_PRESS,

    /** 稳定按住达到800 ms；达到阈值时产生一次，继续按住不会重复。 */
    KEY_EVENT_LONG_PRESS
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
 * InputTask应每KEY_INPUT_SCAN_PERIOD_MS调用一次。稳定释放一个短按时会
 * 同时产生SHORT_PRESS和RELEASED，因此调用方应提供至少
 * KEY_INPUT_MAX_EVENTS_PER_SCAN个KeyEvent_t的数组。
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
