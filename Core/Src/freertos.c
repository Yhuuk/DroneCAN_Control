/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_settings.h"
#include "app_messages.h"
#include "brightness_ui.h"
#include "can_port.h"
#include "dronecan_config.h"
#include "dronecan_node.h"
#include "joystick.h"
#include "joystick_calibration.h"
#include "joystick_calibration_result_ui.h"
#include "joystick_calibration_ui.h"
#include "key_input.h"
#include "lcd_init.h"
#include "main_ui.h"
#include "motor_direction_ui.h"
#include "settings_ui.h"
#include "throttle_control.h"
#include "throttle_ui.h"
#include "ui_input_event.h"
#include <dronecan_dshot.DirectionQuery.h>

#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
 * @brief UiTask私有的方向命令提交状态。
 *
 * 该结构体只在UiTask上下文中读写，不会被CanTask或软件定时器直接访问，
 * 因此无需volatile和互斥锁。
 * 
 */
typedef struct
{
  /** true表示已经提交命令，但尚未收到CanTask的本地处理结果。 */
  bool waiting_for_result;

  /** true表示2秒重复发送保护窗口仍然有效。 */
  bool protection_active;

  /** 当前等待结果的命令关联号。 也就是正要准备发送转向命令的编号*/
  uint16_t pending_token;

  /** 用于校验CanTask返回结果是否确实属于当前命令。 */
  uint8_t pending_motor_mask;
  MotorDirection_t pending_direction;

  /** 下一条命令使用的关联号；0保留为“无关联号”。 */
  uint16_t next_token;

  /** 使用RTOS Tick表示的保护截止时刻。 */
  uint32_t protection_deadline_tick;

  /** true表示保护期结束后需要自动查询刚修改通道的实际方向。 */
  bool auto_query_pending;

  /** 自动查询目标；bit0~bit7分别对应电机1~8。 */
  uint8_t auto_query_motor_mask;
} UiDirectionCommandControl_t;

typedef struct
{
  bool active;
  uint16_t pending_token;
  uint16_t next_token;
} UiDirectionQueryControl_t;

typedef struct
{
  bool active;
  bool awaiting_response;
  uint8_t last_operation;
  uint8_t motor_mask;
  uint16_t request_token;
  uint16_t request_id;
  uint16_t next_request_id;
  uint32_t response_deadline_tick;
  uint32_t next_poll_tick;
  uint32_t overall_deadline_tick;
} CanDirectionQueryControl_t;

/** @brief UiTask当前显示的页面。 */
typedef enum
{
  UI_PAGE_MAIN = 0,
  UI_PAGE_MOTOR_DIRECTION,
  UI_PAGE_THROTTLE,
  UI_PAGE_SETTINGS,
  UI_PAGE_JOYSTICK_CALIBRATION,
  UI_PAGE_JOYSTICK_CALIBRATION_RESULT,
  /** 主页面第4个“状态”入口暂时承载OLED亮度测试页面。 */
  UI_PAGE_BRIGHTNESS
} UiPage_t;

/**
 * @brief CanTask内部的油门发布状态。
 *
 * LOCKED_SILENT：关锁且保持总线静默，不发布RawCommand；
 * UNLOCKED_ACTIVE：解锁后以固定周期发布当前8路目标；
 * LOCKING_ZERO_FLUSH：刚从解锁切到关锁，先发布若干次全零再静默。
 */
typedef enum
{
  CAN_THROTTLE_PUBLISH_LOCKED_SILENT = 0,
  CAN_THROTTLE_PUBLISH_UNLOCKED_ACTIVE,
  CAN_THROTTLE_PUBLISH_LOCKING_ZERO_FLUSH
} CanThrottlePublishState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/** InputTask每10个RTOS Tick扫描一次按键；当前1 Tick等于1 ms。 */
#define INPUT_KEY_SCAN_PERIOD_TICKS KEY_INPUT_SCAN_PERIOD_MS

/**
 * 方向命令是低频用户操作，长度8足以吸收短时间内的按键事件，同时又不会
 * 允许大量过期方向命令在队列中堆积。
 */
#define CAN_COMMAND_QUEUE_LENGTH 8U

/**
 * 按键动作非常低频，16条队列深度足以覆盖UiTask刷新整屏期间连续产生的
 * 输入。队列满时InputTask不会阻塞，而是记录并丢弃新事件，避免破坏固定
 * 10 ms按键扫描周期。
 */
#define UI_EVENT_QUEUE_LENGTH 16U

/** 主页面油门进度条以25 Hz局部刷新，锁图标也随同检查但仅在变化时绘制。 */
#define MAIN_UI_REFRESH_PERIOD_MS 40U

/** 油门页进度条以25 Hz刷新，兼顾连续观感与当前阻塞SPI负载。 */
#define THROTTLE_UI_BAR_REFRESH_PERIOD_MS 40U

/** 精确油门数字以20 Hz刷新，避免数字跳动过快且降低无效绘制量。 */
#define THROTTLE_UI_VALUE_REFRESH_PERIOD_MS 50U

/** 中心校准按InputTask发布周期每10 ms尝试接收一份新ADC快照。 */
#define JOYSTICK_CALIBRATION_SAMPLE_PERIOD_MS 10U

/** 原始X/Y数值以20 Hz局部刷新；采样统计本身仍保持100 Hz。 */
#define JOYSTICK_CALIBRATION_DISPLAY_PERIOD_MS 50U

/** 亮度测试页每次短按UP/DOWN改变约6.3%，并在0和255处饱和。 */
#define BRIGHTNESS_UI_ADJUST_STEP 16U

/** LIM/STEP停止变化5秒后自动保存；退出参数编辑时还会立即保存。 */
#define APP_SETTINGS_AUTOSAVE_DELAY_MS 5000U

/** 以100 Hz周期广播最新8路油门目标，与InputTask的10 ms快照周期一致。 */
#define THROTTLE_RAW_COMMAND_PUBLISH_PERIOD_MS 10U

/** 关锁时连续成功入队5次全零RawCommand，然后停止发布并释放总线控制权。 */
#define THROTTLE_LOCK_ZERO_FLUSH_COUNT 5U

/** 方向反馈LED每100 ms切换一次，形成清晰的快速闪烁。 */
#define DIRECTION_LED_FLASH_HALF_PERIOD_MS 100U

/** 一次反馈固定包含3次完整的“亮→灭”。 */
#define DIRECTION_LED_FLASH_COUNT 3U

/**蜂鸣器快速响400ms，400ms这个鸣叫时间我觉得刚刚好 */
#define DIRECTION_BUZZ_DURATION_MS 400U

/** 油门锁切换只需短促提示，100 ms足够分辨且不会干扰连续操作。 */
#define THROTTLE_LOCK_BUZZ_DURATION_MS 100U

/**
 * 接收板执行一次DShot方向修改约需1.59秒。在上一条命令提交后的2秒内
 * 禁止再次提交方向命令，避免接收板忙碌时重复触发或切换其它通道。
 */
#define DIRECTION_COMMAND_PROTECTION_TIME_MS 2000U

/** 查询动画每250 ms多点亮一个圆点，750 ms完成一次1→2→3循环。 */
#define DIRECTION_QUERY_ANIMATION_PERIOD_MS 250U
/** ACCEPTED后先避开接收板固定3.5 s准备阶段，再请求最终结果。 */
#define DIRECTION_QUERY_INITIAL_RESULT_DELAY_MS 4000U
/** IN_PROGRESS或一次响应丢失后，以500 ms间隔继续查询。 */
#define DIRECTION_QUERY_POLL_INTERVAL_MS 500U
/** 单次服务请求等待响应的时间。 */
#define DIRECTION_QUERY_RESPONSE_TIMEOUT_MS 500U
/** 包含最坏重试时间的整次查询上限。 */
#define DIRECTION_QUERY_OVERALL_TIMEOUT_MS 15000U
#define DIRECTION_QUERY_ALL_MOTORS_MASK 0xFFU

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/**
 * UiTask是该队列的生产者，CanTask是唯一消费者。队列传递结构体副本，
 * UiTask不直接接触libcanard；CanTask负责协议封装和CAN发送资源。
 */
osMessageQueueId_t CanCommandQueueHandle;

/**
 * InputTask和CanTask是该队列的生产者，UiTask是唯一消费者。队列中保存
 * UiEventMessage_t副本：按键动作和CAN命令结果都由UiTask串行处理，
 * 不共享局部变量地址，也不要求多个任务直接操作UI状态。
 */
osMessageQueueId_t UiEventQueueHandle;

/**
 * 方向LED软件定时器。回调运行在FreeRTOS定时器服务任务中，不占用UiTask
 * 等待时间，也不需要额外的硬件定时器中断。
 */
osTimerId_t DirectionLedTimerHandle;


/***
 * 
 * 这是一次性定时器，用于在确认方向指令时提供声音反馈
 */
osTimerId_t DirectionBuzzTimerHandle;
/** 仅产生UI动画Tick，不在定时器回调中直接访问LCD。 */
osTimerId_t DirectionQueryAnimationTimerHandle;

/**
 * 方向命令提交后的一次性等待定时器。到期时只通知UiTask开始查询，
 * 定时器回调本身既不访问LCD，也不直接操作CAN/libcanard。
 */
osTimerId_t DirectionAutoQueryTimerHandle;


/* 以下变量只用于调试器观察运行情况，不参与控制逻辑。 */
volatile uint32_t g_key_press_count[KEY_ID_COUNT];
volatile uint32_t g_key_short_press_count[KEY_ID_COUNT];
volatile uint32_t g_key_long_press_count[KEY_ID_COUNT];
volatile uint32_t g_ui_event_queued_count;
volatile uint32_t g_ui_event_queue_full_count;
volatile uint32_t g_ui_direction_long_confirm_count;
/** 最近一次LCD初始/局部刷新的耗时；当前1个RTOS Tick等于1 ms。 */
volatile uint32_t g_ui_last_refresh_time_ms;
/** 截至当前观察到的最长LCD刷新耗时，便于检查偶发卡顿。 */
volatile uint32_t g_ui_max_refresh_time_ms;
/** 已经成功启动过多少轮“方向LED闪烁3次”反馈。 */
volatile uint32_t g_direction_led_feedback_count;
/** 当前一轮已经完成的亮灭次数，正常结束时等于3。 */
volatile uint8_t g_direction_led_completed_flashes;
/** LED当前逻辑状态；方向LED低电平有效。 */
static bool g_direction_led_is_on;
volatile uint32_t g_can_command_accepted_count;
volatile uint32_t g_can_command_rejected_count;
volatile uint32_t g_can_tx_busy_count;
volatile uint32_t g_can_tx_error_count;
volatile uint32_t g_can_rx_error_count;
volatile int16_t g_last_direction_enqueue_result;
/** RawCommand发布调试量：最近值、最近入队结果以及成功/失败次数。 */
volatile uint16_t g_last_throttle_raw_command;
volatile int16_t g_last_throttle_enqueue_result;
volatile uint32_t g_throttle_publish_count;
volatile uint32_t g_throttle_publish_error_count;
/** 当前发布状态和关锁全零剩余次数，仅用于调试器观察。 */
volatile CanThrottlePublishState_t g_throttle_publish_state;
volatile uint8_t g_throttle_zero_flush_remaining;
/** UiTask已经成功放入CanCommandQueue的方向命令数量。 */
volatile uint32_t g_ui_direction_command_submitted_count;
/** 因CanCommandQueue已满而未能提交的方向命令数量。 */
volatile uint32_t g_ui_direction_command_queue_full_count;
/** 因2秒保护窗口或等待CanTask结果而被忽略的重复长按数量。 */
volatile uint32_t g_ui_direction_command_protected_count;
/** UiTask收到并匹配成功的CanTask受理结果数量。 */
volatile uint32_t g_ui_direction_command_accepted_count;
/** UiTask收到并匹配成功的CanTask拒绝结果数量。 */
volatile uint32_t g_ui_direction_command_rejected_count;
/** 关联号不匹配的迟到或无效CanTask结果数量。 */
volatile uint32_t g_ui_direction_command_stale_result_count;
/** CanTask无法把关键命令结果放入UiEventQueue的次数。 */
volatile uint32_t g_can_result_event_queue_full_count;
volatile uint32_t g_direction_query_started_count;
volatile uint32_t g_direction_query_completed_count;
volatile uint32_t g_direction_query_failed_count;
volatile uint32_t g_direction_query_poll_count;
volatile uint32_t g_direction_query_response_timeout_count;
volatile uint32_t g_direction_query_stale_response_count;
volatile uint32_t g_direction_query_ui_event_drop_count;
volatile uint8_t g_last_direction_query_status;
volatile uint8_t g_last_direction_query_valid_mask;
volatile uint8_t g_last_direction_query_reversed_mask;
volatile uint8_t g_last_direction_query_timeout_mask;
volatile uint8_t g_last_direction_query_crc_error_mask;
volatile uint8_t g_last_direction_query_unsupported_mask;
volatile uint8_t g_last_direction_query_protocol_error_mask;
volatile uint8_t g_last_direction_query_maintenance_error;
/** 成功安排/启动方向修改后自动单通道查询的次数。 */
volatile uint32_t g_direction_auto_query_scheduled_count;
volatile uint32_t g_direction_auto_query_started_count;
/** 自动查询到期事件因UiEventQueue满而丢失的次数。 */
volatile uint32_t g_direction_auto_query_event_drop_count;
/** 最近一次成功发送给SH8501A的0~255亮度值及发送失败次数。 */
volatile uint8_t g_lcd_brightness = 0xFFU;
volatile uint32_t g_lcd_brightness_error_count;

/* USER CODE END Variables */
/* Definitions for UiTask */
osThreadId_t UiTaskHandle;
const osThreadAttr_t UiTask_attributes = {
  .name = "UiTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for CanTask */
osThreadId_t CanTaskHandle;
const osThreadAttr_t CanTask_attributes = {
  .name = "CanTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for InputTask */
osThreadId_t InputTaskHandle;
const osThreadAttr_t InputTask_attributes = {
  .name = "InputTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SPI1BusMutex */
osMutexId_t SPI1BusMutexHandle;
const osMutexAttr_t SPI1BusMutex_attributes = {
  .name = "SPI1BusMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static void InputTask_PostUiEvent(const KeyEvent_t *key_event);
static uint32_t UiTask_TicksUntil(uint32_t now_tick,
                                  uint32_t deadline_tick);
static void UiTask_AdvancePeriodicDeadline(uint32_t *deadline_tick,
                                           uint32_t period_ticks,
                                           uint32_t now_tick);
static bool UiTask_UpdateThrottleView(ThrottleUiView_t *view);
static bool UiTask_UpdateMainThrottleView(MainUiView_t *view);
static bool UiTask_HandleMainInput(MainUiView_t *view,
                                   const UiInputEvent_t *event);
static bool UiTask_HandleSettingsInput(SettingsUiView_t *view,
                                       const UiInputEvent_t *event);
static bool UiTask_HandleBrightnessInput(BrightnessUiView_t *view,
                                         const UiInputEvent_t *event);
static void UiTask_PrepareDirectionPageEntry(MotorDirectionUiView_t *view);
static void UiTask_PrepareThrottlePageEntry(ThrottleUiView_t *view);
static bool UiTask_ShouldLeaveThrottlePage(
    const ThrottleUiView_t *view,
    const UiInputEvent_t *event);
static bool UiTask_HandleThrottleInput(ThrottleUiView_t *view,
                                       const UiInputEvent_t *event);
static bool UiTask_ShouldLeaveDirectionPage(
    const MotorDirectionUiView_t *view,
    const UiInputEvent_t *event);
static bool UiTask_HandleInputEvent(MotorDirectionUiView_t *view,
                                    UiDirectionCommandControl_t *control,
                                    UiDirectionQueryControl_t *query_control,
                                    const UiInputEvent_t *event);
static bool UiTask_SubmitDirectionCommand(
    const MotorDirectionUiView_t *view,
    UiDirectionCommandControl_t *control);
static bool UiTask_HandleCanCommandResult(
    MotorDirectionUiView_t *view,
    UiDirectionCommandControl_t *control,
    const CanCommandResult_t *result);
static bool UiTask_DirectionCommandIsProtected(
    UiDirectionCommandControl_t *control);
static void DirectionLedFeedback_Start(void);
static void DirectionLedTimerCallback(void *argument);
static void DirectionBuzzTimerCallback(void *argument);
static void DirectionBuzzFeedback(void);
static void BuzzerFeedback_Start(uint32_t duration_ms);
static void ThrottleLockHardwareFeedback(bool unlocked);
static bool UiTask_SubmitDirectionQuery(
    MotorDirectionUiView_t *view,
    UiDirectionQueryControl_t *control);
static bool UiTask_StartDirectionQuery(
    MotorDirectionUiView_t *view,
    UiDirectionQueryControl_t *control,
    uint8_t motor_mask);
static bool UiTask_HandleDirectionQueryEvent(
    MotorDirectionUiView_t *view,
    UiDirectionQueryControl_t *control,
    const DirectionQueryEvent_t *event);
static bool UiTask_GetExtremeStepConfig(
    JoystickCalibrationUiStep_t step,
    JoystickCalibrationExtremeSlot_t *slot,
    JoystickCalibrationAxis_t *axis,
    JoystickCalibrationExtremeDirection_t *direction);
static bool UiTask_AreCalibrationCandidatesValid(
    const JoystickCenterCalibration_t *center,
    const JoystickExtremeCalibration_t
        extremes[JOYSTICK_CALIBRATION_SLOT_COUNT]);
static void UiTask_PrepareCalibrationResultView(
    JoystickCalibrationResultUiView_t *view,
    const JoystickCenterCalibration_t *center,
    const JoystickExtremeCalibration_t
        extremes[JOYSTICK_CALIBRATION_SLOT_COUNT],
    const AppSettings_t *settings);
static void UiTask_ResetCalibrationSession(
    JoystickCenterCalibration_t *center,
    JoystickExtremeCalibration_t
        extremes[JOYSTICK_CALIBRATION_SLOT_COUNT]);
static void DirectionQueryAnimationTimerCallback(void *argument);
static void DirectionAutoQueryTimerCallback(void *argument);
static void CanTask_HandleCommand(const CanCommand_t *command,
                                  CanCommandResult_t *result);
static void CanTask_PostCommandResult(const CanCommandResult_t *result);
static void CanTask_StartDirectionQuery(
    CanDirectionQueryControl_t *control,
    const CanCommand_t *command);
static void CanTask_PollDirectionQuery(CanDirectionQueryControl_t *control);
static void CanTask_PostDirectionQueryEvent(
    const CanDirectionQueryControl_t *control,
    DirectionQueryEventType_t event_type,
    uint8_t response_status,
    const DroneCANDirectionQueryResponse_t *response);

/* USER CODE END FunctionPrototypes */

void StartUiTask(void *argument);
void StartCanTask(void *argument);
void StartInputTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  AppSettings_t stored_settings;

  /* 在任何任务访问共享油门状态前，先明确置为锁定和零输出。 */
  ThrottleControl_Init();

  /*
   * AppSettings已在main()、FreeRTOS启动前从FRAM加载。这里把掉电保存的
   * LIM发布到控制层，确保CanTask第一次读取快照时就是恢复后的限制值。
   * 电机MASK属于本次运行的安全选择，上电仍固定恢复为0，不做掉电保存。
   */
  AppSettings_Get(&stored_settings);
  ThrottleControl_SetConfiguration(
      THROTTLE_CONTROL_DEFAULT_MOTOR_MASK,
      stored_settings.throttle_limit);

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of SPI1BusMutex */
  SPI1BusMutexHandle = osMutexNew(&SPI1BusMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /*
   * 创建周期软件定时器，但此时不启动。只有方向层长按Confirm后才启动，
   * 完成3次闪烁后由回调自动停止。
   * 
   * osTimerPeriodic 这个参数表示 周期性定时器，每次回调后会自动重新计时。
   * 回调函数 DirectionLedTimerCallback 会在定时器到期时被调用，用于切换方向LED的状态。
   */
  DirectionLedTimerHandle = osTimerNew(
      DirectionLedTimerCallback,
      osTimerPeriodic,
      NULL,
      NULL);

  /**
   * 创建一个一次性定时器，用于在确认方向指令时提供声音反馈。该定时器在启动后只会触发一次回调函数 DirectionBuzzTimerCallback。
   * 
   * osTimerOnce 这个参数表示 一次性定时器，定时器到期后只会触发一次回调函数。·
   */
  DirectionBuzzTimerHandle = osTimerNew(
      DirectionBuzzTimerCallback,
      osTimerOnce,
      NULL,
      NULL);

  DirectionQueryAnimationTimerHandle = osTimerNew(
      DirectionQueryAnimationTimerCallback,
      osTimerPeriodic,
      NULL,
      NULL);

  DirectionAutoQueryTimerHandle = osTimerNew(
      DirectionAutoQueryTimerCallback,
      osTimerOnce,
      NULL,
      NULL);

  if (DirectionLedTimerHandle == NULL)
  {
    /* 软件定时器创建失败通常表示FreeRTOS堆空间不足。 */
    Error_Handler();
  }

  if(DirectionBuzzTimerHandle == NULL)
  {
    /* 软件定时器创建失败通常表示FreeRTOS堆空间不足。 */
    Error_Handler();
  }

  if (DirectionQueryAnimationTimerHandle == NULL)
  {
    Error_Handler();
  }

  if (DirectionAutoQueryTimerHandle == NULL)
  {
    Error_Handler();
  }

  /* 方向LED为低电平点亮，空闲状态必须保持高电平熄灭。 */
  HAL_GPIO_WritePin(Direction_LED_GPIO_Port,
                    Direction_LED_Pin,
                    GPIO_PIN_SET);

  /* 油门LED同样低电平点亮；系统上电默认锁定，因此必须保持熄灭。 */
  HAL_GPIO_WritePin(Throttle_LED_GPIO_Port,
                    Throttle_LED_Pin,
                    GPIO_PIN_SET);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /*
   * 创建InputTask到CanTask的单向命令队列。
   * - message_count：最多保存8条尚未处理的命令；
   * - message_size ：每条消息是一个完整的CanCommand_t结构体；
   * - attr         ：NULL表示使用默认动态分配属性。
   */
  CanCommandQueueHandle = osMessageQueueNew(
      CAN_COMMAND_QUEUE_LENGTH,
      sizeof(CanCommand_t),
      NULL);

  if (CanCommandQueueHandle == NULL)
  {
    /*
     * 队列创建失败通常意味着FreeRTOS堆空间不足。没有命令队列就不能
     * 安全地把按键命令交给CanTask，因此当前调试阶段直接进入错误处理。
     */
    Error_Handler();
  }

  /*
   * 创建发送给UiTask的统一事件队列：
   * - InputTask放入已确认的短按/长按，不传递GPIO电平和消抖状态；
   * - CanTask放入方向命令的本地受理/拒绝结果；
   * - 每条消息均带类型，UiTask可永久阻塞等待，不需要轮询两个队列。
   */
  UiEventQueueHandle = osMessageQueueNew(
      UI_EVENT_QUEUE_LENGTH,
      sizeof(UiEventMessage_t),
      NULL);

  if (UiEventQueueHandle == NULL)
  {
    /* 队列创建失败通常表示FreeRTOS堆空间不足。 */
    Error_Handler();
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of UiTask */
  UiTaskHandle = osThreadNew(StartUiTask, NULL, &UiTask_attributes);

  /* creation of CanTask */
  CanTaskHandle = osThreadNew(StartCanTask, NULL, &CanTask_attributes);

  /* creation of InputTask */
  InputTaskHandle = osThreadNew(StartInputTask, NULL, &InputTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartUiTask */
/**
  * @brief  Function implementing the UiTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartUiTask */
void StartUiTask(void *argument)
{
  /* USER CODE BEGIN StartUiTask */
  MainUiView_t main_view = {
      .node_id = DRONECAN_CONTROLLER_NODE_ID,
      .can_online = true,
      .throttle_unlocked = false,
      .throttle_percent = 0U,
      .focus = MAIN_UI_FOCUS_DIRECTION
  };
  UiPage_t current_page = UI_PAGE_MAIN;
  SettingsUiView_t settings_view = {
      .focus = SETTINGS_UI_FOCUS_SYSTEM,
      .first_visible_item = 0U
  };
  /* 亮度页既保留原“状态”入口，也可由设置列表进入。 */
  UiPage_t brightness_return_page = UI_PAGE_MAIN;
  ThrottleUiView_t throttle_view = {
      .throttle_unlocked = false,
      .edit_mode = false,
      .motor_mask = THROTTLE_CONTROL_DEFAULT_MOTOR_MASK,
      .limit = THROTTLE_CONTROL_DEFAULT_LIMIT,
      .step = 100U,
      .raw_command = 0U,
      .throttle_percent = 0U,
      .focus = THROTTLE_UI_FOCUS_ALL
  };
  BrightnessUiView_t brightness_view = {
      .brightness = 0xFFU
  };
  JoystickCalibrationUiView_t calibration_view = {
      .raw_x = 0U,
      .raw_y = 0U,
      .step = JOYSTICK_CALIBRATION_UI_STEP_CENTER,
      .focus = JOYSTICK_CALIBRATION_UI_FOCUS_OK,
      .state = JOYSTICK_CALIBRATION_UI_READY
  };
  JoystickCalibrationResultUiView_t calibration_result_view = {
      .x_axis = {
          .minimum = JOYSTICK_DEFAULT_DIRECTION_MINIMUM,
          .center = JOYSTICK_DEFAULT_DIRECTION_CENTER,
          .maximum = JOYSTICK_DEFAULT_DIRECTION_MAXIMUM,
          .dead_zone = JOYSTICK_DEFAULT_DIRECTION_DEAD_ZONE
      },
      .y_axis = {
          .minimum = JOYSTICK_DEFAULT_THROTTLE_MINIMUM,
          .center = JOYSTICK_DEFAULT_THROTTLE_CENTER,
          .maximum = JOYSTICK_DEFAULT_THROTTLE_MAXIMUM,
          .dead_zone = JOYSTICK_DEFAULT_THROTTLE_DEAD_ZONE
      },
      .focus = JOYSTICK_CALIBRATION_RESULT_FOCUS_SAVE,
      .saved = false
  };
  /*
   * 中心值和四个方向极限均只保存在UiTask本轮校准会话的RAM中。
   * 本次不调用AppSettings，也不会改写joystick.c正在使用的正式参数。
   */
  JoystickCenterCalibration_t center_calibration;
  JoystickExtremeCalibration_t
      extreme_calibrations[JOYSTICK_CALIBRATION_SLOT_COUNT];
  MotorDirectionUiView_t view = {
      .power_state = MOTOR_DIRECTION_UI_POWER_OFF,
      .focus = MOTOR_DIRECTION_UI_FOCUS_SWITCH,
      .selected_motor = 1U,
      .selected_direction = MOTOR_DIRECTION_UI_NORMAL,
      .direction_query_in_progress = false,
      .query_animation_dot_count = 3U,
      .queried_direction_valid_mask = 0U,
      .queried_direction_reversed_mask = 0U
  };
  UiDirectionCommandControl_t direction_control = {
      .waiting_for_result = false,
      .protection_active = false,
      .pending_token = 0U,
      .pending_motor_mask = 0U,
      .pending_direction = MOTOR_DIRECTION_NORMAL,
      .next_token = 1U,
      .protection_deadline_tick = 0U,
      .auto_query_pending = false,
      .auto_query_motor_mask = 0U
  };
  UiDirectionQueryControl_t query_control = {
      .active = false,
      .pending_token = 0U,
      .next_token = 1U
  };
  AppSettings_t persistent_settings;

  //定义 一个Ui事件消息结构体 变量
  UiEventMessage_t event_message;
  uint32_t refresh_start_tick;
  uint32_t refresh_duration;
  uint32_t next_main_refresh_tick;
  uint32_t next_throttle_bar_refresh_tick;
  uint32_t next_throttle_value_refresh_tick;
  uint32_t next_calibration_sample_tick;
  uint32_t next_calibration_display_tick;
  uint32_t queue_wait_ticks;
  uint8_t displayed_throttle_percent = throttle_view.throttle_percent;
  uint16_t displayed_raw_command = throttle_view.raw_command;
  osStatus_t queue_status;

  (void)argument;

  /* 把开机从FRAM恢复的LIM/STEP应用到油门页面本地视图。 */
  AppSettings_Get(&persistent_settings);
  throttle_view.limit = persistent_settings.throttle_limit;
  throttle_view.step = persistent_settings.throttle_step;

  /* 读取控制层的安全初值，确保主页面锁图标与实际输出状态一致。 */
  (void)UiTask_UpdateMainThrottleView(&main_view);
  UiTask_ResetCalibrationSession(&center_calibration,
                                 extreme_calibrations);

  /*
     * UiTask是调度器启动后唯一调用显示绘制函数的任务。开机首先绘制主页面；
     * Node ID暂取固定配置126，锁图标和THR进度读取全局油门控制状态。
   */
  refresh_start_tick = osKernelGetTickCount();
  MainUI_Draw(&main_view);
  refresh_duration = osKernelGetTickCount() - refresh_start_tick;
  g_ui_last_refresh_time_ms = refresh_duration;
  g_ui_max_refresh_time_ms = refresh_duration;
  next_main_refresh_tick =
      osKernelGetTickCount() + pdMS_TO_TICKS(MAIN_UI_REFRESH_PERIOD_MS);
  next_throttle_bar_refresh_tick = 0U;
  next_throttle_value_refresh_tick = 0U;
  next_calibration_sample_tick = 0U;
  next_calibration_display_tick = 0U;

  /* Infinite loop */
  for(;;)
  {
    /*
     * 连续油门显示使用绝对截止时间，而不是依赖消息队列的一次相对超时。
     * 因此即使期间不断收到按键消息，40/50 ms显示周期也不会被重新起算。
     */
    uint32_t now_tick = osKernelGetTickCount();

    /*
     * 只有5秒截止时间到达时才真正访问FRAM。保存失败会保留pending状态，
     * 存储模块按1秒间隔重试，不会在本循环中无间隔反复占用SPI1。
     */
    (void)AppSettings_ProcessDeferredSave(now_tick);

    if ((current_page == UI_PAGE_MAIN) &&
        ((int32_t)(now_tick - next_main_refresh_tick) >= 0))
    {
      const MainUiView_t previous_main_view = main_view;

      if (UiTask_UpdateMainThrottleView(&main_view))
      {
        refresh_start_tick = osKernelGetTickCount();
        MainUI_UpdateThrottleStatus(&previous_main_view, &main_view);
        refresh_duration = osKernelGetTickCount() - refresh_start_tick;
        g_ui_last_refresh_time_ms = refresh_duration;
        if (refresh_duration > g_ui_max_refresh_time_ms)
        {
          g_ui_max_refresh_time_ms = refresh_duration;
        }
      }

      UiTask_AdvancePeriodicDeadline(
          &next_main_refresh_tick,
          pdMS_TO_TICKS(MAIN_UI_REFRESH_PERIOD_MS),
          now_tick);
    }
    else if (current_page == UI_PAGE_THROTTLE)
    {
      const bool bar_due =
          ((int32_t)(now_tick - next_throttle_bar_refresh_tick) >= 0);
      const bool value_due =
          ((int32_t)(now_tick - next_throttle_value_refresh_tick) >= 0);

      if (bar_due || value_due)
      {
        const ThrottleUiView_t previous_throttle_view = throttle_view;
        bool refresh_performed = false;

        refresh_start_tick = osKernelGetTickCount();

        /* 一次读取最新快照，供同一时刻到期的进度条和数字共同使用。 */
        if (UiTask_UpdateThrottleView(&throttle_view))
        {
          /* 锁、MASK或LIM等低频状态仍由通用局部更新接口处理。 */
          ThrottleUI_Update(&previous_throttle_view, &throttle_view);
          refresh_performed = true;
        }

        if (bar_due)
        {
          if (displayed_throttle_percent !=
              throttle_view.throttle_percent)
          {
            ThrottleUI_UpdateThrottleBar(&throttle_view);
            displayed_throttle_percent = throttle_view.throttle_percent;
            refresh_performed = true;
          }

          UiTask_AdvancePeriodicDeadline(
              &next_throttle_bar_refresh_tick,
              pdMS_TO_TICKS(THROTTLE_UI_BAR_REFRESH_PERIOD_MS),
              now_tick);
        }

        if (value_due)
        {
          if (displayed_raw_command != throttle_view.raw_command)
          {
            ThrottleUI_UpdateThrottleValue(&throttle_view);
            displayed_raw_command = throttle_view.raw_command;
            refresh_performed = true;
          }

          UiTask_AdvancePeriodicDeadline(
              &next_throttle_value_refresh_tick,
              pdMS_TO_TICKS(THROTTLE_UI_VALUE_REFRESH_PERIOD_MS),
              now_tick);
        }

        if (refresh_performed)
        {
          refresh_duration = osKernelGetTickCount() - refresh_start_tick;
          g_ui_last_refresh_time_ms = refresh_duration;
          if (refresh_duration > g_ui_max_refresh_time_ms)
          {
            g_ui_max_refresh_time_ms = refresh_duration;
          }
        }
      }
    }
    else if (current_page == UI_PAGE_JOYSTICK_CALIBRATION)
    {
      JoystickCalibrationExtremeSlot_t active_slot;
      JoystickCalibrationAxis_t active_axis;
      JoystickCalibrationExtremeDirection_t active_direction;
      JoystickExtremeCalibration_t *active_extreme_calibration = NULL;
      const bool center_sampling =
          (calibration_view.step ==
           JOYSTICK_CALIBRATION_UI_STEP_CENTER) &&
          (center_calibration.state ==
           JOYSTICK_CENTER_CALIBRATION_SAMPLING);

      if (UiTask_GetExtremeStepConfig(calibration_view.step,
                                      &active_slot,
                                      &active_axis,
                                      &active_direction))
      {
        active_extreme_calibration =
            &extreme_calibrations[(uint8_t)active_slot];
      }

      const bool extreme_sampling =
          (active_extreme_calibration != NULL) &&
          (active_extreme_calibration->state ==
           JOYSTICK_EXTREME_CALIBRATION_SAMPLING);
      const bool sample_due =
          (center_sampling || extreme_sampling) &&
          ((int32_t)(now_tick - next_calibration_sample_tick) >= 0);
      const bool display_due =
          ((int32_t)(now_tick - next_calibration_display_tick) >= 0);

      if (sample_due || display_due)
      {
        JoystickRawValues_t raw_values;
        const bool raw_available = Joystick_GetLatestRaw(&raw_values);

        if (sample_due)
        {
          bool sample_completed = false;

          if (raw_available)
          {
            if (calibration_view.step ==
                JOYSTICK_CALIBRATION_UI_STEP_CENTER)
            {
              sample_completed = JoystickCenterCalibration_Process(
                  &center_calibration,
                  g_joystick_snapshot_count,
                  raw_values.direction_raw,
                  raw_values.throttle_raw);
            }
            else if (active_extreme_calibration != NULL)
            {
              /*
               * 通用极限采样器按当前步骤选择X/Y：raw_x来自DMA[0]，
               * raw_y来自DMA[1]；极大/极小方向由步骤映射统一判断。
               */
              sample_completed = JoystickExtremeCalibration_Process(
                  active_extreme_calibration,
                  g_joystick_snapshot_count,
                  raw_values.direction_raw,
                  raw_values.throttle_raw);
            }
          }

          if (sample_completed)
          {
            const JoystickCalibrationUiView_t previous_calibration_view =
                calibration_view;

            /* DMA[0]/Rank1为X方向，DMA[1]/Rank2为Y油门。 */
            calibration_view.raw_x = raw_values.direction_raw;
            calibration_view.raw_y = raw_values.throttle_raw;
            if (calibration_view.step ==
                JOYSTICK_CALIBRATION_UI_STEP_CENTER)
            {
              calibration_view.state =
                  (center_calibration.state ==
                   JOYSTICK_CENTER_CALIBRATION_SUCCEEDED)
                      ? JOYSTICK_CALIBRATION_UI_SUCCEEDED
                      : JOYSTICK_CALIBRATION_UI_FAILED;
            }
            else if (active_extreme_calibration != NULL)
            {
              calibration_view.state =
                  (active_extreme_calibration->state ==
                   JOYSTICK_EXTREME_CALIBRATION_SUCCEEDED)
                      ? JOYSTICK_CALIBRATION_UI_SUCCEEDED
                      : JOYSTICK_CALIBRATION_UI_FAILED;
            }
            JoystickCalibrationUI_Update(&previous_calibration_view,
                                         &calibration_view);
          }

          UiTask_AdvancePeriodicDeadline(
              &next_calibration_sample_tick,
              pdMS_TO_TICKS(JOYSTICK_CALIBRATION_SAMPLE_PERIOD_MS),
              now_tick);
        }

        if (display_due)
        {
          const JoystickCalibrationUiView_t previous_calibration_view =
              calibration_view;

          if (raw_available)
          {
            calibration_view.raw_x = raw_values.direction_raw;
            calibration_view.raw_y = raw_values.throttle_raw;
          }
          JoystickCalibrationUI_Update(&previous_calibration_view,
                                       &calibration_view);
          UiTask_AdvancePeriodicDeadline(
              &next_calibration_display_tick,
              pdMS_TO_TICKS(JOYSTICK_CALIBRATION_DISPLAY_PERIOD_MS),
              now_tick);
        }
      }
    }

    now_tick = osKernelGetTickCount();
    if (current_page == UI_PAGE_MAIN)
    {
      queue_wait_ticks =
          UiTask_TicksUntil(now_tick, next_main_refresh_tick);
    }
    else if (current_page == UI_PAGE_THROTTLE)
    {
      const uint32_t bar_wait =
          UiTask_TicksUntil(now_tick, next_throttle_bar_refresh_tick);
      const uint32_t value_wait =
          UiTask_TicksUntil(now_tick, next_throttle_value_refresh_tick);

      queue_wait_ticks = (bar_wait < value_wait) ? bar_wait : value_wait;
    }
    else if (current_page == UI_PAGE_JOYSTICK_CALIBRATION)
    {
      JoystickCalibrationExtremeSlot_t active_slot;
      JoystickCalibrationAxis_t active_axis;
      JoystickCalibrationExtremeDirection_t active_direction;
      bool extreme_sampling = false;
      const uint32_t display_wait = UiTask_TicksUntil(
          now_tick, next_calibration_display_tick);

      if (UiTask_GetExtremeStepConfig(calibration_view.step,
                                      &active_slot,
                                      &active_axis,
                                      &active_direction))
      {
        extreme_sampling =
            extreme_calibrations[(uint8_t)active_slot].state ==
            JOYSTICK_EXTREME_CALIBRATION_SAMPLING;
      }

      const bool calibration_sampling =
          ((calibration_view.step ==
            JOYSTICK_CALIBRATION_UI_STEP_CENTER) &&
           (center_calibration.state ==
            JOYSTICK_CENTER_CALIBRATION_SAMPLING)) ||
          extreme_sampling;

      if (calibration_sampling)
      {
        const uint32_t sample_wait = UiTask_TicksUntil(
            now_tick, next_calibration_sample_tick);
        queue_wait_ticks = (sample_wait < display_wait)
                               ? sample_wait
                               : display_wait;
      }
      else
      {
        queue_wait_ticks = display_wait;
      }
    }
    else
    {
      /* 方向/设置等页面没有连续显示项，可等待下一条UI消息。 */
      queue_wait_ticks = osWaitForever;
    }

    /*
     * 即使当前位于没有周期刷新的方向页，也不能永久睡眠而错过配置保存。
     * 将配置截止时间合并为UiTask本次队列等待的最短超时。
     */
    {
      const uint32_t settings_wait_ticks =
          AppSettings_TicksUntilSave(now_tick);
      if (settings_wait_ticks < queue_wait_ticks)
      {
        queue_wait_ticks = settings_wait_ticks;
      }
    }

    queue_status = osMessageQueueGet(
        UiEventQueueHandle,
        &event_message,
        NULL,
        queue_wait_ticks);

    if (queue_status == osErrorTimeout)
    {
      /* 回到循环顶部，由绝对截止时间决定本次应刷新的动态区域。 */
      continue;
    }

    if (queue_status == osOK)
    {
      if (event_message.message_type == UI_EVENT_MESSAGE_INPUT)
      {
        const UiInputEvent_t *const input = &event_message.data.input;

        /*
         * 面板SWITCH是全局油门安全开关，与当前显示页面无关。InputTask已
         * 在长按800 ms成立时完成切换；这里消费通知并刷新显示，不再交给
         * 各页面自己的按键状态机。
         */
        if ((input->key_id == KEY_ID_SWITCH) &&
            (input->action == UI_INPUT_ACTION_LONG_PRESS))
        {
          const MainUiView_t previous_main_view = main_view;
          const ThrottleUiView_t previous_throttle_view = throttle_view;

          /* InputTask已经及时完成状态切换；UiTask只同步显示，不重复切换。 */
          const bool main_changed =
              UiTask_UpdateMainThrottleView(&main_view);
          const bool throttle_changed =
              UiTask_UpdateThrottleView(&throttle_view);

          if (main_changed && (current_page == UI_PAGE_MAIN))
          {
            MainUI_UpdateThrottleStatus(&previous_main_view, &main_view);
          }
          else if (throttle_changed &&
                   (current_page == UI_PAGE_THROTTLE))
          {
            ThrottleUI_Update(&previous_throttle_view, &throttle_view);
          }
          continue;
        }

        if (current_page == UI_PAGE_MAIN)
        {
          if ((input->action == UI_INPUT_ACTION_SHORT_PRESS) &&
              (input->key_id == KEY_ID_CONFIRM))
          {
            switch (main_view.focus)
            {
              case MAIN_UI_FOCUS_DIRECTION:
                /*
                 * 每次进入方向页都恢复到安全的OFF开关层。查询动画和已经
                 * 读取的8路结果由方向业务继续持有，不因页面切换而丢失。
                 */
                UiTask_PrepareDirectionPageEntry(&view);
                current_page = UI_PAGE_MOTOR_DIRECTION;

                refresh_start_tick = osKernelGetTickCount();
                MotorDirectionUI_Draw(&view);
                refresh_duration =
                    osKernelGetTickCount() - refresh_start_tick;
                g_ui_last_refresh_time_ms = refresh_duration;
                if (refresh_duration > g_ui_max_refresh_time_ms)
                {
                  g_ui_max_refresh_time_ms = refresh_duration;
                }
                break;

              case MAIN_UI_FOCUS_THROTTLE:
                /*
                 * 每次从主页面进入时焦点回到ALL，但保留上一次通道掩码、
                 * LIM和Step；实际控制状态从ThrottleControl快照同步。
                 */
                UiTask_PrepareThrottlePageEntry(&throttle_view);
                current_page = UI_PAGE_THROTTLE;

                refresh_start_tick = osKernelGetTickCount();
                ThrottleUI_Draw(&throttle_view);
                refresh_duration =
                    osKernelGetTickCount() - refresh_start_tick;
                g_ui_last_refresh_time_ms = refresh_duration;
                if (refresh_duration > g_ui_max_refresh_time_ms)
                {
                  g_ui_max_refresh_time_ms = refresh_duration;
                }
                /* 整页已包含最新动态值，后续从完整绘制结束时重新计时。 */
                displayed_throttle_percent =
                    throttle_view.throttle_percent;
                displayed_raw_command = throttle_view.raw_command;
                now_tick = osKernelGetTickCount();
                next_throttle_bar_refresh_tick =
                    now_tick +
                    pdMS_TO_TICKS(THROTTLE_UI_BAR_REFRESH_PERIOD_MS);
                next_throttle_value_refresh_tick =
                    now_tick +
                    pdMS_TO_TICKS(THROTTLE_UI_VALUE_REFRESH_PERIOD_MS);
                break;

              case MAIN_UI_FOCUS_SETTINGS:
                /* 每次从主页面进入时，从参考图第一行System开始选择。 */
                settings_view.focus = SETTINGS_UI_FOCUS_SYSTEM;
                settings_view.first_visible_item = 0U;
                current_page = UI_PAGE_SETTINGS;

                refresh_start_tick = osKernelGetTickCount();
                SettingsUI_Draw(&settings_view);
                refresh_duration =
                    osKernelGetTickCount() - refresh_start_tick;
                g_ui_last_refresh_time_ms = refresh_duration;
                if (refresh_duration > g_ui_max_refresh_time_ms)
                {
                  g_ui_max_refresh_time_ms = refresh_duration;
                }
                break;

              case MAIN_UI_FOCUS_STATUS:
                /*
                 * 第4个状态入口临时作为亮度测试入口。亮度值在UiTask整个
                 * 生命周期内保留，离开再进入不会自动恢复到最大值。
                 */
                brightness_return_page = UI_PAGE_MAIN;
                current_page = UI_PAGE_BRIGHTNESS;
                refresh_start_tick = osKernelGetTickCount();
                BrightnessUI_Draw(&brightness_view);
                refresh_duration =
                    osKernelGetTickCount() - refresh_start_tick;
                g_ui_last_refresh_time_ms = refresh_duration;
                if (refresh_duration > g_ui_max_refresh_time_ms)
                {
                  g_ui_max_refresh_time_ms = refresh_duration;
                }
                break;

              default:
                break;
            }
          }
          else
          {
            const MainUiView_t previous_main_view = main_view;

            if (UiTask_HandleMainInput(&main_view, input))
            {
              refresh_start_tick = osKernelGetTickCount();
              MainUI_UpdateFocus(&previous_main_view, &main_view);
              refresh_duration =
                  osKernelGetTickCount() - refresh_start_tick;
              g_ui_last_refresh_time_ms = refresh_duration;
              if (refresh_duration > g_ui_max_refresh_time_ms)
              {
                g_ui_max_refresh_time_ms = refresh_duration;
              }
            }
          }
        }
        else if (current_page == UI_PAGE_MOTOR_DIRECTION)
        {
          const MotorDirectionUiView_t previous_view = view;

          if (UiTask_ShouldLeaveDirectionPage(&view, input))
          {
            /*
             * 返回哪个功能入口，就把主页面焦点恢复到对应入口。目前已经
             * 接通的子页面只有方向页，因此这里明确恢复到“转向”。
             */
            main_view.focus = MAIN_UI_FOCUS_DIRECTION;
            current_page = UI_PAGE_MAIN;
            (void)UiTask_UpdateMainThrottleView(&main_view);

            refresh_start_tick = osKernelGetTickCount();
            MainUI_Draw(&main_view);
            refresh_duration = osKernelGetTickCount() - refresh_start_tick;
            g_ui_last_refresh_time_ms = refresh_duration;
            if (refresh_duration > g_ui_max_refresh_time_ms)
            {
              g_ui_max_refresh_time_ms = refresh_duration;
            }
            next_main_refresh_tick =
                osKernelGetTickCount() +
                pdMS_TO_TICKS(MAIN_UI_REFRESH_PERIOD_MS);
          }
          else if (UiTask_HandleInputEvent(&view,
                                           &direction_control,
                                           &query_control,
                                           input))
          {
            /*
             * 未被页面路由消费的按键继续交给原方向状态机，所以开关、
             * 状态查询、通道选择、NOR/REV选择和发送逻辑均保持原样。
             */
            refresh_start_tick = osKernelGetTickCount();
            MotorDirectionUI_Update(&previous_view, &view);
            refresh_duration = osKernelGetTickCount() - refresh_start_tick;
            g_ui_last_refresh_time_ms = refresh_duration;
            if (refresh_duration > g_ui_max_refresh_time_ms)
            {
              g_ui_max_refresh_time_ms = refresh_duration;
            }
          }
        }
        else if (current_page == UI_PAGE_THROTTLE)
        {
          const ThrottleUiView_t previous_throttle_view = throttle_view;

          if (UiTask_ShouldLeaveThrottlePage(&throttle_view, input))
          {
            /*
             * 长按BACK可能从参数编辑层直接离开页面。若有尚未提交的
             * LIM/STEP变化，此处采用第二种“明确退出时立即保存”方式。
             */
            if (AppSettings_IsSavePending())
            {
              persistent_settings.throttle_limit = throttle_view.limit;
              persistent_settings.throttle_step = throttle_view.step;
              (void)AppSettings_SaveNow(
                  &persistent_settings,
                  osKernelGetTickCount());
            }

            main_view.focus = MAIN_UI_FOCUS_THROTTLE;
            current_page = UI_PAGE_MAIN;
            (void)UiTask_UpdateMainThrottleView(&main_view);

            refresh_start_tick = osKernelGetTickCount();
            MainUI_Draw(&main_view);
            refresh_duration = osKernelGetTickCount() - refresh_start_tick;
            g_ui_last_refresh_time_ms = refresh_duration;
            if (refresh_duration > g_ui_max_refresh_time_ms)
            {
              g_ui_max_refresh_time_ms = refresh_duration;
            }
            next_main_refresh_tick =
                osKernelGetTickCount() +
                pdMS_TO_TICKS(MAIN_UI_REFRESH_PERIOD_MS);
          }
          else if (UiTask_HandleThrottleInput(&throttle_view, input))
          {
            const bool parameter_changed =
                (previous_throttle_view.limit != throttle_view.limit) ||
                (previous_throttle_view.step != throttle_view.step);
            const bool parameter_edit_finished =
                previous_throttle_view.edit_mode &&
                !throttle_view.edit_mode;

            if (parameter_changed)
            {
              /*
               * 每次UP/DOWN都只更新RAM并把截止时间推迟5秒；连续调节
               * 不会产生连续FRAM写入，停止变化后才自动保存最新值。
               */
              persistent_settings.throttle_limit = throttle_view.limit;
              persistent_settings.throttle_step = throttle_view.step;
              AppSettings_RequestDeferredSave(
                  &persistent_settings,
                  osKernelGetTickCount(),
                  pdMS_TO_TICKS(APP_SETTINGS_AUTOSAVE_DELAY_MS));
            }

            if (parameter_edit_finished &&
                AppSettings_IsSavePending())
            {
              /* 短按Confirm或BACK退出LIM/STEP编辑时立即保存。 */
              persistent_settings.throttle_limit = throttle_view.limit;
              persistent_settings.throttle_step = throttle_view.step;
              (void)AppSettings_SaveNow(
                  &persistent_settings,
                  osKernelGetTickCount());
            }

            refresh_start_tick = osKernelGetTickCount();
            ThrottleUI_Update(&previous_throttle_view, &throttle_view);
            refresh_duration = osKernelGetTickCount() - refresh_start_tick;
            g_ui_last_refresh_time_ms = refresh_duration;
            if (refresh_duration > g_ui_max_refresh_time_ms)
            {
              g_ui_max_refresh_time_ms = refresh_duration;
            }
          }
        }
        else if (current_page == UI_PAGE_SETTINGS)
        {
          const SettingsUiView_t previous_settings_view = settings_view;

          if ((input->key_id == KEY_ID_BACK) &&
              ((input->action == UI_INPUT_ACTION_SHORT_PRESS) ||
               (input->action == UI_INPUT_ACTION_LONG_PRESS)))
          {
            /* 设置页没有编辑子层级，短按或长按BACK都返回设置入口。 */
            main_view.focus = MAIN_UI_FOCUS_SETTINGS;
            current_page = UI_PAGE_MAIN;
            (void)UiTask_UpdateMainThrottleView(&main_view);

            refresh_start_tick = osKernelGetTickCount();
            MainUI_Draw(&main_view);
            refresh_duration = osKernelGetTickCount() - refresh_start_tick;
            g_ui_last_refresh_time_ms = refresh_duration;
            if (refresh_duration > g_ui_max_refresh_time_ms)
            {
              g_ui_max_refresh_time_ms = refresh_duration;
            }
            next_main_refresh_tick =
                osKernelGetTickCount() +
                pdMS_TO_TICKS(MAIN_UI_REFRESH_PERIOD_MS);
          }
          else if ((input->action == UI_INPUT_ACTION_SHORT_PRESS) &&
                   (input->key_id == KEY_ID_CONFIRM) &&
                   (settings_view.focus ==
                    SETTINGS_UI_FOCUS_JOYSTICK_CALIBRATION))
          {
            JoystickRawValues_t raw_values;

            /*
             * 每次从设置页进入都创建一轮新的RAM临时会话。默认焦点在OK，
             * 但不会自动开始采样，必须等待用户短按Confirm。
             */
            UiTask_ResetCalibrationSession(&center_calibration,
                                           extreme_calibrations);
            calibration_view.step =
                JOYSTICK_CALIBRATION_UI_STEP_CENTER;
            calibration_view.focus = JOYSTICK_CALIBRATION_UI_FOCUS_OK;
            calibration_view.state = JOYSTICK_CALIBRATION_UI_READY;
            if (Joystick_GetLatestRaw(&raw_values))
            {
              calibration_view.raw_x = raw_values.direction_raw;
              calibration_view.raw_y = raw_values.throttle_raw;
            }

            current_page = UI_PAGE_JOYSTICK_CALIBRATION;
            refresh_start_tick = osKernelGetTickCount();
            JoystickCalibrationUI_Draw(&calibration_view);
            refresh_duration =
                osKernelGetTickCount() - refresh_start_tick;
            g_ui_last_refresh_time_ms = refresh_duration;
            if (refresh_duration > g_ui_max_refresh_time_ms)
            {
              g_ui_max_refresh_time_ms = refresh_duration;
            }

            now_tick = osKernelGetTickCount();
            next_calibration_sample_tick =
                now_tick +
                pdMS_TO_TICKS(JOYSTICK_CALIBRATION_SAMPLE_PERIOD_MS);
            next_calibration_display_tick =
                now_tick +
                pdMS_TO_TICKS(JOYSTICK_CALIBRATION_DISPLAY_PERIOD_MS);
          }
          else if ((input->action == UI_INPUT_ACTION_SHORT_PRESS) &&
                   (input->key_id == KEY_ID_CONFIRM) &&
                   (settings_view.focus == SETTINGS_UI_FOCUS_BRIGHTNESS))
          {
            /*
             * 只接入工程中已经存在的亮度业务。System、Node ID和声音
             * 尚无完整业务，本次Confirm不虚构处理逻辑。
             */
            brightness_return_page = UI_PAGE_SETTINGS;
            current_page = UI_PAGE_BRIGHTNESS;

            refresh_start_tick = osKernelGetTickCount();
            BrightnessUI_Draw(&brightness_view);
            refresh_duration = osKernelGetTickCount() - refresh_start_tick;
            g_ui_last_refresh_time_ms = refresh_duration;
            if (refresh_duration > g_ui_max_refresh_time_ms)
            {
              g_ui_max_refresh_time_ms = refresh_duration;
            }
          }
          else if (UiTask_HandleSettingsInput(&settings_view, input))
          {
            refresh_start_tick = osKernelGetTickCount();
            SettingsUI_UpdateFocus(&previous_settings_view, &settings_view);
            refresh_duration = osKernelGetTickCount() - refresh_start_tick;
            g_ui_last_refresh_time_ms = refresh_duration;
            if (refresh_duration > g_ui_max_refresh_time_ms)
            {
              g_ui_max_refresh_time_ms = refresh_duration;
            }
          }
        }
        else if (current_page == UI_PAGE_JOYSTICK_CALIBRATION)
        {
          const JoystickCalibrationUiView_t previous_calibration_view =
              calibration_view;
          const bool back_requested =
              (input->key_id == KEY_ID_BACK) &&
              ((input->action == UI_INPUT_ACTION_SHORT_PRESS) ||
               (input->action == UI_INPUT_ACTION_LONG_PRESS));
          const bool esc_confirmed =
              (input->key_id == KEY_ID_CONFIRM) &&
              (input->action == UI_INPUT_ACTION_SHORT_PRESS) &&
              (calibration_view.focus ==
               JOYSTICK_CALIBRATION_UI_FOCUS_ESC);
          const bool next_confirmed =
              (input->key_id == KEY_ID_CONFIRM) &&
              (input->action == UI_INPUT_ACTION_SHORT_PRESS) &&
              (calibration_view.focus ==
               JOYSTICK_CALIBRATION_UI_FOCUS_NEXT);

          if (back_requested || esc_confirmed)
          {
            /*
             * ESC/BACK均取消本次临时候选值并回设置页；settings_view没有
             * 被重置，所以焦点仍停在“摇杆校准”。
             */
            UiTask_ResetCalibrationSession(&center_calibration,
                                           extreme_calibrations);
            current_page = UI_PAGE_SETTINGS;
            refresh_start_tick = osKernelGetTickCount();
            SettingsUI_Draw(&settings_view);
            refresh_duration =
                osKernelGetTickCount() - refresh_start_tick;
            g_ui_last_refresh_time_ms = refresh_duration;
            if (refresh_duration > g_ui_max_refresh_time_ms)
            {
              g_ui_max_refresh_time_ms = refresh_duration;
            }
          }
          else if ((input->action == UI_INPUT_ACTION_SHORT_PRESS) &&
                   ((input->key_id == KEY_ID_UP) ||
                    (input->key_id == KEY_ID_DOWN)))
          {
            /*
             * 三个按钮循环选择：
             * DOWN：OK -> NEXT -> ESC -> OK；UP沿相反方向循环。
             */
            if (input->key_id == KEY_ID_DOWN)
            {
              if (calibration_view.focus ==
                  JOYSTICK_CALIBRATION_UI_FOCUS_OK)
              {
                calibration_view.focus =
                    JOYSTICK_CALIBRATION_UI_FOCUS_NEXT;
              }
              else if (calibration_view.focus ==
                       JOYSTICK_CALIBRATION_UI_FOCUS_NEXT)
              {
                calibration_view.focus =
                    JOYSTICK_CALIBRATION_UI_FOCUS_ESC;
              }
              else
              {
                calibration_view.focus =
                    JOYSTICK_CALIBRATION_UI_FOCUS_OK;
              }
            }
            else
            {
              if (calibration_view.focus ==
                  JOYSTICK_CALIBRATION_UI_FOCUS_OK)
              {
                calibration_view.focus =
                    JOYSTICK_CALIBRATION_UI_FOCUS_ESC;
              }
              else if (calibration_view.focus ==
                       JOYSTICK_CALIBRATION_UI_FOCUS_ESC)
              {
                calibration_view.focus =
                    JOYSTICK_CALIBRATION_UI_FOCUS_NEXT;
              }
              else
              {
                calibration_view.focus =
                    JOYSTICK_CALIBRATION_UI_FOCUS_OK;
              }
            }

            if (previous_calibration_view.focus !=
                calibration_view.focus)
            {
              JoystickCalibrationUI_Update(&previous_calibration_view,
                                           &calibration_view);
            }
          }
          else if ((input->action == UI_INPUT_ACTION_SHORT_PRESS) &&
                   (input->key_id == KEY_ID_CONFIRM) &&
                   (calibration_view.focus ==
                    JOYSTICK_CALIBRATION_UI_FOCUS_OK))
          {
            bool sampling_started = false;

            /*
             * 五个步骤都由用户按OK触发。正在采样时Start返回false，
             * 因而忽略重复Confirm；失败或成功后可重新采集100组。
             */
            if (calibration_view.step ==
                JOYSTICK_CALIBRATION_UI_STEP_CENTER)
            {
              sampling_started = JoystickCenterCalibration_Start(
                  &center_calibration,
                  g_joystick_snapshot_count);
            }
            else
            {
              JoystickCalibrationExtremeSlot_t slot;
              JoystickCalibrationAxis_t axis;
              JoystickCalibrationExtremeDirection_t direction;

              if (UiTask_GetExtremeStepConfig(calibration_view.step,
                                              &slot,
                                              &axis,
                                              &direction))
              {
                const uint16_t center_value =
                    (axis == JOYSTICK_CALIBRATION_AXIS_X)
                        ? center_calibration.candidate_x
                        : center_calibration.candidate_y;

                sampling_started = JoystickExtremeCalibration_Start(
                    &extreme_calibrations[(uint8_t)slot],
                    g_joystick_snapshot_count,
                    axis,
                    direction,
                    center_value,
                    center_calibration.candidate_valid);
              }
            }

            if (sampling_started)
            {
              calibration_view.state =
                  JOYSTICK_CALIBRATION_UI_SAMPLING;
              next_calibration_sample_tick =
                  osKernelGetTickCount() +
                  pdMS_TO_TICKS(
                      JOYSTICK_CALIBRATION_SAMPLE_PERIOD_MS);
              JoystickCalibrationUI_Update(&previous_calibration_view,
                                           &calibration_view);
            }
          }
          else if (next_confirmed)
          {
            if (calibration_view.step ==
                JOYSTICK_CALIBRATION_UI_STEP_CENTER)
            {
              if ((center_calibration.state ==
                   JOYSTICK_CENTER_CALIBRATION_SUCCEEDED) &&
                  center_calibration.candidate_valid)
              {
                /*
                 * 第一步候选中心继续保留在RAM，作为四个方向有效行程
                 * 判断基准；进入第二步后仍须由用户按OK才开始采样。
                 */
                JoystickExtremeCalibration_Init(
                    &extreme_calibrations[JOYSTICK_CALIBRATION_SLOT_UP]);
                calibration_view.step =
                    JOYSTICK_CALIBRATION_UI_STEP_UP;
                calibration_view.focus =
                    JOYSTICK_CALIBRATION_UI_FOCUS_OK;
                calibration_view.state =
                    JOYSTICK_CALIBRATION_UI_READY;
                JoystickCalibrationUI_Update(
                    &previous_calibration_view,
                    &calibration_view);
              }
              else
              {
                calibration_view.state =
                    JOYSTICK_CALIBRATION_UI_INCOMPLETE;
                JoystickCalibrationUI_Update(
                    &previous_calibration_view,
                    &calibration_view);
              }
            }
            else
            {
              JoystickCalibrationExtremeSlot_t current_slot;
              JoystickCalibrationAxis_t current_axis;
              JoystickCalibrationExtremeDirection_t current_direction;

              if (UiTask_GetExtremeStepConfig(calibration_view.step,
                                              &current_slot,
                                              &current_axis,
                                              &current_direction) &&
                  (extreme_calibrations[(uint8_t)current_slot].state ==
                   JOYSTICK_EXTREME_CALIBRATION_SUCCEEDED) &&
                  extreme_calibrations[(uint8_t)current_slot]
                      .candidate_valid)
              {
                if (calibration_view.step <
                    JOYSTICK_CALIBRATION_UI_STEP_RIGHT)
                {
                  const JoystickCalibrationUiStep_t next_step =
                      (JoystickCalibrationUiStep_t)
                          ((uint8_t)calibration_view.step + 1U);
                  JoystickCalibrationExtremeSlot_t next_slot;
                  JoystickCalibrationAxis_t next_axis;
                  JoystickCalibrationExtremeDirection_t next_direction;

                  if (UiTask_GetExtremeStepConfig(next_step,
                                                  &next_slot,
                                                  &next_axis,
                                                  &next_direction))
                  {
                    /*
                     * 只初始化即将进入的槽位，已完成步骤的候选极限值
                     * 继续留在RAM；新页面默认聚焦OK且不会自动采样。
                     */
                    JoystickExtremeCalibration_Init(
                        &extreme_calibrations[(uint8_t)next_slot]);
                    calibration_view.step = next_step;
                    calibration_view.focus =
                        JOYSTICK_CALIBRATION_UI_FOCUS_OK;
                    calibration_view.state =
                        JOYSTICK_CALIBRATION_UI_READY;
                    JoystickCalibrationUI_Update(
                        &previous_calibration_view,
                        &calibration_view);
                  }
                }
                else if (UiTask_AreCalibrationCandidatesValid(
                             &center_calibration,
                             extreme_calibrations))
                {
                  /*
                   * CAL:5/5成功后，NEXT才允许进入结果页。结果页直接读取
                   * 五步RAM候选值；此处仍不写FRAM、不替换正式校准参数。
                   */
                  UiTask_PrepareCalibrationResultView(
                      &calibration_result_view,
                      &center_calibration,
                      extreme_calibrations,
                      &persistent_settings);
                  current_page =
                      UI_PAGE_JOYSTICK_CALIBRATION_RESULT;

                  refresh_start_tick = osKernelGetTickCount();
                  JoystickCalibrationResultUI_Draw(
                      &calibration_result_view);
                  refresh_duration =
                      osKernelGetTickCount() - refresh_start_tick;
                  g_ui_last_refresh_time_ms = refresh_duration;
                  if (refresh_duration > g_ui_max_refresh_time_ms)
                  {
                    g_ui_max_refresh_time_ms = refresh_duration;
                  }
                }
              }
              /* 当前步骤未成功时不切页，也不会覆盖已有候选数据。 */
            }
          }
        }
        else if (current_page ==
                 UI_PAGE_JOYSTICK_CALIBRATION_RESULT)
        {
          const JoystickCalibrationResultUiView_t
              previous_calibration_result_view =
                  calibration_result_view;
          const bool back_requested =
              (input->key_id == KEY_ID_BACK) &&
              ((input->action == UI_INPUT_ACTION_SHORT_PRESS) ||
               (input->action == UI_INPUT_ACTION_LONG_PRESS));
          const bool cancel_confirmed =
              (input->key_id == KEY_ID_CONFIRM) &&
              (input->action == UI_INPUT_ACTION_SHORT_PRESS) &&
              (calibration_result_view.focus ==
               JOYSTICK_CALIBRATION_RESULT_FOCUS_CANCEL);

          if (back_requested || cancel_confirmed)
          {
            /*
             * 未保存时CANCEL/BACK只丢弃本轮临时值；已经保存后，正式
             * AppSettings和运行时参数均已更新，所以这里只结束校准会话。
             * settings_view保持不变，返回后焦点仍在“摇杆校准”。
             */
            UiTask_ResetCalibrationSession(&center_calibration,
                                           extreme_calibrations);
            current_page = UI_PAGE_SETTINGS;

            refresh_start_tick = osKernelGetTickCount();
            SettingsUI_Draw(&settings_view);
            refresh_duration =
                osKernelGetTickCount() - refresh_start_tick;
            g_ui_last_refresh_time_ms = refresh_duration;
            if (refresh_duration > g_ui_max_refresh_time_ms)
            {
              g_ui_max_refresh_time_ms = refresh_duration;
            }
          }
          else if ((input->action == UI_INPUT_ACTION_SHORT_PRESS) &&
                   ((input->key_id == KEY_ID_UP) ||
                    (input->key_id == KEY_ID_DOWN)))
          {
            /* 两个按钮构成闭环，UP和DOWN都切换到另一个按钮。 */
            calibration_result_view.focus =
                (calibration_result_view.focus ==
                 JOYSTICK_CALIBRATION_RESULT_FOCUS_SAVE)
                    ? JOYSTICK_CALIBRATION_RESULT_FOCUS_CANCEL
                    : JOYSTICK_CALIBRATION_RESULT_FOCUS_SAVE;
            JoystickCalibrationResultUI_Update(
                &previous_calibration_result_view,
                &calibration_result_view);
          }
          else if ((input->action == UI_INPUT_ACTION_SHORT_PRESS) &&
                   (input->key_id == KEY_ID_CONFIRM) &&
                   (calibration_result_view.focus ==
                    JOYSTICK_CALIBRATION_RESULT_FOCUS_SAVE) &&
                   !calibration_result_view.saved &&
                   UiTask_AreCalibrationCandidatesValid(
                       &center_calibration,
                       extreme_calibrations))
          {
            AppSettings_t candidate_settings = persistent_settings;

            /*
             * X轴对应direction，Y轴对应throttle。只替换六个校准点和
             * 完整校准标志；死区、DMA映射及方向反转标志全部沿用旧值。
             */
            candidate_settings.joystick_calibration.direction.minimum =
                calibration_result_view.x_axis.minimum;
            candidate_settings.joystick_calibration.direction.center =
                calibration_result_view.x_axis.center;
            candidate_settings.joystick_calibration.direction.maximum =
                calibration_result_view.x_axis.maximum;
            candidate_settings.joystick_calibration.throttle.minimum =
                calibration_result_view.y_axis.minimum;
            candidate_settings.joystick_calibration.throttle.center =
                calibration_result_view.y_axis.center;
            candidate_settings.joystick_calibration.throttle.maximum =
                calibration_result_view.y_axis.maximum;
            candidate_settings.joystick_calibration.calibration_valid =
                true;

            /*
             * SaveConfirmed会写非活动A/B槽、提交并回读CRC/字段。只有
             * 整个事务成功后才更新运行RAM；失败则停留本页允许再次SAVE。
             */
            if (AppSettings_SaveConfirmed(&candidate_settings) == HAL_OK)
            {
              if (!Joystick_ApplyCalibrationSettings(
                      &candidate_settings.joystick_calibration))
              {
                /*
                 * AppSettings和Joystick使用相同合法性约束，正常情况下
                 * 不可能到达；若程序约束以后失配，禁止继续错误运行。
                 */
                Error_Handler();
              }
              else
              {
                persistent_settings = candidate_settings;
                calibration_result_view.saved = true;
                JoystickCalibrationResultUI_Update(
                    &previous_calibration_result_view,
                    &calibration_result_view);
              }
            }
          }
        }
        else if (current_page == UI_PAGE_BRIGHTNESS)
        {
          const BrightnessUiView_t previous_brightness_view =
              brightness_view;

          if ((input->key_id == KEY_ID_BACK) &&
              ((input->action == UI_INPUT_ACTION_SHORT_PRESS) ||
               (input->action == UI_INPUT_ACTION_LONG_PRESS)))
          {
            refresh_start_tick = osKernelGetTickCount();
            if (brightness_return_page == UI_PAGE_SETTINGS)
            {
              /* 从设置列表进入亮度页时，BACK回到原来的“亮度”焦点。 */
              current_page = UI_PAGE_SETTINGS;
              SettingsUI_Draw(&settings_view);
            }
            else
            {
              /* 保留原“状态”入口的返回行为，不改变现有测试通路。 */
              main_view.focus = MAIN_UI_FOCUS_STATUS;
              current_page = UI_PAGE_MAIN;
              (void)UiTask_UpdateMainThrottleView(&main_view);
              MainUI_Draw(&main_view);
              next_main_refresh_tick =
                  osKernelGetTickCount() +
                  pdMS_TO_TICKS(MAIN_UI_REFRESH_PERIOD_MS);
            }
            refresh_duration = osKernelGetTickCount() - refresh_start_tick;
            g_ui_last_refresh_time_ms = refresh_duration;
            if (refresh_duration > g_ui_max_refresh_time_ms)
            {
              g_ui_max_refresh_time_ms = refresh_duration;
            }
          }
          else if (UiTask_HandleBrightnessInput(&brightness_view, input))
          {
            refresh_start_tick = osKernelGetTickCount();
            BrightnessUI_Update(&previous_brightness_view,
                                &brightness_view);
            refresh_duration = osKernelGetTickCount() - refresh_start_tick;
            g_ui_last_refresh_time_ms = refresh_duration;
            if (refresh_duration > g_ui_max_refresh_time_ms)
            {
              g_ui_max_refresh_time_ms = refresh_duration;
            }
          }
        }
      }
      else if (event_message.message_type ==
               UI_EVENT_MESSAGE_CAN_COMMAND_RESULT)
      {
        const MotorDirectionUiView_t previous_view = view;
        /**
         * direction_control 这个参数在函数中根据情况在赋值
         */
        if (UiTask_HandleCanCommandResult(
                &view,
                &direction_control,
                &event_message.data.can_command_result) &&
            (current_page == UI_PAGE_MOTOR_DIRECTION))
        {
          MotorDirectionUI_Update(&previous_view, &view);
        }
      }
      else if (event_message.message_type ==
               UI_EVENT_MESSAGE_DIRECTION_QUERY)
      {
        const MotorDirectionUiView_t previous_view = view;

        if (UiTask_HandleDirectionQueryEvent(
                &view,
                &query_control,
                &event_message.data.direction_query) &&
            (current_page == UI_PAGE_MOTOR_DIRECTION))
        {
          MotorDirectionUI_Update(&previous_view, &view);
        }
      }
      else if (event_message.message_type ==
               UI_EVENT_MESSAGE_DIRECTION_AUTO_QUERY_DUE)
      {
        /*
         * 一次性定时器只负责唤醒UiTask。真正的查询仍由UiTask提交给
         * CanTask，保证查询状态和LCD状态只在本任务中串行修改。
         */
        if (direction_control.auto_query_pending)
        {
          const MotorDirectionUiView_t previous_view = view;
          const uint8_t motor_mask =
              direction_control.auto_query_motor_mask;

          direction_control.auto_query_pending = false;
          direction_control.auto_query_motor_mask = 0U;

          if (UiTask_StartDirectionQuery(&view,
                                         &query_control,
                                         motor_mask))
          {
            g_direction_auto_query_started_count++;
            if (current_page == UI_PAGE_MOTOR_DIRECTION)
            {
              MotorDirectionUI_Update(&previous_view, &view);
            }
          }
        }
      }
      else if (event_message.message_type ==
               UI_EVENT_MESSAGE_DIRECTION_QUERY_ANIMATION_TICK)
      {
        if (query_control.active && view.direction_query_in_progress)
        {
          const MotorDirectionUiView_t previous_view = view;

          view.query_animation_dot_count =
              (view.query_animation_dot_count >= 3U)
                  ? 1U
                  : (uint8_t)(view.query_animation_dot_count + 1U);
          if (current_page == UI_PAGE_MOTOR_DIRECTION)
          {
            MotorDirectionUI_Update(&previous_view, &view);
          }
        }
      }
      else
      {
        /* 防御损坏或版本不匹配的队列消息，不访问union中的无效成员。 */
      }
    }
  }
  /* USER CODE END StartUiTask */
}

/* USER CODE BEGIN Header_StartCanTask */
/**
* @brief Function implementing the CanTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanTask */
void StartCanTask(void *argument)
{
  /* USER CODE BEGIN StartCanTask */
  HAL_StatusTypeDef status = HAL_OK;
  CanCommand_t command;
  CanCommandResult_t command_result;
  DroneCANThrottleCommand_t throttle_command = {0};
  ThrottleControlSnapshot_t throttle_snapshot = {
      .unlocked = false,
      .raw_command = 0U,
      .limit = THROTTLE_CONTROL_DEFAULT_LIMIT,
      .motor_mask = THROTTLE_CONTROL_DEFAULT_MOTOR_MASK
  };
  CanThrottlePublishState_t throttle_publish_state =
      CAN_THROTTLE_PUBLISH_LOCKED_SILENT;
  uint8_t zero_flush_remaining = 0U;
  uint32_t next_throttle_publish_tick;
  CanDirectionQueryControl_t query_control = {
      .active = false,
      .awaiting_response = false,
      .last_operation = 0U,
      .motor_mask = 0U,
      .request_token = 0U,
      .request_id = 0U,
      .next_request_id = 1U,
      .response_deadline_tick = 0U,
      .next_poll_tick = 0U,
      .overall_deadline_tick = 0U
  };

  (void)argument;

  status = CAN_Port_Init();

  if(status != HAL_OK) {
      // Handle error
      Error_Handler();

  }

  DroneCAN_Node_Init();
  next_throttle_publish_tick = osKernelGetTickCount();
  g_throttle_publish_state = throttle_publish_state;
  g_throttle_zero_flush_remaining = zero_flush_remaining;


  /* Infinite loop */
  for(;;)
  {
    /*
     * 使用0超时非阻塞读取：队列为空时CanTask仍需继续处理libcanard的
     * 发送队列，不能因为等待新按键命令而停止向CAN硬件邮箱搬运帧。
     */
    if (osMessageQueueGet(CanCommandQueueHandle,    // 从哪个队列取
                          &command,                // 复制到哪个变量
                          NULL,                    // 不需要读取消息优先级
                          0U) == osOK)              // 队列为空时不等待
    {
      if (command.command_type == CAN_COMMAND_START_DIRECTION_QUERY)
      {
        CanTask_StartDirectionQuery(&query_control, &command);
      }
      else
      {
        CanTask_HandleCommand(&command, &command_result);
        CanTask_PostCommandResult(&command_result);
      }
    }

    status = DroneCAN_ProcessRx();
    if (status != HAL_OK)
    {
      g_can_rx_error_count++;
    }

    CanTask_PollDirectionQuery(&query_control);

    /*
     * 油门是连续状态而不是离散命令，因此CanTask直接读取最新快照。
     * 解锁时每10 ms构造一次完整8路RawCommand；关锁后先可靠地发布5次
     * 全零，再停止发布。油门不经过普通命令队列，避免积压过期摇杆值。
     */
    const uint32_t now_tick = osKernelGetTickCount();
    if ((int32_t)(now_tick - next_throttle_publish_tick) >= 0)
    {
      bool should_publish = false;
      bool publish_for_zero_flush = false;

      if (!ThrottleControl_GetSnapshot(&throttle_snapshot))
      {
        /* ADC快照异常按锁定处理，绝不能沿用上一轮非零油门。 */
        throttle_snapshot.unlocked = false;
        throttle_snapshot.raw_command = 0U;
      }

      switch (throttle_publish_state)
      {
        case CAN_THROTTLE_PUBLISH_LOCKED_SILENT:
          if (throttle_snapshot.unlocked)
          {
            /* 解锁后的第一周期就发布；安全解锁条件保证初始值通常为0。 */
            throttle_publish_state =
                CAN_THROTTLE_PUBLISH_UNLOCKED_ACTIVE;
            should_publish = true;
          }
          break;

        case CAN_THROTTLE_PUBLISH_UNLOCKED_ACTIVE:
          if (throttle_snapshot.unlocked)
          {
            should_publish = true;
          }
          else
          {
            /* 关锁不能只停止发送：先用数次全零覆盖接收端的最后非零值。 */
            throttle_publish_state =
                CAN_THROTTLE_PUBLISH_LOCKING_ZERO_FLUSH;
            zero_flush_remaining = THROTTLE_LOCK_ZERO_FLUSH_COUNT;
            should_publish = true;
            publish_for_zero_flush = true;
          }
          break;

        case CAN_THROTTLE_PUBLISH_LOCKING_ZERO_FLUSH:
          if (throttle_snapshot.unlocked)
          {
            /* 防御快速重新解锁：立即恢复正常发布，不继续发送关锁零帧。 */
            throttle_publish_state =
                CAN_THROTTLE_PUBLISH_UNLOCKED_ACTIVE;
            zero_flush_remaining = 0U;
            should_publish = true;
          }
          else
          {
            should_publish = true;
            publish_for_zero_flush = true;
          }
          break;

        default:
          /* 状态损坏时回到最安全的静默锁定状态。 */
          throttle_publish_state = CAN_THROTTLE_PUBLISH_LOCKED_SILENT;
          zero_flush_remaining = 0U;
          break;
      }

      if (should_publish)
      {
        for (uint8_t channel = 0U;
             channel < DRONECAN_ESC_CHANNEL_COUNT;
             ++channel)
        {
          /*
           * RawCommand没有协议级掩码字段，因此始终编码8个数组元素。
           * 关锁清零阶段无条件写8路0；正常解锁阶段只有MASK选中的通道
           * 使用当前油门，未选中通道明确写0。
           */
          throttle_command.motor[channel] = publish_for_zero_flush
              ? 0U
              : (((throttle_snapshot.motor_mask &
                   (uint8_t)(1UL << channel)) != 0U)
                     ? throttle_snapshot.raw_command
                     : 0U);
        }

        g_last_throttle_raw_command = publish_for_zero_flush
                                          ? 0U
                                          : throttle_snapshot.raw_command;
        g_last_throttle_enqueue_result =
            DroneCAN_PublishRawCommand(&throttle_command);
        if (g_last_throttle_enqueue_result > 0)
        {
          ++g_throttle_publish_count;

          /*
           * 只有成功加入libcanard队列才算一次清零发送；失败时保留计数，
           * 下一周期继续重试，不能在零帧实际未提交时提前进入静默。
           */
          if (publish_for_zero_flush && (zero_flush_remaining > 0U))
          {
            --zero_flush_remaining;
            if (zero_flush_remaining == 0U)
            {
              throttle_publish_state =
                  CAN_THROTTLE_PUBLISH_LOCKED_SILENT;
            }
          }
        }
        else
        {
          ++g_throttle_publish_error_count;
        }
      }
      else
      {
        /* 关锁静默不是发送错误；清除最近值，便于Watch窗口明确识别。 */
        g_last_throttle_raw_command = 0U;
        g_last_throttle_enqueue_result = 0;
      }

      g_throttle_publish_state = throttle_publish_state;
      g_throttle_zero_flush_remaining = zero_flush_remaining;

      next_throttle_publish_tick +=
          pdMS_TO_TICKS(THROTTLE_RAW_COMMAND_PUBLISH_PERIOD_MS);

      /* 若任务曾被长时间延迟，跳过旧周期，避免恢复后连续补发过期油门。 */
      if ((int32_t)(now_tick - next_throttle_publish_tick) >= 0)
      {
        next_throttle_publish_tick =
            now_tick +
            pdMS_TO_TICKS(THROTTLE_RAW_COMMAND_PUBLISH_PERIOD_MS);
      }
    }

    status = DroneCAN_ProcessTx();
    
      if (status == HAL_BUSY)
        {
            /*
             * CAN邮箱暂时没有空间，保留libcanard帧，
             * 下一轮继续尝试。
             */
            g_can_tx_busy_count++;
        }
        else if (status != HAL_OK)
        {
            /*
             * 记录错误，后续增加错误恢复。
             */
            g_can_tx_error_count++;
        }

    osDelay(1);
  }
  /* USER CODE END StartCanTask */
}

/* USER CODE BEGIN Header_StartInputTask */
/**
* @brief Function implementing the InputTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartInputTask */
void StartInputTask(void *argument)
{
  /* USER CODE BEGIN StartInputTask */
  KeyEvent_t key_events[KEY_INPUT_MAX_EVENTS_PER_SCAN];
  uint32_t next_wake_tick;

  (void)argument;

  /*
   * 初始化全部5个按键。上电时已经被按住的按键不会立即产生事件，
   * 必须先稳定释放、再重新按下，从而避免启动阶段误发方向命令。
   */
  KeyInput_Init();
  next_wake_tick = osKernelGetTickCount();

  /* Infinite loop */
  for(;;)
  {
    /*
     * ADC+DMA在后台连续采样；这里每10 ms只发布一次最新的两通道快照。
     * 该操作不等待、不使用队列，也不会改变按键状态机的固定扫描周期。
     */
    Joystick_Process();

    const uint8_t event_count = KeyInput_Scan(
        key_events,
        (uint8_t)KEY_INPUT_MAX_EVENTS_PER_SCAN);

    /*
     * KeyInput_Scan同时维护5个按键。InputTask只负责统计事件并把已经
     * 判定完成的短按/长按交给UiTask，不在这里解释菜单含义，也不发送
     * CAN命令。这样按键扫描周期不会被LCD整屏刷新阻塞。
     */
    for (uint8_t index = 0U; index < event_count; ++index)
    {
      const KeyEvent_t *const event = &key_events[index];

      if ((uint32_t)event->key_id >= (uint32_t)KEY_ID_COUNT)
      {
        /* 防御无效按键编号，避免调试计数数组越界。 */
        continue;
      }

      if (event->event_type == KEY_EVENT_SHORT_PRESS)
      {
        g_key_short_press_count[event->key_id]++;
        InputTask_PostUiEvent(event);
        continue;
      }

      if (event->event_type == KEY_EVENT_LONG_PRESS)
      {
        /*
         * 油门SWITCH属于全局实时安全输入，不能等待低优先级UiTask完成
         * 可能耗时的LCD刷新。InputTask在固定10 ms上下文中立即切换状态，
         * 随后的UI事件只用于刷新锁图标，不再执行第二次切换。
         */
        if (event->key_id == KEY_ID_SWITCH)
        {
          const bool was_unlocked = ThrottleControl_IsUnlocked();

          if (was_unlocked)
          {
            ThrottleControl_Lock();
          }
          else
          {
            (void)ThrottleControl_TryUnlock();
          }

          /*
           * 只有锁状态真正发生变化才更新LED并鸣叫。摇杆不在安全位置导致
           * 解锁被拒绝时既不点亮LED，也不产生“成功”提示音。
           */
          const bool is_unlocked = ThrottleControl_IsUnlocked();
          if (is_unlocked != was_unlocked)
          {
            ThrottleLockHardwareFeedback(is_unlocked);
          }
        }

        g_key_long_press_count[event->key_id]++;
        InputTask_PostUiEvent(event);
        continue;
      }

      if (event->event_type != KEY_EVENT_PRESSED)
      {
        /* RELEASED暂时不参与业务，后续UiTask可按需使用。 */
        continue;
      }

      g_key_press_count[event->key_id]++;
    }


    /*
     * 使用绝对周期延时，避免状态机执行时间长期累积到扫描周期中。
     * 当前FreeRTOS Tick为1 kHz，因此10 Tick对应10 ms。
     */
    next_wake_tick += INPUT_KEY_SCAN_PERIOD_TICKS;
    (void)osDelayUntil(next_wake_tick);
  }
  /* USER CODE END StartInputTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief 把CAL:2~5映射为RAM槽位、ADC轴和相对中心的极值方向。
 */
static bool UiTask_GetExtremeStepConfig(
    JoystickCalibrationUiStep_t step,
    JoystickCalibrationExtremeSlot_t *slot,
    JoystickCalibrationAxis_t *axis,
    JoystickCalibrationExtremeDirection_t *direction)
{
  if ((slot == NULL) || (axis == NULL) || (direction == NULL))
  {
    return false;
  }

  switch (step)
  {
    case JOYSTICK_CALIBRATION_UI_STEP_UP:
      *slot = JOYSTICK_CALIBRATION_SLOT_UP;
      *axis = JOYSTICK_CALIBRATION_AXIS_Y;
      *direction = JOYSTICK_CALIBRATION_EXTREME_MAXIMUM;
      return true;

    case JOYSTICK_CALIBRATION_UI_STEP_DOWN:
      *slot = JOYSTICK_CALIBRATION_SLOT_DOWN;
      *axis = JOYSTICK_CALIBRATION_AXIS_Y;
      *direction = JOYSTICK_CALIBRATION_EXTREME_MINIMUM;
      return true;

    case JOYSTICK_CALIBRATION_UI_STEP_LEFT:
      *slot = JOYSTICK_CALIBRATION_SLOT_LEFT;
      *axis = JOYSTICK_CALIBRATION_AXIS_X;
      *direction = JOYSTICK_CALIBRATION_EXTREME_MAXIMUM;
      return true;

    case JOYSTICK_CALIBRATION_UI_STEP_RIGHT:
      *slot = JOYSTICK_CALIBRATION_SLOT_RIGHT;
      *axis = JOYSTICK_CALIBRATION_AXIS_X;
      *direction = JOYSTICK_CALIBRATION_EXTREME_MINIMUM;
      return true;

    case JOYSTICK_CALIBRATION_UI_STEP_CENTER:
    default:
      return false;
  }
}

/**
 * @brief 最终确认前再次校验五个步骤及双轴MIN<CEN<MAX关系。
 *
 * 单步采样器已经做过稳定性、ADC范围和最小有效行程检查；这里仍进行一次
 * 会话级交叉校验，防止页面状态或RAM内容异常时把不完整结果写入FRAM。
 */
static bool UiTask_AreCalibrationCandidatesValid(
    const JoystickCenterCalibration_t *center,
    const JoystickExtremeCalibration_t
        extremes[JOYSTICK_CALIBRATION_SLOT_COUNT])
{
  uint16_t x_minimum;
  uint16_t x_maximum;
  uint16_t y_minimum;
  uint16_t y_maximum;

  if ((center == NULL) || (extremes == NULL) ||
      (center->state != JOYSTICK_CENTER_CALIBRATION_SUCCEEDED) ||
      !center->candidate_valid)
  {
    return false;
  }

  for (uint8_t slot_index = 0U;
       slot_index < (uint8_t)JOYSTICK_CALIBRATION_SLOT_COUNT;
       ++slot_index)
  {
    if ((extremes[slot_index].state !=
         JOYSTICK_EXTREME_CALIBRATION_SUCCEEDED) ||
        !extremes[slot_index].candidate_valid)
    {
      return false;
    }
  }

  /* 物理右/下是ADC较小端，物理左/上是ADC较大端。 */
  x_minimum =
      extremes[JOYSTICK_CALIBRATION_SLOT_RIGHT].candidate_value;
  x_maximum =
      extremes[JOYSTICK_CALIBRATION_SLOT_LEFT].candidate_value;
  y_minimum =
      extremes[JOYSTICK_CALIBRATION_SLOT_DOWN].candidate_value;
  y_maximum =
      extremes[JOYSTICK_CALIBRATION_SLOT_UP].candidate_value;

  return (x_minimum < center->candidate_x) &&
         (center->candidate_x < x_maximum) &&
         (y_minimum < center->candidate_y) &&
         (center->candidate_y < y_maximum) &&
         (x_maximum <= JOYSTICK_CENTER_ADC_MAXIMUM) &&
         (y_maximum <= JOYSTICK_CENTER_ADC_MAXIMUM);
}

/**
 * @brief 将本次五步RAM候选值和当前死区组织成结果页显示模型。
 */
static void UiTask_PrepareCalibrationResultView(
    JoystickCalibrationResultUiView_t *view,
    const JoystickCenterCalibration_t *center,
    const JoystickExtremeCalibration_t
        extremes[JOYSTICK_CALIBRATION_SLOT_COUNT],
    const AppSettings_t *settings)
{
  if ((view == NULL) || (center == NULL) || (extremes == NULL) ||
      (settings == NULL))
  {
    return;
  }

  view->x_axis.minimum =
      extremes[JOYSTICK_CALIBRATION_SLOT_RIGHT].candidate_value;
  view->x_axis.center = center->candidate_x;
  view->x_axis.maximum =
      extremes[JOYSTICK_CALIBRATION_SLOT_LEFT].candidate_value;
  view->x_axis.dead_zone =
      settings->joystick_calibration.direction.dead_zone;

  view->y_axis.minimum =
      extremes[JOYSTICK_CALIBRATION_SLOT_DOWN].candidate_value;
  view->y_axis.center = center->candidate_y;
  view->y_axis.maximum =
      extremes[JOYSTICK_CALIBRATION_SLOT_UP].candidate_value;
  view->y_axis.dead_zone =
      settings->joystick_calibration.throttle.dead_zone;

  view->focus = JOYSTICK_CALIBRATION_RESULT_FOCUS_SAVE;
  view->saved = false;
}

/** @brief 清除本轮五步RAM候选值，不影响正式运行参数或FRAM。 */
static void UiTask_ResetCalibrationSession(
    JoystickCenterCalibration_t *center,
    JoystickExtremeCalibration_t
        extremes[JOYSTICK_CALIBRATION_SLOT_COUNT])
{
  if ((center == NULL) || (extremes == NULL))
  {
    return;
  }

  JoystickCenterCalibration_Init(center);
  for (uint8_t slot_index = 0U;
       slot_index < (uint8_t)JOYSTICK_CALIBRATION_SLOT_COUNT;
       ++slot_index)
  {
    JoystickExtremeCalibration_Init(&extremes[slot_index]);
  }
}

/**
 * @brief 计算距离绝对截止时间还剩多少RTOS Tick。
 *
 * 使用有符号差值可正确处理32位Tick自然回绕；已经到期时返回0，使UiTask
 * 不阻塞并立即回到循环顶部执行刷新。
 */
static uint32_t UiTask_TicksUntil(uint32_t now_tick,
                                  uint32_t deadline_tick)
{
  const int32_t remaining = (int32_t)(deadline_tick - now_tick);

  return (remaining > 0) ? (uint32_t)remaining : 0U;
}

/**
 * @brief 把周期截止时间推进到未来，且不补画已经错过的历史帧。
 *
 * LCD刷新若偶尔超过一个周期，只显示最新油门快照；连续补画旧值既增加
 * 延迟，也会占用本应留给CAN和输入任务的CPU时间。
 */
static void UiTask_AdvancePeriodicDeadline(uint32_t *deadline_tick,
                                           uint32_t period_ticks,
                                           uint32_t now_tick)
{
  if ((deadline_tick == NULL) || (period_ticks == 0U))
  {
    return;
  }

  *deadline_tick += period_ticks;
  if ((int32_t)(now_tick - *deadline_tick) >= 0)
  {
    *deadline_tick = now_tick + period_ticks;
  }
}

/**
 * @brief 将油门控制层快照同步到油门页面，不改变本页焦点和编辑状态。
 * @return true表示锁、掩码、LIM或实际油门发生变化，需要局部刷新。
 */
static bool UiTask_UpdateThrottleView(ThrottleUiView_t *view)
{
  ThrottleControlSnapshot_t snapshot = {
      .unlocked = false,
      .raw_command = 0U,
      .limit = THROTTLE_CONTROL_DEFAULT_LIMIT,
      .motor_mask = THROTTLE_CONTROL_DEFAULT_MOTOR_MASK
  };
  uint8_t throttle_percent = 0U;
  bool changed;

  if (view == NULL)
  {
    return false;
  }

  (void)ThrottleControl_GetSnapshot(&snapshot);
  if (snapshot.limit > 0U)
  {
    /* 进度条显示当前LIM内的摇杆百分比，旁边数字显示精确RawCommand。 */
    throttle_percent =
        (uint8_t)((((uint32_t)snapshot.raw_command * 100U) +
                   (snapshot.limit / 2U)) /
                  snapshot.limit);
    if (throttle_percent > 100U)
    {
      throttle_percent = 100U;
    }
  }

  changed = (view->throttle_unlocked != snapshot.unlocked) ||
            (view->motor_mask != snapshot.motor_mask) ||
            (view->limit != snapshot.limit) ||
            (view->raw_command != snapshot.raw_command) ||
            (view->throttle_percent != throttle_percent);

  view->throttle_unlocked = snapshot.unlocked;
  view->motor_mask = snapshot.motor_mask;
  view->limit = snapshot.limit;
  view->raw_command = snapshot.raw_command;
  view->throttle_percent = throttle_percent;
  return changed;
}

/**
 * @brief 把全局油门控制快照同步到主页面显示模型。
 * @return true表示锁状态或0~100油门值发生变化，需要局部刷新主页面。
 *
 * 主页面显示的是实际准备发送的RawCommand，而不是未经锁定判断的摇杆值；
 * 因此锁定状态下即使拨动摇杆，底部THR仍保持0。
 */
static bool UiTask_UpdateMainThrottleView(MainUiView_t *view)
{
  ThrottleControlSnapshot_t snapshot = {
      .unlocked = false,
      .raw_command = 0U,
      .limit = THROTTLE_CONTROL_DEFAULT_LIMIT,
      .motor_mask = THROTTLE_CONTROL_DEFAULT_MOTOR_MASK
  };
  uint8_t throttle_percent = 0U;
  bool changed;

  if (view == NULL)
  {
    return false;
  }

  /* 摇杆快照无效时GetSnapshot会保留零输出，锁图标仍反映控制层状态。 */
  (void)ThrottleControl_GetSnapshot(&snapshot);

  if (snapshot.limit > 0U)
  {
    throttle_percent =
        (uint8_t)((((uint32_t)snapshot.raw_command * 100U) +
                   (snapshot.limit / 2U)) /
                  snapshot.limit);
    if (throttle_percent > 100U)
    {
      throttle_percent = 100U;
    }
  }

  changed = (view->throttle_unlocked != snapshot.unlocked) ||
            (view->throttle_percent != throttle_percent);
  view->throttle_unlocked = snapshot.unlocked;
  view->throttle_percent = throttle_percent;
  return changed;
}

/**
 * @brief 把按键层事件转换为UI输入事件并 非阻塞地放入UiEventQueue。
 *
 * PRESSED和RELEASED是物理边沿，本页面不使用；调用方只会传入SHORT_PRESS
 * 或LONG_PRESS。0超时表示队列满时立即返回，不能阻塞固定周期的InputTask。
 */
static void InputTask_PostUiEvent(const KeyEvent_t *key_event)
{
  UiEventMessage_t event_message;
  UiInputEvent_t *ui_event;

  if (key_event == NULL)
  {
    return;
  }

  event_message.message_type = UI_EVENT_MESSAGE_INPUT;

  /**
   * 这里就是把event_message.data.input的地址给到了ui_event，ui_event它后面的赋值就是给到了event_message.data.input
   */
  ui_event = &event_message.data.input;
  ui_event->key_id = key_event->key_id;
  if (key_event->event_type == KEY_EVENT_SHORT_PRESS)
  {
    ui_event->action = UI_INPUT_ACTION_SHORT_PRESS;
  }
  else if (key_event->event_type == KEY_EVENT_LONG_PRESS)
  {
    ui_event->action = UI_INPUT_ACTION_LONG_PRESS;
  }
  else
  {
    return;
  }

  if (osMessageQueuePut(UiEventQueueHandle,
                        &event_message,
                        0U,
                        0U) == osOK)
  {
    g_ui_event_queued_count++;
  }
  else
  {
    g_ui_event_queue_full_count++;
  }
}

/**
 * @brief 处理主页面的入口焦点选择。
 *
 * 当前主页面只响应UP和DOWN的短按事件。四个入口按
 * “转向→油门→设置→状态”的顺序排列：DOWN向后选择，UP向前选择，
 * 到达两端后首尾循环。其他按键和长按事件不改变任何状态。
 *
 * @param[in,out] view 主页面显示状态。
 * @param[in] event InputTask已经完成消抖和短/长按判定的输入事件。
 * @return true表示焦点发生变化，需要更新屏幕；false表示无需刷新。
 */
static bool UiTask_HandleMainInput(MainUiView_t *view,
                                   const UiInputEvent_t *event)
{
  if ((view == NULL) || (event == NULL) ||
      (event->action != UI_INPUT_ACTION_SHORT_PRESS))
  {
    return false;
  }

  if (event->key_id == KEY_ID_UP)
  {
    switch (view->focus)
    {
      case MAIN_UI_FOCUS_DIRECTION:
        view->focus = MAIN_UI_FOCUS_STATUS;
        break;
      case MAIN_UI_FOCUS_THROTTLE:
        view->focus = MAIN_UI_FOCUS_DIRECTION;
        break;
      case MAIN_UI_FOCUS_SETTINGS:
        view->focus = MAIN_UI_FOCUS_THROTTLE;
        break;
      case MAIN_UI_FOCUS_STATUS:
        view->focus = MAIN_UI_FOCUS_SETTINGS;
        break;
      default:
        view->focus = MAIN_UI_FOCUS_DIRECTION;
        break;
    }

    return true;
  }

  if (event->key_id == KEY_ID_DOWN)
  {
    switch (view->focus)
    {
      case MAIN_UI_FOCUS_DIRECTION:
        view->focus = MAIN_UI_FOCUS_THROTTLE;
        break;
      case MAIN_UI_FOCUS_THROTTLE:
        view->focus = MAIN_UI_FOCUS_SETTINGS;
        break;
      case MAIN_UI_FOCUS_SETTINGS:
        view->focus = MAIN_UI_FOCUS_STATUS;
        break;
      case MAIN_UI_FOCUS_STATUS:
        view->focus = MAIN_UI_FOCUS_DIRECTION;
        break;
      default:
        view->focus = MAIN_UI_FOCUS_DIRECTION;
        break;
    }

    return true;
  }

  return false;
}

/**
 * @brief 处理设置列表五个选项之间的焦点循环。
 *
 * 本函数只维护设置页的显示焦点，不执行任何设置业务。Confirm由UiTask的
 * 页面路由单独处理，因此尚未实现的选项不会因为新增UI而获得虚构功能。
 *
 * @param[in,out] view 设置页面显示状态。
 * @param[in] event InputTask已经完成消抖和短/长按判定的输入事件。
 * @return true表示焦点发生变化，需要局部刷新；false表示无需刷新。
 */
static bool UiTask_HandleSettingsInput(SettingsUiView_t *view,
                                       const UiInputEvent_t *event)
{
  uint8_t focus_index;

  if ((view == NULL) || (event == NULL) ||
      (event->action != UI_INPUT_ACTION_SHORT_PRESS))
  {
    return false;
  }

  if (event->key_id == KEY_ID_UP)
  {
    if (view->focus == SETTINGS_UI_FOCUS_SYSTEM)
    {
      /* 首项继续向上时循环到末项，并显示最后三个条目。 */
      view->focus = SETTINGS_UI_FOCUS_SOUND;
      view->first_visible_item =
          (uint8_t)(SETTINGS_UI_FOCUS_COUNT -
                    SETTINGS_UI_VISIBLE_ITEM_COUNT);
      return true;
    }

    view->focus = (SettingsUiFocus_t)((uint32_t)view->focus - 1U);
    focus_index = (uint8_t)view->focus;

    /*
     * 焦点仍在当前三行内时只移动焦点，不滚动窗口；只有越过最上行
     * 才把窗口向上移动一项。这正是“亮度→Node ID仍保持原三项”的逻辑。
     */
    if (focus_index < view->first_visible_item)
    {
      view->first_visible_item = focus_index;
    }
    return true;
  }

  if (event->key_id == KEY_ID_DOWN)
  {
    if (view->focus == SETTINGS_UI_FOCUS_SOUND)
    {
      /* 末项继续向下时循环回首项，并恢复最前面的三个条目。 */
      view->focus = SETTINGS_UI_FOCUS_SYSTEM;
      view->first_visible_item = 0U;
      return true;
    }

    view->focus = (SettingsUiFocus_t)((uint32_t)view->focus + 1U);
    focus_index = (uint8_t)view->focus;
    if (focus_index >=
        (uint8_t)(view->first_visible_item +
                  SETTINGS_UI_VISIBLE_ITEM_COUNT))
    {
      view->first_visible_item =
          (uint8_t)(focus_index -
                    (SETTINGS_UI_VISIBLE_ITEM_COUNT - 1U));
    }
    return true;
  }

  return false;
}

/**
 * @brief 处理临时亮度页面的UP/DOWN短按并立即写入SH8501A。
 *
 * 页面只修改OLED控制器的全局DBV，不修改任何RGB565显存内容。先确认
 * 0x51指令发送成功，再提交新的页面状态，避免显示值与硬件亮度不一致。
 *
 * @return true表示亮度成功变化，需要局部刷新页面。
 */
static bool UiTask_HandleBrightnessInput(BrightnessUiView_t *view,
                                         const UiInputEvent_t *event)
{
  uint8_t target;

  if ((view == NULL) || (event == NULL) ||
      (event->action != UI_INPUT_ACTION_SHORT_PRESS))
  {
    return false;
  }

  target = view->brightness;
  if (event->key_id == KEY_ID_UP)
  {
    target = (target > (uint8_t)(0xFFU - BRIGHTNESS_UI_ADJUST_STEP))
                 ? 0xFFU
                 : (uint8_t)(target + BRIGHTNESS_UI_ADJUST_STEP);
  }
  else if (event->key_id == KEY_ID_DOWN)
  {
    target = (target < BRIGHTNESS_UI_ADJUST_STEP)
                 ? 0U
                 : (uint8_t)(target - BRIGHTNESS_UI_ADJUST_STEP);
  }
  else
  {
    return false;
  }

  if (target == view->brightness)
  {
    return false;
  }

  if (LCD_SetBrightness(target) != HAL_OK)
  {
    ++g_lcd_brightness_error_count;
    return false;
  }

  view->brightness = target;
  g_lcd_brightness = target;
  return true;
}

/**
 * @brief 准备方向页面每次进入时的安全导航状态。
 *
 * 只复位本页的开关、焦点和临时选择，不清除已经查询到的8路方向结果，
 * 也不打断可能仍在进行的查询/命令控制流程。这样退出再进入时总是从
 * OFF开关开始操作，同时后台DroneCAN状态仍保持连续。
 */
static void UiTask_PrepareDirectionPageEntry(MotorDirectionUiView_t *view)
{
  if (view == NULL)
  {
    return;
  }

  view->power_state = MOTOR_DIRECTION_UI_POWER_OFF;
  view->focus = MOTOR_DIRECTION_UI_FOCUS_SWITCH;
  view->selected_motor = 1U;
  view->selected_direction = MOTOR_DIRECTION_UI_NORMAL;
}

/**
 * @brief 准备每次进入油门页面时的导航状态。
 *
 * 通道掩码、LIM和Step不会因离开页面而丢失；只把焦点恢复到ALL并退出
 * 参数编辑状态。锁、油门值和共享配置从ThrottleControl重新读取。
 */
static void UiTask_PrepareThrottlePageEntry(ThrottleUiView_t *view)
{
  if (view == NULL)
  {
    return;
  }

  view->focus = THROTTLE_UI_FOCUS_ALL;
  view->edit_mode = false;
  if ((view->step != 1U) && (view->step != 10U) &&
      (view->step != 100U) && (view->step != 1000U))
  {
    view->step = 100U;
  }

  (void)UiTask_UpdateThrottleView(view);
}

/**
 * @brief 判断BACK是否应直接把油门页面路由回主页面。
 *
 * 长按BACK在任意层级立即返回；短按BACK只有在顶部导航且未编辑参数时
 * 返回主页面。参数编辑中的短按BACK由页面状态机用于退出编辑，电机层的
 * 短按BACK用于回到ALL。
 */
static bool UiTask_ShouldLeaveThrottlePage(
    const ThrottleUiView_t *view,
    const UiInputEvent_t *event)
{
  if ((view == NULL) || (event == NULL) ||
      (event->key_id != KEY_ID_BACK))
  {
    return false;
  }

  if (event->action == UI_INPUT_ACTION_LONG_PRESS)
  {
    return true;
  }

  return (event->action == UI_INPUT_ACTION_SHORT_PRESS) &&
         !view->edit_mode &&
         (view->focus <= THROTTLE_UI_FOCUS_STEP);
}

/**
 * @brief 推进油门页面焦点、参数编辑和通道掩码状态机。
 *
 * 共享配置仅通过ThrottleControl_SetConfiguration()发布；CanTask在下一次
 * 10 ms油门周期读取快照，把掩码外通道写0。UiTask不直接操作libcanard。
 */
static bool UiTask_HandleThrottleInput(ThrottleUiView_t *view,
                                       const UiInputEvent_t *event)
{
  bool configuration_changed = false;

  if ((view == NULL) || (event == NULL) ||
      ((uint32_t)event->key_id >= (uint32_t)KEY_ID_COUNT))
  {
    return false;
  }

  /* ALL只有长按Confirm才改变全部通道，避免普通确认动作误选8路电机。 */
  if (event->action == UI_INPUT_ACTION_LONG_PRESS)
  {
    if ((event->key_id == KEY_ID_CONFIRM) &&
        (view->focus == THROTTLE_UI_FOCUS_ALL))
    {
      /*
       * 非零油门时允许一键取消全选，但禁止从部分/全不选扩展为8路，
       * 防止新通道在选择瞬间带油启动。
       */
      if ((view->motor_mask != 0xFFU) && (view->raw_command != 0U))
      {
        return false;
      }
      view->motor_mask =
          (view->motor_mask == 0xFFU) ? 0x00U : 0xFFU;
      configuration_changed = true;
    }
    else
    {
      return false;
    }
  }
  else if (event->action != UI_INPUT_ACTION_SHORT_PRESS)
  {
    return false;
  }
  else if (view->edit_mode)
  {
    if (view->focus == THROTTLE_UI_FOCUS_LIMIT)
    {
      if (event->key_id == KEY_ID_UP)
      {
        /* 增大LIM会抬高实际输出，只允许在当前油门为0时执行。 */
        if (view->raw_command != 0U)
        {
          return false;
        }
        const uint32_t increased =
            (uint32_t)view->limit + (uint32_t)view->step;
        view->limit = (increased > THROTTLE_CONTROL_RAW_COMMAND_MAX)
                          ? THROTTLE_CONTROL_RAW_COMMAND_MAX
                          : (uint16_t)increased;
        configuration_changed = true;
      }
      else if (event->key_id == KEY_ID_DOWN)
      {
        view->limit = (view->limit > view->step)
                          ? (uint16_t)(view->limit - view->step)
                          : 0U;
        configuration_changed = true;
      }
      else if ((event->key_id == KEY_ID_BACK) ||
               (event->key_id == KEY_ID_CONFIRM))
      {
        view->edit_mode = false;
        return true;
      }
      else
      {
        return false;
      }
    }
    else if (view->focus == THROTTLE_UI_FOCUS_STEP)
    {
      if (event->key_id == KEY_ID_UP)
      {
        if (view->step < 10U)       { view->step = 10U; }
        else if (view->step < 100U) { view->step = 100U; }
        else                        { view->step = 1000U; }
        return true;
      }
      if (event->key_id == KEY_ID_DOWN)
      {
        if (view->step > 100U)     { view->step = 100U; }
        else if (view->step > 10U) { view->step = 10U; }
        else                       { view->step = 1U; }
        return true;
      }
      if ((event->key_id == KEY_ID_BACK) ||
          (event->key_id == KEY_ID_CONFIRM))
      {
        view->edit_mode = false;
        return true;
      }
      return false;
    }
    else
    {
      /* 防御损坏状态：只有LIM和Step允许进入编辑模式。 */
      view->edit_mode = false;
      return true;
    }
  }
  else if (view->focus <= THROTTLE_UI_FOCUS_STEP)
  {
    if (event->key_id == KEY_ID_UP)
    {
      view->focus = (view->focus == THROTTLE_UI_FOCUS_ALL)
                        ? THROTTLE_UI_FOCUS_STEP
                        : (ThrottleUiFocus_t)(view->focus - 1);
      return true;
    }
    if (event->key_id == KEY_ID_DOWN)
    {
      view->focus = (view->focus == THROTTLE_UI_FOCUS_STEP)
                        ? THROTTLE_UI_FOCUS_ALL
                        : (ThrottleUiFocus_t)(view->focus + 1);
      return true;
    }
    if (event->key_id == KEY_ID_CONFIRM)
    {
      if (view->focus == THROTTLE_UI_FOCUS_ALL)
      {
        /* ALL短按只进入电机层，不改变当前掩码。 */
        view->focus = THROTTLE_UI_FOCUS_MOTOR_1;
      }
      else
      {
        view->edit_mode = true;
      }
      return true;
    }
    return false;
  }
  else
  {
    uint8_t motor_index =
        (uint8_t)(view->focus - THROTTLE_UI_FOCUS_MOTOR_1);

    if (event->key_id == KEY_ID_UP)
    {
      motor_index = (motor_index == 0U) ? 7U : (uint8_t)(motor_index - 1U);
      view->focus =
          (ThrottleUiFocus_t)(THROTTLE_UI_FOCUS_MOTOR_1 + motor_index);
      return true;
    }
    if (event->key_id == KEY_ID_DOWN)
    {
      motor_index = (uint8_t)((motor_index + 1U) % 8U);
      view->focus =
          (ThrottleUiFocus_t)(THROTTLE_UI_FOCUS_MOTOR_1 + motor_index);
      return true;
    }
    if (event->key_id == KEY_ID_CONFIRM)
    {
      const uint8_t motor_bit = (uint8_t)(1UL << motor_index);

      /* 取消通道随时允许；新增通道必须先把实际油门降到0。 */
      if (((view->motor_mask & motor_bit) == 0U) &&
          (view->raw_command != 0U))
      {
        return false;
      }
      view->motor_mask ^= motor_bit;
      configuration_changed = true;
    }
    else if (event->key_id == KEY_ID_BACK)
    {
      view->focus = THROTTLE_UI_FOCUS_ALL;
      return true;
    }
    else
    {
      return false;
    }
  }

  if (configuration_changed)
  {
    ThrottleControl_SetConfiguration(view->motor_mask, view->limit);
    /* 立即重新计算显示值，不必等下一次100 ms周期刷新。 */
    (void)UiTask_UpdateThrottleView(view);
    return true;
  }

  return false;
}

/**
 * @brief 判断一条BACK事件是否应由页面路由直接返回主页面。
 *
 * 长按BACK在方向页任意位置都返回；短按BACK只有在刚进入页面的默认导航
 * 状态（OFF且焦点位于开关）才返回。其他短按BACK返回false，随后继续交给
 * UiTask_HandleInputEvent()执行方向页原有的逐层返回逻辑。
 *
 * @param view 当前方向页面状态。
 * @param event 已经完成消抖和短/长按判定的按键事件。
 * @return true表示页面路由应返回主页面；false表示继续留在方向页面。
 */
static bool UiTask_ShouldLeaveDirectionPage(
    const MotorDirectionUiView_t *view,
    const UiInputEvent_t *event)
{
  if ((view == NULL) || (event == NULL) ||
      (event->key_id != KEY_ID_BACK))
  {
    return false;
  }

  if (event->action == UI_INPUT_ACTION_LONG_PRESS)
  {
    return true;
  }

  return (event->action == UI_INPUT_ACTION_SHORT_PRESS) &&
         (view->power_state == MOTOR_DIRECTION_UI_POWER_OFF) &&
         (view->focus == MOTOR_DIRECTION_UI_FOCUS_SWITCH);
}

/**
 * @brief 根据一条短按/长按事件推进电机方向页面状态机。
 * @param[in,out] view  当前视图状态；发生有效操作时在原结构体上修改。
 * @param[in,out] control 方向命令的等待结果及2秒保护状态。
 * @param[in] event     InputTask通过队列提交的一条完整输入事件。
 * @return true表示界面内容或焦点发生变化，需要重绘；false表示忽略事件。
 *
 * UiTask只向CanCommandQueue提交明确的通道和目标方向，不直接调用DroneCAN
 * 或libcanard。真正的协议封装与发送仍由CanTask独占完成。
 */
static bool UiTask_HandleInputEvent(MotorDirectionUiView_t *view,
                                    UiDirectionCommandControl_t *control,
                                    UiDirectionQueryControl_t *query_control,
                                    const UiInputEvent_t *event)
{
  if ((view == NULL) || (control == NULL) || (query_control == NULL) ||
      (event == NULL) ||
      ((uint32_t)event->key_id >= (uint32_t)KEY_ID_COUNT))
  {
    return false;
  }

  /* 除方向层长按Confirm外，当前页面的业务全部由短按完成。 */
  if (event->action == UI_INPUT_ACTION_LONG_PRESS)
  {
    if ((view->focus == MOTOR_DIRECTION_UI_FOCUS_STATUS_DOTS) &&
        (event->key_id == KEY_ID_CONFIRM))
    {
      /* 刚发送过换向命令的2秒保护期内，不启动会占用DShot线的查询。 */
      if (UiTask_DirectionCommandIsProtected(control))
      {
        return false;
      }
      return UiTask_SubmitDirectionQuery(view, query_control);
    }

    if ((view->focus == MOTOR_DIRECTION_UI_FOCUS_DIRECTION) &&
        (event->key_id == KEY_ID_CONFIRM))
    {
      /*
       * 只有方向选择层的Confirm长按才是危险操作确认手势。提交成功不
       * 立即闪灯鸣叫；必须等待CanTask返回“已加入libcanard队列”。
       */
      g_ui_direction_long_confirm_count++;
      (void)UiTask_SubmitDirectionCommand(view, control);
    }
    return false;
  }

  if (event->action != UI_INPUT_ACTION_SHORT_PRESS)
  {
    return false;
  }

  switch (view->focus)
  {
    case MOTOR_DIRECTION_UI_FOCUS_SWITCH:
      /*
       * OFF顶栏中，UP和DOWN都把焦点切换到三个状态点组成的整体选项。
       * Confirm仍只负责打开方向功能并直接进入1号通道。
       */
      if (((event->key_id == KEY_ID_UP) ||
           (event->key_id == KEY_ID_DOWN)) &&
          (view->power_state == MOTOR_DIRECTION_UI_POWER_OFF))
      {
        view->focus = MOTOR_DIRECTION_UI_FOCUS_STATUS_DOTS;
        return true;
      }
      if (event->key_id == KEY_ID_CONFIRM)
      {
        /* 查询期间接收板占用DShot维护资源，不允许进入方向修改功能。 */
        if (query_control->active)
        {
          break;
        }
        view->power_state = MOTOR_DIRECTION_UI_POWER_ON;
        view->focus = MOTOR_DIRECTION_UI_FOCUS_MOTOR;
        view->selected_motor = 1U;
        view->selected_direction = MOTOR_DIRECTION_UI_NORMAL;
        return true;
      }
      break;

    case MOTOR_DIRECTION_UI_FOCUS_STATUS_DOTS:
      /*
       * 三个点作为一个顶栏焦点项。UP和DOWN均返回开关；短按Confirm和
       * Back不响应，长按Confirm已在上方长按分支中启动8路方向查询。
       *
       * view->power_state == MOTOR_DIRECTION_UI_POWER_OFF这个条件不添加也是一样，因为当焦点指示器在状态点上时，power_state一定是OFF状态，只有在OFF状态下才会进入这个case
       */
      if ((event->key_id == KEY_ID_UP) ||
           (event->key_id == KEY_ID_DOWN))
          // (view->power_state == MOTOR_DIRECTION_UI_POWER_OFF))
      {
        view->focus = MOTOR_DIRECTION_UI_FOCUS_SWITCH;
        return true;
      }
      break;

    case MOTOR_DIRECTION_UI_FOCUS_MOTOR:
      if (event->key_id == KEY_ID_UP)
      {
        /* UP选择前一通道，1的前一项循环到8。 */
        view->selected_motor = (view->selected_motor <= 1U)
                                   ? 8U
                                   : (uint8_t)(view->selected_motor - 1U);
        return true;
      }
      if (event->key_id == KEY_ID_DOWN)
      {
        /* DOWN选择后一通道，8的后一项循环到1。 */
        view->selected_motor = (view->selected_motor >= 8U)
                                   ? 1U
                                   : (uint8_t)(view->selected_motor + 1U);
        return true;
      }
      if (event->key_id == KEY_ID_CONFIRM)
      {
        /* 进入方向层时默认落在NOR，随后UP=NOR、DOWN=REV。 */
        view->focus = MOTOR_DIRECTION_UI_FOCUS_DIRECTION;
        view->selected_direction = MOTOR_DIRECTION_UI_NORMAL;
        return true;
      }
      if (event->key_id == KEY_ID_BACK)
      {
        /* 通道层BACK回到开关，并立即恢复安全的OFF状态。 */
        view->power_state = MOTOR_DIRECTION_UI_POWER_OFF;
        view->focus = MOTOR_DIRECTION_UI_FOCUS_SWITCH;
        view->selected_motor = 1U;
        view->selected_direction = MOTOR_DIRECTION_UI_NORMAL;
        return true;
      }
      break;

    case MOTOR_DIRECTION_UI_FOCUS_DIRECTION:
      if (event->key_id == KEY_ID_UP)
      {
        if (view->selected_direction != MOTOR_DIRECTION_UI_NORMAL)
        {
          view->selected_direction = MOTOR_DIRECTION_UI_NORMAL;
          return true;
        }
      }
      else if (event->key_id == KEY_ID_DOWN)
      {
        if (view->selected_direction != MOTOR_DIRECTION_UI_REVERSED)
        {
          view->selected_direction = MOTOR_DIRECTION_UI_REVERSED;
          return true;
        }
      }
      else if (event->key_id == KEY_ID_BACK)
      {
        /* 返回相同通道的编号选择，不改变当前通道。 */
        view->focus = MOTOR_DIRECTION_UI_FOCUS_MOTOR;
        return true;
      }
      else
      {
        /* 方向层短按Confirm及未定义的Switch按键均不响应。 */
      }
      break;

    default:
      /* 防御非法焦点值，恢复到OFF开关状态。 */
      view->power_state = MOTOR_DIRECTION_UI_POWER_OFF;
      view->focus = MOTOR_DIRECTION_UI_FOCUS_SWITCH;
      view->selected_motor = 1U;
      view->selected_direction = MOTOR_DIRECTION_UI_NORMAL;
      return true;
  }

  return false;
}

/**
 * @brief 在三点焦点上确认后提交一次8路方向查询。
 *
 * 查询命令仍通过CanCommandQueue交给CanTask，UiTask不直接访问libcanard。
 * 成功入队后立即清除旧方向颜色并启动1→2→3点动画；队列满则保持原界面。
 */
static bool UiTask_SubmitDirectionQuery(
    MotorDirectionUiView_t *view,
    UiDirectionQueryControl_t *control)
{
  if ((view == NULL) || (control == NULL) || control->active ||
      (view->power_state != MOTOR_DIRECTION_UI_POWER_OFF) ||
      (view->focus != MOTOR_DIRECTION_UI_FOCUS_STATUS_DOTS))
  {
    return false;
  }

  return UiTask_StartDirectionQuery(view,
                                    control,
                                    DIRECTION_QUERY_ALL_MOTORS_MASK);
}

/**
 * @brief 启动一次由motor_mask指定通道的方向查询并打开三点动画。
 *
 * 手动查询传入0xFF；方向修改后的自动查询只传入刚修改通道对应的一位。
 * 本函数不限制当前焦点和页面开关状态，以便用户仍停留在方向选择层时也能
 * 自动验证结果。旧结果仅清除待查通道，其余通道的已验证颜色保持不变。
 */
static bool UiTask_StartDirectionQuery(
    MotorDirectionUiView_t *view,
    UiDirectionQueryControl_t *control,
    uint8_t motor_mask)
{
  CanCommand_t command;

  if ((view == NULL) || (control == NULL) || control->active ||
      (motor_mask == 0U))
  {
    return false;
  }

  command.command_type = CAN_COMMAND_START_DIRECTION_QUERY;
  command.request_token = control->next_token;
  command.motor_mask = motor_mask;
  command.direction = MOTOR_DIRECTION_NORMAL; /* 查询命令不使用该字段。 */

  if (osMessageQueuePut(CanCommandQueueHandle,
                        &command,
                        0U,
                        0U) != osOK)
  {
    g_ui_direction_command_queue_full_count++;
    return false;
  }

  control->active = true;
  control->pending_token = command.request_token;
  control->next_token++;
  if (control->next_token == 0U)
  {
    control->next_token = 1U;
  }

  view->direction_query_in_progress = true;
  view->query_animation_dot_count = 1U;
  view->queried_direction_valid_mask &= (uint8_t)~motor_mask;
  view->queried_direction_reversed_mask &=
      view->queried_direction_valid_mask;

  if (DirectionQueryAnimationTimerHandle != NULL)
  {
    (void)osTimerStop(DirectionQueryAnimationTimerHandle);
    (void)osTimerStart(
        DirectionQueryAnimationTimerHandle,
        pdMS_TO_TICKS(DIRECTION_QUERY_ANIMATION_PERIOD_MS));
  }

  return true;
}

/** @brief 把CanTask查询结果应用到UI状态，并停止查询动画。 */
static bool UiTask_HandleDirectionQueryEvent(
    MotorDirectionUiView_t *view,
    UiDirectionQueryControl_t *control,
    const DirectionQueryEvent_t *event)
{
  if ((view == NULL) || (control == NULL) || (event == NULL) ||
      (!control->active) ||
      (event->request_token != control->pending_token))
  {
    return false;
  }

  g_last_direction_query_status = event->response_status;

  if (event->event_type == DIRECTION_QUERY_EVENT_ACTIVE)
  {
    return false;
  }

  control->active = false;
  control->pending_token = 0U;
  view->direction_query_in_progress = false;
  view->query_animation_dot_count = 3U;

  if (DirectionQueryAnimationTimerHandle != NULL)
  {
    (void)osTimerStop(DirectionQueryAnimationTimerHandle);
  }

  if (event->event_type == DIRECTION_QUERY_EVENT_COMPLETE)
  {
    const uint8_t query_mask = event->query_motor_mask;
    const uint8_t valid_mask = event->valid_mask & query_mask;

    /*
     * 单通道自动查询只替换目标通道，不破坏此前已经验证的其它7路结果；
     * 手动8路查询的query_mask为0xFF，因此仍会整体更新全部通道。
     */
    view->queried_direction_valid_mask =
        (view->queried_direction_valid_mask & (uint8_t)~query_mask) |
        valid_mask;
    view->queried_direction_reversed_mask =
        (view->queried_direction_reversed_mask & (uint8_t)~query_mask) |
        (event->reversed_mask & valid_mask);
    g_direction_query_completed_count++;
  }
  else
  {
    /*
     * 待查通道在启动查询时已经恢复成未知紫色。查询失败时不再清除其它
     * 通道，避免一次单通道验证失败破坏此前有效的8路显示结果。
     */
    g_direction_query_failed_count++;
  }

  return true;
}

/**
 * @brief 查询动画软件定时器回调。
 *
 * 回调只投递一个轻量UI事件，不直接画LCD，保证所有绘图仍由UiTask串行
 * 完成。队列满时丢弃本帧动画不会影响CAN查询状态机。
 */
static void DirectionQueryAnimationTimerCallback(void *argument)
{
  UiEventMessage_t event_message;

  (void)argument;
  event_message.message_type =
      UI_EVENT_MESSAGE_DIRECTION_QUERY_ANIMATION_TICK;

  if (osMessageQueuePut(UiEventQueueHandle,
                        &event_message,
                        0U,
                        0U) != osOK)
  {
    g_direction_query_ui_event_drop_count++;
  }
}

/**
 * @brief 方向命令保护时间结束后的软件定时器回调。
 *
 * 回调运行在FreeRTOS定时器服务任务中，所以只投递一个无数据事件；目标
 * 通道保存在UiTask私有的direction_control中，随后由UiTask安全地读取。
 */
static void DirectionAutoQueryTimerCallback(void *argument)
{
  UiEventMessage_t event_message;

  (void)argument;
  event_message.message_type = UI_EVENT_MESSAGE_DIRECTION_AUTO_QUERY_DUE;

  if (osMessageQueuePut(UiEventQueueHandle,
                        &event_message,
                        0U,
                        0U) != osOK)
  {
    g_direction_auto_query_event_drop_count++;
  }
}

/**
 * @brief 检查方向命令的2秒重复发送保护是否仍然有效。
 *
 * 使用有符号Tick差值判断截止时间，可正确处理32位Tick自然回绕。保护
 * 到期时同时清除可能因结果事件丢失而遗留的waiting_for_result，避免UI
 * 永久锁死；在这2秒内仍不会再次发送，符合接收板执行时序的安全要求。
 * 
 * UiTask_DirectionCommandIsProtected 的true表示 当前禁止再次发送方向修改指令，false 表示可以发送新的方向修改命令
 */
static bool UiTask_DirectionCommandIsProtected(
    UiDirectionCommandControl_t *control)
{
  const uint32_t now_tick = osKernelGetTickCount();

  if (control == NULL)
  {
    return true;
  }

  if (!control->protection_active)
  {
    return false;
  }

  if ((int32_t)(now_tick - control->protection_deadline_tick) < 0)
  {
    return true;
  }

  control->protection_active = false;
  control->waiting_for_result = false;
  control->pending_token = 0U;
  control->pending_motor_mask = 0U;
  return false;
}

/**
 * @brief 把当前UI选择转换为一条明确的方向命令并交给CanTask。
 * @return true表示完整命令已复制进CanCommandQueue；false表示未提交。
 *
 * motor_mask的bit0~bit7分别对应1~8通道。这里发送的是SET_NORMAL或
 * SET_REVERSED目标状态，而不是不确定当前状态的“toggle”。队列使用0超时，
 * UiTask不会因为CanTask异常而阻塞；队列满时本次操作明确失败且不反馈成功。
 */
static bool UiTask_SubmitDirectionCommand(
    const MotorDirectionUiView_t *view,
    UiDirectionCommandControl_t *control)
{
  CanCommand_t command;
  const uint32_t now_tick = osKernelGetTickCount();

  if ((view == NULL) || (control == NULL) ||
      (view->power_state != MOTOR_DIRECTION_UI_POWER_ON) ||
      (view->focus != MOTOR_DIRECTION_UI_FOCUS_DIRECTION) ||
      (view->selected_motor < 1U) || (view->selected_motor > 8U))
  {
    return false;
  }

  if (UiTask_DirectionCommandIsProtected(control))
  {
    g_ui_direction_command_protected_count++;
    return false;
  }

  /**
   * control->next_token 是代表什么？
   * view->selected_motor - 1U 是什么意思
   * 
   * 1UL << (view->selected_motor - 1U) 是代表 1 向左移多少位
   */
  command.command_type = CAN_COMMAND_SET_DIRECTION;
  command.request_token = control->next_token;
  command.motor_mask = (uint8_t)(1UL << (view->selected_motor - 1U));
  command.direction = (view->selected_direction == MOTOR_DIRECTION_UI_REVERSED)
                          ? MOTOR_DIRECTION_REVERSED
                          : MOTOR_DIRECTION_NORMAL;

  if (osMessageQueuePut(CanCommandQueueHandle,
                        &command,
                        0U,
                        0U) != osOK)
  {
    g_ui_direction_command_queue_full_count++;
    return false;
  }

  /*
   * 从命令成功交给CanTask的时刻就开始2秒保护。这样即使结果事件意外
   * 丢失，也不会在接收板可能仍忙碌时立即发送第二条方向命令。
   */
  control->waiting_for_result = true;
  control->protection_active = true;
  control->pending_token = command.request_token;
  control->pending_motor_mask = command.motor_mask;
  control->pending_direction = command.direction;
  control->protection_deadline_tick =
      now_tick + (uint32_t)pdMS_TO_TICKS(
                         DIRECTION_COMMAND_PROTECTION_TIME_MS);

  control->next_token++;
  if (control->next_token == 0U)
  {
    /* 0保留给初始化/无效状态，16位自然回绕后从1继续。 */
    control->next_token = 1U;
  }

  g_ui_direction_command_submitted_count++;
  return true;
}

/**
 * @brief 处理CanTask返回的方向命令本地受理结果。
 *
 * 只有关联号、命令类型、通道掩码和方向全部匹配当前等待项，结果才有效。
 * 成功反馈在此处启动，因此蜂鸣和LED表示“CanTask/libcanard已受理”，而
 * 不是仅仅表示按键长按被识别。拒绝时立即解除保护，允许用户修正后重试。
 */
static bool UiTask_HandleCanCommandResult(
    MotorDirectionUiView_t *view,
    UiDirectionCommandControl_t *control,
    const CanCommandResult_t *result)
{
  if ((view == NULL) || (control == NULL) || (result == NULL))
  {
    return false;
  }

  if ((!control->waiting_for_result) ||
      (result->request_token != control->pending_token) ||
      (result->command_type != CAN_COMMAND_SET_DIRECTION) ||
      (result->motor_mask != control->pending_motor_mask) ||
      (result->direction != control->pending_direction))
  {
    g_ui_direction_command_stale_result_count++;
    return false;
  }

  control->waiting_for_result = false;

  if ((result->status == CAN_COMMAND_RESULT_ACCEPTED) &&
      (result->transport_result > 0))
  {
    const uint32_t now_tick = osKernelGetTickCount();
    const int32_t remaining_ticks =
        (int32_t)(control->protection_deadline_tick - now_tick);
    const uint32_t auto_query_delay_ticks =
        (remaining_ticks > 0) ? (uint32_t)remaining_ticks : 1U;

    /* 保护截止时间保持不变，成功反馈不延长2秒执行窗口。 */
    g_ui_direction_command_accepted_count++;
    /*
     * DirectionCommand没有远端完成应答，不能把目标方向直接当成已验证结果。
     * 清除受影响通道的查询有效位，使其恢复紫色，直到下一次实际查询。
     */
    view->queried_direction_valid_mask &= (uint8_t)~result->motor_mask;
    view->queried_direction_reversed_mask &=
        view->queried_direction_valid_mask;

    /*
     * 接收板完整换向状态机约需1590 ms。这里等到既有2秒保护截止时间，
     * 再自动查询刚修改的一路，保留约410 ms执行余量。一次性定时器不会
     * 阻塞UiTask；到期后顶栏三点沿用现有查询动画。
     */
    control->auto_query_pending = true;
    control->auto_query_motor_mask = result->motor_mask;
    if (DirectionAutoQueryTimerHandle != NULL)
    {
      (void)osTimerStop(DirectionAutoQueryTimerHandle);
      if (osTimerStart(DirectionAutoQueryTimerHandle,
                       auto_query_delay_ticks) == osOK)
      {
        g_direction_auto_query_scheduled_count++;
      }
      else
      {
        control->auto_query_pending = false;
        control->auto_query_motor_mask = 0U;
      }
    }
    DirectionLedFeedback_Start();
    DirectionBuzzFeedback();
    return true;
  }

  /* 未成功加入libcanard队列，不存在接收板忙碌风险，可立即重试。 */
  control->protection_active = false;
  control->pending_token = 0U;
  control->pending_motor_mask = 0U;
  control->auto_query_pending = false;
  control->auto_query_motor_mask = 0U;
  g_ui_direction_command_rejected_count++;
  return false;
}

/**
 * @brief 启动或重新启动方向LED快速闪烁3次的非阻塞反馈。
 *
 * 本函数只立即点亮LED并启动FreeRTOS软件定时器，随后马上返回。UiTask
 * 不会在这里等待，因此LED闪烁期间仍可继续接收和处理按键事件。
 */
static void DirectionLedFeedback_Start(void)
{
  if (DirectionLedTimerHandle == NULL)
  {
    return;
  }

  /*
   * 若上一轮尚未完成，就停止并从第一次重新计数。软件定时器控制API只
   * 向定时器服务任务提交命令，不会像HAL_Delay那样占住当前执行流程。
   * 
   * osTimerIsRunning 检查定时器是否正在运行，如果返回非零值，表示定时器正在运行。
   * osTimerStop 停止定时器的运行，如果定时器正在运行，它会被停止，并且不会再触发回调函数。
   */
  if (osTimerIsRunning(DirectionLedTimerHandle) != 0U)
  {
    (void)osTimerStop(DirectionLedTimerHandle);
  }

  g_direction_led_completed_flashes = 0U;
  g_direction_led_is_on = true;

  /* LED低电平有效：RESET立即点亮，第一次100 ms计时从这里开始。 */
  HAL_GPIO_WritePin(Direction_LED_GPIO_Port,
                    Direction_LED_Pin,
                    GPIO_PIN_RESET);

  if (osTimerStart(
          DirectionLedTimerHandle,
          pdMS_TO_TICKS(DIRECTION_LED_FLASH_HALF_PERIOD_MS)) == osOK)
  {
    g_direction_led_feedback_count++;
  }
  else
  {
    /* 启动失败时恢复熄灭，不能让反馈灯永久保持点亮。 */
    g_direction_led_is_on = false;
    HAL_GPIO_WritePin(Direction_LED_GPIO_Port,
                      Direction_LED_Pin,
                      GPIO_PIN_SET);
  }
}

/**
 * @brief FreeRTOS软件定时器回调，每100 ms切换一次方向LED。
 *
 * 回调运行在定时器服务任务中，必须保持短小，禁止HAL_Delay、LCD刷新、
 * CAN发送等耗时操作。这里只有一次GPIO写入、一次计数和必要的停止命令。
 */
static void DirectionLedTimerCallback(void *argument)
{
  (void)argument;

  if (g_direction_led_is_on)
  {
    /* 一次点亮阶段结束：拉高GPIO熄灭，并计为完成一次闪烁。 */
    HAL_GPIO_WritePin(Direction_LED_GPIO_Port,
                      Direction_LED_Pin,
                      GPIO_PIN_SET);
    g_direction_led_is_on = false;
    g_direction_led_completed_flashes++;

    if (g_direction_led_completed_flashes >= DIRECTION_LED_FLASH_COUNT)
    {
      /* 第3次熄灭后停止周期定时器，最终状态明确保持为灭。 */
      (void)osTimerStop(DirectionLedTimerHandle);
    }
  }
  else
  {
    /* 经过100 ms灭灯间隔后，再开始下一次点亮。 */
    HAL_GPIO_WritePin(Direction_LED_GPIO_Port,
                      Direction_LED_Pin,
                      GPIO_PIN_RESET);
    g_direction_led_is_on = true;
  }
}


/**
 * @brief 启动方向命令受理后的非阻塞蜂鸣反馈。
 *
 * 本函数只打开蜂鸣器并启动一次性软件定时器，随后立即返回。定时器到期
 * 后只调用一次DirectionBuzzTimerCallback关闭蜂鸣器，不会阻塞UiTask。
 */
static void DirectionBuzzFeedback(void)
{
  BuzzerFeedback_Start(DIRECTION_BUZZ_DURATION_MS);
}

/**
 * @brief 非阻塞启动或重启一次蜂鸣提示。
 *
 * 方向确认和油门锁切换共用同一个物理蜂鸣器及一次性软件定时器。如果新
 * 提示到来时蜂鸣器仍在响，就以新时长重新计时，避免两个定时器竞争同一
 * GPIO，也不会使用HAL_Delay阻塞InputTask或UiTask。
 */
static void BuzzerFeedback_Start(uint32_t duration_ms)
{
  if (DirectionBuzzTimerHandle == NULL)
  {
    return;
  }

  if (duration_ms == 0U)
  {
    HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, GPIO_PIN_RESET);
    return;
  }

  //如果上一次定时器还在运行，先停止它，避免蜂鸣器长鸣
  if (osTimerIsRunning(DirectionBuzzTimerHandle) != 0U)
  {
    (void)osTimerStop(DirectionBuzzTimerHandle);
  }

  HAL_GPIO_WritePin(Buzz_GPIO_Port,
                      Buzz_Pin,
                      GPIO_PIN_SET);

  /* 启动一次性定时器，达到调用方指定时间后自动熄灭蜂鸣器。 */
  if (osTimerStart(
          DirectionBuzzTimerHandle,
          pdMS_TO_TICKS(duration_ms)) != osOK)
  {
    /* 启动失败时立即熄灭蜂鸣器，避免长鸣。 */
    HAL_GPIO_WritePin(Buzz_GPIO_Port,
                      Buzz_Pin,
                      GPIO_PIN_RESET);
  }

}

/**
 * @brief 同步油门锁的实体LED，并为一次成功切换产生100 ms提示音。
 *
 * 油门LED为低电平有效：解锁写RESET持续点亮，关锁写SET立即熄灭。
 * 本函数由InputTask在锁状态已经成功改变后调用，不负责修改软件锁状态。
 */
static void ThrottleLockHardwareFeedback(bool unlocked)
{
  HAL_GPIO_WritePin(Throttle_LED_GPIO_Port,
                    Throttle_LED_Pin,
                    unlocked ? GPIO_PIN_RESET : GPIO_PIN_SET);
  BuzzerFeedback_Start(THROTTLE_LOCK_BUZZ_DURATION_MS);
}


/**
 * @brief FreeRTOS软件定时器回调，在确认方向指令时提供声音反馈。
 */
static void DirectionBuzzTimerCallback(void *argument)
{
  (void)argument;

  /* 在确认方向指令时提供声音反馈，回调运行在定时器服务任务中。 */
  HAL_GPIO_WritePin(Buzz_GPIO_Port,
                    Buzz_Pin,
                    GPIO_PIN_RESET);

  /* osTimerOnce到期后自动停止，不需要在回调内再次调用osTimerStop。 */
}

static void CanTask_PostDirectionQueryEvent(
    const CanDirectionQueryControl_t *control,
    DirectionQueryEventType_t event_type,
    uint8_t response_status,
    const DroneCANDirectionQueryResponse_t *response)
{
  UiEventMessage_t message = {0};
  DirectionQueryEvent_t *event;

  if (control == NULL)
  {
    return;
  }

  message.message_type = UI_EVENT_MESSAGE_DIRECTION_QUERY;
  event = &message.data.direction_query;
  event->request_token = control->request_token;
  event->request_id = control->request_id;
  event->event_type = event_type;
  event->response_status = response_status;
  event->query_motor_mask = control->motor_mask;

  if (response != NULL)
  {
    event->query_motor_mask = response->query_motor_mask;
    event->valid_mask = response->valid_mask;
    event->reversed_mask = response->reversed_mask;
    event->timeout_mask = response->timeout_mask;
    event->crc_error_mask = response->crc_error_mask;
    event->unsupported_mask = response->unsupported_mask;
    event->protocol_error_mask = response->protocol_error_mask;
    event->maintenance_error = response->maintenance_error;
  }

  /*
   * ACTIVE只是进度提示，可以直接丢弃；COMPLETE/FAILED会停止动画并更新
   * 安全状态，最多等待10 ms给UiTask腾出队列空间，显著降低终态丢失风险。
   */
  if (osMessageQueuePut(
          UiEventQueueHandle,
          &message,
          0U,
          (event_type == DIRECTION_QUERY_EVENT_ACTIVE)
              ? 0U
              : pdMS_TO_TICKS(10U)) != osOK)
  {
    g_direction_query_ui_event_drop_count++;
  }
}

static void CanTask_StartDirectionQuery(
    CanDirectionQueryControl_t *control,
    const CanCommand_t *command)
{
  int16_t enqueue_result;
  const uint32_t now_tick = osKernelGetTickCount();

  if ((control == NULL) || (command == NULL) || control->active ||
      (command->motor_mask == 0U))
  {
    return;
  }

  control->active = true;
  control->awaiting_response = true;
  control->last_operation =
      DRONECAN_DSHOT_DIRECTIONQUERY_REQUEST_OPERATION_START_QUERY;
  control->motor_mask = command->motor_mask;
  control->request_token = command->request_token;
  control->request_id = control->next_request_id;

  enqueue_result = DroneCAN_SendDirectionQuery(
      control->last_operation,
      control->motor_mask,
      control->request_id);
  if (enqueue_result <= 0)
  {
    CanTask_PostDirectionQueryEvent(
        control,
        DIRECTION_QUERY_EVENT_FAILED,
        DIRECTION_QUERY_LOCAL_STATUS_TX_ERROR,
        NULL);
    control->active = false;
    return;
  }

  control->next_request_id++;
  if (control->next_request_id == 0U)
  {
    control->next_request_id = 1U;
  }

  control->response_deadline_tick =
      now_tick + pdMS_TO_TICKS(DIRECTION_QUERY_RESPONSE_TIMEOUT_MS);
  control->overall_deadline_tick =
      now_tick + pdMS_TO_TICKS(DIRECTION_QUERY_OVERALL_TIMEOUT_MS);
  g_direction_query_started_count++;
}

static bool CanTask_DirectionQueryCompleteResponseIsValid(
    const CanDirectionQueryControl_t *control,
    const DroneCANDirectionQueryResponse_t *response)
{
  uint8_t failure_mask;
  uint8_t all_result_mask;

  if ((control == NULL) || (response == NULL) ||
      (response->active_source_node_id != DRONECAN_CONTROLLER_NODE_ID) ||
      (response->active_request_id != control->request_id) ||
      (response->query_motor_mask != control->motor_mask) ||
      (response->maintenance_error != 0U) ||
      ((response->reversed_mask & (uint8_t)~response->valid_mask) != 0U))
  {
    return false;
  }

  failure_mask = response->timeout_mask |
                 response->crc_error_mask |
                 response->unsupported_mask |
                 response->protocol_error_mask;
  all_result_mask = response->valid_mask | failure_mask;

  if (((response->valid_mask & failure_mask) != 0U) ||
      ((all_result_mask & (uint8_t)~control->motor_mask) != 0U) ||
      (all_result_mask != control->motor_mask))
  {
    return false;
  }

  /* 四类最终失败位图按DSDL约束必须两两互斥。 */
  if (((response->timeout_mask & response->crc_error_mask) != 0U) ||
      ((response->timeout_mask & response->unsupported_mask) != 0U) ||
      ((response->timeout_mask & response->protocol_error_mask) != 0U) ||
      ((response->crc_error_mask & response->unsupported_mask) != 0U) ||
      ((response->crc_error_mask & response->protocol_error_mask) != 0U) ||
      ((response->unsupported_mask & response->protocol_error_mask) != 0U))
  {
    return false;
  }

  return true;
}

static void CanTask_PollDirectionQuery(CanDirectionQueryControl_t *control)
{
  DroneCANDirectionQueryResponse_t response;
  const uint32_t now_tick = osKernelGetTickCount();
  int16_t enqueue_result;

  if ((control == NULL) || (!control->active))
  {
    return;
  }

  if (DroneCAN_TakeDirectionQueryResponse(&response))
  {
    /* 保存原始响应位图，便于硬件联调时直接在调试器中观察。 */
    g_last_direction_query_valid_mask = response.valid_mask;
    g_last_direction_query_reversed_mask = response.reversed_mask;
    g_last_direction_query_timeout_mask = response.timeout_mask;
    g_last_direction_query_crc_error_mask = response.crc_error_mask;
    g_last_direction_query_unsupported_mask = response.unsupported_mask;
    g_last_direction_query_protocol_error_mask = response.protocol_error_mask;
    g_last_direction_query_maintenance_error = response.maintenance_error;

    if (response.request_id != control->request_id)
    {
      g_direction_query_stale_response_count++;
    }
    else if (response.protocol_version !=
             DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_PROTOCOL_VERSION)
    {
      CanTask_PostDirectionQueryEvent(
          control,
          DIRECTION_QUERY_EVENT_FAILED,
          DIRECTION_QUERY_LOCAL_STATUS_INVALID_RESPONSE,
          &response);
      control->active = false;
      return;
    }
    else
    {
      g_last_direction_query_status = response.status;

      if (response.status ==
          DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_ACCEPTED)
      {
        control->awaiting_response = false;
        control->next_poll_tick =
            now_tick +
            pdMS_TO_TICKS(DIRECTION_QUERY_INITIAL_RESULT_DELAY_MS);
        CanTask_PostDirectionQueryEvent(
            control, DIRECTION_QUERY_EVENT_ACTIVE, response.status, &response);
      }
      else if (response.status ==
               DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_IN_PROGRESS)
      {
        control->awaiting_response = false;
        control->next_poll_tick =
            now_tick + pdMS_TO_TICKS(DIRECTION_QUERY_POLL_INTERVAL_MS);
        CanTask_PostDirectionQueryEvent(
            control, DIRECTION_QUERY_EVENT_ACTIVE, response.status, &response);
      }
      else if (response.status ==
               DRONECAN_DSHOT_DIRECTIONQUERY_RESPONSE_STATUS_COMPLETE)
      {
        if (CanTask_DirectionQueryCompleteResponseIsValid(control, &response))
        {
          CanTask_PostDirectionQueryEvent(
              control,
              DIRECTION_QUERY_EVENT_COMPLETE,
              response.status,
              &response);
        }
        else
        {
          CanTask_PostDirectionQueryEvent(
              control,
              DIRECTION_QUERY_EVENT_FAILED,
              DIRECTION_QUERY_LOCAL_STATUS_INVALID_RESPONSE,
              &response);
        }
        control->active = false;
        return;
      }
      else
      {
        /* BUSY、NOT_SAFE、INVALID、NOT_FOUND、INTERNAL_ERROR等均终止本次查询。 */
        CanTask_PostDirectionQueryEvent(
            control,
            DIRECTION_QUERY_EVENT_FAILED,
            response.status,
            &response);
        control->active = false;
        return;
      }
    }
  }

  if ((int32_t)(now_tick - control->overall_deadline_tick) >= 0)
  {
    CanTask_PostDirectionQueryEvent(
        control,
        DIRECTION_QUERY_EVENT_FAILED,
        DIRECTION_QUERY_LOCAL_STATUS_TIMEOUT,
        NULL);
    control->active = false;
    return;
  }

  if (control->awaiting_response)
  {
    if ((int32_t)(now_tick - control->response_deadline_tick) < 0)
    {
      return;
    }

    g_direction_query_response_timeout_count++;
    enqueue_result = DroneCAN_SendDirectionQuery(
        control->last_operation,
        (control->last_operation ==
         DRONECAN_DSHOT_DIRECTIONQUERY_REQUEST_OPERATION_START_QUERY)
            ? control->motor_mask
            : 0U,
        control->request_id);
    control->response_deadline_tick =
        now_tick + pdMS_TO_TICKS(DIRECTION_QUERY_RESPONSE_TIMEOUT_MS);
    if ((enqueue_result > 0) &&
        (control->last_operation ==
         DRONECAN_DSHOT_DIRECTIONQUERY_REQUEST_OPERATION_GET_RESULT))
    {
      g_direction_query_poll_count++;
    }
    return;
  }

  if ((int32_t)(now_tick - control->next_poll_tick) >= 0)
  {
    control->last_operation =
        DRONECAN_DSHOT_DIRECTIONQUERY_REQUEST_OPERATION_GET_RESULT;
    enqueue_result = DroneCAN_SendDirectionQuery(
        control->last_operation, 0U, control->request_id);
    if (enqueue_result > 0)
    {
      control->awaiting_response = true;
      control->response_deadline_tick =
          now_tick + pdMS_TO_TICKS(DIRECTION_QUERY_RESPONSE_TIMEOUT_MS);
      g_direction_query_poll_count++;
    }
    else
    {
      /* libcanard暂时无法入队时500 ms后重试，仍受15 s总超时约束。 */
      control->next_poll_tick =
          now_tick + pdMS_TO_TICKS(DIRECTION_QUERY_POLL_INTERVAL_MS);
    }
  }
}


/**
 * @brief 在CanTask上下文中把应用命令转换为DroneCAN方向消息。
 *
 * 只有CanTask调用DroneCAN_SetMotorsNormal/Reversed，确保CanardInstance、
 * Transfer-ID、request_id和libcanard发送队列始终只有一个任务访问，
 * 因而不需要为libcanard再添加互斥锁。
 */
static void CanTask_HandleCommand(const CanCommand_t *command,
                                  CanCommandResult_t *result)
{
  int16_t transport_result;

  if (result == NULL)
  {
    return;
  }

  /*
   * 先构造一份完整的默认拒绝结果，确保任何提前返回路径都能通知UiTask，
   * 不会让UI一直停留在“等待CanTask结果”的状态。
   */
  result->request_token = (command != NULL) ? command->request_token : 0U;
  result->command_type = (command != NULL)
                             ? command->command_type
                             : CAN_COMMAND_SET_DIRECTION;
  result->motor_mask = (command != NULL) ? command->motor_mask : 0U;
  result->direction = (command != NULL)
                          ? command->direction
                          : MOTOR_DIRECTION_NORMAL;
  result->status = CAN_COMMAND_RESULT_REJECTED;
  result->transport_result = 0;

  if ((command == NULL) ||
      (command->command_type != CAN_COMMAND_SET_DIRECTION) ||
      (command->motor_mask == 0U))
  {
    g_can_command_rejected_count++;
    return;
  }

  if (command->direction == MOTOR_DIRECTION_NORMAL)
  {
    transport_result = DroneCAN_SetMotorsNormal(command->motor_mask);
  }
  else if (command->direction == MOTOR_DIRECTION_REVERSED)
  {
    transport_result = DroneCAN_SetMotorsReversed(command->motor_mask);
  }
  else
  {
    g_can_command_rejected_count++;
    return;
  }

  /*
   * DirectionCommand为7字节单帧消息，正常情况下result应为1，表示成功
   * 加入一个libcanard软件队列帧；真正装入CAN邮箱由ProcessTx完成。
   */
  result->transport_result = transport_result;
  g_last_direction_enqueue_result = transport_result;

  if (transport_result > 0)
  {
    result->status = CAN_COMMAND_RESULT_ACCEPTED;
    g_can_command_accepted_count++;
  }
  else
  {
    g_can_command_rejected_count++;
  }
}

/**
 * @brief 把CanTask处理结果非阻塞地交回UiTask。
 *
 * 结果和按键共用UiEventQueue，但通过message_type区分。CanTask不能为了UI
 * 反馈等待队列空间，否则会停止搬运libcanard发送帧；极端队列满时记录错误，
 * UiTask仍会依靠2秒保护超时自动解除等待，不会永久锁死。
 */
static void CanTask_PostCommandResult(const CanCommandResult_t *result)
{
  UiEventMessage_t event_message;

  if (result == NULL)
  {
    return;
  }

  event_message.message_type = UI_EVENT_MESSAGE_CAN_COMMAND_RESULT;
  event_message.data.can_command_result = *result;

  if (osMessageQueuePut(UiEventQueueHandle,
                        &event_message,
                        0U,
                        0U) != osOK)
  {
    g_can_result_event_queue_full_count++;
  }
}

/* USER CODE END Application */

