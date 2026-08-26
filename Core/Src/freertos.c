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
#include "app_messages.h"
#include "can_port.h"
#include "dronecan_node.h"
#include "key_input.h"
#include "motor_direction_ui.h"
#include "ui_input_event.h"

#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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

/** 方向反馈LED每100 ms切换一次，形成清晰的快速闪烁。 */
#define DIRECTION_LED_FLASH_HALF_PERIOD_MS 100U

/** 一次反馈固定包含3次完整的“亮→灭”。 */
#define DIRECTION_LED_FLASH_COUNT 3U

/**蜂鸣器快速响100ms */
#define DIRECTION_BUZZ_DURATION_MS 100U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/**
 * 后续UiTask是该队列的生产者，CanTask是消费者。当前UI阶段尚不向该
 * 队列提交命令，但保留已经完成的CAN任务架构供下一阶段接入。
 */
osMessageQueueId_t CanCommandQueueHandle;

/**
 * InputTask是该队列的唯一生产者，UiTask是唯一消费者。队列中保存的是
 * UiInputEvent_t结构体副本，不共享局部变量地址，也不要求两个任务加锁。
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
volatile int16_t g_last_direction_enqueue_result;

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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static void InputTask_PostUiEvent(const KeyEvent_t *key_event);
static bool UiTask_HandleInputEvent(MotorDirectionUiView_t *view,
                                    const UiInputEvent_t *event);
static void DirectionLedFeedback_Start(void);
static void DirectionLedTimerCallback(void *argument);
static void DirectionBuzzTimerCallback(void *argument);
static void DirectionBuzzFeedback(void);
static void CanTask_HandleCommand(const CanCommand_t *command);

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

  /* USER CODE END Init */

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

  /* 方向LED为低电平点亮，空闲状态必须保持高电平熄灭。 */
  HAL_GPIO_WritePin(Direction_LED_GPIO_Port,
                    Direction_LED_Pin,
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
   * 创建InputTask到UiTask的输入队列：
   * - 每条消息是一个UiInputEvent_t结构体副本；
   * - 这里只传递已确认的短按/长按，不传递GPIO电平和消抖内部状态；
   * - UiTask因此可以阻塞等待事件，不需要周期轮询按键。
   */
  UiEventQueueHandle = osMessageQueueNew(
      UI_EVENT_QUEUE_LENGTH,
      sizeof(UiInputEvent_t),
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
  MotorDirectionUiView_t view = {
      .power_state = MOTOR_DIRECTION_UI_POWER_OFF,
      .focus = MOTOR_DIRECTION_UI_FOCUS_SWITCH,
      .selected_motor = 1U,
      .selected_direction = MOTOR_DIRECTION_UI_NORMAL
  };
  UiInputEvent_t event;
  uint32_t refresh_start_tick;
  uint32_t refresh_duration;

  (void)argument;

  /*
   * UiTask是调度器启动后唯一调用显示绘制函数的任务。初始页面为OFF，
   * 焦点包围左上角开关，符合“未确认ON时不能移动通道焦点”的安全规则。
   */
  refresh_start_tick = osKernelGetTickCount();
  MotorDirectionUI_Draw(&view);
  refresh_duration = osKernelGetTickCount() - refresh_start_tick;
  g_ui_last_refresh_time_ms = refresh_duration;
  g_ui_max_refresh_time_ms = refresh_duration;

  /* Infinite loop */
  for(;;)
  {
    /*
     * 没有按键动作时永久阻塞，任务不占用CPU时间。InputTask写入一条
     * UiInputEvent_t后本任务被唤醒；只有状态真的改变才刷新屏幕。
     */
    if (osMessageQueueGet(UiEventQueueHandle,
                          &event,
                          NULL,
                          osWaitForever) == osOK)
    {
      const MotorDirectionUiView_t previous_view = view;

      if (UiTask_HandleInputEvent(&view, &event))
      {
        /*
         * 按键动作后只重画旧焦点、新焦点以及可能变化的开关区域，避免
         * 每次操作都重新发送完整屏幕的57,600字节。
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

  (void)argument;

  status = CAN_Port_Init();

  if(status != HAL_OK) {
      // Handle error
      Error_Handler();

  }

  DroneCAN_Node_Init();


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
      CanTask_HandleCommand(&command);
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
 * @brief 把按键层事件转换为UI输入事件并非阻塞地放入UiEventQueue。
 *
 * PRESSED和RELEASED是物理边沿，本页面不使用；调用方只会传入SHORT_PRESS
 * 或LONG_PRESS。0超时表示队列满时立即返回，不能阻塞固定周期的InputTask。
 */
static void InputTask_PostUiEvent(const KeyEvent_t *key_event)
{
  UiInputEvent_t ui_event;

  if (key_event == NULL)
  {
    return;
  }

  ui_event.key_id = key_event->key_id;
  if (key_event->event_type == KEY_EVENT_SHORT_PRESS)
  {
    ui_event.action = UI_INPUT_ACTION_SHORT_PRESS;
  }
  else if (key_event->event_type == KEY_EVENT_LONG_PRESS)
  {
    ui_event.action = UI_INPUT_ACTION_LONG_PRESS;
  }
  else
  {
    return;
  }

  if (osMessageQueuePut(UiEventQueueHandle,
                        &ui_event,
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
 * @brief 根据一条短按/长按事件推进电机方向页面状态机。
 * @param[in,out] view  当前视图状态；发生有效操作时在原结构体上修改。
 * @param[in] event     InputTask通过队列提交的一条完整输入事件。
 * @return true表示界面内容或焦点发生变化，需要重绘；false表示忽略事件。
 *
 * 当前阶段只完成UI，不调用CanCommandQueue或任何DroneCAN函数。方向层长按
 * Confirm仅累计调试计数，下一阶段再从这里提交明确的电机通道和方向命令。
 */
static bool UiTask_HandleInputEvent(MotorDirectionUiView_t *view,
                                    const UiInputEvent_t *event)
{
  if ((view == NULL) || (event == NULL) ||
      ((uint32_t)event->key_id >= (uint32_t)KEY_ID_COUNT))
  {
    return false;
  }

  /* 除方向层长按Confirm外，当前页面的业务全部由短按完成。 */
  if (event->action == UI_INPUT_ACTION_LONG_PRESS)
  {
    if ((view->focus == MOTOR_DIRECTION_UI_FOCUS_DIRECTION) &&
        (event->key_id == KEY_ID_CONFIRM))
    {
      /* 已识别发送手势；本阶段明确不发送CAN，仅供调试器确认。 */
      g_ui_direction_long_confirm_count++;
      DirectionLedFeedback_Start();
      DirectionBuzzFeedback();
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
      /* OFF时UP/DOWN/BACK均忽略；短按Confirm才进入通道选择。 */
      if (event->key_id == KEY_ID_CONFIRM)
      {
        view->power_state = MOTOR_DIRECTION_UI_POWER_ON;
        view->focus = MOTOR_DIRECTION_UI_FOCUS_MOTOR;
        view->selected_motor = 1U;
        view->selected_direction = MOTOR_DIRECTION_UI_NORMAL;
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
 * 这是方向确认后的声音反馈函数，启动一次性定时器，100 ms后自动熄灭蜂鸣器。
 * 
 * DirectionBuzzFeedback()就是给函数调用的，osTimerStart用来启动一次。
 * DirectionBuzzTimerCallback()是 FreeRtos内部每隔100ms调用，所以100ms以后我就osTimerStop停止这个任务
 */
static void DirectionBuzzFeedback(void)
{
  if (DirectionBuzzTimerHandle == NULL)
  {
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

  /* 启动一次性定时器，100 ms后自动熄灭蜂鸣器。 */
  if (osTimerStart(
          DirectionBuzzTimerHandle,
          pdMS_TO_TICKS(DIRECTION_BUZZ_DURATION_MS)) != osOK)
  {
    /* 启动失败时立即熄灭蜂鸣器，避免长鸣。 */
    HAL_GPIO_WritePin(Buzz_GPIO_Port,
                      Buzz_Pin,
                      GPIO_PIN_RESET);
  }

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

  /***
   * 
   *一次性定时器无需手动停止
   *  100 ms后自动熄灭蜂鸣器。*/
  // (void)osTimerStop(DirectionBuzzTimerHandle);
}


/**
 * @brief 在CanTask上下文中把应用命令转换为DroneCAN方向消息。
 *
 * 只有CanTask调用DroneCAN_SetMotorsNormal/Reversed，确保CanardInstance、
 * Transfer-ID、request_id和libcanard发送队列始终只有一个任务访问，
 * 因而不需要为libcanard再添加互斥锁。
 */
static void CanTask_HandleCommand(const CanCommand_t *command)
{
  int16_t result;

  if ((command == NULL) ||
      (command->command_type != CAN_COMMAND_SET_DIRECTION) ||
      (command->motor_mask == 0U))
  {
    g_can_command_rejected_count++;
    return;
  }

  if (command->direction == MOTOR_DIRECTION_NORMAL)
  {
    result = DroneCAN_SetMotorsNormal(command->motor_mask);
  }
  else if (command->direction == MOTOR_DIRECTION_REVERSED)
  {
    result = DroneCAN_SetMotorsReversed(command->motor_mask);
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
  g_last_direction_enqueue_result = result;

  if (result > 0)
  {
    g_can_command_accepted_count++;
  }
  else
  {
    g_can_command_rejected_count++;
  }
}

/* USER CODE END Application */

