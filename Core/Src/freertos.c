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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/**
 * InputTask是该队列的生产者，CanTask是消费者。队列复制CanCommand_t的
 * 内容，在两个任务之间传递“设置哪些电机、设置成什么方向”的参数。
 */
osMessageQueueId_t CanCommandQueueHandle;

/* 以下变量只用于调试器观察运行情况，不参与控制逻辑。 */
volatile uint32_t g_key_press_count[KEY_ID_COUNT];
volatile uint32_t g_key_short_press_count[KEY_ID_COUNT];
volatile uint32_t g_key_long_press_count[KEY_ID_COUNT];
volatile uint32_t g_input_command_queued_count;
volatile uint32_t g_input_queue_full_count;
volatile uint32_t g_input_direction_conflict_count;
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

static void InputTask_PostDirectionCommand(MotorDirection_t direction);
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
  /* start timers, add new ones, ... */
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
    bool up_pressed = false;
    bool down_pressed = false;
    const uint8_t event_count = KeyInput_Scan(
        key_events,
        (uint8_t)KEY_INPUT_MAX_EVENTS_PER_SCAN);

    /*
     * KeyInput_Scan会同时维护5个按键。当前阶段只有UP和DOWN映射为电机
     * 方向命令；Confirm、Back、Switch的稳定按下事件只累计调试计数，
     * 后续加入OLED菜单时可直接复用，无需重写GPIO消抖层。
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
        continue;
      }

      if (event->event_type == KEY_EVENT_LONG_PRESS)
      {
        g_key_long_press_count[event->key_id]++;
        continue;
      }

      if (event->event_type != KEY_EVENT_PRESSED)
      {
        /* RELEASED暂时不参与业务，后续UiTask可按需使用。 */
        continue;
      }

      g_key_press_count[event->key_id]++;

      if (event->key_id == KEY_ID_UP)
      {
        up_pressed = true;
      }
      else if (event->key_id == KEY_ID_DOWN)
      {
        down_pressed = true;
      }
      else
      {
        /* Confirm、Back、Switch暂不执行方向控制。 */
      }
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
 * @brief 将InputTask识别出的方向意图放入CanTask命令队列。
 *
 * 本函数不调用任何libcanard函数。发送到队列的是结构体副本，所以局部
 * 变量command在函数返回后失效不会影响CanTask读取消息。
 */
// static void InputTask_PostDirectionCommand(MotorDirection_t direction)
// {
//   const CanCommand_t command = {
//       .command_type = CAN_COMMAND_SET_DIRECTION,
//       .motor_mask = 0x01U, /* 当前阶段固定选择第1路电机。 */
//       .direction = direction
//   };

//   /*
//    * 方向按键是离散事件，不应阻塞InputTask等待队列空间。队列满时记录
//    * 错误并丢弃本次请求，避免旧命令积压后在用户未预期的时刻执行。
//    */
//   if (osMessageQueuePut(CanCommandQueueHandle,
//                         &command,
//                         0U,
//                         0U) == osOK)
//   {
//     g_input_command_queued_count++;
//   }
//   else
//   {
//     g_input_queue_full_count++;
//   }
// }

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

