# 说明

## 向上取整
```
#define KEY_DEBOUNCE_SCAN_COUNT                                      \
    ((KEY_INPUT_DEBOUNCE_TIME_MS + KEY_INPUT_SCAN_PERIOD_MS - 1U) / \
     KEY_INPUT_SCAN_PERIOD_MS)
```
这一段就是 KEY_INPUT_DEBOUNCE_TIME_MS 对 KEY_INPUT_SCAN_PERIOD_MS的向上取整

## FreeRtos
| 类型 | 代表的 RTOS 对象 | 主要用途 |
| --- | --- | --- |
| `osThreadId_t` | Thread / 任务 | 标识一个线程或任务，用于挂起、恢复、修改优先级等操作 |
| `osTimerId_t` | 软件定时器 | 标识一个软件定时器，到设定时间后执行回调函数 |
| `osEventFlagsId_t` | 事件标志组 | 用多个 bit 表示不同事件，可等待一个或多个事件发生 |
| `osMutexId_t` | 互斥锁 | 保护共享资源，防止多个任务同时访问同一个资源 |
| `osSemaphoreId_t` | 信号量 | 用于任务同步、事件通知或资源数量计数 |
| `osMemoryPoolId_t` | 内存池 | 管理固定大小的内存块，适合实时系统中快速申请和释放内存 |
| `osMessageQueueId_t` | 消息队列 | 在任务之间安全传递数据，同时具备缓存和同步作用 |

1. osThreadNew 是创建一个任务
2. 
```
osMessageQueueGet(UiEventQueueHandle,
                          &event,
                          NULL,
                          osWaitForever) == osOK
``` 
和
```
osMessageQueuePut(UiEventQueueHandle,
                        &ui_event,
                        0U,
                        0U) == osOK
```
 就是同一个消息队列中的生产者和消费者。

 3. `osKernelGetTickCount()` 是获取当前 rtos 的系统节拍计数值。它在系统启动后一直递增。
 4. 

 | 成员 | 作用 |
|---|---|
| `waiting_for_result` | 是否已经把命令交给CanTask，并等待CanTask返回本地处理结果 |
| `protection_active` | 是否处于2秒重复命令保护期 |
| `pending_token` | 当前正在等待的命令编号 |
| `pending_motor_mask` | 当前等待命令对应的电机 |
| `pending_direction` | 当前等待命令设置的方向 |
| `next_token` | 下一条新命令应该使用的编号 |
| `protection_deadline_tick` | 保护结束的系统Tick时刻 |