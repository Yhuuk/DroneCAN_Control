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


## `Release` 编译烧录
1. 编译
```bash
cmake --preset Release

cmake --build --preset Release --target DrondCAN_Control
```
2. 烧录
```bash
& "E:/cubemx_keli5/cubemx/openocd-20240916/OpenOCD-20240916-0.12.0/bin/openocd.exe" `
-s "E:/cubemx_keli5/cubemx/openocd-20240916/OpenOCD-20240916-0.12.0/share/openocd/scripts" `
-f "interface/cmsis-dap.cfg" `
-c "transport select swd" `
-f "target/stm32l4x.cfg" `
-c "adapter speed 1000" `
-c "program build/Release/DrondCAN_Control.elf verify reset exit"
```

**说明**: 目前`tasks.json` 中配置的`编译、烧录` 快捷键是 `Debug`，所以如果使用 `Release` 就可以使用上面的指令完成

## 调试
1. 打开命令面板`Ctrl+Shift+P`,执行 `Cortex-Debug: Toggle hex display in Variables window`,就可以在`Cortex Live Watch`窗口中观察到`16进制`显示


## `FRAM` 的使用说明
1. MB85RS16状态寄存器主要包含：

| 位 | 名称 | 含义 |
|---|---|---|
| bit7 | WPEN | 是否启用硬件WP引脚对状态寄存器的写保护 |
| bit6～4 | 固定/保留 | 一般读取为0 |
| bit3 | BP1 | 存储区块保护位 |
| bit2 | BP0 | 存储区块保护位 |
| bit1 | WEL | 写使能锁存位 |
| bit0 | 固定/保留 | 一般读取为0 |

```text
WEL = 0：当前不允许写
WEL = 1：已经执行WREN，允许进行下一次写入
```
执行`WREN`,`WEL`应该变成`1`。完成一次`WRITE`后，`WEL`自动变回`0`。

`BP1/BP0` 用来保护FRAM的部分或全部存储空间
```text
BP1 BP0 = 00：整个FRAM都可以写
BP1 BP0 = 01：顶部一部分受保护
BP1 BP0 = 10：顶部一半受保护
BP1 BP0 = 11：全部受保护
```

2. 指令的功能

| 操作 | 命令 |
|---|---:|
| 写使能WREN | `0x06` |
| 写禁止WRDI | `0x04` |
| 读状态寄存器RDSR | `0x05` |
| 写状态寄存器WRSR | `0x01` |
| 读取数据READ | `0x03` |
| 写入数据WRITE | `0x02` |
| 读取设备ID RDID | `0x9F` |

## 摇杆校准

### 极限值是否收缩
1. 极限值没有向中心收缩，而是采取100组稳定后的ADC原始数据取平均，并且进行了四舍五入的算法
2. 没有固定收缩 20、40、100 ADC。或者没有按 95%、98% 等比例收缩。
3. 实测现在的校准符合实际，校准之后，摇杆推到极限，其油门数值也能拉满
4. 归一化算法位于 [joystick.c (line 122)](E:/cubemx_keli5/cubemx_keil5_Projects/DrondCAN_Control/Core/Src/joystick.c:122)。

### 摇杆校准的采集的数据保存在哪里
1. 本轮校准的临时数据全部是 `StartUiTask()` 的局部 RAM 变量：`JoystickExtremeCalibration_t extreme_calibrations[JOYSTICK_CALIBRATION_SLOT_COUNT]`
2. 五步校准完成后，在`SAVE` 保存数据写入 `FRAM` 之前会进行整体检查。代码在[freertos.c (line 2379)]

### `FRAM` 存储、`RAM` 参数及立即生效机制
1. 完整调用链路
```text
用户短按 Confirm，焦点位于 SAVE
  ↓
确认 saved == false
  ↓
UiTask_AreCalibrationCandidatesValid()
  ↓
将结果页六个值复制到 candidate_settings
  ↓
AppSettings_SaveConfirmed()
  ↓
AppSettings_WriteNewRecord()
  ↓
写入非活动FRAM槽
  ↓
写提交标记
  ↓
FRAM回读 + CRC + 字段一致性验证
  ↓
返回 HAL_OK
  ↓
Joystick_ApplyCalibrationSettings()
  ↓
candidate_settings.joystick_calibration
复制到 g_joystick_calibration
  ↓
下一次 Joystick_Process() 使用新参数
```
所以明确的顺序是 
```text
先确认 FRAM 成功
再更新运行 RAM
最后把 SAVE 显示为绿色
```

### `RAM` 热更新机制
1. `热更新` 指的是 `MCU` 不复位、不重新初始化`ADC`、不重新启动`DMA`，在程序运行过程中，把新校准的配置复制到摇杆算法实际使用的`RAM`变量中
2. 热更新函数和目标是 `Joystick_ApplyCalibrationSettings()` 和 `static JoystickCalibrationSettings_t g_joystick_calibration;`