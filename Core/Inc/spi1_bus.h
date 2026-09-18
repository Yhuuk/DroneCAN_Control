#ifndef SPI1_BUS_H
#define SPI1_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief 独占共享的 SPI1 总线。
 *
 * @retval HAL_OK    已取得总线，调用者可以开始一个完整 SPI 事务。
 * @retval HAL_ERROR FreeRTOS 已运行，但互斥锁尚未创建或获取失败。
 *
 * @note
 * - 在 FreeRTOS 调度器启动前，系统只有 main() 一个执行流，不存在任务
 *   抢占，因此函数直接返回 HAL_OK，不访问尚未创建的互斥锁。
 * - 调度器运行后，本函数会一直等待 SPI1BusMutex，直到取得总线。
 * - 只能在普通任务或启动代码中调用，不能在中断服务函数中调用。
 * - 获取成功后必须调用 SPI1_Bus_Release()，并且不要重复嵌套获取；
 *   CubeMX 创建的是普通（非递归）互斥锁。
 */
HAL_StatusTypeDef SPI1_Bus_Acquire(void);

/**
 * @brief 释放由 SPI1_Bus_Acquire() 取得的 SPI1 总线。
 * @note  调度器启动前调用本函数不会执行任何操作。
 */
void SPI1_Bus_Release(void);

#ifdef __cplusplus
}
#endif

#endif /* SPI1_BUS_H */
