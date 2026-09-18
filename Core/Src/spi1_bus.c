#include "spi1_bus.h"

#include "cmsis_os.h"

/* 该句柄由 CubeMX 在 freertos.c/MX_FREERTOS_Init() 中创建。 */
extern osMutexId_t SPI1BusMutexHandle;

HAL_StatusTypeDef SPI1_Bus_Acquire(void)
{
    /*
     * LCD_Init() 和 FRAM_Init() 当前在 osKernelInitialize() 之前执行。
     * 此时互斥锁对象尚不存在，但也没有任何并发任务，所以不需要加锁。
     */
    if (osKernelGetState() != osKernelRunning)
    {
        return HAL_OK;
    }

    if (SPI1BusMutexHandle == NULL)
    {
        return HAL_ERROR;
    }

    return (osMutexAcquire(SPI1BusMutexHandle, osWaitForever) == osOK)
        ? HAL_OK
        : HAL_ERROR;
}

void SPI1_Bus_Release(void)
{
    if ((osKernelGetState() == osKernelRunning) &&
        (SPI1BusMutexHandle != NULL))
    {
        (void)osMutexRelease(SPI1BusMutexHandle);
    }
}
