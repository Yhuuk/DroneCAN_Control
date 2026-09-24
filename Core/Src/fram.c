#include "fram.h"

#include "spi.h"
#include "spi1_bus.h"

#include <string.h>

#define FRAM_SPI_TIMEOUT_MS       100U

/**
 * FRAM_CMD_WREN 是写使能操作码
 * FRAM_CMD_RDSR 读状态寄存器的指令
 * FRAM_CMD_READ 读寄存器
 * FRAM_CMD_WRITE 写寄存器
 * FRAM_CMD_RDID 是读取设备ID 指令
 * 
 * WEL = 0：当前不允许写
   WEL = 1：已经执行WREN，允许进行下一次写入
 */
#define FRAM_CMD_WREN             0x06U
#define FRAM_CMD_RDSR             0x05U
#define FRAM_CMD_READ             0x03U
#define FRAM_CMD_WRITE            0x02U
#define FRAM_CMD_RDID             0x9FU

/* BP1/BP0不为0时，FRAM的一部分或全部存储区会受到写保护。 
   BP0 是bit2，BP1是bit3
*/
#define FRAM_STATUS_BP_MASK       0x0CU 

volatile uint8_t g_fram_initialized = 0U;
volatile uint8_t g_fram_device_id[FRAM_DEVICE_ID_LENGTH] = {0U};
volatile uint8_t g_fram_status_register = 0U;
volatile uint32_t g_fram_write_count = 0U;
volatile uint32_t g_fram_error_count = 0U;

/**
 * 该ID 是器件类型和厂商识别信息，这个是MB85RS16的
 * 
 * MB85RS256B 的device_id 为04 7F 05 09
 * 所以但我使用MB85RS256B这款的时候，就不能在初始化中以读取到的ID为是为初始化的标准之一
 */
// static const uint8_t g_fram_expected_device_id[FRAM_DEVICE_ID_LENGTH] = {
//     0x04U, 0x7FU, 0x01U, 0x01U
// };

static void FRAM_Select(void)
{
    /*
     * FRAM驱动只管理自己的CS。OLED与FRAM不能同时访问SPI1这一约束，
     * 由所有驱动共同使用的SPI1BusMutex保证，而不是由FRAM操作OLED引脚。
     */
    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);
}

/**
 * FRAM的 CS拉高，放弃SPI
 */
static void FRAM_Deselect(void)
{
    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
}

/**
 * 查看改地址后面的长度是否会超出总长度
 */
static HAL_StatusTypeDef FRAM_CheckRange(uint16_t address, uint16_t length)
{
    if (length == 0U)
    {
        return HAL_OK;
    }

    if ((address >= FRAM_CAPACITY_BYTES) ||
        (length > (uint16_t)(FRAM_CAPACITY_BYTES - address)))
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief 在已经持有SPI1互斥锁时发送写使能命令。
 *
 * WREN本身是一个独立CS事务；CS上升沿后WEL置位。MB85RS16完成下一次
 * WRITE事务后会自动清除WEL，因此不能只在MCU开机时发送一次WREN。
 */
static HAL_StatusTypeDef FRAM_WriteEnableLocked(void)
{
    uint8_t command = FRAM_CMD_WREN;
    HAL_StatusTypeDef status;

    FRAM_Select();
    status = HAL_SPI_Transmit(&hspi1, &command, 1U, FRAM_SPI_TIMEOUT_MS);
    FRAM_Deselect();

    return status;
}

HAL_StatusTypeDef FRAM_ReadDeviceId(uint8_t device_id[FRAM_DEVICE_ID_LENGTH])
{
    uint8_t command = FRAM_CMD_RDID;
    HAL_StatusTypeDef status;

    if (device_id == NULL)
    {
        ++g_fram_error_count;
        return HAL_ERROR;
    }

    if (SPI1_Bus_Acquire() != HAL_OK)
    {
        ++g_fram_error_count;
        return HAL_ERROR;
    }

    FRAM_Select();
    status = HAL_SPI_Transmit(&hspi1, &command, 1U, FRAM_SPI_TIMEOUT_MS);
    if (status == HAL_OK)
    {
        memset(device_id, 0xFF, FRAM_DEVICE_ID_LENGTH);
        status = HAL_SPI_Receive(&hspi1,
                                 device_id,
                                 FRAM_DEVICE_ID_LENGTH,
                                 FRAM_SPI_TIMEOUT_MS);
    }
    FRAM_Deselect();
    SPI1_Bus_Release();

    if (status != HAL_OK)
    {
        ++g_fram_error_count;
    }
    return status;
}

HAL_StatusTypeDef FRAM_ReadStatus(uint8_t *status_register)
{
    uint8_t command = FRAM_CMD_RDSR;
    HAL_StatusTypeDef status;

    /**
     * status_register == NULL 表示的是status_register 是否是一个空指针，而不是说status_register的内容是不是空的
     */
    if (status_register == NULL)
    {
        ++g_fram_error_count;
        return HAL_ERROR;
    }

    if (SPI1_Bus_Acquire() != HAL_OK)
    {
        ++g_fram_error_count;
        return HAL_ERROR;
    }

    FRAM_Select();
    status = HAL_SPI_Transmit(&hspi1, &command, 1U, FRAM_SPI_TIMEOUT_MS);
    if (status == HAL_OK)
    {
        *status_register = 0xFFU;
        status = HAL_SPI_Receive(&hspi1,
                                 status_register,
                                 1U,
                                 FRAM_SPI_TIMEOUT_MS);
    }
    FRAM_Deselect();
    SPI1_Bus_Release();

    if (status != HAL_OK)
    {
        ++g_fram_error_count;
    }
    return status;
}

HAL_StatusTypeDef FRAM_Read(uint16_t address, uint8_t *data, uint16_t length)
{
    uint8_t header[3];
    HAL_StatusTypeDef status;

    if (length == 0U)
    {
        return HAL_OK;
    }
    if ((data == NULL) || (FRAM_CheckRange(address, length) != HAL_OK))
    {
        ++g_fram_error_count;
        return HAL_ERROR;
    }

    header[0] = FRAM_CMD_READ;
    header[1] = (uint8_t)(address >> 8U);
    header[2] = (uint8_t)address;

    if (SPI1_Bus_Acquire() != HAL_OK)
    {
        ++g_fram_error_count;
        return HAL_ERROR;
    }

    FRAM_Select();
    status = HAL_SPI_Transmit(&hspi1,
                              header,
                              (uint16_t)sizeof(header),
                              FRAM_SPI_TIMEOUT_MS);
    if (status == HAL_OK)
    {
        memset(data, 0xFF, length);
        status = HAL_SPI_Receive(&hspi1,
                                 data,
                                 length,
                                 FRAM_SPI_TIMEOUT_MS);
    }
    FRAM_Deselect();
    SPI1_Bus_Release();

    if (status != HAL_OK)
    {
        ++g_fram_error_count;
    }
    return status;
}

HAL_StatusTypeDef FRAM_Write(uint16_t address,
                             const uint8_t *data,
                             uint16_t length)
{
    uint8_t header[3];
    HAL_StatusTypeDef status;

    if (length == 0U)
    {
        return HAL_OK;
    }
    if ((data == NULL) || (FRAM_CheckRange(address, length) != HAL_OK))
    {
        ++g_fram_error_count;
        return HAL_ERROR;
    }

    header[0] = FRAM_CMD_WRITE;
    header[1] = (uint8_t)(address >> 8U);
    header[2] = (uint8_t)address;

    if (SPI1_Bus_Acquire() != HAL_OK)
    {
        ++g_fram_error_count;
        return HAL_ERROR;
    }

    /*
     * 一直持有同一把互斥锁完成“WREN + WRITE”。虽然芯片要求它们使用
     * 两个独立CS脉冲，但软件层面不允许OLED或其它任务插入两者之间。
     */
    status = FRAM_WriteEnableLocked();
    if (status == HAL_OK)
    {
        FRAM_Select();
        status = HAL_SPI_Transmit(&hspi1,
                                  header,
                                  (uint16_t)sizeof(header),
                                  FRAM_SPI_TIMEOUT_MS);
        if (status == HAL_OK)
        {
            status = HAL_SPI_Transmit(&hspi1,
                                      (uint8_t *)data,
                                      length,
                                      FRAM_SPI_TIMEOUT_MS);
        }
        FRAM_Deselect();
    }

    SPI1_Bus_Release();

    if (status == HAL_OK)
    {
        ++g_fram_write_count;
    }
    else
    {
        ++g_fram_error_count;
    }
    return status;
}

HAL_StatusTypeDef FRAM_Init(void)
{
    uint8_t device_id[FRAM_DEVICE_ID_LENGTH];
    uint8_t status_register;

    g_fram_initialized = 0U;
    g_fram_status_register = 0U;
    g_fram_write_count = 0U;
    g_fram_error_count = 0U;
    for (uint8_t index = 0U; index < FRAM_DEVICE_ID_LENGTH; ++index)
    {
        g_fram_device_id[index] = 0U;
    }

    /* FRAM初始化只设置自己的CS；OLED的CS由OLED驱动负责。 */
    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);

    if (FRAM_ReadDeviceId(device_id) != HAL_OK)
    {
        return HAL_ERROR;
    }
    for (uint8_t index = 0U; index < FRAM_DEVICE_ID_LENGTH; ++index)
    {
        g_fram_device_id[index] = device_id[index];
    }
    /**
     *判断读取到的ID是否是 04 7F 01 01
     *这是我MB85RS16的ID,现在我两块板子上的FRAM大小不同，所以ID不用，这里就不能使用获取到的ID进行初始化。
     */
    // if (memcmp(device_id,
    //            g_fram_expected_device_id,
    //            FRAM_DEVICE_ID_LENGTH) != 0)
    // {
    //     ++g_fram_error_count;
    //     return HAL_ERROR;
    // }

    if (FRAM_ReadStatus(&status_register) != HAL_OK)
    {
        return HAL_ERROR;
    }
    g_fram_status_register = status_register;

    /* 不擅自改写状态寄存器；检测到块保护时明确报告初始化失败。 */
    if ((status_register & FRAM_STATUS_BP_MASK) != 0U)
    {
        ++g_fram_error_count;
        return HAL_ERROR;
    }

    g_fram_initialized = 1U;
    return HAL_OK;
}
