#ifndef FRAM_H
#define FRAM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define FRAM_CAPACITY_BYTES       2048U
#define FRAM_DEVICE_ID_LENGTH     4U

/*
 * 调试器 Watch 窗口可直接观察这些量：
 * - g_fram_initialized == 1 表示器件 ID 和状态寄存器检查通过；
 * - g_fram_device_id 的正常值应为 04 7F 01 01；
 * - g_fram_error_count 记录底层通信或参数错误次数。
 */
extern volatile uint8_t g_fram_initialized;
extern volatile uint8_t g_fram_device_id[FRAM_DEVICE_ID_LENGTH];
extern volatile uint8_t g_fram_status_register;
extern volatile uint32_t g_fram_write_count;
extern volatile uint32_t g_fram_error_count;

/**
 * @brief 初始化并识别 MB85RS16，不改写 FRAM 用户数据。
 * @retval HAL_OK ID正确且存储区未被状态寄存器写保护。
 */
HAL_StatusTypeDef FRAM_Init(void);

/** @brief 读取4字节器件ID，MB85RS16正常应返回 04 7F 01 01。 */
HAL_StatusTypeDef FRAM_ReadDeviceId(uint8_t device_id[FRAM_DEVICE_ID_LENGTH]);

/** @brief 读取FRAM状态寄存器。 */
HAL_StatusTypeDef FRAM_ReadStatus(uint8_t *status);

/**
 * @brief 从FRAM读取连续数据。
 * @param address 0x0000~0x07FF范围内的起始字节地址。
 * @param data    接收缓冲区。
 * @param length  读取字节数，不允许越过FRAM末尾。
 */
HAL_StatusTypeDef FRAM_Read(uint16_t address, uint8_t *data, uint16_t length);

/**
 * @brief 向FRAM写入连续数据。
 * @param address 0x0000~0x07FF范围内的起始字节地址。
 * @param data    待写数据缓冲区。
 * @param length  写入字节数，不允许越过FRAM末尾。
 *
 * @note 每次调用都会自动先发送一次 WREN，再发送 WRITE、16位地址和数据。
 *       调用者不需要、也不应另外手动发送 WREN。
 */
HAL_StatusTypeDef FRAM_Write(uint16_t address,
                             const uint8_t *data,
                             uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* FRAM_H */
