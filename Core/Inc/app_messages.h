#ifndef APP_MESSAGES_H
#define APP_MESSAGES_H

// Message definitions go here
#include <stdint.h>

typedef enum{
    CAN_COMMAND_SET_DIRECTION = 1
} CanCommandType_t;

typedef enum{
    MOTOR_DIRECTION_NORMAL = 1,
    MOTOR_DIRECTION_REVERSED = 2
} MotorDirection_t;

typedef struct{
    CanCommandType_t command_type;
    uint8_t motor_mask;
    MotorDirection_t direction;
} CanCommand_t;




#endif // APP_MESSAGES_H