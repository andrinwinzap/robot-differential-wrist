#ifndef HOME_H
#define HOME_H

#include "as5600.h"
#include "config.h"
#include "freertos/semphr.h"

typedef struct
{
    SemaphoreHandle_t homing_semaphore;
    pid_position_ctrl_t *pid_position_ctrl;
    pid_speed_ctrl_t *pid_speed_ctrl;
    as5600_t *encoderA;
    as5600_t *encoderB;
} homing_params_t;

void homing_task(void *param);

#endif // HOME_H