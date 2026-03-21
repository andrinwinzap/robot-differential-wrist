#ifndef HOME_H
#define HOME_H

#include "config.h"
#include "wrist.h"
#include "freertos/semphr.h"

typedef struct
{
    SemaphoreHandle_t homing_semaphore;
    wrist_t *wrist;
} homing_params_t;

bool homing_task(void *param);

#endif // HOME_H