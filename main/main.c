#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tb6612.h"

tb6612_motor_t motor1 = {
    .dir_pin = GPIO_NUM_19,
    .pwm_gpio = GPIO_NUM_18,
    .unit = MCPWM_UNIT_0,
    .timer = MCPWM_TIMER_0,
    .op = MCPWM_OPR_A,
    .max_rpm = 40.0f,
};

tb6612_motor_t motor2 = {
    .dir_pin = GPIO_NUM_17,
    .pwm_gpio = GPIO_NUM_16,
    .unit = MCPWM_UNIT_0,
    .timer = MCPWM_TIMER_1,
    .op = MCPWM_OPR_A,
    .max_rpm = 40.0f,
};

void app_main(void)
{
    tb6612_motor_init(&motor1);
    tb6612_motor_init(&motor2);

    while (1)
    {
        tb6612_motor_set_speed(&motor1, 3.14f);
        tb6612_motor_set_speed(&motor2, -2.0f);
        vTaskDelay(pdMS_TO_TICKS(2000));

        tb6612_motor_set_speed(&motor1, 0.0f);
        tb6612_motor_set_speed(&motor2, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
