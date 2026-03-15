#ifndef TB6612_H
#define TB6612_H

#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include <stdbool.h>

typedef struct
{
    gpio_num_t dir_pin;
    gpio_num_t pwm_pin;
    mcpwm_unit_t unit;
    mcpwm_timer_t timer;
    mcpwm_operator_t op;
    bool invert_dir;
} tb6612_motor_t;

void tb6612_motor_init(tb6612_motor_t *motor,
                       gpio_num_t dir_pin,
                       gpio_num_t pwm_pin,
                       mcpwm_unit_t unit,
                       mcpwm_timer_t timer,
                       mcpwm_operator_t op,
                       bool invert_dir);

void tb6612_motor_set_speed(const tb6612_motor_t *motor, float speed);

#endif // TB6612_H