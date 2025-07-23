#ifndef TB6612_H
#define TB6612_H

#include "driver/gpio.h"
#include "driver/mcpwm.h"

typedef struct
{
    gpio_num_t dir_pin;
    gpio_num_t pwm_gpio;
    mcpwm_unit_t unit;
    mcpwm_timer_t timer;
    mcpwm_operator_t op;
    float max_rpm;
} tb6612_motor_t;

void tb6612_motor_init(const tb6612_motor_t *motor);
void tb6612_motor_set_speed(const tb6612_motor_t *motor, float speed);

#endif