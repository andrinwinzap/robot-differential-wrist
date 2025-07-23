#include <math.h>
#include "tb6612.h"

static inline float rpm_to_rad_per_sec(float rpm)
{
    return (rpm * 2.0f * M_PI) / 60.0f;
}

void tb6612_motor_init(const tb6612_motor_t *motor)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << motor->dir_pin),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);

    mcpwm_gpio_init(motor->unit, MCPWM0A, motor->pwm_gpio);

    mcpwm_config_t pwm_config = {
        .frequency = 1000,
        .cmpr_a = 0.0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0};
    mcpwm_init(motor->unit, motor->timer, &pwm_config);
}

void tb6612_motor_set_speed(const tb6612_motor_t *motor, float speed)
{
    float max_speed = rpm_to_rad_per_sec(motor->max_rpm);

    if (speed > max_speed)
        speed = max_speed;
    if (speed < -max_speed)
        speed = -max_speed;

    float duty = (speed / max_speed) * 100.0f;

    if (duty > 0.0f)
    {
        gpio_set_level(motor->dir_pin, 1);
        mcpwm_set_duty(motor->unit, motor->timer, motor->op, duty);
    }
    else if (duty < 0.0f)
    {
        gpio_set_level(motor->dir_pin, 0);
        mcpwm_set_duty(motor->unit, motor->timer, motor->op, -duty);
    }
    else
    {
        mcpwm_set_duty(motor->unit, motor->timer, motor->op, 0.0f);
    }
}
