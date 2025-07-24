#include <math.h>
#include "tb6612.h"

static inline float rpm_to_rad_per_sec(float rpm)
{
    return (rpm * 2.0f * M_PI) / 60.0f;
}

void tb6612_motor_init(tb6612_motor_t *motor, gpio_num_t in1_pin, gpio_num_t in2_pin, gpio_num_t pwm_pin, mcpwm_unit_t unit, mcpwm_timer_t timer, mcpwm_operator_t op, float max_rpm)
{
    motor->in1_pin = in1_pin;
    motor->in2_pin = in2_pin;
    motor->pwm_pin = pwm_pin;
    motor->unit = unit;
    motor->timer = timer;
    motor->op = op;
    motor->max_rpm = max_rpm;

    // Configure IN1 and IN2 as output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << motor->in1_pin) | (1ULL << motor->in2_pin),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);

    // Determine correct PWM signal
    mcpwm_io_signals_t pwm_signal =
        (motor->op == MCPWM_OPR_A) ? (motor->timer == MCPWM_TIMER_0 ? MCPWM0A : motor->timer == MCPWM_TIMER_1 ? MCPWM1A
                                                                                                              : MCPWM2A)
                                   : (motor->timer == MCPWM_TIMER_0 ? MCPWM0B : motor->timer == MCPWM_TIMER_1 ? MCPWM1B
                                                                                                              : MCPWM2B);

    mcpwm_gpio_init(motor->unit, pwm_signal, motor->pwm_pin);

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

    float duty = fabsf(speed / max_speed) * 100.0f;

    if (speed > 0.0f)
    {
        gpio_set_level(motor->in1_pin, 1);
        gpio_set_level(motor->in2_pin, 0);
        mcpwm_set_duty(motor->unit, motor->timer, motor->op, duty);
    }
    else if (speed < 0.0f)
    {
        gpio_set_level(motor->in1_pin, 0);
        gpio_set_level(motor->in2_pin, 1);
        mcpwm_set_duty(motor->unit, motor->timer, motor->op, duty);
    }
    else
    {
        gpio_set_level(motor->in1_pin, 0);
        gpio_set_level(motor->in2_pin, 0); // Coast
        mcpwm_set_duty(motor->unit, motor->timer, motor->op, 0.0f);
    }
}
