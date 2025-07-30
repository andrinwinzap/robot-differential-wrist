#include "tb6612.h"
#include <math.h>

void tb6612_motor_init(tb6612_motor_t *motor,
                       gpio_num_t in1_pin, gpio_num_t in2_pin,
                       gpio_num_t pwm_pin,
                       mcpwm_unit_t unit,
                       mcpwm_timer_t timer,
                       mcpwm_operator_t op)
{
    motor->in1_pin = in1_pin;
    motor->in2_pin = in2_pin;
    motor->pwm_pin = pwm_pin;
    motor->unit = unit;
    motor->timer = timer;
    motor->op = op;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << in1_pin) | (1ULL << in2_pin),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);

    mcpwm_io_signals_t pwm_signal =
        (op == MCPWM_OPR_A) ? (timer == MCPWM_TIMER_0 ? MCPWM0A : timer == MCPWM_TIMER_1 ? MCPWM1A
                                                                                         : MCPWM2A)
                            : (timer == MCPWM_TIMER_0 ? MCPWM0B : timer == MCPWM_TIMER_1 ? MCPWM1B
                                                                                         : MCPWM2B);

    mcpwm_gpio_init(unit, pwm_signal, pwm_pin);

    mcpwm_config_t pwm_config = {
        .frequency = 1000,
        .cmpr_a = 0.0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0};
    mcpwm_init(unit, timer, &pwm_config);
}

void tb6612_motor_set_speed(const tb6612_motor_t *motor, float speed)
{
    if (speed > 1.0f)
        speed = 1.0f;
    if (speed < -1.0f)
        speed = -1.0f;

    if (speed == 0.0f)
    {
        // Active brake
        gpio_set_level(motor->in1_pin, 0);
        gpio_set_level(motor->in2_pin, 0);
        mcpwm_set_duty(motor->unit, motor->timer, motor->op, 0.0f); // No PWM during brake
    }
    else
    {
        float min_duty = 0.05f; // Minimum effective duty
        float duty = (min_duty + (1.0f - min_duty) * fabsf(speed)) * 100.0f;

        if (speed > 0.0f)
        {
            gpio_set_level(motor->in1_pin, 1);
            gpio_set_level(motor->in2_pin, 0);
        }
        else
        {
            gpio_set_level(motor->in1_pin, 0);
            gpio_set_level(motor->in2_pin, 1);
        }

        mcpwm_set_duty(motor->unit, motor->timer, motor->op, duty);
    }
}