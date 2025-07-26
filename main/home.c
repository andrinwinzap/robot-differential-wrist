#include "home.h"
#include "esp_log.h"
#include "math.h"

static const char *TAG = "HOMING";

bool a_axis_endstop()
{
    return gpio_get_level(ENDSTOP_A_PIN);
}

bool b_axis_endstop()
{
    return !gpio_get_level(ENDSTOP_B_PIN);
}

bool reached_target(as5600_t *encoder, float target)
{
    return (fabs(as5600_get_position(encoder) - target) < POSITION_TOLERANCE);
}

void homing_task(void *pvParams)
{
    homing_params_t *params = (homing_params_t *)pvParams;
    enum
    {
        FINDING_A_AXIS_END_COARSE,
        BACKING_OFF_A_ENDSTOP,
        FINDING_A_AXIS_END_FINE,
        MOVING_TO_B_END,
        MOVING_NEXT_TO_B_END_SWITCH,
        FINDING_B_AXIS_RISING_EDGE,
        FINDING_B_AXIS_FALLING_EDGE,
        MOVING_TO_ZERO,
        FINISHED
    } state = FINDING_A_AXIS_END_COARSE;

    int8_t dir;
    float rising_edge;
    float falling_edge;

    params->wrist->axis_a.speed_ctrl = COARSE_HOMING_SPEED;
    while (true)
    {
        if (xSemaphoreTake(params->homing_semaphore, portMAX_DELAY) == pdTRUE)
        {

            switch (state)
            {
            case FINDING_A_AXIS_END_COARSE:
            {
                if (a_axis_endstop())
                {
                    params->wrist->axis_a.speed_ctrl = 0.0;
                    float coarse_pos = as5600_get_position(&params->wrist->axis_a.encoder);
                    params->wrist->axis_a.pos_ctrl = coarse_pos - 0.3;

                    state = BACKING_OFF_A_ENDSTOP;
                }
                break;
            }
            case BACKING_OFF_A_ENDSTOP:
            {
                if (reached_target(&params->wrist->axis_a.encoder, params->wrist->axis_a.pos_ctrl))
                {
                    params->wrist->axis_a.speed_ctrl = FINE_HOMING_SPEED;

                    state = FINDING_A_AXIS_END_FINE;
                }
                break;
            }
            case FINDING_A_AXIS_END_FINE:
            {
                if (a_axis_endstop())
                {
                    params->wrist->axis_a.speed_ctrl = 0.0;
                    as5600_set_position(&params->wrist->axis_a.encoder, M_PI / 2.0f);
                    params->wrist->axis_a.pos_ctrl = -M_PI / 2.0f;

                    state = MOVING_TO_B_END;
                }
                break;
            }
            case MOVING_TO_B_END:
            {
                if (reached_target(&params->wrist->axis_a.encoder, params->wrist->axis_a.pos_ctrl))
                {
                    state = MOVING_NEXT_TO_B_END_SWITCH;
                }
                break;
            }

            case MOVING_NEXT_TO_B_END_SWITCH:
            {
                if (reached_target(&params->wrist->axis_b.encoder, params->wrist->axis_b.pos_ctrl))
                {
                    float pos = as5600_get_position(&params->wrist->axis_b.encoder);
                    dir = (pos < 0) - (pos > 0);
                    params->wrist->axis_b.pos_ctrl = -dir * 0.2f;

                    params->wrist->axis_b.speed_ctrl = COARSE_HOMING_SPEED * dir;
                    state = FINDING_B_AXIS_RISING_EDGE;
                }
                break;
            }

            case FINDING_B_AXIS_RISING_EDGE:
            {
                if (b_axis_endstop())
                {
                    rising_edge = as5600_get_position(&params->wrist->axis_b.encoder);
                    params->wrist->axis_b.speed_ctrl = FINE_HOMING_SPEED * dir;
                    ESP_LOGI(TAG, "RISING EDGE: %.4f", rising_edge);

                    state = FINDING_B_AXIS_FALLING_EDGE;
                }

                break;
            }

            case FINDING_B_AXIS_FALLING_EDGE:
            {
                if (!b_axis_endstop())
                {
                    params->wrist->axis_b.speed_ctrl = 0.0;
                    falling_edge = as5600_get_position(&params->wrist->axis_b.encoder);
                    ESP_LOGI(TAG, "Falling EDGE: %.4f", falling_edge);

                    float calibrated_position = (falling_edge - rising_edge) / 2.0f;
                    as5600_set_position(&params->wrist->axis_b.encoder, calibrated_position);

                    params->wrist->axis_b.pos_ctrl = 0.5f;
                    params->wrist->axis_a.pos_ctrl = 0.0f;

                    state = MOVING_TO_ZERO;
                }
                break;
            }

            case MOVING_TO_ZERO:
            {
                if (reached_target(&params->wrist->axis_a.encoder, params->wrist->axis_a.pos_ctrl) && reached_target(&params->wrist->axis_b.encoder, params->wrist->axis_b.pos_ctrl))
                {
                    state = FINISHED;
                }
                break;
            }

            default:
                break;
            }
        }
    }
}