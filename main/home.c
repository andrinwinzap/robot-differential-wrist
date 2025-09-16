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

bool reached_target_pos(axis_t *axis)
{
    return (fabs(axis->pos - axis->pos_ctrl) < POSITION_TOLERANCE);
}

void homing_task(void *pvParams)
{
    homing_params_t *params = (homing_params_t *)pvParams;

    typedef enum
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
    } homing_state_t;

    homing_state_t state = FINDING_A_AXIS_END_COARSE;
    homing_state_t last_logged_state = -1; // So we log the first state too

    int8_t dir;
    float rising_edge, falling_edge;

    params->wrist->axis_a.vel_ctrl = COARSE_HOMING_SPEED;

    ESP_LOGI(TAG, "Position Tolerance: %f", POSITION_TOLERANCE);
    float pos = params->wrist->axis_b.pos;
    dir = (pos < 0) - (pos > 0);
    params->wrist->axis_b.pos_ctrl = 1.5f;
    while (true)
    {
        if (xSemaphoreTake(params->homing_semaphore, portMAX_DELAY) == pdTRUE)
        {
            ESP_LOGI(TAG, "A Pos: %f, A Pos Ctrl: %f, A Vel: %f, A Vel Ctrl: %f, B Pos: %f, B Pos Ctrl: %f, B Vel: %f, B Vel Ctrl: %f,", params->wrist->axis_a.pos, params->wrist->axis_a.pos_ctrl, params->wrist->axis_a.vel, params->wrist->axis_a.vel_ctrl, params->wrist->axis_b.pos, params->wrist->axis_b.pos_ctrl, params->wrist->axis_b.vel, params->wrist->axis_b.vel_ctrl);
            if (state != last_logged_state)
            {
                ESP_LOGI(TAG, "State: %d", state);
                last_logged_state = state;
            }

            switch (state)
            {
            case FINDING_A_AXIS_END_COARSE:
                if (a_axis_endstop())
                {
                    params->wrist->axis_a.vel_ctrl = 0.0;
                    float coarse_pos = params->wrist->axis_a.pos;
                    params->wrist->axis_a.pos_ctrl = coarse_pos - 0.2;
                    ESP_LOGI(TAG, "A endstop (coarse) hit at %.4f", coarse_pos);
                    state = BACKING_OFF_A_ENDSTOP;
                }
                break;

            case BACKING_OFF_A_ENDSTOP:
                if (reached_target_pos(&params->wrist->axis_a))
                {
                    params->wrist->axis_a.vel_ctrl = FINE_HOMING_SPEED;
                    state = FINDING_A_AXIS_END_FINE;
                }
                break;

            case FINDING_A_AXIS_END_FINE:
                if (a_axis_endstop())
                {
                    params->wrist->axis_a.vel_ctrl = 0.0;
                    as5600_set_position(&params->wrist->axis_a.encoder, ENDSTOP_A_POSITION);
                    params->wrist->axis_a.pos = ENDSTOP_A_POSITION;
                    params->wrist->axis_a.pos_ctrl = AXIS_A_MAX + 0.00;
                    ESP_LOGI(TAG, "A endstop (fine) hit, zeroed to %.4f", ENDSTOP_A_POSITION);
                    state = MOVING_TO_B_END;
                }
                break;

            case MOVING_TO_B_END:
                if (reached_target_pos(&params->wrist->axis_a))
                {
                    state = MOVING_NEXT_TO_B_END_SWITCH;
                }
                break;

            case MOVING_NEXT_TO_B_END_SWITCH:
                if (reached_target_pos(&params->wrist->axis_b))
                {
                    ESP_LOGI(TAG, "B Axis initial: %d", b_axis_endstop());
                    params->wrist->axis_b.vel_ctrl = -FINE_HOMING_SPEED;
                    state = FINDING_B_AXIS_RISING_EDGE;
                }
                break;

            case FINDING_B_AXIS_RISING_EDGE:
                if (b_axis_endstop())
                {
                    rising_edge = params->wrist->axis_b.pos;
                    ESP_LOGI(TAG, "B rising edge at %.4f", rising_edge);
                    state = FINDING_B_AXIS_FALLING_EDGE;
                }
                break;

            case FINDING_B_AXIS_FALLING_EDGE:
                if (!b_axis_endstop())
                {
                    params->wrist->axis_b.vel_ctrl = 0.0;
                    falling_edge = params->wrist->axis_b.pos;
                    ESP_LOGI(TAG, "B falling edge at %.4f", falling_edge);

                    float center = ((falling_edge + rising_edge) / 2.0f);
                    float calibrated = ENDSTOP_B_POSITION + (params->wrist->axis_b.pos - center);
                    as5600_set_position(&params->wrist->axis_b.encoder, calibrated);
                    params->wrist->axis_b.pos = calibrated;
                    params->wrist->axis_a.pos_ctrl = 0.0f;
                    params->wrist->axis_b.pos_ctrl = 0.0f;

                    state = MOVING_TO_ZERO;
                }
                break;

            case MOVING_TO_ZERO:
                if (reached_target_pos(&params->wrist->axis_a) &&
                    reached_target_pos(&params->wrist->axis_b))
                {
                    ESP_LOGI(TAG, "Homing complete. Both axes at zero.");
                    state = FINISHED;
                }
                break;

            case FINISHED:
                return;
            }
        }
    }
}
