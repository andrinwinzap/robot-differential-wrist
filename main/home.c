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

void home(pid_position_ctrl_t *pid_position_ctrl, as5600_t *encoderA, as5600_t *encoderB)
{
    enum
    {
        FINDING_A_AXIS_END,
        MOVING_TO_B_END,
        MOVING_NEXT_TO_B_END_SWITCH,
        FINDING_B_AXIS_RISING_EDGE,
        FINDING_B_AXIS_FALLING_EDGE,
        MOVING_TO_ZERO
    } state = FINDING_A_AXIS_END;

    int8_t dir;
    float rising_edge;
    float falling_edge;

    while (true)
    {
        switch (state)
        {
        case FINDING_A_AXIS_END:
        {
            if (a_axis_endstop())
            {
                as5600_set_position(encoderA, M_PI / 2.0f);
                pid_position_ctrl->differential = -M_PI / 2.0f;
                ESP_LOGI(TAG, "AXIS A HOMED");
                state = MOVING_TO_B_END;
            }
            pid_position_ctrl->differential += HOMING_SPEED * 0.01;
            vTaskDelay(pdMS_TO_TICKS(10));
            break;
        }
        case MOVING_TO_B_END:
        {
            if (fabs(as5600_get_position(encoderA) - pid_position_ctrl->differential) < 0.1)
            {
                float pos = as5600_get_position(encoderB);
                dir = (pos < 0) - (pos > 0);

                pid_position_ctrl->common = -dir * 0.1f;
                state = MOVING_NEXT_TO_B_END_SWITCH;
            }
            break;
        }

        case MOVING_NEXT_TO_B_END_SWITCH:
        {
            if (fabs(as5600_get_position(encoderB) - pid_position_ctrl->common) < 0.01)
            {
                ESP_LOGI("i'm here", );
                state = FINDING_B_AXIS_RISING_EDGE;
            }
            break;
        }

        case FINDING_B_AXIS_RISING_EDGE:
        {
            if (b_axis_endstop())
            {
                rising_edge = as5600_get_position(encoderB);
                ESP_LOGI(TAG, "RISING EDGE: %.4f", rising_edge);
                state = FINDING_B_AXIS_FALLING_EDGE;
            }

            pid_position_ctrl->common += HOMING_SPEED * 0.01 * 0.1 * dir;
            vTaskDelay(pdMS_TO_TICKS(10));
            break;
        }

        case FINDING_B_AXIS_FALLING_EDGE:
        {
            if (!b_axis_endstop())
            {
                falling_edge = as5600_get_position(encoderB);
                ESP_LOGI(TAG, "Falling EDGE: %.4f", falling_edge);

                float calibrated_position = (falling_edge - rising_edge) / 2.0f;
                as5600_set_position(encoderB, calibrated_position);

                pid_position_ctrl->common = 0.5f;
                pid_position_ctrl->differential = 0.0f;

                state = MOVING_TO_ZERO;
            }

            pid_position_ctrl->common -= HOMING_SPEED * 0.01 * dir;
            vTaskDelay(pdMS_TO_TICKS(10));
            break;
        }

        case MOVING_TO_ZERO:
        {
            if ((fabs(as5600_get_position(encoderA) - pid_position_ctrl->differential) < 0.1) && (fabs(as5600_get_position(encoderB) - pid_position_ctrl->common) < 0.1))
            {
                return;
            }
            break;
        }
        default:
            break;
        }
        taskYIELD();
    }
}