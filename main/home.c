#include "home.h"
#include "esp_log.h"
#include "math.h"

static const char *TAG = "HOMING";

void home(pid_position_ctrl_t *pid_position_ctrl, as5600_t *encoderA, as5600_t *encoderB)
{
    while (!gpio_get_level(ENDSTOP_A_PIN))
    {
        pid_position_ctrl->differential += HOMING_SPEED * 0.01;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    as5600_set_position(encoderA, M_PI / 2.0f);
    pid_position_ctrl->differential = -M_PI / 2.0f;

    ESP_LOGI(TAG, "AXIS A HOMED");

    while (fabs(as5600_get_position(encoderA) - pid_position_ctrl->differential) > 0.1)
        ;

    pid_position_ctrl->common = 0.0f;
    while (fabs(as5600_get_position(encoderB)) > 0.1)
        ;

    while (gpio_get_level(ENDSTOP_B_PIN))
    {
        pid_position_ctrl->common += HOMING_SPEED * 0.01;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    as5600_set_position(encoderB, 0.0f);
    pid_position_ctrl->common = 0.5f;

    pid_position_ctrl->differential = 0.0f;
}
