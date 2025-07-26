#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "as5600.h"
#include "esp_log.h"
#include "tb6612.h"
#include "diff_speed_ctrl.h"
#include "pid.h"
#include "nvs_flash.h"
#include "esp_timer.h"

#define TAG "AS5600_DUAL"

#define I2C_MASTER_NUM_1 I2C_NUM_0
#define I2C_MASTER_SDA_IO_1 10
#define I2C_MASTER_SCL_IO_1 9

#define I2C_MASTER_NUM_0 I2C_NUM_1
#define I2C_MASTER_SDA_IO_0 18
#define I2C_MASTER_SCL_IO_0 17

#define I2C_FREQ_HZ 400000

#define ENDSTOP_A_PIN GPIO_NUM_11
#define ENDSTOP_B_PIN GPIO_NUM_8

#define COMMON_GEAR_RATIO

#define PID_LOOP_FREQUENCY 100
#define PID_FREQ_ALPHA 0.1f
#define HOMING_SPEED 1.5 // rad/s

#define CTRL_SIGNAL_THRESHOLD 0.03f

tb6612_motor_t motor1, motor2;
pid_controller_t pidA, pidB;

float common_position_target = 0.0f;
float differential_position_target = 0.0f;

void motor1_cb(float speed)
{
    tb6612_motor_set_speed(&motor1, speed);
}

void motor2_cb(float speed)
{
    tb6612_motor_set_speed(&motor2, speed);
}

diff_speed_ctrl_t speed_controller = {
    .cb1 = &motor1_cb,
    .cb2 = &motor2_cb,
    .gear_ratio = 20.0f / 29.0f,
    .common_speed = 0.0f,
    .differential_speed = 0.0f};

as5600_t encoderA, encoderB;

void gpio_input_init(gpio_num_t pin)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

void i2c_bus_init(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(i2c_num, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_num, conf.mode, 0, 0, 0));
}

void pid_loop_task(void *param)
{
    float dt_s = 1.0f / PID_LOOP_FREQUENCY;
    float dt_ms = 1000 / PID_LOOP_FREQUENCY;

    double measured_pid_frequency = 0.0;

    for (;;)
    {
        as5600_update(&encoderA);
        as5600_update(&encoderB);

        // Differential
        float differential_position_measured = as5600_get_position(&encoderA);
        float differential_control_signal = pid_update(&pidA, differential_position_target, differential_position_measured, dt_s);
        if (fabsf(differential_control_signal) < CTRL_SIGNAL_THRESHOLD)
            differential_control_signal = 0.0f;
        set_differential_speed(&speed_controller, differential_control_signal);

        // Common
        float common_position_measured = as5600_get_position(&encoderB);
        float common_control_signal = pid_update(&pidB, common_position_target, common_position_measured, dt_s);
        if (fabsf(common_control_signal) < CTRL_SIGNAL_THRESHOLD)
            common_control_signal = 0.0f;
        set_common_speed(&speed_controller, common_control_signal);

        int64_t now_us = esp_timer_get_time();
        static int64_t last_us = 0;

        if (last_us > 0)
        {
            float loop_time_ms = (now_us - last_us) / 1000.0f;
            float current_frequency = 1000.0f / loop_time_ms;
            measured_pid_frequency = PID_FREQ_ALPHA * current_frequency + (1.0f - PID_FREQ_ALPHA) * measured_pid_frequency;
        }
        last_us = now_us;

        static int64_t last_log_us = 0;
        if ((now_us - last_log_us) >= 1000000) // 1 second
        {
            ESP_LOGI(TAG, "PID Loop Frequency: %.2f Hz", measured_pid_frequency);
            last_log_us = now_us;
        }

        vTaskDelay(pdMS_TO_TICKS(dt_ms));
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS partition truncated or new version found, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized successfully");

    gpio_input_init(ENDSTOP_A_PIN);
    gpio_input_init(ENDSTOP_B_PIN);

    i2c_bus_init(I2C_MASTER_NUM_0, I2C_MASTER_SDA_IO_0, I2C_MASTER_SCL_IO_0);
    i2c_bus_init(I2C_MASTER_NUM_1, I2C_MASTER_SDA_IO_1, I2C_MASTER_SCL_IO_1);

    tb6612_motor_init(&motor1, GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_6, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    tb6612_motor_init(&motor2, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14, MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);

    pid_init(&pidA, 10.0f, 0.0f, 0.0f, 0.0f);
    pid_init(&pidB, 10.0f, 0.0f, 0.0f, 0.0f);

    bool ok1 = as5600_init(&encoderA, I2C_MASTER_NUM_0, AS5600_DEFAULT_ADDR, 0.1f, 0.01f, 1.0f, -1);
    bool ok2 = as5600_init(&encoderB, I2C_MASTER_NUM_1, AS5600_DEFAULT_ADDR, 0.1f, 0.01f, 1.0f, -1);

    if (!ok1 || !ok2)
    {
        ESP_LOGE(TAG, "Failed to initialize one or both encoders (ok1=%d, ok2=%d)", ok1, ok2);
        return;
    }

    ESP_LOGI(TAG, "Encoders initialized on I2C_NUM_0 and I2C_NUM_1.");

    differential_position_target = as5600_get_position(&encoderA);
    common_position_target = as5600_get_position(&encoderB);

    xTaskCreatePinnedToCore(
        pid_loop_task,
        "pid_loop",
        8000,
        NULL,
        10,
        NULL,
        1);

    while (!gpio_get_level(ENDSTOP_A_PIN))
    {
        differential_position_target += HOMING_SPEED * 0.01;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    as5600_set_position(&encoderA, M_PI / 2.0f);
    differential_position_target = -M_PI / 2.0f;

    ESP_LOGI(TAG, "AXIS A HOMED");

    while (fabs(as5600_get_position(&encoderA) - differential_position_target) > 0.1)
        ;

    common_position_target = 0.0f;
    while (fabs(as5600_get_position(&encoderB) > 0.1))
        ;

    while (gpio_get_level(ENDSTOP_B_PIN))
    {
        common_position_target += HOMING_SPEED * 0.01;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    as5600_set_position(&encoderB, 0.0f);
    common_position_target = 0.5f;

    differential_position_target = 0.0f;
}
