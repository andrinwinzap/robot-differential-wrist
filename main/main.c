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
#include "driver/timer.h"
#include "freertos/semphr.h"

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

#define PID_LOOP_FREQUENCY 1000
#define PID_FREQ_ALPHA 0.9f
#define HOMING_SPEED 1.5 // rad/s

#define CTRL_SIGNAL_THRESHOLD 0.1f

#define TIMER_GROUP TIMER_GROUP_0
#define TIMER_IDX TIMER_0

SemaphoreHandle_t pid_semaphore;

tb6612_motor_t motor1, motor2;
pid_controller_t pidA, pidB;

float common_position_target = 0.0f;
float differential_position_target = 0.0f;

void IRAM_ATTR timer_isr(void *arg)
{
    BaseType_t high_task_wakeup = pdFALSE;
    timer_group_clr_intr_status_in_isr(TIMER_GROUP, TIMER_IDX);
    timer_group_enable_alarm_in_isr(TIMER_GROUP, TIMER_IDX);
    xSemaphoreGiveFromISR(pid_semaphore, &high_task_wakeup);
    if (high_task_wakeup)
    {
        portYIELD_FROM_ISR();
    }
}

void init_pid_timer()
{
    timer_config_t config = {
        .divider = 80, // 1 tick = 1 microsecond (assuming 80 MHz APB clock)
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = true,
    };
    ESP_ERROR_CHECK(timer_init(TIMER_GROUP, TIMER_IDX, &config));

    // Set alarm every (1.0 / PID_LOOP_FREQUENCY) seconds
    double alarm_time_sec = 1.0 / PID_LOOP_FREQUENCY;
    ESP_ERROR_CHECK(timer_set_counter_value(TIMER_GROUP, TIMER_IDX, 0));
    ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP, TIMER_IDX, alarm_time_sec * 1000000)); // microseconds
    ESP_ERROR_CHECK(timer_enable_intr(TIMER_GROUP, TIMER_IDX));
    ESP_ERROR_CHECK(timer_isr_register(TIMER_GROUP, TIMER_IDX, timer_isr, NULL, ESP_INTR_FLAG_IRAM, NULL));
    ESP_ERROR_CHECK(timer_start(TIMER_GROUP, TIMER_IDX));
}

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

    double measured_pid_frequency = 0.0;
    int64_t last_us = 0;
    int64_t last_log_us = 0;

    static float prev_position_a = 0.0f;
    static float prev_position_b = 0.0f;

    unsigned long change_count_a = 0;
    unsigned long change_count_b = 0;
    unsigned long total_updates_a = 0;
    unsigned long total_updates_b = 0;

    const float POSITION_EPSILON = 1e-6f;

    for (;;)
    {
        if (xSemaphoreTake(pid_semaphore, portMAX_DELAY) == pdTRUE)
        {
            as5600_update(&encoderA);
            as5600_update(&encoderB);

            // Differential (A)
            float differential_position_measured = as5600_get_position(&encoderA);
            total_updates_a++;
            if (fabsf(differential_position_measured - prev_position_a) >= POSITION_EPSILON)
                change_count_a++;
            prev_position_a = differential_position_measured;

            float differential_control_signal = pid_update(&pidA, differential_position_target, differential_position_measured, dt_s);
            if (fabsf(differential_control_signal) < CTRL_SIGNAL_THRESHOLD)
                differential_control_signal = 0.0f;
            set_differential_speed(&speed_controller, differential_control_signal);

            // Common (B)
            float common_position_measured = as5600_get_position(&encoderB);
            total_updates_b++;
            if (fabsf(common_position_measured - prev_position_b) >= POSITION_EPSILON)
                change_count_b++;
            prev_position_b = common_position_measured;

            float common_control_signal = pid_update(&pidB, common_position_target, common_position_measured, dt_s);
            if (fabsf(common_control_signal) < CTRL_SIGNAL_THRESHOLD)
                common_control_signal = 0.0f;
            set_common_speed(&speed_controller, common_control_signal);

            // Frequency estimation
            int64_t now_us = esp_timer_get_time();
            if (last_us > 0)
            {
                float loop_time_ms = (now_us - last_us) / 1000.0f;
                float current_frequency = 1000.0f / loop_time_ms;
                measured_pid_frequency = PID_FREQ_ALPHA * current_frequency + (1.0f - PID_FREQ_ALPHA) * measured_pid_frequency;
            }
            last_us = now_us;

            // Log once per second
            if ((now_us - last_log_us) >= 1000000)
            {
                float change_percent_a = total_updates_a ? (100.0f * change_count_a / total_updates_a) : 0.0f;
                float change_percent_b = total_updates_b ? (100.0f * change_count_b / total_updates_b) : 0.0f;

                ESP_LOGI(TAG, "PID Freq: %.2f Hz | A change: %lu/%lu (%.2f%%) | B change: %lu/%lu (%.2f%%)",
                         measured_pid_frequency,
                         change_count_a, total_updates_a, change_percent_a,
                         change_count_b, total_updates_b, change_percent_b);

                // Reset counters after logging
                change_count_a = 0;
                change_count_b = 0;
                total_updates_a = 0;
                total_updates_b = 0;
                last_log_us = now_us;
            }
        }
        taskYIELD();
    }
}

void app_main(void)
{
    pid_semaphore = xSemaphoreCreateBinary();
    init_pid_timer();

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

    pid_init(&pidA, 30.0f, 0.0f, 0.0f, 0.0f);
    pid_init(&pidB, 30.0f, 0.0f, 0.0f, 0.0f);

    bool ok1 = as5600_init(&encoderA, I2C_MASTER_NUM_0, AS5600_DEFAULT_ADDR, 1.0f, 0.0f, 1.0f, -1);
    bool ok2 = as5600_init(&encoderB, I2C_MASTER_NUM_1, AS5600_DEFAULT_ADDR, 1.0f, 0.0f, 1.0f, -1);

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
    while (fabs(as5600_get_position(&encoderB)) > 0.1)
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
