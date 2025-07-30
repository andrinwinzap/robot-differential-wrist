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
#include "home.h"
#include "config.h"
#include "wrist.h"

#define TAG "AS5600_DUAL"

SemaphoreHandle_t pid_semaphore;
SemaphoreHandle_t homing_semaphore;

wrist_t wrist;

static homing_params_t homing_params;

void IRAM_ATTR timer_isr(void *arg)
{
    BaseType_t high_task_wakeup = pdFALSE;

    timer_group_clr_intr_status_in_isr(TIMER_GROUP, TIMER_IDX);
    timer_group_enable_alarm_in_isr(TIMER_GROUP, TIMER_IDX);

    xSemaphoreGiveFromISR(pid_semaphore, &high_task_wakeup);
    xSemaphoreGiveFromISR(homing_semaphore, &high_task_wakeup);

    if (high_task_wakeup)
    {
        portYIELD_FROM_ISR();
    }
}

void init_control_timer()
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
    tb6612_motor_set_speed(&wrist.motor_1, speed);
}

void motor2_cb(float speed)
{
    tb6612_motor_set_speed(&wrist.motor_2, speed);
}

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
    const int64_t PID_LOG_INTERVAL_US = (int64_t)(1000000.0f / PID_LOG_FREQUENCY_HZ);

    double measured_pid_frequency = 0.0;
    double measured_loop_time_us = 0.0; // Changed to microseconds
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
            int64_t loop_start_us = esp_timer_get_time();

            wrist.axis_a.pos_ctrl += wrist.axis_a.speed_ctrl * dt_s;
            wrist.axis_b.pos_ctrl += wrist.axis_b.speed_ctrl * dt_s;

            as5600_update(&wrist.axis_a.encoder);
            as5600_update(&wrist.axis_b.encoder);

            // Differential (A)
            float differential_position_measured = as5600_get_position(&wrist.axis_a.encoder);
            total_updates_a++;
            if (fabsf(differential_position_measured - prev_position_a) >= POSITION_EPSILON)
                change_count_a++;
            prev_position_a = differential_position_measured;

            float differential_control_signal = pid_update(&wrist.axis_a.pid, wrist.axis_a.pos_ctrl, differential_position_measured);
            if (fabsf(differential_control_signal) < CTRL_SIGNAL_THRESHOLD)
                differential_control_signal = 0.0f;
            set_differential_speed(&wrist.diff_speed_ctrl, differential_control_signal);

            // Common (B)
            float common_position_measured = as5600_get_position(&wrist.axis_b.encoder);
            total_updates_b++;
            if (fabsf(common_position_measured - prev_position_b) >= POSITION_EPSILON)
                change_count_b++;
            prev_position_b = common_position_measured;

            float common_control_signal = pid_update(&wrist.axis_b.pid, wrist.axis_b.pos_ctrl, common_position_measured);
            if (fabsf(common_control_signal) < CTRL_SIGNAL_THRESHOLD)
                common_control_signal = 0.0f;
            set_common_speed(&wrist.diff_speed_ctrl, common_control_signal);

            int64_t now_us = esp_timer_get_time();
            float loop_time_us = (float)(now_us - loop_start_us); // Microseconds
            measured_loop_time_us = PID_LOOP_TIME_ALPHA * loop_time_us + (1.0f - PID_LOOP_TIME_ALPHA) * measured_loop_time_us;

            if (last_us > 0)
            {
                float time_between_loops_ms = (now_us - last_us) / 1000.0f;
                float current_frequency = 1000.0f / time_between_loops_ms;
                measured_pid_frequency = PID_FREQ_ALPHA * current_frequency + (1.0f - PID_FREQ_ALPHA) * measured_pid_frequency;
            }
            last_us = now_us;

            if ((now_us - last_log_us) >= PID_LOG_INTERVAL_US)
            {
                float change_percent_a = total_updates_a ? (100.0f * change_count_a / total_updates_a) : 0.0f;
                float change_percent_b = total_updates_b ? (100.0f * change_count_b / total_updates_b) : 0.0f;

                ESP_LOGD(TAG,
                         "PID Freq: %.2f Hz | Loop Time: %.0f us | Encoder A change: %.2f%% | Encoder B change: %.2f%%",
                         measured_pid_frequency, measured_loop_time_us, change_percent_a, change_percent_b);

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
    homing_semaphore = xSemaphoreCreateBinary();
    init_control_timer();

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

    tb6612_motor_init(&wrist.motor_1, GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_6, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    tb6612_motor_init(&wrist.motor_2, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14, MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);

    wrist.axis_a.pos_ctrl = 0.0f;
    wrist.axis_a.speed_ctrl = 0.0f;

    wrist.axis_b.pos_ctrl = 0.0f;
    wrist.axis_b.speed_ctrl = 0.0f;

    wrist.diff_speed_ctrl.cb1 = &motor1_cb;
    wrist.diff_speed_ctrl.cb2 = &motor2_cb;
    wrist.diff_speed_ctrl.gear_ratio = 20.0f / 29.0f;
    wrist.diff_speed_ctrl.common_speed = 0.0f;
    wrist.diff_speed_ctrl.differential_speed = 0.0f;

    float dt = 1.0f / PID_LOOP_FREQUENCY;

    pid_init(&wrist.axis_a.pid, 10.0f, 0.0f, 0.0f, dt);
    pid_init(&wrist.axis_b.pid, 10.0f, 0.0f, 0.0f, dt);

    bool ok1 = as5600_init(&wrist.axis_a.encoder, I2C_MASTER_NUM_0, AS5600_DEFAULT_ADDR, 1.0f, 0.0f, 1.0f, -1);
    bool ok2 = as5600_init(&wrist.axis_b.encoder, I2C_MASTER_NUM_1, AS5600_DEFAULT_ADDR, 1.0f, 0.0f, 1.0f, -1);

    if (!ok1 || !ok2)
    {
        ESP_LOGE(TAG, "Failed to initialize one or both encoders (ok1=%d, ok2=%d)", ok1, ok2);
        return;
    }

    ESP_LOGI(TAG, "Encoders initialized on I2C_NUM_0 and I2C_NUM_1.");

    wrist.axis_a.pos_ctrl = as5600_get_position(&wrist.axis_a.encoder);
    wrist.axis_b.pos_ctrl = as5600_get_position(&wrist.axis_b.encoder);

    xTaskCreatePinnedToCore(
        pid_loop_task,
        "pid_loop",
        8000,
        NULL,
        10,
        NULL,
        1);

    homing_params.homing_semaphore = homing_semaphore;
    homing_params.wrist = &wrist;

    xTaskCreatePinnedToCore(
        homing_task,
        "homing_task",
        4096,
        &homing_params,
        5,
        NULL,
        1);
}
