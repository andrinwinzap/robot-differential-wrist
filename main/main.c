#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/gptimer.h"
#include "driver/uart.h"

#include "nvs_flash.h"

#include "as5600.h"
#include "tb6612.h"
#include "diff_speed_ctrl.h"
#include "pid.h"

#include "home.h"
#include "config.h"
#include "wrist.h"
#include "macros.h"

#define TOPIC_BUFFER_SIZE 64

rcl_publisher_t axis_a_position_publisher;
std_msgs__msg__Float32 axis_a_position_publisher_msg;
char axis_a_position_publisher_topic[TOPIC_BUFFER_SIZE];

rcl_publisher_t axis_b_position_publisher;
std_msgs__msg__Float32 axis_b_position_publisher_msg;
char axis_b_position_publisher_topic[TOPIC_BUFFER_SIZE];

rcl_subscription_t axis_a_position_subscriber;
std_msgs__msg__Float32 axis_a_position_subscriber_msg;
char axis_a_position_subscriber_topic[TOPIC_BUFFER_SIZE];

rcl_subscription_t axis_b_position_subscriber;
std_msgs__msg__Float32 axis_b_position_subscriber_msg;
char axis_b_position_subscriber_topic[TOPIC_BUFFER_SIZE];

SemaphoreHandle_t pid_semaphore;
SemaphoreHandle_t homing_semaphore;

wrist_t wrist;

static homing_params_t homing_params;

static const char *TAG = "Wrist";

static size_t uart_port = UART_NUM_1;

static gptimer_handle_t control_timer = NULL;

void axis_a_position_subscriber_callback(const void *msgin)
{
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
    float pos = msg->data;
    if (pos > A_AXIS_MAX)
    {
        pos = A_AXIS_MAX;
    }
    else if (pos < A_AXIS_MIN)
    {
        pos = A_AXIS_MIN;
    }
    wrist.axis_a.pos_ctrl = pos;
}

void axis_b_position_subscriber_callback(const void *msgin)
{
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
    float pos = msg->data;
    if (pos > B_AXIS_MAX)
    {
        pos = B_AXIS_MAX;
    }
    else if (pos < B_AXIS_MIN)
    {
        pos = B_AXIS_MIN;
    }
    wrist.axis_b.pos_ctrl = pos;
}

float axis_a_get_position(void)
{
    return as5600_get_position(&wrist.axis_a.encoder);
}

float axis_b_get_position(void)
{
    return as5600_get_position(&wrist.axis_b.encoder);
}

bool IRAM_ATTR gptimer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;

    xSemaphoreGiveFromISR(pid_semaphore, &high_task_wakeup);
    xSemaphoreGiveFromISR(homing_semaphore, &high_task_wakeup);

    return high_task_wakeup == pdTRUE;
}

void init_control_timer()
{
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1 MHz -> 1 tick = 1 microsecond
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &control_timer));

    gptimer_event_callbacks_t callbacks = {
        .on_alarm = gptimer_callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(control_timer, &callbacks, NULL));

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 1000000 / PID_LOOP_FREQUENCY,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(control_timer, &alarm_config));

    ESP_ERROR_CHECK(gptimer_enable(control_timer));
    ESP_ERROR_CHECK(gptimer_start(control_timer));
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

void nvs_init()
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
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        axis_a_position_publisher_msg.data = axis_a_get_position();
        RCSOFTCHECK(rcl_publish(&axis_a_position_publisher, &axis_a_position_publisher_msg, NULL));

        axis_b_position_publisher_msg.data = axis_b_get_position();
        RCSOFTCHECK(rcl_publish(&axis_b_position_publisher, &axis_b_position_publisher_msg, NULL));
    }
}

void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "wrist", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &axis_a_position_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        axis_a_position_publisher_topic));

    RCCHECK(rclc_publisher_init_default(
        &axis_b_position_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        axis_b_position_publisher_topic));

    RCCHECK(rclc_subscription_init_default(
        &axis_a_position_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        axis_a_position_subscriber_topic));

    RCCHECK(rclc_subscription_init_default(
        &axis_b_position_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        axis_b_position_subscriber_topic));

    rcl_timer_t timer;
    const unsigned int timer_timeout = 20;
    RCCHECK(rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback,
        true));

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &axis_a_position_subscriber,
        &axis_a_position_subscriber_msg,
        axis_a_position_subscriber_callback,
        ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &axis_b_position_subscriber,
        &axis_b_position_subscriber_msg,
        axis_b_position_subscriber_callback,
        ON_NEW_DATA));

    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);
    }

    // free resources
    RCCHECK(rcl_publisher_fini(&axis_a_position_publisher, &node));
    RCCHECK(rcl_publisher_fini(&axis_b_position_publisher, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
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

                ESP_LOGI(TAG,
                         "PID Freq: %.2f Hz | Loop Time: %.0f us | "
                         "Ctrl A: %.4f | Ctrl B: %.4f | "
                         "Enc A Δ: %.2f%% | Enc B Δ: %.2f%%",
                         measured_pid_frequency, measured_loop_time_us,
                         differential_control_signal, common_control_signal,
                         change_percent_a, change_percent_b);

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
    ESP_LOGI(TAG, "Starting Setup...");

    snprintf(axis_a_position_publisher_topic, TOPIC_BUFFER_SIZE, "/%s/%s/get_position", robot_name, axis_a_name);
    snprintf(axis_a_position_subscriber_topic, TOPIC_BUFFER_SIZE, "/%s/%s/set_position", robot_name, axis_a_name);
    snprintf(axis_b_position_publisher_topic, TOPIC_BUFFER_SIZE, "/%s/%s/get_position", robot_name, axis_b_name);
    snprintf(axis_b_position_subscriber_topic, TOPIC_BUFFER_SIZE, "/%s/%s/set_position", robot_name, axis_b_name);

    pid_semaphore = xSemaphoreCreateBinary();
    homing_semaphore = xSemaphoreCreateBinary();

    init_control_timer();
    nvs_init();

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

    pid_init(&wrist.axis_a.pid, 3.0f, .0f, .0f, 1.0f / PID_LOOP_FREQUENCY);
    pid_init(&wrist.axis_b.pid, 3.0f, .0f, .0f, 1.0f / PID_LOOP_FREQUENCY);

    bool ok1 = as5600_init(&wrist.axis_a.encoder, I2C_MASTER_NUM_0, AS5600_DEFAULT_ADDR, .1f, 0.0f, 1.0f, -1, true);
    bool ok2 = as5600_init(&wrist.axis_b.encoder, I2C_MASTER_NUM_1, AS5600_DEFAULT_ADDR, .1f, 0.0f, 1.0f, -1, true);

    if (!ok1 || !ok2)
    {
        ESP_LOGE(TAG, "Failed to initialize one or both encoders (ok1=%d, ok2=%d)", ok1, ok2);
        return;
    }

    ESP_LOGI(TAG, "Encoders initialized on I2C_NUM_0 and I2C_NUM_1.");

    wrist.axis_a.pos_ctrl = as5600_get_position(&wrist.axis_a.encoder);
    wrist.axis_b.pos_ctrl = as5600_get_position(&wrist.axis_b.encoder);

    homing_params.homing_semaphore = homing_semaphore;
    homing_params.wrist = &wrist;

    xTaskCreatePinnedToCore(
        pid_loop_task,
        "pid_loop",
        16384,
        NULL,
        10,
        NULL,
        1);

    xTaskCreatePinnedToCore(
        homing_task,
        "homing_task",
        8192,
        &homing_params,
        5,
        NULL,
        0);

#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    rmw_uros_set_custom_transport(
        true,
        (void *)&uart_port,
        esp32_serial_open,
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read);
#else
#error micro-ROS transports misconfigured
#endif // RMW_UXRCE_TRANSPORT_CUSTOM

    xTaskCreatePinnedToCore(
        micro_ros_task,
        "uros_task",
        16384,
        NULL,
        5,
        NULL,
        0);
}
