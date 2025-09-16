#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/gptimer.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"
#include "nvs_flash.h"
#include "as5600.h"
#include "tb6612.h"
#include "diff_speed_ctrl.h"
#include "pid.h"
#include "macros.h"
#include "config.h"
#include "wrist.h"
#include "home.h"

#define TOPIC_BUFFER_SIZE 64
#define COMMAND_BUFFER_LEN 2
#define STATE_BUFFER_LEN 2

rcl_publisher_t axis_a_state_publisher;
std_msgs__msg__Float32MultiArray axis_a_state_publisher_msg;
char axis_a_state_publisher_topic[TOPIC_BUFFER_SIZE];
static float axis_a_state_buffer[STATE_BUFFER_LEN];

rcl_subscription_t axis_a_command_subscriber;
std_msgs__msg__Float32MultiArray axis_a_command_subscriber_msg;
char axis_a_command_subscriber_topic[TOPIC_BUFFER_SIZE];
static float axis_a_command_buffer[COMMAND_BUFFER_LEN];

rcl_publisher_t axis_b_state_publisher;
std_msgs__msg__Float32MultiArray axis_b_state_publisher_msg;
char axis_b_state_publisher_topic[TOPIC_BUFFER_SIZE];
static float axis_b_state_buffer[STATE_BUFFER_LEN];

rcl_subscription_t axis_b_command_subscriber;
std_msgs__msg__Float32MultiArray axis_b_command_subscriber_msg;
char axis_b_command_subscriber_topic[TOPIC_BUFFER_SIZE];
static float axis_b_command_buffer[COMMAND_BUFFER_LEN];

SemaphoreHandle_t pid_semaphore;
SemaphoreHandle_t homing_semaphore;

static gptimer_handle_t control_timer = NULL;

static size_t uart_port = UART_NUM_1;

wrist_t wrist;

static homing_params_t homing_params;

static const char *TAG = "Wrist";

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
        .alarm_count = 1000000 / PID_LOOP_FREQUENCY_HZ,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(control_timer, &alarm_config));

    ESP_ERROR_CHECK(gptimer_enable(control_timer));
    ESP_ERROR_CHECK(gptimer_start(control_timer));
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

void motor1_cb(float speed)
{
    tb6612_motor_set_speed(&wrist.motor_1, speed);
}

void motor2_cb(float speed)
{
    tb6612_motor_set_speed(&wrist.motor_2, speed);
}

void pid_loop_task(void *param)
{
    float dt_s = 1.0f / PID_LOOP_FREQUENCY_HZ;
    const int64_t PID_LOG_INTERVAL_US = (int64_t)(1000000.0f / PID_LOG_FREQUENCY_HZ);

    double pid_freq = 0.0;
    double loop_time_us = 0.0;
    int64_t last_us = 0;
    int64_t last_log_us = 0;

    float diff_pos_feedback;
    float diff_pos_delta;
    float diff_vel_feedback = 0;
    float diff_vel_sig;
    float diff_pwm_sig;

    float comm_vel_sig;
    float comm_pos_delta;
    float comm_pwm_sig;
    float comm_pos_feedback;
    float comm_vel_feedback = 0;

    as5600_update(&wrist.axis_a.encoder);
    wrist.axis_a.pos = as5600_get_position(&wrist.axis_a.encoder);
    as5600_update(&wrist.axis_b.encoder);
    wrist.axis_b.pos = as5600_get_position(&wrist.axis_b.encoder);

    for (;;)
    {
        if (xSemaphoreTake(pid_semaphore, portMAX_DELAY) == pdTRUE)
        {
            int64_t loop_start_us = esp_timer_get_time();

            as5600_update(&wrist.axis_a.encoder);
            diff_pos_feedback = as5600_get_position(&wrist.axis_a.encoder);
            diff_pos_delta = diff_pos_feedback - wrist.axis_a.pos;
            diff_vel_feedback = AS5600_A_VELOCITY_FILTER_ALPHA * (diff_pos_delta / dt_s) + (1 - AS5600_A_VELOCITY_FILTER_ALPHA) * diff_vel_feedback;
            wrist.axis_a.pos = diff_pos_feedback;
            wrist.axis_a.vel = diff_vel_feedback;

            as5600_update(&wrist.axis_b.encoder);
            comm_pos_feedback = as5600_get_position(&wrist.axis_b.encoder);
            comm_pos_delta = comm_pos_feedback - wrist.axis_b.pos;
            comm_vel_feedback = AS5600_B_VELOCITY_FILTER_ALPHA * (comm_pos_delta / dt_s) + (1 - AS5600_B_VELOCITY_FILTER_ALPHA) * comm_vel_feedback;
            wrist.axis_b.pos = comm_pos_feedback;
            wrist.axis_b.vel = comm_vel_feedback;

            //ESP_LOGI(TAG, "A: %f, B: %f", diff_pos_feedback, comm_pos_feedback);
            //    Differential (A)
            diff_vel_sig = pid_update(&wrist.axis_a.pos_pid, wrist.axis_a.pos_ctrl, diff_pos_feedback, wrist.axis_a.vel_ctrl);
            diff_pwm_sig = pid_update(&wrist.axis_a.vel_pid, diff_vel_sig, diff_vel_feedback, diff_vel_sig);

            set_diff_pwm(&wrist.diff_pwm_ctrl, diff_pwm_sig);

            // Common (B)
            comm_vel_sig = pid_update(&wrist.axis_b.pos_pid, wrist.axis_b.pos_ctrl, comm_pos_feedback, wrist.axis_b.vel_ctrl);
            comm_pwm_sig = pid_update(&wrist.axis_b.vel_pid, comm_vel_sig, comm_vel_feedback, comm_vel_sig);

            set_comm_pwm(&wrist.diff_pwm_ctrl, comm_pwm_sig);

            wrist.axis_a.pos_ctrl += wrist.axis_a.vel_ctrl * dt_s;
            wrist.axis_b.pos_ctrl += wrist.axis_b.vel_ctrl * dt_s;

            int64_t now_us = esp_timer_get_time();
            loop_time_us = PID_LOOP_TIME_ALPHA * (float)(now_us - loop_start_us) + (1.0f - PID_LOOP_TIME_ALPHA) * loop_time_us;

            if (last_us > 0)
            {
                float time_between_loops_ms = (now_us - last_us) / 1000.0f;
                float current_frequency = 1000.0f / time_between_loops_ms;
                pid_freq = PID_FREQ_ALPHA * current_frequency + (1.0f - PID_FREQ_ALPHA) * pid_freq;
            }
            last_us = now_us;

            if ((now_us - last_log_us) >= PID_LOG_INTERVAL_US)
            {
                ESP_LOGD(TAG,
                         "PID Freq: %.2f Hz | Loop: %.0f us | "
                         "A: vel_meas=%.4f vel_ctrl=%.4f PWM=%.4f"
                         "B: vel_meas=%.4f vel_ctrl=%.4f PWM=%.4f",
                         pid_freq, loop_time_us,
                         diff_vel_feedback, diff_vel_sig, diff_pwm_sig,
                         comm_vel_feedback, comm_vel_sig, comm_pwm_sig);
                last_log_us = now_us;
            }
        }

        taskYIELD();
    }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        axis_a_state_publisher_msg.data.data[0] = wrist.axis_a.pos;
        axis_a_state_publisher_msg.data.data[1] = wrist.axis_a.vel;
        RCSOFTCHECK(rcl_publish(&axis_a_state_publisher, &axis_a_state_publisher_msg, NULL));

        axis_b_state_publisher_msg.data.data[0] = wrist.axis_b.pos;
        axis_b_state_publisher_msg.data.data[1] = wrist.axis_b.vel;
        RCSOFTCHECK(rcl_publish(&axis_b_state_publisher, &axis_b_state_publisher_msg, NULL));
    }
}

void axis_a_command_subscriber_callback(const void *msgin)
{
    const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    if (msg->data.size < 2)
    {
        ESP_LOGW(TAG, "Received command with insufficient data size: %zu", msg->data.size);
        return;
    }

    float pos = msg->data.data[0];
    if (pos > AXIS_A_MAX)
    {
        pos = AXIS_A_MAX;
    }
    else if (pos < AXIS_A_MIN)
    {
        pos = AXIS_A_MIN;
    }
    wrist.axis_a.pos_ctrl = pos;

    float vel = msg->data.data[1];
    wrist.axis_a.vel_ctrl = vel;
}

void axis_b_command_subscriber_callback(const void *msgin)
{
    const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    if (msg->data.size < 2)
    {
        ESP_LOGW(TAG, "Received command with insufficient data size: %zu", msg->data.size);
        return;
    }

    float pos = msg->data.data[0];
    if (pos > AXIS_B_MAX)
    {
        pos = AXIS_B_MAX;
    }
    else if (pos < AXIS_B_MIN)
    {
        pos = AXIS_B_MIN;
    }
    wrist.axis_b.pos_ctrl = pos;

    float vel = msg->data.data[1];
    wrist.axis_b.vel_ctrl = vel;
}

void micro_ros_task(void *arg)
{
try_uros_task:
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "wrist", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &axis_a_state_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        axis_a_state_publisher_topic));

    std_msgs__msg__Float32MultiArray__init(&axis_a_state_publisher_msg);
    axis_a_state_publisher_msg.data.data = axis_a_state_buffer;
    axis_a_state_publisher_msg.data.capacity = STATE_BUFFER_LEN;
    axis_a_state_publisher_msg.data.size = STATE_BUFFER_LEN;

    RCCHECK(rclc_publisher_init_default(
        &axis_b_state_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        axis_b_state_publisher_topic));

    std_msgs__msg__Float32MultiArray__init(&axis_b_state_publisher_msg);
    axis_b_state_publisher_msg.data.data = axis_b_state_buffer;
    axis_b_state_publisher_msg.data.capacity = STATE_BUFFER_LEN;
    axis_b_state_publisher_msg.data.size = STATE_BUFFER_LEN;

    RCCHECK(rclc_subscription_init_default(
        &axis_a_command_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        axis_a_command_subscriber_topic));

    std_msgs__msg__Float32MultiArray__init(&axis_a_command_subscriber_msg);
    axis_a_command_subscriber_msg.data.data = axis_a_command_buffer;
    axis_a_command_subscriber_msg.data.capacity = COMMAND_BUFFER_LEN;
    axis_a_command_subscriber_msg.data.size = 0;

    std_msgs__msg__Float32MultiArray__init(&axis_b_command_subscriber_msg);
    axis_b_command_subscriber_msg.data.data = axis_b_command_buffer;
    axis_b_command_subscriber_msg.data.capacity = COMMAND_BUFFER_LEN;
    axis_b_command_subscriber_msg.data.size = 0;

    RCCHECK(rclc_subscription_init_default(
        &axis_b_command_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        axis_b_command_subscriber_topic));

    rcl_timer_t timer;
    const unsigned int timer_timeout = 100;
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
        &axis_a_command_subscriber,
        &axis_a_command_subscriber_msg,
        axis_a_command_subscriber_callback,
        ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &axis_b_command_subscriber,
        &axis_b_command_subscriber_msg,
        axis_b_command_subscriber_callback,
        ON_NEW_DATA));

    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }

    // free resources
    RCCHECK(rcl_publisher_fini(&axis_a_state_publisher, &node));
    RCCHECK(rcl_publisher_fini(&axis_b_state_publisher, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Setup...");

    snprintf(axis_a_state_publisher_topic, TOPIC_BUFFER_SIZE, "/%s/%s/get_state", ROBOT_NAME, AXIS_A_NAME);
    snprintf(axis_a_command_subscriber_topic, TOPIC_BUFFER_SIZE, "/%s/%s/send_command", ROBOT_NAME, AXIS_A_NAME);
    snprintf(axis_b_state_publisher_topic, TOPIC_BUFFER_SIZE, "/%s/%s/get_state", ROBOT_NAME, AXIS_B_NAME);
    snprintf(axis_b_command_subscriber_topic, TOPIC_BUFFER_SIZE, "/%s/%s/send_command", ROBOT_NAME, AXIS_B_NAME);

    pid_semaphore = xSemaphoreCreateBinary();
    homing_semaphore = xSemaphoreCreateBinary();

    init_control_timer();
    nvs_init();

    gpio_input_init(ENDSTOP_A_PIN);
    gpio_input_init(ENDSTOP_B_PIN);

    i2c_bus_init(AS5600_A_I2C_PORT, AS5600_A_I2C_SDA, AS5600_A_I2C_SCL);
    i2c_bus_init(AS5600_B_I2C_PORT, AS5600_B_I2C_SDA, AS5600_B_I2C_SCL);

    tb6612_motor_init(&wrist.motor_1, TB6612_A_IN1, TB6612_A_IN2, TB6612_A_PWM, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    tb6612_motor_init(&wrist.motor_2, TB6612_B_IN1, TB6612_B_IN2, TB6612_B_PWM, MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);

    wrist.axis_a.pos_ctrl = 0.0f;
    wrist.axis_a.vel_ctrl = 0.0f;

    wrist.axis_b.pos_ctrl = 0.0f;
    wrist.axis_b.vel_ctrl = 0.0f;

    wrist.diff_pwm_ctrl.cb1 = &motor1_cb;
    wrist.diff_pwm_ctrl.cb2 = &motor2_cb;
    wrist.diff_pwm_ctrl.gear_ratio = GEAR_RATIO;

    wrist.diff_pwm_ctrl.common_speed = 0.0f;
    wrist.diff_pwm_ctrl.differential_speed = 0.0f;

    pid_init(&wrist.axis_a.pos_pid, AXIS_A_POSITION_KP, AXIS_A_POSITION_KI, AXIS_A_POSITION_KD, AXIS_A_POSITION_KF, AXIS_A_MAX_SPEED, 1.0f / PID_LOOP_FREQUENCY_HZ);
    pid_init(&wrist.axis_b.pos_pid, AXIS_B_POSITION_KP, AXIS_B_POSITION_KI, AXIS_B_POSITION_KD, AXIS_B_POSITION_KF, AXIS_B_MAX_SPEED, 1.0f / PID_LOOP_FREQUENCY_HZ);
    pid_init(&wrist.axis_a.vel_pid, AXIS_A_VELOCITY_KP, AXIS_A_VELOCITY_KI, AXIS_A_VELOCITY_KD, AXIS_A_VELOCITY_KF, 1.0f, 1.0f / PID_LOOP_FREQUENCY_HZ);
    pid_init(&wrist.axis_b.vel_pid, AXIS_B_VELOCITY_KP, AXIS_B_VELOCITY_KI, AXIS_B_VELOCITY_KD, AXIS_B_VELOCITY_KF, 1.0f, 1.0f / PID_LOOP_FREQUENCY_HZ);

    bool ok1 = as5600_init(&wrist.axis_a.encoder, AS5600_A_I2C_PORT, AS5600_DEFAULT_ADDR, AS5600_A_SCALE_FACTOR, INVERT_AS5600_A, AS5600_A_ENABLE_NVS);
    bool ok2 = as5600_init(&wrist.axis_b.encoder, AS5600_B_I2C_PORT, AS5600_DEFAULT_ADDR, AS5600_B_SCALE_FACTOR, INVERT_AS5600_B, AS5600_B_ENABLE_NVS);

    if (!ok1 || !ok2)
    {
        ESP_LOGE(TAG, "Failed to initialize one or both encoders (ok1=%d, ok2=%d)", ok1, ok2);
        return;
    }

    ESP_LOGI(TAG, "Encoders initialized");

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

    homing_task(&homing_params);

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

    ESP_LOGI(TAG, "Setup Complete");
}
