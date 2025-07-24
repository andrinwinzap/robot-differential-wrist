// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "tb6612.h"

// tb6612_motor_t motorA = {
//     .in1_pin = GPIO_NUM_4,
//     .in2_pin = GPIO_NUM_5,
//     .pwm_pin = GPIO_NUM_6,
//     .unit = MCPWM_UNIT_0,
//     .timer = MCPWM_TIMER_0,
//     .op = MCPWM_OPR_A,
//     .max_rpm = 40.0f,
// };

// tb6612_motor_t motorB = {
//     .in1_pin = GPIO_NUM_12,
//     .in2_pin = GPIO_NUM_13,
//     .pwm_pin = GPIO_NUM_14,
//     .unit = MCPWM_UNIT_0,
//     .timer = MCPWM_TIMER_1,
//     .op = MCPWM_OPR_A,
//     .max_rpm = 40.0f,
// };

// void app_main(void)
// {
//     tb6612_motor_init(&motorA);
//     tb6612_motor_init(&motorB);

//     while (1)
//     {
//         printf("motorA: forward, motorB: reverse\n");
//         tb6612_motor_set_speed(&motorA, 3.14f); // forward
//         tb6612_motor_set_speed(&motorB, -2.0f); // reverse
//         vTaskDelay(pdMS_TO_TICKS(2000));

//         printf("Stopping both motors\n");
//         tb6612_motor_set_speed(&motorA, 0.0f);
//         tb6612_motor_set_speed(&motorB, 0.0f);
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "as5600.h"
#include "esp_log.h"
#include "tb6612.h"

#define TAG "AS5600_DUAL"

#define I2C_MASTER_NUM_0 I2C_NUM_0
#define I2C_MASTER_SDA_IO_0 18
#define I2C_MASTER_SCL_IO_0 17

#define I2C_MASTER_NUM_1 I2C_NUM_1
#define I2C_MASTER_SDA_IO_1 10
#define I2C_MASTER_SCL_IO_1 9

#define I2C_FREQ_HZ 400000

#define ENDSTOP_PIN GPIO_NUM_11
#define HALL_PIN GPIO_NUM_8

tb6612_motor_t motorA, motorB;
as5600_t encoder1, encoder2;

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

void app_main(void)
{
    gpio_input_init(ENDSTOP_PIN);
    gpio_input_init(HALL_PIN);

    i2c_bus_init(I2C_MASTER_NUM_0, I2C_MASTER_SDA_IO_0, I2C_MASTER_SCL_IO_0);
    i2c_bus_init(I2C_MASTER_NUM_1, I2C_MASTER_SDA_IO_1, I2C_MASTER_SCL_IO_1);

    tb6612_motor_init(&motorA, GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_6, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 40.0f);
    tb6612_motor_init(&motorB, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14, MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 40.0f);

    tb6612_motor_set_speed(&motorA, 3.14f);
    tb6612_motor_set_speed(&motorB, -2.0f);

    bool ok1 = as5600_init(&encoder1, I2C_MASTER_NUM_0, AS5600_DEFAULT_ADDR, 0.1f, 0.01f, 1.0f);
    bool ok2 = as5600_init(&encoder2, I2C_MASTER_NUM_1, AS5600_DEFAULT_ADDR, 0.1f, 0.01f, 1.0f);

    if (!ok1 || !ok2)
    {
        ESP_LOGE(TAG, "Failed to initialize one or both encoders (ok1=%d, ok2=%d)", ok1, ok2);
        return;
    }

    ESP_LOGI(TAG, "Encoders initialized on I2C_NUM_0 and I2C_NUM_1.");

    while (1)
    {
        int endstop = gpio_get_level(ENDSTOP_PIN);
        int hall = !gpio_get_level(HALL_PIN);

        as5600_update(&encoder1);
        as5600_update(&encoder2);

        float pos1 = as5600_get_position(&encoder1);
        float vel1 = as5600_get_velocity(&encoder1);
        float pos2 = as5600_get_position(&encoder2);
        float vel2 = as5600_get_velocity(&encoder2);

        ESP_LOGI(TAG, "E1: %.3frad %.3frad/s | E2: %.3frad %.3frad/s | Endstop: %d | Hall: %d", pos1, vel1, pos2, vel2, endstop, hall);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
