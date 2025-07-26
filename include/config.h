#ifndef CONFIG_H
#define CONFIG_H

#define ENDSTOP_A_PIN GPIO_NUM_11
#define ENDSTOP_B_PIN GPIO_NUM_8
#define HOMING_SPEED 1.5 // rad/s

#define I2C_MASTER_NUM_1 I2C_NUM_0
#define I2C_MASTER_SDA_IO_1 10
#define I2C_MASTER_SCL_IO_1 9

#define I2C_MASTER_NUM_0 I2C_NUM_1
#define I2C_MASTER_SDA_IO_0 18
#define I2C_MASTER_SCL_IO_0 17

#define I2C_FREQ_HZ 400000

#define COMMON_GEAR_RATIO

#define PID_LOOP_FREQUENCY 1000
#define PID_LOG_FREQUENCY_HZ 10
#define PID_FREQ_ALPHA 0.9f
#define PID_LOOP_TIME_ALPHA 0.9f

#define CTRL_SIGNAL_THRESHOLD 0.1f

#define TIMER_GROUP TIMER_GROUP_0
#define TIMER_IDX TIMER_0

typedef struct
{
    float differential;
    float common;
} pid_position_ctrl_t;

#endif // CONFIG_H