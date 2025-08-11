#ifndef CONFIG_H
#define CONFIG_H

#define COARSE_HOMING_SPEED 1.5 // rad/s
#define FINE_HOMING_SPEED 0.5   // rad/s
#define POSITION_TOLERANCE 1.0e-2

#define I2C_FREQ_HZ 400000

#define COMMON_GEAR_RATIO 20.0f / 29.0f

#define PID_LOOP_FREQUENCY 100
#define PID_LOG_FREQUENCY_HZ 10
#define PID_FREQ_ALPHA 0.9f
#define PID_LOOP_TIME_ALPHA 0.9f

#define VEL_SIG_THRESHOLD 0.0f

#define TIMER_GROUP TIMER_GROUP_0
#define TIMER_IDX TIMER_0

static const char *robot_name = "robot";

// AXIS A
static const char *axis_a_name = "joint_5";

#define ENDSTOP_A_PIN GPIO_NUM_11
#define A_AXIS_MAX M_PI / 2.0f - 0.03
#define A_AXIS_MIN -M_PI / 2.0f + 0.03
#define ENDSTOP_A_POSITION A_AXIS_MAX - 0.01

#define AS5600_A_I2C_PORT I2C_NUM_0
#define AS5600_A_I2C_SDA 18
#define AS5600_A_I2C_SCL 17
#define AS5600_A_VELOCITY_FILTER_ALPHA 0.1f
#define AS5600_A_SCALE_FACTOR 1.0f
#define INVERT_AS5600_A true
#define AS5600_A_ENABLE_NVS true

#define TB6612_A_IN1 GPIO_NUM_4
#define TB6612_A_IN2 GPIO_NUM_5
#define TB6612_A_PWM GPIO_NUM_6

#define AXIS_A_POSITION_KP 1.0f
#define AXIS_A_POSITION_KI 0.0f
#define AXIS_A_POSITION_KD 0.0f
#define AXIS_A_POSITION_KF 1.0f

#define AXIS_A_VELOCITY_KP 1.0f
#define AXIS_A_VELOCITY_KI 2.0f
#define AXIS_A_VELOCITY_KD 0.0f
#define AXIS_A_VELOCITY_KF 0.4f

// AXIS B
static const char *axis_b_name = "joint_6";

#define ENDSTOP_B_PIN GPIO_NUM_8
#define B_AXIS_MAX M_PI
#define B_AXIS_MIN 0
#define ENDSTOP_B_POSITION M_PI / 9.0f

#define AS5600_B_I2C_PORT I2C_NUM_1
#define AS5600_B_I2C_SDA 10
#define AS5600_B_I2C_SCL 9
#define AS5600_B_VELOCITY_FILTER_ALPHA 0.1f
#define AS5600_B_SCALE_FACTOR 1.0f
#define INVERT_AS5600_B true
#define AS5600_B_ENABLE_NVS true

#define AXIS_B_POSITION_KP 2.0f
#define AXIS_B_POSITION_KI .0f
#define AXIS_B_POSITION_KD 0.0f
#define AXIS_B_POSITION_KF 1.0f

#define AXIS_B_VELOCITY_KP 2.0f
#define AXIS_B_VELOCITY_KI 2.0f
#define AXIS_B_VELOCITY_KD 0.0f
#define AXIS_B_VELOCITY_KF 0.4f

#define TB6612_B_IN1 GPIO_NUM_12
#define TB6612_B_IN2 GPIO_NUM_13
#define TB6612_B_PWM GPIO_NUM_14

#endif // CONFIG_H