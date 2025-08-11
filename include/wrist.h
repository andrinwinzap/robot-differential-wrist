#ifndef WRIST_H
#define WRIST_H

#include "as5600.h"
#include "pid.h"
#include "tb6612.h"
#include "diff_speed_ctrl.h"

typedef struct
{
    volatile float pos;
    volatile float vel;
    volatile float pos_ctrl;
    volatile float vel_ctrl;
    as5600_t encoder;
    pid_controller_t vel_pid;
    pid_controller_t pos_pid;
} axis_t;

typedef struct
{
    axis_t axis_a;
    axis_t axis_b;
    tb6612_motor_t motor_1;
    tb6612_motor_t motor_2;
    diff_speed_ctrl_t diff_pwm_ctrl;
} wrist_t;

#endif // WRIST_H