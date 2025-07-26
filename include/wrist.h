#ifndef WRIST_H
#define WRIST_H

#include "as5600.h"
#include "pid.h"
#include "tb6612.h"
#include "diff_speed_ctrl.h"

typedef struct
{
    float pos_ctrl;
    float speed_ctrl;
    as5600_t encoder;
    pid_controller_t pid;
} axis_t;

typedef struct
{
    axis_t axis_a;
    axis_t axis_b;
    tb6612_motor_t motor_1;
    tb6612_motor_t motor_2;
    diff_speed_ctrl_t diff_speed_ctrl;
} wrist_t;

#endif // WRIST_H