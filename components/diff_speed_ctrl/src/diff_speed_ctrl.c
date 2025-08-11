#include "diff_speed_ctrl.h"
#include <math.h>

void update_outputs(diff_speed_ctrl_t *ctrl)
{
    float add_scaled = ctrl->common_speed / ctrl->gear_ratio;

    float speed1 = add_scaled + ctrl->differential_speed;
    float speed2 = add_scaled - ctrl->differential_speed;

    float max_abs = fmaxf(fabsf(speed1), fabsf(speed2));

    if (max_abs > 1)
    {
        speed1 /= max_abs;
        speed2 /= max_abs;
    }

    if (ctrl->cb1)
        ctrl->cb1(speed1);
    if (ctrl->cb2)
        ctrl->cb2(speed2);
}

void set_comm_pwm(diff_speed_ctrl_t *ctrl, float speed)
{
    ctrl->common_speed = speed;
    update_outputs(ctrl);
}

void set_diff_pwm(diff_speed_ctrl_t *ctrl, float speed)
{
    ctrl->differential_speed = speed;
    update_outputs(ctrl);
}
