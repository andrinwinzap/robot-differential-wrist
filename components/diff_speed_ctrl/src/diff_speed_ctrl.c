#include "diff_speed_ctrl.h"

void update_outputs(diff_speed_ctrl_t *ctrl)
{
    if (ctrl->cb1)
        ctrl->cb1(ctrl->common_speed + ctrl->differential_speed);

    if (ctrl->cb2)
        ctrl->cb2(ctrl->common_speed - ctrl->differential_speed);
}

void set_common_speed(diff_speed_ctrl_t *ctrl, float speed)
{
    ctrl->common_speed = speed;
    update_outputs(ctrl);
}

void set_differential_speed(diff_speed_ctrl_t *ctrl, float speed)
{
    ctrl->differential_speed = speed;
    update_outputs(ctrl);
}
