#ifndef DIFF_SPEED_CTRL_H
#define DIFF_SPEED_CTRL_H

typedef void (*speed_cb_t)(float speed);

typedef struct
{
    speed_cb_t cb1;
    speed_cb_t cb2;
    float gear_ratio;
    float common_speed;
    float differential_speed;
} diff_speed_ctrl_t;

void set_common_speed(diff_speed_ctrl_t *ctrl, float speed);

void set_differential_speed(diff_speed_ctrl_t *ctrl, float speed);

void update_outputs(diff_speed_ctrl_t *ctrl);

#endif // DIFF_SPEED_CTRL_H
