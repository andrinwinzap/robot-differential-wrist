#ifndef HOME_H
#define HOME_H

#include "as5600.h"
#include "config.h"

void home(pid_position_ctrl_t *pid_position_ctrl, as5600_t *encoderA, as5600_t *encoderB);

#endif // HOME_H