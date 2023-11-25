#ifndef __FIRE_TASK_H
#define __FIRE_TASK_H
#include "gimbal_struct_variables.h"
void fire_Task(void const *argument);
const gimbal_fire_control_t **get_fire_control_point(void);

#define FIRE_SPEED_0 0
#define FIRE_SPEED_TRY 10
#define FIRE_SPEED_10 2550
#define FIRE_SPEED_16 4500

#endif
