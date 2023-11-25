#ifndef __FIRE_TASK_H
#define __FIRE_TASK_H
#include "Chassis_movement_task.h"
typedef struct{
    Robot_cmd_t     *Robot_cmd;
    Motor_information_t   Fire_Motor;
    int32_t set_position;             
}fire_task_t;
void Fire_Task(void const *argument);

#endif

