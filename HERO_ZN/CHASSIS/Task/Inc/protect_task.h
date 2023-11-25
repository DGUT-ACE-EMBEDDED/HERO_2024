#ifndef __PROTECT_TASK_H
#define __PROTECT_TASK_H

#include "stdint.h"
#include "stm32f4xx.h" 
#include "main.h"
#include "cmsis_os.h"
#include "cmsis_armcc.h"

void Protect_Task(void *pvParameters);
extern uint32_t (*rc_lost_time)(void);

extern void out_of_control(void);
extern void normal_control(void);

#endif

