#include "fire_Task.h"
#include "bsp_dr16.h"
#include "gimbal_task.h"
#include "stdlib.h"
#include "string.h"
#include "can1_receive.h"
#include "can1_send.h"
#include "bsp_dr16.h"
#include "bsp_Motor_Encoder.h"
#include "gimbal_config.h"
#include "bsp_referee.h"
/* ************************freertos******************** */
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

gimbal_fire_control_t *fire_control_p;

int fire = 4500;

static gimbal_fire_control_t *fire_task_init(void);
static void fire_pid_calculate(gimbal_fire_control_t *fire_pid_calculate_f);
static void fire_behaviour_choose(gimbal_fire_control_t *fire_behaviour_choose_f);

void fire_Task(void const *argument)
{
	fire_control_p = fire_task_init();
	while (1)
	{
		fire_behaviour_choose(fire_control_p); // 还差裁判系统 TODO:

		fire_pid_calculate(fire_control_p);
		
		can2_gimbal_setmsg(fire_control_p->left_motor.set_current, fire_control_p->right_motor.set_current);
		vTaskDelay(1);
	}
}
gimbal_fire_control_t *fire_task_init(void)
{
	gimbal_fire_control_t *fire_task_init_p;
	fire_task_init_p = malloc(sizeof(gimbal_fire_control_t));
	memset(fire_task_init_p, 0, sizeof(gimbal_fire_control_t));

	// 获得拨弹指针
	fire_task_init_p->right_motor.motor_measure = get_right_motor_measure_point();
	fire_task_init_p->left_motor.motor_measure = get_left_motor_measure_point();
	
	fire_task_init_p->referee = Get_referee_Address();
	fire_task_init_p->fire_rc = RC_Get_RC_Pointer();

	//fire_task_init_p->fire_motor_encoder = Encoder_Init(M2006, 3);

	// fire
	PidInit(&fire_task_init_p->left_motor_speed_pid, 10, 0, 0, Output_Limit);
	PidInit(&fire_task_init_p->right_motor_speed_pid, 10, 0, 0, Output_Limit);
	PidInitMode(&fire_task_init_p->left_motor_speed_pid, Output_Limit, 16000, 0);
	PidInitMode(&fire_task_init_p->right_motor_speed_pid, Output_Limit, 16000, 0);

	fire_task_init_p->fire_behaviour = FIRE_FULL_AUTO;
	fire_task_init_p->feed_buttle = false;

	return fire_task_init_p;
}

void fire_behaviour_choose(gimbal_fire_control_t *fire_behaviour_choose_f)
{
    fire_behaviour_choose_f->left_motor.Speed_Set = FIRE_SPEED_TRY;
    fire_behaviour_choose_f->right_motor.Speed_Set = -FIRE_SPEED_TRY;

////		fire_behaviour_choose_f->right_motor.Speed_Set = -FIRE_SPEED_10;
////	// 裁判系统弹速设置
////	switch (fire_behaviour_choose_f->referee->Robot_Status.shooter_id1_42mm_speed_limit)
////	{
////	case 10:
////		fire_behaviour_choose_f->left_motor.Speed_Set = FIRE_SPEED_10;
////		fire_behaviour_choose_f->right_motor.Speed_Set = -FIRE_SPEED_10;
////		break;
////	case 16:
////		fire_behaviour_choose_f->left_motor.Speed_Set = FIRE_SPEED_16;
////		fire_behaviour_choose_f->right_motor.Speed_Set = -FIRE_SPEED_16;
////		break;
////	default:
////		fire_behaviour_choose_f->left_motor.Speed_Set = fire;
////		fire_behaviour_choose_f->right_motor.Speed_Set = -fire;
////		break;
////	}
}
void fire_pid_calculate(gimbal_fire_control_t *fire_pid_calculate_f)
{
	if(fire_pid_calculate_f->fire_rc->Fire_ON_OFF_FLAG == 1)
	{
		fire_pid_calculate_f->left_motor.set_current = motor_speed_control(&fire_pid_calculate_f->left_motor_speed_pid,
																	   fire_pid_calculate_f->left_motor.Speed_Set,
																	   fire_pid_calculate_f->left_motor.motor_measure->speed);
		fire_pid_calculate_f->right_motor.set_current = motor_speed_control(&fire_pid_calculate_f->right_motor_speed_pid,
																		fire_pid_calculate_f->right_motor.Speed_Set,
																		fire_pid_calculate_f->right_motor.motor_measure->speed);
	}
	else if(fire_pid_calculate_f->fire_rc->Fire_ON_OFF_FLAG == 0)
	{
		fire_pid_calculate_f->left_motor.set_current =0;
		fire_pid_calculate_f->right_motor.set_current =0;
	}
}
const gimbal_fire_control_t **get_fire_control_point(void)
{
	return (const gimbal_fire_control_t **)&fire_control_p;
}
