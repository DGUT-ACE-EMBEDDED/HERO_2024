#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "stdint.h"
#include "FSM_Chassis.h"
#include "pid.h"
#include "bsp_Motor_Encoder.h"
#include "bsp_referee.h"
#include "imu_task.h"
#include "CAN1.h"
/* ************************freertos******************** */
#include "FreeRTOS.h"
#include "freertos.h"
#include "queue.h"
#include "semphr.h"

#define Y  1
#define RF 0
#define LF 1
#define LB 2
#define RB 3

typedef enum
{
	FIRE_ACTION,	   
	FIRE_LOCK_POSITION, //锁住
	FIRE_NO_ACTION,
} fire_state_e;

typedef enum
{
	ALL_OK,
    RF_MISS,
    LF_MISS,
    LB_MISS,
    RB_MISS
} motor_state_e;

/*---全新的数据结构体---*/
/*
 *划分为：必要信息、电机信息、火控信息、PID、SET部分
 */
typedef struct
{
    const RC_ctrl_t *RC;      //遥控器
	REFEREE_t *referee;       //裁判系统
    const INS_t *INS;         //陀螺仪
    fp32 Difference_Angle_between_Chassis_Gimbal;//与底盘角度差
} Necessary_information_t;//必要信息

typedef struct
{
    Encoder_t *Motor_encoder[5];//编码器
    int Motor_encoder_clear_flag;//编码器清除标志位
    /*---电机电调数据---*/
    const Motor_3508 *LF_Motor;
	const Motor_3508 *RF_Motor;
	const Motor_3508 *LB_Motor;
	const Motor_3508 *RB_Motor;
	const Motor_3508 *FIRE_Motor;

    uint32_t *last_call_back_time;//电机失联计时数组
    //const CAN1_t  *CAN1_Chassis;//CAN中断处理中的电机失联数组
    motor_state_e motor_state;//底盘电机在线状态
	
} Motor_information_t;//电机信息
typedef struct
{
    fire_state_e fire_state;
	int32_t fire_set_position;//拨弹盘转子位置
	int one_shoot_flag;//单发标志
	int fire_on_flag;//上弹标志
	int fire_detect_flag;//退弹标志
    
} Fire_information_t;//火控数据
typedef struct
{
    /*---速度PID---*/
    pid_parameter_t z_speed;      //自转速度
    pid_parameter_t LF_Speed_PID;
	pid_parameter_t RF_Speed_PID;
	pid_parameter_t LB_Speed_PID;
	pid_parameter_t RB_Speed_PID;
	pid_parameter_t FIRE_Speed_PID;
	/*---位置PID---*/
	pid_parameter_t	LF_Position_PID;
	pid_parameter_t	RF_Position_PID;	
	pid_parameter_t LB_Position_PID;
	pid_parameter_t RB_Position_PID;
	pid_parameter_t FIRE_Position_PID;	
    
} PID_information_t;//PID数据

typedef struct
{
    Chassis_State_t State;      //底盘行动
    Chassis_Mode_t Mode;        //底盘模式
    
    
    //将兵车视为质点建立地平面坐标系，向右X正向，向上Y正向，以底盘头为Y正向
    /*---速度设定---*/
    fp32 plane_x_speed_set;//俯视地平面横轴，向右为正
    fp32 plane_y_speed_set;//俯视地平面竖轴，向上为正
	fp32 plane_z_speed_set;//俯视地平面旋转轴，逆时针为正
    
	fp32 LF_speed_set;
	fp32 RF_speed_set;
	fp32 LB_speed_set;
	fp32 RB_speed_set;
    
    /*---电流设定---*/
    int16_t LF_current_input;
	int16_t RF_current_input;
	int16_t LB_current_input;
	int16_t RB_current_input;
	int16_t FIRE_current_input;
}  set_information_t;

typedef struct
{
    Necessary_information_t  Necessary_information;
    Motor_information_t      Motor_information;
    Fire_information_t       Fire_information;
    PID_information_t        PID_information;
    set_information_t        Set_information;
}
New_Chassis_Task_t;
/*---全新的数据结构体---*/
typedef struct
{
	int16_t LF_current_input;
	int16_t RF_current_input;
	int16_t LB_current_input;
	int16_t RB_current_input;
	int16_t FIRE_current_input;
	
	pid_parameter_t LF_Speed_PID;
	pid_parameter_t RF_Speed_PID;
	pid_parameter_t LB_Speed_PID;
	pid_parameter_t RB_Speed_PID;
	
	pid_parameter_t	LF_Position_PID;
	pid_parameter_t	RF_Position_PID;	
	pid_parameter_t LB_Position_PID;
	pid_parameter_t RB_Position_PID;
	
	pid_parameter_t FIRE_Speed_PID;
	pid_parameter_t FIRE_Position_PID;	
	
	const Motor_3508 *LF_Motor;
	const Motor_3508 *RF_Motor;
	const Motor_3508 *LB_Motor;
	const Motor_3508 *RB_Motor;
	const Motor_3508 *FIRE_Motor;
	Encoder_t *Motor_encoder[5];
    motor_state_e motor_state;
	
	FSM_Chassis_t *FSM_Chassis;
	RC_ctrl_t     *RC;
	REFEREE_t 		*referee;
	const INS_t *INS;
	const CAN1_t  *CAN1_Chassis;
	
	fire_state_e fire_state;
	int32_t fire_set_position;
	int one_shoot_flag;
	int fire_on_flag;
	int fire_detect_flag;
	
}Chassis_Task_t;

void Task_Chassis(void const *argument);
void Chassis_Init(void);
void Chassis_Motor_PID_Init(void);
void Motion_Calc(void);
void Motor_Online_Detection(void);

/*底盘数据结构体初始化函数*/
void Necessary_information_init(void);
void Motor_information_init(void);
void Fire_information_init(void);
void PID_information_init(void);
void Set_information_init(void);


#endif


