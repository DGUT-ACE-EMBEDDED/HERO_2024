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
	FIRE_LOCK_POSITION, //��ס
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

/*---ȫ�µ����ݽṹ��---*/
/*
 *����Ϊ����Ҫ��Ϣ�������Ϣ�������Ϣ��PID��SET����
 */
typedef struct
{
    const RC_ctrl_t *RC;      //ң����
	REFEREE_t *referee;       //����ϵͳ
    const INS_t *INS;         //������
    fp32 Difference_Angle_between_Chassis_Gimbal;//����̽ǶȲ�
} Necessary_information_t;//��Ҫ��Ϣ

typedef struct
{
    Encoder_t *Motor_encoder[5];//������
    int Motor_encoder_clear_flag;//�����������־λ
    /*---����������---*/
    const Motor_3508 *LF_Motor;
	const Motor_3508 *RF_Motor;
	const Motor_3508 *LB_Motor;
	const Motor_3508 *RB_Motor;
	const Motor_3508 *FIRE_Motor;

    uint32_t *last_call_back_time;//���ʧ����ʱ����
    //const CAN1_t  *CAN1_Chassis;//CAN�жϴ����еĵ��ʧ������
    motor_state_e motor_state;//���̵������״̬
	
} Motor_information_t;//�����Ϣ
typedef struct
{
    fire_state_e fire_state;
	int32_t fire_set_position;//������ת��λ��
	int one_shoot_flag;//������־
	int fire_on_flag;//�ϵ���־
	int fire_detect_flag;//�˵���־
    
} Fire_information_t;//�������
typedef struct
{
    /*---�ٶ�PID---*/
    pid_parameter_t z_speed;      //��ת�ٶ�
    pid_parameter_t LF_Speed_PID;
	pid_parameter_t RF_Speed_PID;
	pid_parameter_t LB_Speed_PID;
	pid_parameter_t RB_Speed_PID;
	pid_parameter_t FIRE_Speed_PID;
	/*---λ��PID---*/
	pid_parameter_t	LF_Position_PID;
	pid_parameter_t	RF_Position_PID;	
	pid_parameter_t LB_Position_PID;
	pid_parameter_t RB_Position_PID;
	pid_parameter_t FIRE_Position_PID;	
    
} PID_information_t;//PID����

typedef struct
{
    Chassis_State_t State;      //�����ж�
    Chassis_Mode_t Mode;        //����ģʽ
    
    
    //��������Ϊ�ʵ㽨����ƽ������ϵ������X��������Y�����Ե���ͷΪY����
    /*---�ٶ��趨---*/
    fp32 plane_x_speed_set;//���ӵ�ƽ����ᣬ����Ϊ��
    fp32 plane_y_speed_set;//���ӵ�ƽ�����ᣬ����Ϊ��
	fp32 plane_z_speed_set;//���ӵ�ƽ����ת�ᣬ��ʱ��Ϊ��
    
	fp32 LF_speed_set;
	fp32 RF_speed_set;
	fp32 LB_speed_set;
	fp32 RB_speed_set;
    
    /*---�����趨---*/
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
/*---ȫ�µ����ݽṹ��---*/
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

/*�������ݽṹ���ʼ������*/
void Necessary_information_init(void);
void Motor_information_init(void);
void Fire_information_init(void);
void PID_information_init(void);
void Set_information_init(void);


#endif


