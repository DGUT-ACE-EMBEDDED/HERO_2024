#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "gimbal_struct_variables.h"
#include "gimbal_config.h"

#include "can1_receive.h"
#include "can1_send.h"
#include "can2_receive.h"
#include "can2_send.h"

#include "data_handle.h"//usb���⴮��
#include "cybergear.h"
/*���ݴ�����*/
#define pitch_height_limit 100000
#define pitch_low_limit 0

typedef struct 
{
    //˫�����趨λ��
    struct Gimbal_Set_Angle_t
    {
        fp32    pitch;
        fp32    yaw;
        fp32    last_pitch;
    }Gimbal_Set_Angle;
    
    //imu�����Լ��������
    struct Gimbal_Data_t
    {
        Motor6020_t   Yaw_Data;
        Motor2006_t   Pitch_Data;
        Encoder_t   *pitch_encoder;
        Encoder_t   *yaw_encoder;
        const INS_t   *Imu_c;
        //���ʧ�����
        struct Disconnection_t
        {
            uint16_t *yaw;
            uint16_t *pitch;
        }Disconnection;
        
    }Gimbal_Data;
    
    //ң��������
    RC_ctrl_t   *Gimbal_RC;
    //С�������ݰ�
    GIMBAL_DATA_BAG *AUTO_RC;
    //��̨ģʽ
    gimbal_behaviour_e behaviour;
    
    //PID
    struct Gimbal_PID_t
    {
        //����PID
        pid_parameter_t     PITCH_postion_PID;
        pid_parameter_t     PITCH_speed_PID;
        pid_parameter_t     YAW_postion_PID;
        pid_parameter_t     YAW_speed_PID;
        //����PID
        pid_parameter_t     AUTO_PITCH_postion_PID;
        pid_parameter_t     AUTO_PITCH_speed_PID;
        pid_parameter_t     AUTO_YAW_postion_PID;
        pid_parameter_t     AUTO_YAW_speed_PID;
        first_order_filter_type_t AUTO_PITCH_filter;
    }Gimbal_PID;
    
}Gimbal_t;



void Gimbal_Task(void const *argument);
void gimbal_init(void);
void gimbal_set_angle(void);
#endif
