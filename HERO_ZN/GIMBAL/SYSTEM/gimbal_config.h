/*code is far away from bug with the animal protecting
 *  ��������������
 *�����ߩ��������ߩ�
 *������������������ ��
 *������������������
 *�����ש������ס���
 *������������������
 *���������ߡ�������
 *������������������
 *������������������
 *��������������PC��BJ����
 *��������������������BUG��
 *����������������������
 *���������������������ǩ�
 *������������������������
 *���������������ש�����
 *���������ϩϡ����ϩ�
 *���������ߩ������ߩ�
 *������
 */

// #include "gimbal_config.h"
#ifndef __GIMBAL_CONFIG_H
#define __GIMBAL_CONFIG_H

#define FIRE_WORK

//#define VIRTUAL_DELAY_COMPENSATE

#define YAW_USE_PID 			0
#define YAW_USE_LQR 			1
#define YAW_CONTROLER 		YAW_USE_LQR

#define PITCH_ZERO_OFFSET 100.0f //p����λƫ��ֵ

/**********************pitch��PID����**********************/
#define GIMBAL_PITCH_P_P 1000.0f
#define GIMBAL_PITCH_P_I 0.0f
#define GIMBAL_PITCH_P_D 100.0f

#define GIMBAL_PITCH_S_P 10.0f
#define GIMBAL_PITCH_S_I 0.0f
#define GIMBAL_PITCH_S_D 0.0f

/**********************pitch������PID����**********************/
#define GIMBAL_PITCH_visual_P_P 800.0f
#define GIMBAL_PITCH_visual_P_I 0.08f
#define GIMBAL_PITCH_visual_P_D 0.0f

#define GIMBAL_PITCH_visual_S_P 35.0f
#define GIMBAL_PITCH_visual_S_I 0.0f
#define GIMBAL_PITCH_visual_S_D 0.0f

/**********************Yaw��PID����**********************/
#define GIMBAL_YAW_P_P 300.0f
#define GIMBAL_YAW_P_I 0.0f
#define GIMBAL_YAW_P_D 100.0f

#define GIMBAL_YAW_S_P 10.0f
#define GIMBAL_YAW_S_I 0.0f
#define GIMBAL_YAW_S_D 0.0f

#define GIMBAL_YAW_visual_P_P 0.0f
#define GIMBAL_YAW_visual_P_I 5.0f
#define GIMBAL_YAW_visual_P_D 0.0f

#define GIMBAL_YAW_visual_S_P 1.0f
#define GIMBAL_YAW_visual_S_I 0.0f
#define GIMBAL_YAW_visual_S_D 0.0f
/**********************��̨pitch�Ƕ�����**********************/
#define PITCH_ANGLE_LIMIT_UP 25.0f
#define PITCH_ANGLE_LIMIT_DOWN -18.0f

/**********************�������ң���ٶ�����**********************/
#define MOUSE_YAW_SPEED 0.011f   //���yaw���ٶ�����
#define MOUSE_PITCH_SPEED 0.009f //���pitch���ٶ�����
#define RC_YAW_SPEED 0.0003f     //ң����yaw���ٶ�����
#define RC_PITCH_SPEED 0.0005f   //ң����pitch���ٶ�����

#endif
