#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "maths.h"
#include "gimbal_config.h"
#include "virtual_task.h"
#include "filter.h"
#include "lqr.h"
#define abs(x) (x)>=0? (x):-(x)

float Gimbal_pitch = 0.0f;
float Gimbal_yaw = 0.0f;

static void f_GIMBAL_MANUAL(gimbal_control_t *f_GIMBAL_MANUAL_f);
static void f_GIMBAL_AUTOATTACK(gimbal_control_t *f_GIMBAL_AUTOATTACK_f);
static void f_GIMBAL_AUTOBUFF(gimbal_control_t *f_GIMBAL_AUTOBUFF_f);
static float torque_to_voltage_6020(float torque);
static float float_min_distance(float target, float actual, float minValue, float maxValue);
float *get_Gimbal_pitch_point(void)
{
    return &Gimbal_pitch;
}

float *get_Gimbal_yaw_point(void)
{
    return &Gimbal_yaw;
}

void gimbal_behaviour_choose(gimbal_control_t *gimbal_behaviour_choose_f)
{
    gimbal_behaviour_e last_behaviour;
    static gimbal_behaviour_e rc_behaviour = GIMBAL_MANUAL;
    static gimbal_behaviour_e kb_behaviour = GIMBAL_MANUAL;

    // 手柄
    last_behaviour = rc_behaviour;
    switch (gimbal_behaviour_choose_f->Gimbal_RC->rc.s2)
    {
    case RC_SW_UP:
        rc_behaviour = GIMBAL_AUTOATTACK;
        break;
    case RC_SW_MID:
        rc_behaviour = GIMBAL_MANUAL;
        break;
    case RC_SW_DOWN:
        rc_behaviour = GIMBAL_MANUAL;
        break;
    default:
        break;
    }
    // 如果挡位发生改变，设置对应的模式
    if (last_behaviour != rc_behaviour)
    {
        gimbal_behaviour_choose_f->gimbal_behaviour = rc_behaviour;
    }

    // 键鼠
    last_behaviour = kb_behaviour;
    //**c
//    if(gimbal_behaviour_choose_f->Gimbal_RC->kb.bit.C)
//    {
//        kb_behaviour = GIMBAL_MANUAL;
//    }
		if(gimbal_behaviour_choose_f->Gimbal_RC->mouse.press_r)
    {
        kb_behaviour = GIMBAL_AUTOATTACK;
    }
		else
		{
				kb_behaviour = GIMBAL_MANUAL;
		}
    // 如果模式发生改变，设置对应的模式
    if (last_behaviour != kb_behaviour)
    {
        gimbal_behaviour_choose_f->gimbal_behaviour = kb_behaviour;
    }
}

void gimbal_behaviour_react(gimbal_control_t *gimbal_behaviour_react_f)
{
		Gimbal_pitch -= gimbal_behaviour_react_f->Gimbal_RC->mouse.y * MOUSE_PITCH_SPEED;
    Gimbal_pitch += gimbal_behaviour_react_f->Gimbal_RC->rc.ch[1] * RC_PITCH_SPEED;

    Gimbal_yaw -= gimbal_behaviour_react_f->Gimbal_RC->mouse.x * MOUSE_YAW_SPEED;
    Gimbal_yaw -= gimbal_behaviour_react_f->Gimbal_RC->rc.ch[0] * RC_YAW_SPEED;
    switch (gimbal_behaviour_react_f->gimbal_behaviour)
    {
    case GIMBAL_MANUAL:
        f_GIMBAL_MANUAL(gimbal_behaviour_react_f);
        break;
    case GIMBAL_AUTOATTACK:
        f_GIMBAL_AUTOATTACK(gimbal_behaviour_react_f);
        break;
    case GIMBAL_AUTOBUFF:
        f_GIMBAL_AUTOBUFF(gimbal_behaviour_react_f);
        break;
    default:
        break;
    }
		
		value_limit(Gimbal_pitch, PITCH_ANGLE_LIMIT_DOWN, PITCH_ANGLE_LIMIT_UP);
		#if(YAW_CONTROLER == YAW_USE_PID)
				Gimbal_yaw = loop_fp32_constrain(Gimbal_yaw, 0.0f, 360.0f);
		#elif(YAW_CONTROLER == YAW_USE_LQR)
				Gimbal_yaw = loop_fp32_constrain(Gimbal_yaw, -180.0f, 180.0f);
		#endif
}

void f_GIMBAL_MANUAL(gimbal_control_t *f_GIMBAL_MANUAL_f)
{
	
}
void f_GIMBAL_AUTOATTACK(gimbal_control_t *f_GIMBAL_AUTOATTACK_f)
{
	if(((*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_yaw != 0) || ((*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_pitch != 0))
	{
		#ifdef VIRTUAL_DELAY_COMPENSATE
					Gimbal_pitch = (*f_GIMBAL_AUTOATTACK_f->auto_c)->gimbal_use_control[0];
					Gimbal_yaw = (*f_GIMBAL_AUTOATTACK_f->auto_c)->gimbal_use_control[1];
					Gimbal_pitch = first_order_filter(&f_GIMBAL_AUTOATTACK_f->Pitch_c.visual_pitch__first_order_filter,Gimbal_pitch);
					gimbal_clear_virtual_recive();
		#else
					Gimbal_pitch = f_GIMBAL_AUTOATTACK_f->Imu_c->Pitch + (*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_pitch;

					Gimbal_yaw = f_GIMBAL_AUTOATTACK_f->Imu_c->Yaw + (*f_GIMBAL_AUTOATTACK_f->auto_c)->auto_yaw;
					Gimbal_pitch = first_order_filter(&f_GIMBAL_AUTOATTACK_f->Pitch_c.visual_pitch__first_order_filter,Gimbal_pitch);
					gimbal_clear_virtual_recive();
		#endif
	}
}
void f_GIMBAL_AUTOBUFF(gimbal_control_t *f_GIMBAL_AUTOBUFF_f)
{
}
void gimbal_pid_calculate(gimbal_control_t *gimbal_pid_calculate_f)
{
	 /*-----------------------PITCH--------------------------*/
if(gimbal_pid_calculate_f->gimbal_behaviour == GIMBAL_AUTOATTACK || gimbal_pid_calculate_f->gimbal_behaviour == GIMBAL_AUTOBUFF)//自瞄时使用i
				{
    gimbal_pid_calculate_f->Pitch_c.pitch_motor.set_current = 
                    motor_position_speed_control(&gimbal_pid_calculate_f->Pitch_c.pitch_motor_auto_speed_pid,
                    &gimbal_pid_calculate_f->Pitch_c.pitch_motor_auto_position_pid,
				    Gimbal_pitch,
					gimbal_pid_calculate_f->Imu_c->Pitch,
				    gimbal_pid_calculate_f->Pitch_c.pitch_motor.motor_measure->speed);	
				}
				else
				{
			  gimbal_pid_calculate_f->Pitch_c.pitch_motor.set_current = motor_position_speed_control(&gimbal_pid_calculate_f->Pitch_c.pitch_motor_speed_pid,
																																														 &gimbal_pid_calculate_f->Pitch_c.pitch_motor_position_pid,
																																														 Gimbal_pitch,
																																														 gimbal_pid_calculate_f->Imu_c->Pitch,
																																														 gimbal_pid_calculate_f->Pitch_c.pitch_motor.motor_measure->speed);	
				}
	/*-----------------------YAW--------------------------*/
		#if(YAW_CONTROLER == YAW_USE_PID)
				gimbal_pid_calculate_f->Yaw_c.yaw_motor.set_voltage = motor_position_speed_control(&gimbal_pid_calculate_f->Yaw_c.yaw_motor_visu-al_speed_pid,
																																													 &gimbal_pid_calculate_f->Yaw_c.yaw_motor_visual_position_pid,
																																													 // 视当前位置为0，寻目标的劣弧角度即为控制量
																																													 user_abs(Gimbal_yaw - gimbal_pid_calculate_f->Imu_c->Yaw) > 180 ? ((Gimbal_yaw - gimbal_pid_calculate_f->Imu_c->Yaw) > 0 ? ((Gimbal_yaw - gimbal_pid_calculate_f->Imu_c->Yaw) - 360.0f) : (360.0f - (Gimbal_yaw - gimbal_pid_calculate_f->Imu_c->Yaw))) : (Gimbal_yaw - gimbal_pid_calculate_f->Imu_c->Yaw),
																																													 0,
																																													 gimbal_pid_calculate_f->Yaw_c.yaw_motor.motor_measure->speed);
		#elif(YAW_CONTROLER == YAW_USE_LQR)
				gimbal_pid_calculate_f->Yaw_c.motor_target = float_min_distance(Gimbal_yaw,gimbal_pid_calculate_f->Imu_c->Yaw, -180, 180);
				double Yaw_system_state[2] = {((-gimbal_pid_calculate_f->Yaw_c.motor_target) / 57.295779513f), -gimbal_pid_calculate_f->Imu_c->Gyro[2]};
				
				
				LQR_Data_Update(&gimbal_pid_calculate_f->Yaw_c.motor_lqr, Yaw_system_state);
				LQR_Calculate(&gimbal_pid_calculate_f->Yaw_c.motor_lqr);
				if(gimbal_pid_calculate_f->gimbal_behaviour == GIMBAL_AUTOATTACK || gimbal_pid_calculate_f->gimbal_behaviour == GIMBAL_AUTOBUFF)//自瞄时使用i
				{
					PidCalculate(&gimbal_pid_calculate_f->Yaw_c.yaw_lqr_only_i_pid, Gimbal_yaw, gimbal_pid_calculate_f->Imu_c->Yaw );
				}
				else
				{
					pid_clear(&gimbal_pid_calculate_f->Yaw_c.yaw_lqr_only_i_pid);
				}
				gimbal_pid_calculate_f->Yaw_c.motor_lqr.Output[0] = sliding_mean_filter(&gimbal_pid_calculate_f->Yaw_c.motor_filter, gimbal_pid_calculate_f->Yaw_c.motor_lqr.Output[0],10);
				gimbal_pid_calculate_f->Yaw_c.motor_output = torque_to_voltage_6020(gimbal_pid_calculate_f->Yaw_c.motor_lqr.Output[0]) - gimbal_pid_calculate_f->Yaw_c.yaw_lqr_only_i_pid.out;
				gimbal_pid_calculate_f->Yaw_c.yaw_motor.set_current = abs_limit(gimbal_pid_calculate_f->Yaw_c.motor_output, 25000);
		#endif
}
float float_min_distance(float target, float actual, float minValue, float maxValue)
{
	if (maxValue < minValue)
    {
        return 0;
    }
	
	target = loop_float_constrain(target, minValue,maxValue);
	
	if (abs(target - actual) > (maxValue - minValue) / 2.0f)
	{
		if(maxValue - actual < (maxValue - minValue) / 2.0f)
		{
			return maxValue - actual + target - minValue;
		}
		else
		{
			return minValue - actual + target - maxValue;
		}
	}
	else 
	{
		return target - actual;
	}
	
}
float torque_to_voltage_6020(float torque)
{
	float voltage = 0.0f;
		
	float current = torque / 0.741f * 10000;
	voltage = (current - 128.3507f) / 0.7778f;
	voltage = abs_limit(voltage,25000);
	
	return voltage;
		
}

///*
//功能    若电机堵塞,电流置零

//参数    1.电机相关的结构体
//        2.堵塞检测模式  (1.基于速度检测;2.基于转子位置；3.基于陀螺仪)
//        3.堵塞容忍度    (数值越大检测越慢)
//*/
//void gimbal_pitch_stall_detection(gimbal_control_t *gimbal_pid_calculate_f,int choose,int stall_time)
//{

//    if (choose == 1)//基于速度检测
//    {
//        
//        //第一次运行速度检测，全部初始化
//        static int times_of_First_detection_speed = 0;
//        if (!times_of_First_detection_speed)
//        {
//            for (int i = 0; i < stall_time; i++)
//                gimbal_pid_calculate_f->Pitch_c.pitch_motor.pitch_speed[i] = 32767;//int16的最大值
//        }
//        
//        //搬运历史电机转速数据   
//        gimbal_pid_calculate_f->Pitch_c.pitch_motor.pitch_speed[0] = gimbal_pid_calculate_f->Pitch_c.pitch_motor.motor_measure->speed;
//        for (int i = 1; i < stall_time; i++)
//        {
//            gimbal_pid_calculate_f->Pitch_c.pitch_motor.pitch_speed[i] = gimbal_pid_calculate_f->Pitch_c.pitch_motor.pitch_speed[i-1];
//        }
//        
//        if (times_of_First_detection_speed < stall_time)
//        {
//            times_of_First_detection_speed++;
//            return;
//        }
//        
//        int zero_speed = 0;
//        for (int i = 0; i < stall_time; i++)
//        {
//            int normal_change = 10;
//            //电机速度连续为0,基于可能会有杂波导致数字浮动，应设置合理的检测值 change
//            if (gimbal_pid_calculate_f->Pitch_c.pitch_motor.pitch_speed[i] <= normal_change)    zero_speed++;
//        }
//        if (zero_speed == stall_time)    gimbal_pid_calculate_f->Pitch_c.pitch_motor.set_current = 0;//pitch轴电机堵死，电流紧急清零
//    }
//    if (choose == 2)//基于电机转子检测
//    {
//        
//        //第一次运行电子转子位置检测，全部初始化
//        static int times_of_detection_position = 0;
//        if (times_of_detection_position)
//        {
//            for (int i = 0; i < stall_time; i++)
//                gimbal_pid_calculate_f->Pitch_c.pitch_motor.pitch_postion[i] = 65535;//uint16的最大值
//        }
//        
//        //搬运历史电机转子位置数据
//        gimbal_pid_calculate_f->Pitch_c.pitch_motor.pitch_postion[0] = gimbal_pid_calculate_f->Pitch_c.pitch_motor.motor_measure->position;
//        for (int i = 1; i < stall_time; i++)
//        {
//            gimbal_pid_calculate_f->Pitch_c.pitch_motor.pitch_postion[i] = gimbal_pid_calculate_f->Pitch_c.pitch_motor.pitch_postion[i-1];
//        }
//        
//        if (times_of_detection_position < stall_time)
//        {
//            times_of_detection_position++;
//            return;
//        }
//        
//        int no_change_postion = 0;
//        for (int i = 1; i < stall_time; i++)
//        {
//            int normal_change = 10;
//            //电机转子位置连续未改变,基于可能会有杂波导致数字浮动，应设置合理的检测值 change
//            if (gimbal_pid_calculate_f->Pitch_c.pitch_motor.pitch_postion[i] - gimbal_pid_calculate_f->Pitch_c.pitch_motor.pitch_postion[i-1] < normal_change)    no_change_postion++;
//        }
//        if (no_change_postion == stall_time-1)    gimbal_pid_calculate_f->Pitch_c.pitch_motor.set_current = 0;//pitch轴电机堵死，电流紧急清零
//    }
//    if (choose == 3)//基于imu检测
//    {
//        
//        //第一次运行imu检测，全部初始化
//        static int time_of_detection_imu = 1;
//        if (!time_of_detection_imu)
//        {
//            for (int i = 0; i < stall_time; i++)
//                gimbal_pid_calculate_f->Pitch_c.pitch_motor.pitch_imu[i] = 0;//uint16的最大值
//        }
//        

//        //搬运历史imu数据
//        gimbal_pid_calculate_f->Pitch_c.pitch_motor.pitch_imu[0] = gimbal_pid_calculate_f->Imu_c->Pitch;
//        for (int i = 1; i < stall_time; i++)
//        {
//            gimbal_pid_calculate_f->Pitch_c.pitch_motor.pitch_imu[i] = gimbal_pid_calculate_f->Pitch_c.pitch_motor.pitch_imu[i-1];
//        }
//        
//        
//        if (time_of_detection_imu < stall_time)
//        {
//            time_of_detection_imu++;
//            return;
//        }
//        
//        int no_change_imu = 0;
//        for (int i = 1; i < stall_time; i++)
//        {
//            //电机转子位置连续未改变,基于可能会有杂波导致数字浮动，应设置合理的检测值 change
//            int normal_close = 10;
//            if (gimbal_pid_calculate_f->Pitch_c.pitch_motor.pitch_imu[i] - gimbal_pid_calculate_f->Pitch_c.pitch_motor.pitch_imu[i-1] < normal_close)    no_change_imu++;
//        }
//        if (no_change_imu == stall_time-1)    gimbal_pid_calculate_f->Pitch_c.pitch_motor.set_current = 0;//pitch轴电机堵死，电流紧急清零
//    }
//    
//}
