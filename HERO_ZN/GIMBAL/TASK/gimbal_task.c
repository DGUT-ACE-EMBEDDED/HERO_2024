#include "gimbal_task.h"

Gimbal_t   Gimbal;//云台数据结构体
int pitch_disconnection_flag = 0;
int yaw_disconnection_flag = 0;
fp32 Gimbal_yaw_last_angle = 0;
/**
 * @brief  云台电机掉线检测
 * @param  
 * @retval void
 */
void Online_detection(void)
{
    *Gimbal.Gimbal_Data.Disconnection.pitch += 1;
    *Gimbal.Gimbal_Data.Disconnection.yaw += 1;
    
    if (*Gimbal.Gimbal_Data.Disconnection.pitch >= 60000)
    {
        *Gimbal.Gimbal_Data.Disconnection.pitch = 60000;//防止数值溢出
        pitch_disconnection_flag = 1;
    }
    else    pitch_disconnection_flag = 0;
    
    if (*Gimbal.Gimbal_Data.Disconnection.yaw >= 60000)
    {
        *Gimbal.Gimbal_Data.Disconnection.yaw = 60000;//防止数值溢出
        yaw_disconnection_flag = 1;
    }
    else    yaw_disconnection_flag = 0;
}

/**
 * @brief  云台初始化
 * @param  
 * @retval void
 */
void Gimbal_init(void)
{
    /*-------------------------------- 初始化云台角度 --------------------------------*/
    Gimbal.Gimbal_Set_Angle.pitch = 0;
    Gimbal.Gimbal_Set_Angle.yaw   = 0;
    /*-------------------------------- 获取各指针 --------------------------------*/
    //遥控器
    Gimbal.Gimbal_RC = RC_Get_RC_Pointer();
    //自瞄数据包
    Gimbal.AUTO_RC = RC_Get_AUTO_Pointer();
    //电机数据
    Gimbal.Gimbal_Data.Pitch_Data.motor_measure = get_pitch_motor_measure_point();
    Gimbal.Gimbal_Data.Yaw_Data.motor_measure = get_yaw_motor_measure_point();
    //陀螺仪
    Gimbal.Gimbal_Data.Imu_c = get_imu_control_point();
    //编码器
    Gimbal.Gimbal_Data.pitch_encoder = get_pitch_motor_encoder_point();
    Gimbal.Gimbal_Data.yaw_encoder = get_yaw_motor_encoder_point();
    //失联计时
    Gimbal.Gimbal_Data.Disconnection.yaw = get_yaw_motor_disconnection_point();
    Gimbal.Gimbal_Data.Disconnection.pitch = get_pitch_motor_disconnection_point();
    /*-------------------------------- 初始化编码器 --------------------------------*/
    Encoder_Init(Gimbal.Gimbal_Data.pitch_encoder,M2006);
    Encoder_Init(Gimbal.Gimbal_Data.yaw_encoder,GM6020);
    /*-------------------------------- 初始化 PID --------------------------------*/
    
    //P轴
    PidInit(&Gimbal.Gimbal_PID.PITCH_speed_PID, GIMBAL_PITCH_S_P, GIMBAL_PITCH_S_I, GIMBAL_PITCH_S_D, Integral_Limit | Output_Limit | OutputFilter);
    PidInit(&Gimbal.Gimbal_PID.PITCH_postion_PID, GIMBAL_PITCH_P_P, GIMBAL_PITCH_P_I, GIMBAL_PITCH_P_D, Integral_Limit | Output_Limit | StepIn);
    
    PidInitMode(&Gimbal.Gimbal_PID.PITCH_speed_PID, Output_Limit, 10000, 0);
    PidInitMode(&Gimbal.Gimbal_PID.PITCH_speed_PID, Integral_Limit, 10000, 0);
	PidInitMode(&Gimbal.Gimbal_PID.PITCH_speed_PID, OutputFilter, 0.9, 0);
    
    PidInitMode(&Gimbal.Gimbal_PID.PITCH_postion_PID, Integral_Limit, 5, 0);
    PidInitMode(&Gimbal.Gimbal_PID.PITCH_postion_PID, Output_Limit, 10000, 0);
    PidInitMode(&Gimbal.Gimbal_PID.PITCH_postion_PID, StepIn, 30, 0);
		
    // Y轴
    PidInit(&Gimbal.Gimbal_PID.YAW_speed_PID, GIMBAL_YAW_S_P, GIMBAL_YAW_S_I, GIMBAL_YAW_S_D, Output_Limit | Integral_Limit);
    PidInit(&Gimbal.Gimbal_PID.YAW_postion_PID, GIMBAL_YAW_P_P,GIMBAL_YAW_P_I, GIMBAL_YAW_P_D, Integral_Limit | Output_Limit | StepIn);
    
    PidInitMode(&Gimbal.Gimbal_PID.YAW_speed_PID, Output_Limit, 30000, 0);
    PidInitMode(&Gimbal.Gimbal_PID.YAW_speed_PID, Integral_Limit, 10000, 0);
    
    PidInitMode(&Gimbal.Gimbal_PID.YAW_postion_PID, StepIn, 60, 0);
    PidInitMode(&Gimbal.Gimbal_PID.YAW_postion_PID, Integral_Limit, 300, 0);
    PidInitMode(&Gimbal.Gimbal_PID.YAW_postion_PID, Output_Limit, 30000, 0);
    
    //P轴自瞄
	PidInit(&Gimbal.Gimbal_PID.AUTO_PITCH_speed_PID, GIMBAL_PITCH_visual_S_P, GIMBAL_PITCH_visual_S_I, GIMBAL_PITCH_visual_S_D, Integral_Limit | Output_Limit);
	PidInit(&Gimbal.Gimbal_PID.AUTO_PITCH_postion_PID, GIMBAL_PITCH_visual_P_P, GIMBAL_PITCH_visual_P_I, GIMBAL_PITCH_visual_P_D, Output_Limit);	
	
    PidInitMode(&Gimbal.Gimbal_PID.AUTO_PITCH_speed_PID, Output_Limit, 10000, 0);
    PidInitMode(&Gimbal.Gimbal_PID.AUTO_PITCH_speed_PID, Integral_Limit, 10000, 0);		
	
    PidInitMode(&Gimbal.Gimbal_PID.AUTO_PITCH_postion_PID, Output_Limit, 10000, 0);

	//Y轴自瞄
    PidInit(&Gimbal.Gimbal_PID.AUTO_YAW_speed_PID, GIMBAL_YAW_visual_S_P, GIMBAL_YAW_visual_S_I, GIMBAL_YAW_visual_S_D, Output_Limit | Integral_Limit );
    PidInit(&Gimbal.Gimbal_PID.AUTO_YAW_postion_PID, GIMBAL_YAW_visual_P_P, GIMBAL_YAW_visual_P_I, GIMBAL_YAW_visual_P_D, Integral_Limit | Output_Limit );		
	
    PidInitMode(&Gimbal.Gimbal_PID.AUTO_YAW_speed_PID, Output_Limit, 30000, 0);
    PidInitMode(&Gimbal.Gimbal_PID.AUTO_YAW_speed_PID, Integral_Limit, 10000, 0);
    
    PidInitMode(&Gimbal.Gimbal_PID.AUTO_YAW_postion_PID, Integral_Limit, 5, 0);
    PidInitMode(&Gimbal.Gimbal_PID.AUTO_YAW_postion_PID, Output_Limit, 30000, 0);
    /*-------------------------------- 初始化 滤波器 --------------------------------*/
    first_order_filter_init(&Gimbal.Gimbal_PID.AUTO_PITCH_filter,0.03);
    /*-------------------------------- 初始化P轴编码器 --------------------------------*/
    while (Gimbal.Gimbal_Data.Imu_c->Pitch != 0)
    {
        Gimbal.Gimbal_Data.Pitch_Data.set_current = 
        motor_position_speed_control(&Gimbal.Gimbal_PID.PITCH_speed_PID,
                                     &Gimbal.Gimbal_PID.PITCH_postion_PID,
                                      Gimbal.Gimbal_Set_Angle.pitch,
                                      Gimbal.Gimbal_Data.Imu_c->Pitch,
                                      Gimbal.Gimbal_Data.Pitch_Data.motor_measure->speed);
        can1_gimbal_setmsg_to_pitch(Gimbal.Gimbal_Data.Pitch_Data.set_current);
    }
    //p轴水平后，清零p轴电机累积编码值
    EncoderValZero(Gimbal.Gimbal_Data.pitch_encoder);
}

/**
 * @brief  云台设置模式
 * @param  
 * @retval void
 */
void Gimbal_set_mode(void)
{
    gimbal_behaviour_e last_behaviour;
    static gimbal_behaviour_e rc_behaviour = GIMBAL_MANUAL;
    static gimbal_behaviour_e kb_behaviour = GIMBAL_MANUAL;

    // 手柄
    last_behaviour = rc_behaviour;
    
    switch (Gimbal.Gimbal_RC->rc.s2)
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
        Gimbal.behaviour = rc_behaviour;
    }

    // 键鼠
    last_behaviour = kb_behaviour;

	if(Gimbal.Gimbal_RC->mouse.press_r)
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
        Gimbal.behaviour = kb_behaviour;
    }
}
void gimbal_clear_recive(void)
{
	if(Gimbal.AUTO_RC == NULL)
	{
		return;
	}
	Gimbal.AUTO_RC->auto_pitch = 0;
	Gimbal.AUTO_RC->auto_yaw = 0;
}
/**
 * @brief  云台设置目标角度
 * @param  
 * @retval void
 */
void Gimbal_set_angle(void)
{
    Gimbal.Gimbal_Set_Angle.last_pitch = Gimbal.Gimbal_Set_Angle.pitch;//记录pitch轴旧角度，后面判断pitch轴升降方向
    if (Gimbal.behaviour == GIMBAL_MANUAL)//手瞄
    {
        Gimbal.Gimbal_Set_Angle.pitch -= Gimbal.Gimbal_RC->mouse.y * MOUSE_PITCH_SPEED;
        Gimbal.Gimbal_Set_Angle.pitch += Gimbal.Gimbal_RC->rc.ch[1] * RC_PITCH_SPEED;

        Gimbal.Gimbal_Set_Angle.yaw -= Gimbal.Gimbal_RC->mouse.x * MOUSE_YAW_SPEED;
        Gimbal.Gimbal_Set_Angle.yaw -= Gimbal.Gimbal_RC->rc.ch[0] * RC_YAW_SPEED;
    }
    else if (Gimbal.behaviour == GIMBAL_AUTOATTACK)//自瞄
    {
        Gimbal.Gimbal_Set_Angle.pitch += Gimbal.AUTO_RC->auto_pitch;
        Gimbal.Gimbal_Set_Angle.yaw += Gimbal.AUTO_RC->auto_yaw;
        //pitch轴滤波
        Gimbal.Gimbal_Set_Angle.pitch = first_order_filter(&Gimbal.Gimbal_PID.AUTO_PITCH_filter,Gimbal.Gimbal_Set_Angle.pitch);
		gimbal_clear_recive();
    }
}

void Gimbal_pitch_limit(void)
{
    if ( Gimbal.Gimbal_Set_Angle.pitch > Gimbal.Gimbal_Set_Angle.last_pitch )//上仰
    {
        //采用累积编码值，判断丝杆转动极限
        if (Gimbal.Gimbal_Data.pitch_encoder->Encode_Actual_Val > pitch_height_limit)//仰角过高
            Gimbal.Gimbal_Set_Angle.pitch = Gimbal.Gimbal_Set_Angle.last_pitch;
    }
    if ( Gimbal.Gimbal_Set_Angle.pitch < Gimbal.Gimbal_Set_Angle.last_pitch )//下俯
    {
        if (Gimbal.Gimbal_Data.pitch_encoder->Encode_Actual_Val < pitch_low_limit)
            Gimbal.Gimbal_Set_Angle.pitch = Gimbal.Gimbal_Set_Angle.last_pitch;
    }
    
}

void Gimbal_PID_calc(void)
{
    
    //Pitch轴速度位置环
    Gimbal.Gimbal_Data.Pitch_Data.set_current = 
    motor_position_speed_control(&Gimbal.Gimbal_PID.PITCH_speed_PID,
                                 &Gimbal.Gimbal_PID.PITCH_postion_PID,
                                  Gimbal.Gimbal_Set_Angle.pitch,
                                  Gimbal.Gimbal_Data.Imu_c->Pitch,
                                  Gimbal.Gimbal_Data.Pitch_Data.motor_measure->speed);
    
    // 视当前位置为0，寻目标的劣弧角度即为控制量
    int yaw_angle_close;
    if (user_abs(Gimbal.Gimbal_Set_Angle.yaw - Gimbal.Gimbal_Data.Imu_c->Yaw) > 180 )
    {
        if ( (Gimbal.Gimbal_Set_Angle.yaw - Gimbal.Gimbal_Data.Imu_c->Yaw) > 0 )
        {
            yaw_angle_close = ((Gimbal.Gimbal_Set_Angle.yaw - Gimbal.Gimbal_Data.Imu_c->Yaw) - 360.0f) ;
        }
        else 
        {
            yaw_angle_close = (360.0f - (Gimbal.Gimbal_Set_Angle.yaw - Gimbal.Gimbal_Data.Imu_c->Yaw)) ;
        }                                        
    }        
    else
    {
        yaw_angle_close = (Gimbal.Gimbal_Set_Angle.yaw - Gimbal.Gimbal_Data.Imu_c->Yaw);
    }
  
    //Yaw轴速度位置环
    Gimbal.Gimbal_Data.Yaw_Data.set_current = 
    motor_position_speed_control(&Gimbal.Gimbal_PID.YAW_speed_PID,
                                 &Gimbal.Gimbal_PID.YAW_postion_PID,
                                  yaw_angle_close,
                                  0,
                                  Gimbal.Gimbal_Data.Pitch_Data.motor_measure->speed);
}

void Gimbal_change_angle(void)
{
    int16_t Pitch_motor_set_current = (int16_t)Gimbal.Gimbal_Data.Pitch_Data.set_current;
    int16_t yaw_motor_set_current   = (int16_t)Gimbal.Gimbal_Data.Yaw_Data.set_current;
    //CAN发送电流
    can1_gimbal_setmsg_to_pitch(Pitch_motor_set_current);
	can2_gimbal_setmsg_to_yaw(yaw_motor_set_current);
}


/**
 * @brief  云台主任务
 * @param
 * @retval void
 */
float torque = 0;
float MechPosition = 0;
float mspeed = 0;
float kp = 10;
float kd = 0;

uint16_t F_2 = 36666;
uint16_t angel_2 = 200;
uint16_t speed_2 = 200;
uint16_t kp_2 = 10;
uint16_t kd_2 = 0;
fp32 loc = 25.6;
fp32 spd = 2.62;
fp32 cur = 1;
parameter_index_e read_e = cur_kp;
fp32 num = 0;
void Gimbal_Task(void const *argument)
{
    Gimbal_init();
    extern MI_Motor_t *MI_Motor;//定义一个米狗电机结构体
    MI_Motor = MI_motor_init(&hcan1);//挂载在can
    while (1)
    {
        taskENTER_CRITICAL();         
        /*--------临界区--------*/
       
       //MI_motor_get_ID(MI_Motor);//获取电机id值,储存到结构体
       //MI_motor_changeID(MI_Motor,127);//更改ID为127
        Set_Mode(MI_Motor,postion);
        MI_motor_enable(MI_Motor);//电机使能
// 
        MI_motor_Write_One_Para(MI_Motor,spd_ref,loc);
        MI_motor_Write_One_Para(MI_Motor,cur_kp,0.5);
//        MI_motor_Write_One_Para(MI_Motor,limit_spd,15,'f');
//        MI_motor_Write_One_Para(MI_Motor,loc_ref,loc,'f');

        MI_motor_stop(MI_Motor);//电机失能
        MI_motor_Read_One_Para(MI_Motor,cur_kp);
//        Gimbal_set_angle();   
//        Gimbal_pitch_limit();//pitch轴丝杆限位
//        Gimbal_PID_calc();
//        Online_detection();//电机在线检测
        /*--------临界区--------*/
		taskEXIT_CRITICAL();        
//		Gimbal_change_angle();//发送角度
        vTaskDelay(1); // 绝对延时//vTaskDelay(2);
    }
}


