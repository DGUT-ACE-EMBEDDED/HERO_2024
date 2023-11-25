#include "fire_task.h"
/* ************************freertos******************** */
#include "FreeRTOS.h"
#include "freertos.h"
#include "queue.h"
#include "semphr.h"

fire_task_t fire_task = {
    .Fire_Motor = {
        .CanId = 1,
        .reverse_flag = 1,
        .Encoder_type = M3508,
        .current_input = 0,
        .voltage_input = 0
    },
    .set_position =  0
};
void fire_init(void)
{
    CMD_Init();
    
    fire_task.Robot_cmd = get_Robot_cmd_point();
    fire_task.Fire_Motor.Motor_Information = Get_Measure_Address(FIRE_MOTOR_ID);
    fire_task.Fire_Motor.Motor_encoder  = Get_Chassis_motor_encoder_point(CHASSIS_MOTOR_FIRE_ENCODER, fire_task.Fire_Motor.Encoder_type, 0);
    
    PidInit(&fire_task.Fire_Motor.Speed_PID, FIRE_MOTOR_S_KP, FIRE_MOTOR_S_KI, FIRE_MOTOR_S_KD, Output_Limit);
	PidInitMode(&fire_task.Fire_Motor.Speed_PID,Output_Limit, FIRE_3508_CURRENT_LIMIT, 0);	
	
	PidInit(&fire_task.Fire_Motor.Position_PID, FIRE_MOTOR_P_KP, FIRE_MOTOR_P_KI, FIRE_MOTOR_P_KD, Integral_Limit | Output_Limit);
	PidInitMode(&fire_task.Fire_Motor.Position_PID,Output_Limit, FIRE_3508_CURRENT_LIMIT, 0);	
	PidInitMode(&fire_task.Fire_Motor.Position_PID,Integral_Limit, 150 , 0);
}

static Fire_State_e ON_OR_OFF = NO_Fire_FORCE;//摩擦轮开关状态
void fire_work(void)
{
    static uint8_t Fist_set = 1;//首次设置目标值
    static uint8_t Block = 0;//堵转标志
    static uint16_t On_Fire_Block_time = 0;//进弹堵转时长
    
    //设置目标位置
    if (fire_task.Robot_cmd->Fire_State == On_Fire)
    {
        if (Fist_set == 1)
        {
            Fist_set = 0;
            Block = 0;
            fire_task.set_position += 36864;
            On_Fire_Block_time = 0;//记录开始时间
        }       
        On_Fire_Block_time++;        
        if (On_Fire_Block_time > 4000){//长时间处于进弹状态，认为是卡弹，退弹重推
            Fist_set = 1;
            Block = 1;
            fire_task.Robot_cmd->Fire_State = On_Empty;//退弹状态
        }
    }
    else if (fire_task.Robot_cmd->Fire_State == On_Empty)
    {
        if (Fist_set == 1)
        {
            Fist_set = 0;
            //因为堵转进的退弹模式，当前堵转的位置回退半个蛋
            if (Block == 1)
                fire_task.set_position = fire_task.Fire_Motor.Motor_encoder->Encode_Record_Val - 20000;
            else                
                fire_task.set_position -= 36864;            //主动进的退弹模式，回退全个蛋
        }      
    }
    else    ON_OR_OFF = fire_task.Robot_cmd->Fire_State;//保存摩擦轮开关状态，留给退弹模式完成后用
    
    //检测进弹行程
    if (fire_task.Robot_cmd->Fire_State == On_Fire && fire_task.Fire_Motor.Motor_encoder->Encode_Record_Val > fire_task.set_position-4000)//检查进弹行程
    {
        fire_task.Robot_cmd->Fire_State = Ready_Fire;
        Fist_set = 1;
    }
    //检测退弹行程
    if (fire_task.Robot_cmd->Fire_State == On_Empty && fire_task.Fire_Motor.Motor_encoder->Encode_Record_Val < fire_task.set_position+4000)//检查退弹行程
    {
        if(Block == 1)   fire_task.Robot_cmd->Fire_State = On_Fire;//堵转完成退弹后重新进弹
        else                fire_task.Robot_cmd->Fire_State = ON_OR_OFF;//主动退弹时的摩擦轮状态
        Fist_set = 1;
    }
            
    //位置速度环计算
    fire_task.Fire_Motor.current_input = motor_position_speed_control( &fire_task.Fire_Motor.Speed_PID,
                                                                       &fire_task.Fire_Motor.Position_PID,
                                                                        fire_task.set_position,
                                                                        fire_task.Fire_Motor.Motor_encoder->Encode_Record_Val,
                                                                        fire_task.Fire_Motor.Motor_Information->speed_data);
}
void Fire_Send_current(void)
{
    CAN1_Fire_Tx(fire_task.Fire_Motor.current_input);
}

void Fire_acion(void)
{
    Fire_Send_current();
    if (ON_OR_OFF == NO_Fire_FORCE) Send_to_Gimbal(false);
    else                            Send_to_Gimbal(true);
}

void Fire_Task(void const *argument)
{
	fire_init();
	while(1)
	{
		taskENTER_CRITICAL(); //进入临界区
        Fire_Set();
		fire_work();
        Fire_acion();
		taskEXIT_CRITICAL(); //退出临界区
		vTaskDelay(1);
	}
}

//#include "chassis_task.h"
//#include "pid.h"
//#include "PID_Data.h"

//extern Chassis_Task_t Chassis_Data;

//static void Fire_Init(Chassis_Task_t *Chassis_Data);
//static void Fire_Work(Chassis_Task_t *Chassis_Data);

//void Fire_Task(void const *argument)
//{
//	Fire_Init(&Chassis_Data);
//	while(1)
//	{
//		taskENTER_CRITICAL(); //进入临界区
//		Fire_Work(&Chassis_Data);
//		taskEXIT_CRITICAL(); //退出临界区
//		vTaskDelay(1);
//	}
//}

//void Fire_Init(Chassis_Task_t *Chassis_Data)
//{
//	Chassis_Data->fire_state = FIRE_NO_ACTION;
//	PidInit(&Chassis_Data->FIRE_Speed_PID, FIRE_MOTOR_S_KP, FIRE_MOTOR_S_KI, FIRE_MOTOR_S_KD, Output_Limit);
//	PidInitMode(&Chassis_Data->FIRE_Speed_PID,Output_Limit, FIRE_3508_CURRENT_LIMIT, 0);	
//	
//	PidInit(&Chassis_Data->FIRE_Position_PID, FIRE_MOTOR_P_KP, FIRE_MOTOR_P_KI, FIRE_MOTOR_P_KD, Integral_Limit | Output_Limit);
//	PidInitMode(&Chassis_Data->FIRE_Position_PID,Output_Limit, FIRE_3508_CURRENT_LIMIT, 0);	
//	PidInitMode(&Chassis_Data->FIRE_Position_PID,Integral_Limit, 150 , 0);
//    
//	EncoderValZero(Chassis_Data->Motor_encoder[4]);
//	Chassis_Data->fire_set_position = 0;
//}

//void Fire_Work(Chassis_Task_t *Chassis_Data)
//{
//    static int count = 0;
//    static int ready_fire = 1;
//    static int32_t last_fire_set_position = 0;
//    
//    fire_state_e last_fire_state = Chassis_Data->fire_state;
//    
//    if(Chassis_Data->RC->rc.s2 == RC_SW_UP || Chassis_Data->RC->rc.s2 == RC_SW_MID)//底盘开始行动
//    {

//        Chassis_Data->fire_state = FIRE_ACTION;
//        
//        //加个拨轮回正检测，防止连发
//        if ( Chassis_Data->fire_on_flag == 0 && Chassis_Data->RC->rc.ch[4] <= 200 && Chassis_Data->RC->mouse.press_l ==0)
//            ready_fire = 1;
//        
//        //击发检测
//        if((Chassis_Data->RC->rc.ch[4] >= 650 || Chassis_Data->RC->mouse.press_l ==1) && Chassis_Data->fire_on_flag == 0 && ready_fire)
//        {
//            count = 0;//清空卡弹计数器
//            ready_fire = 0;
//            Chassis_Data->fire_on_flag = 1; //发射中标志
//            Chassis_Data->fire_detect_flag = 0;//退弹标志
//            last_fire_set_position =  Chassis_Data->fire_set_position;//记录当前位置，卡弹退弹用
//            Chassis_Data->fire_set_position += 36864;
//        }
//        Chassis_Data->FIRE_current_input = motor_position_speed_control(&Chassis_Data->FIRE_Speed_PID, &Chassis_Data->FIRE_Position_PID, Chassis_Data->fire_set_position, Chassis_Data->Motor_encoder[4]->Encode_Record_Val ,Chassis_Data->FIRE_Motor->speed_data);	
//    }
//    else//全部退弹
//    {
//        Chassis_Data->fire_state = FIRE_NO_ACTION;
//        Chassis_Data->fire_on_flag = 0;

//        if ( last_fire_state == FIRE_ACTION && Chassis_Data->fire_detect_flag == 0 )
//        {
//            Chassis_Data->fire_detect_flag = 1;
//            Chassis_Data->fire_set_position -= 36864;
//        }       
//        
//        if ( Chassis_Data->fire_detect_flag == 1 )
//        {
//            Chassis_Data->FIRE_current_input = motor_position_speed_control(&Chassis_Data->FIRE_Speed_PID, 
//                                                                            &Chassis_Data->FIRE_Position_PID, 
//                                                                             Chassis_Data->fire_set_position, 
//                                                                             Chassis_Data->Motor_encoder[4]->Encode_Record_Val ,
//                                                                             Chassis_Data->FIRE_Motor->speed_data);	
//        }
//        else
//        {
//            Chassis_Data->FIRE_current_input = 0;
//        }
//        
//        if ( Chassis_Data->Motor_encoder[4]->Encode_Record_Val <= Chassis_Data->fire_set_position )
//            Chassis_Data->fire_detect_flag = 0;
//        
//    }
//    
//    
//    //检测电机是否卡死，检测原理 ： 若电机卡死 位置环 I 持续积累，积累值长时间到达阈值 判定为电机卡死
//    if (Chassis_Data->fire_on_flag == 1)
//    {
//        if (Chassis_Data->FIRE_Speed_PID.Ierror == 150)    count++;
//        if (count > 3500) //堵转，反转退弹
//        {
//            Chassis_Data->fire_state = FIRE_LOCK_POSITION;
//            Chassis_Data->fire_detect_flag = 1;//退弹标志
//            count = 0;
//        }
//    }
//        
//    //退弹中
//    if ( Chassis_Data->fire_detect_flag == 1 )
//    {
//        if (Chassis_Data->Motor_encoder[4]->Encode_Record_Val > last_fire_set_position-36084/2)//多退半个弹的距离
//        {
//            //Chassis_Data->FIRE_current_input = 0;//电流置0，重力退弹
//            //电机主动反转退弹
//            Chassis_Data->FIRE_current_input = motor_position_speed_control(&Chassis_Data->FIRE_Speed_PID, 
//                                                                            &Chassis_Data->FIRE_Position_PID, 
//                                                                             last_fire_set_position-36084/2, 
//                                                                             Chassis_Data->Motor_encoder[4]->Encode_Record_Val ,
//                                                                             Chassis_Data->FIRE_Motor->speed_data);	
//        }            
//        else 
//        {
//            Chassis_Data->fire_detect_flag = 0;//退弹完成，重新上弹

//            Chassis_Data->FIRE_current_input = motor_position_speed_control(&Chassis_Data->FIRE_Speed_PID, &Chassis_Data->FIRE_Position_PID, Chassis_Data->fire_set_position, Chassis_Data->Motor_encoder[4]->Encode_Record_Val ,Chassis_Data->FIRE_Motor->speed_data);	
//        }            
//    }
//    
//    // 发射完成
//    if(Chassis_Data->Motor_encoder[4]->Encode_Record_Val >= Chassis_Data->fire_set_position)
//    {
//        Chassis_Data->fire_on_flag = 0;
//    }
//}


