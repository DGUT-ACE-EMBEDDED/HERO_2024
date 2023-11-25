//#include "chassis_task.h"
//#include "PID_Data.h"
//#include "cmsis_os.h"
///*********FreeRTOS头文件包含*********/
//#include "FreeRTOSConfig.h"
//#include "FreeRTOS.h"
//#include "task.h"
///********状态机头文件********/
//#include "FSM_Chassis.h"
///********应用层头文件********/
//#include "maths.h"
//#include "bsp_dr16.h"
//#include "pid.h"
//#include "CAN1.h"
//#include "CAN2.h"
//#include "filter.h"

//Chassis_Task_t Chassis_Data;
//New_Chassis_Task_t New_Chassis_Data;

//Recursive_ave_filter_type_t filter1;
//first_order_filter_type_t ditong_filter1;

//uint32_t currentTime;

//uint16_t Chassis_Power_Limit;
//fp32 Chassis_Power_All;



//int power;
//float filted_accel[2],a_motor[1];

//void Chassis_Motor_PID_Init(void);
//void Chassis_Motor_PID_Calc(void);
//void Clear_or_not(void); 
//void Power_Limit(void);
//void chassis_prevent_motion_distortion(void);
//void a_limit(void);

//void Task_Chassis(void const *argument)
//{
//	Chassis_Init();
//	vTaskDelay(500);
//	while(1)
//	{
//        taskENTER_CRITICAL(); 
//        /*--------------------------------临界区--------------------------------*/
//        currentTime = xTaskGetTickCount();                              //当前系统时间
//        Mode_Deal();													//模式处理
//        State_Deal();													//确定运动状态
//        Chassis_X_Y_Speed_Calc();                                       //xy速度解算
//        Clear_or_not();												    //仅用于锁车模式
//        Motion_Calc();												    //麦轮运动解算
//        Chassis_Motor_PID_Calc();							            //PID计算
////        Power_Limit();												    //功率限制
//        chassis_prevent_motion_distortion();	                        //防止运动失真
//        a_limit();														//加速度限制
//        CAN1_Chassis_Tx(New_Chassis_Data.Set_information.LF_current_input,
//                        New_Chassis_Data.Set_information.RF_current_input,
//						New_Chassis_Data.Set_information.LB_current_input,
//						New_Chassis_Data.Set_information.RB_current_input);	
//        Can1_Cap_Setmsg(power);//发送超电充能功率
//        CAN2_Send_Gambal();                                             //转发遥控数据->云台板
//        CAN1_Fire_Tx(Chassis_Data.FIRE_current_input);
//        /*--------------------------------临界区--------------------------------*/
//        taskEXIT_CRITICAL(); 
//        vTaskDelayUntil(&currentTime, 1);//绝对延时//vTaskDelay(2);	
//    }
//}


///*
// *函数名称：Chassis_Init
// *函数参数：void
// *函数返回：void
// *函数功能：底盘初始化
// *特别说明：
// */
//void Chassis_Init(void)
//{
//    /*--------------------底盘数据结构体初始化--------------------*/
//    Necessary_information_init();
//    Motor_information_init();
//    Fire_information_init();
//    PID_information_init();
//    Set_information_init();
//    
//	FSM_Chassis_Init();
//	Chassis_Motor_PID_Init();
//	Recursive_ave_filter_init(&filter1);
//	first_order_filter_init(&ditong_filter1,0.3);

//	/*--------------------初始化编码器--------------------*/
//    
////	Chassis_Data.Motor_encoder[CHASSIS_MOTOR_RF_ENCODER]   = Get_Chassis_motor_encoder_point(CHASSIS_MOTOR_RF_ENCODER);
////    Chassis_Data.Motor_encoder[CHASSIS_MOTOR_RB_ENCODER]   = Get_Chassis_motor_encoder_point(CHASSIS_MOTOR_RB_ENCODER);
////    Chassis_Data.Motor_encoder[CHASSIS_MOTOR_LB_ENCODER]   = Get_Chassis_motor_encoder_point(CHASSIS_MOTOR_LB_ENCODER);
////    Chassis_Data.Motor_encoder[CHASSIS_MOTOR_LF_ENCODER]   = Get_Chassis_motor_encoder_point(CHASSIS_MOTOR_LF_ENCODER);
////    Chassis_Data.Motor_encoder[CHASSIS_MOTOR_FIRE_ENCODER] = Get_Chassis_motor_encoder_point(CHASSIS_MOTOR_FIRE_ENCODER);

////	Encoder_Init(Chassis_Data.Motor_encoder[CHASSIS_MOTOR_RF_ENCODER], M3508);
////	Encoder_Init(Chassis_Data.Motor_encoder[CHASSIS_MOTOR_RB_ENCODER], M3508);
////    Encoder_Init(Chassis_Data.Motor_encoder[CHASSIS_MOTOR_LB_ENCODER], M3508);
////    Encoder_Init(Chassis_Data.Motor_encoder[CHASSIS_MOTOR_LF_ENCODER], M3508);
////    Encoder_Init(Chassis_Data.Motor_encoder[CHASSIS_MOTOR_FIRE_ENCODER], M3508);
////	/*--------------------初始化底盘电机在线状态--------------------*/
////    Chassis_Data.motor_state = ALL_OK;
////	Chassis_Data.FSM_Chassis		=		Get_FSM_Chassis_Address();
////	Chassis_Data.CAN1_Chassis		=		Get_CAN1_Address();
////	Chassis_Data.RC							=		RC_Get_RC_Pointer();
////	Chassis_Data.referee        = 	Get_referee_Address();
////	Chassis_Data.INS            =   get_imu_control_point();
////	Chassis_Data.LF_Motor       =   Get_Measure_Address(CHASSIS_MOTOR_LF_ID);
////	Chassis_Data.RF_Motor				=   Get_Measure_Address(CHASSIS_MOTOR_RF_ID);
////	Chassis_Data.LB_Motor       =   Get_Measure_Address(CHASSIS_MOTOR_LB_ID);
////	Chassis_Data.RB_Motor				=   Get_Measure_Address(CHASSIS_MOTOR_RB_ID);
////	Chassis_Data.FIRE_Motor     =   Get_Measure_Address(FIRE_MOTOR_ID);
//}


///*
// *函数名称：Necessary_information_init
// *函数参数：void
// *函数返回：void
// *函数功能：初始化底盘必要数据结构体
// *特别说明：
// */
//void Necessary_information_init(void)
//{
//    New_Chassis_Data.Necessary_information.Difference_Angle_between_Chassis_Gimbal = 0;
//    New_Chassis_Data.Necessary_information.INS = get_imu_control_point();
//    New_Chassis_Data.Necessary_information.RC = RC_Get_RC_Pointer();
//    New_Chassis_Data.Necessary_information.referee = Get_referee_Address();
//}


///*
// *函数名称：Motor_information_init
// *函数参数：void
// *函数返回：void
// *函数功能：初始化电机数据结构体
// *特别说明：
// */
//void Motor_information_init(void)
//{
//    /*初始化电机电调数据*/
//    New_Chassis_Data.Motor_information.LF_Motor     =   Get_Measure_Address(CHASSIS_MOTOR_LF_ID);
//	New_Chassis_Data.Motor_information.RF_Motor		=   Get_Measure_Address(CHASSIS_MOTOR_RF_ID);
//	New_Chassis_Data.Motor_information.LB_Motor     =   Get_Measure_Address(CHASSIS_MOTOR_LB_ID);
//	New_Chassis_Data.Motor_information.RB_Motor		=   Get_Measure_Address(CHASSIS_MOTOR_RB_ID);
//	New_Chassis_Data.Motor_information.FIRE_Motor   =   Get_Measure_Address(FIRE_MOTOR_ID);
//    /*初始化电机失联计时*/
//    New_Chassis_Data.Motor_information.last_call_back_time = Get_last_call_back_time();
//    /*初始化电机编码器数据*/
//    New_Chassis_Data.Motor_information.Motor_encoder[CHASSIS_MOTOR_RF_ENCODER]   = Get_Chassis_motor_encoder_point(CHASSIS_MOTOR_RF_ENCODER,M3508);
//    New_Chassis_Data.Motor_information.Motor_encoder[CHASSIS_MOTOR_RB_ENCODER]   = Get_Chassis_motor_encoder_point(CHASSIS_MOTOR_RB_ENCODER,M3508);
//    New_Chassis_Data.Motor_information.Motor_encoder[CHASSIS_MOTOR_LB_ENCODER]   = Get_Chassis_motor_encoder_point(CHASSIS_MOTOR_LB_ENCODER,M3508);
//    New_Chassis_Data.Motor_information.Motor_encoder[CHASSIS_MOTOR_LF_ENCODER]   = Get_Chassis_motor_encoder_point(CHASSIS_MOTOR_LF_ENCODER,M3508);
//    New_Chassis_Data.Motor_information.Motor_encoder[CHASSIS_MOTOR_FIRE_ENCODER] = Get_Chassis_motor_encoder_point(CHASSIS_MOTOR_FIRE_ENCODER,M3508);
//    
//    Encoder_Init(New_Chassis_Data.Motor_information.Motor_encoder[CHASSIS_MOTOR_RF_ENCODER], M3508);
//	Encoder_Init(New_Chassis_Data.Motor_information.Motor_encoder[CHASSIS_MOTOR_RB_ENCODER], M3508);
//    Encoder_Init(New_Chassis_Data.Motor_information.Motor_encoder[CHASSIS_MOTOR_LB_ENCODER], M3508);
//    Encoder_Init(New_Chassis_Data.Motor_information.Motor_encoder[CHASSIS_MOTOR_LF_ENCODER], M3508);
//    Encoder_Init(New_Chassis_Data.Motor_information.Motor_encoder[CHASSIS_MOTOR_FIRE_ENCODER], M3508);
//    /*初始化电机在线数据*/
//    New_Chassis_Data.Motor_information.motor_state = ALL_OK;
//}


///*
// *函数名称：PID_information_init
// *函数参数：void
// *函数返回：void
// *函数功能：初始化PID数据结构体
// *特别说明：只初始化了底盘四个驱动电机，拨盘电机的初始化在Fire_Init里面进行
// */
//void PID_information_init(void)
//{
//    /*---速度PID---*/
//    PidInit(&New_Chassis_Data.PID_information.LF_Speed_PID, LF_Speed_PID_P,	LF_Speed_PID_I,	LF_Speed_PID_D,	Output_Limit | StepIn);
//	PidInit(&New_Chassis_Data.PID_information.RF_Speed_PID, RF_Speed_PID_P,	RF_Speed_PID_I,	RF_Speed_PID_D,	Output_Limit | StepIn);	
//	PidInit(&New_Chassis_Data.PID_information.LB_Speed_PID, LB_Speed_PID_P,	LB_Speed_PID_I,	LB_Speed_PID_D,	Output_Limit | StepIn);		
//	PidInit(&New_Chassis_Data.PID_information.RB_Speed_PID, RB_Speed_PID_P,	RB_Speed_PID_I,	RB_Speed_PID_D,	Output_Limit | StepIn);
//    PidInit(&New_Chassis_Data.PID_information.z_speed, 40, 0.1 ,0, Output_Limit | Integral_Limit | Deadzone);
//    
//	PidInitMode(&New_Chassis_Data.PID_information.z_speed, Output_Limit, 10000, 0);
//	PidInitMode(&New_Chassis_Data.PID_information.z_speed, Integral_Limit, 800, 0);
//	PidInitMode(&New_Chassis_Data.PID_information.z_speed, Deadzone, 5, 0);

//	PidInitMode(&New_Chassis_Data.PID_information.LF_Speed_PID, Output_Limit, MOTOR_3508_CURRENT_LIMIT, 0);
//	PidInitMode(&New_Chassis_Data.PID_information.RF_Speed_PID, Output_Limit, MOTOR_3508_CURRENT_LIMIT, 0);
//	PidInitMode(&New_Chassis_Data.PID_information.LB_Speed_PID, Output_Limit, MOTOR_3508_CURRENT_LIMIT, 0);
//	PidInitMode(&New_Chassis_Data.PID_information.RB_Speed_PID, Output_Limit, MOTOR_3508_CURRENT_LIMIT, 0);
//    
//	PidInitMode(&New_Chassis_Data.PID_information.LF_Speed_PID, StepIn, StepIn_Data, 0);
//	PidInitMode(&New_Chassis_Data.PID_information.RF_Speed_PID, StepIn, StepIn_Data, 0);
//	PidInitMode(&New_Chassis_Data.PID_information.LB_Speed_PID, StepIn, StepIn_Data, 0);
//	PidInitMode(&New_Chassis_Data.PID_information.RB_Speed_PID, StepIn, StepIn_Data, 0);
//    /*---位置PID---*/
//	PidInit(&New_Chassis_Data.PID_information.LF_Position_PID, LF_Position_PID_P, LF_Position_PID_I, LF_Position_PID_D, NONE);
//	PidInit(&New_Chassis_Data.PID_information.RF_Position_PID, RF_Position_PID_P, RF_Position_PID_I, RF_Position_PID_D, NONE);
//	PidInit(&New_Chassis_Data.PID_information.LB_Position_PID, LB_Position_PID_P, LB_Position_PID_I, LB_Position_PID_D, NONE);
//	PidInit(&New_Chassis_Data.PID_information.RB_Position_PID, RB_Position_PID_P, RB_Position_PID_I, RB_Position_PID_D, NONE);
//}


//void Fire_information_init(void)
//{
//    New_Chassis_Data.Fire_information.fire_detect_flag = 0;
//    New_Chassis_Data.Fire_information.fire_on_flag = 0;
//    New_Chassis_Data.Fire_information.fire_set_position = 0;
//    New_Chassis_Data.Fire_information.fire_state = FIRE_NO_ACTION;
//    New_Chassis_Data.Fire_information.one_shoot_flag = 0;
//}


//void Set_information_init(void)
//{
//    /*---速度归零---*/
//    New_Chassis_Data.Set_information.plane_x_speed_set = 0;
//    New_Chassis_Data.Set_information.plane_y_speed_set = 0;
//    New_Chassis_Data.Set_information.plane_z_speed_set = 0;
//    
//    New_Chassis_Data.Set_information.RF_speed_set = 0;
//    New_Chassis_Data.Set_information.RB_speed_set = 0;
//    New_Chassis_Data.Set_information.LB_speed_set = 0;
//    New_Chassis_Data.Set_information.LF_speed_set = 0;
//    /*---电流归零---*/
//    New_Chassis_Data.Set_information.RF_current_input = 0;
//    New_Chassis_Data.Set_information.RB_current_input = 0;
//    New_Chassis_Data.Set_information.LB_current_input = 0;
//    New_Chassis_Data.Set_information.LF_current_input = 0;
//    New_Chassis_Data.Set_information.FIRE_current_input = 0;
//    /*---模式、锁车初始化---*/
//    New_Chassis_Data.Set_information.Mode = NO_FORCE;
//    New_Chassis_Data.Set_information.State = SPEED;
//}
///*
// *函数名称：Chassis_Motor_PID_Init
// *函数参数：void
// *函数返回：void
// *函数功能：初始化PID参数
// *特别说明：
// */
//void Chassis_Motor_PID_Init(void)
//{
//	PidInit(&Chassis_Data.LF_Speed_PID, LF_Speed_PID_P,	LF_Speed_PID_I,	LF_Speed_PID_D,	Output_Limit | StepIn);
//	PidInit(&Chassis_Data.RF_Speed_PID, RF_Speed_PID_P,	RF_Speed_PID_I,	RF_Speed_PID_D,	Output_Limit | StepIn);	
//	PidInit(&Chassis_Data.LB_Speed_PID, LB_Speed_PID_P,	LB_Speed_PID_I,	LB_Speed_PID_D,	Output_Limit | StepIn);		
//	PidInit(&Chassis_Data.RB_Speed_PID, RB_Speed_PID_P,	RB_Speed_PID_I,	RB_Speed_PID_D,	Output_Limit | StepIn);
//    
//	PidInitMode(&Chassis_Data.LF_Speed_PID, Output_Limit, MOTOR_3508_CURRENT_LIMIT, 0);
//	PidInitMode(&Chassis_Data.RF_Speed_PID, Output_Limit, MOTOR_3508_CURRENT_LIMIT, 0);
//	PidInitMode(&Chassis_Data.LB_Speed_PID, Output_Limit, MOTOR_3508_CURRENT_LIMIT, 0);
//	PidInitMode(&Chassis_Data.RB_Speed_PID, Output_Limit, MOTOR_3508_CURRENT_LIMIT, 0);
//    
//	PidInitMode(&Chassis_Data.LF_Speed_PID, StepIn, StepIn_Data, 0);
//	PidInitMode(&Chassis_Data.RF_Speed_PID, StepIn, StepIn_Data, 0);
//	PidInitMode(&Chassis_Data.LB_Speed_PID, StepIn, StepIn_Data, 0);
//	PidInitMode(&Chassis_Data.RB_Speed_PID, StepIn, StepIn_Data, 0);

//	PidInit(&Chassis_Data.LF_Position_PID, LF_Position_PID_P, LF_Position_PID_I, LF_Position_PID_D, NONE);
//	PidInit(&Chassis_Data.RF_Position_PID, RF_Position_PID_P, RF_Position_PID_I, RF_Position_PID_D, NONE);
//	PidInit(&Chassis_Data.LB_Position_PID, LB_Position_PID_P, LB_Position_PID_I, LB_Position_PID_D, NONE);
//	PidInit(&Chassis_Data.RB_Position_PID, RB_Position_PID_P, RB_Position_PID_I, RB_Position_PID_D, NONE);
//}

///*
// *函数名称：Chassis_Motor_PID_Calc
// *函数参数：void
// *函数返回：void
// *函数功能：根据底盘状态与底盘模式进行电流PID计算
// *特别说明：
// */
//void Chassis_Motor_PID_Calc(void)
//{
//	if(New_Chassis_Data.Set_information.State == LOCK_POSITION)
//	{
//	New_Chassis_Data.Set_information.LF_current_input = motor_position_speed_control(&New_Chassis_Data.PID_information.LF_Speed_PID, &New_Chassis_Data.PID_information.LF_Position_PID, 0, New_Chassis_Data.Motor_information.Motor_encoder[LF]->Encode_Record_Val, New_Chassis_Data.Motor_information.LF_Motor->speed_data);
//	New_Chassis_Data.Set_information.RF_current_input = motor_position_speed_control(&New_Chassis_Data.PID_information.RF_Speed_PID, &New_Chassis_Data.PID_information.RF_Position_PID, 0, New_Chassis_Data.Motor_information.Motor_encoder[RF]->Encode_Record_Val, New_Chassis_Data.Motor_information.RF_Motor->speed_data);	
//	New_Chassis_Data.Set_information.LB_current_input = motor_position_speed_control(&New_Chassis_Data.PID_information.LB_Speed_PID, &New_Chassis_Data.PID_information.LB_Position_PID, 0, New_Chassis_Data.Motor_information.Motor_encoder[LB]->Encode_Record_Val, New_Chassis_Data.Motor_information.LB_Motor->speed_data);
//	New_Chassis_Data.Set_information.RB_current_input = motor_position_speed_control(&New_Chassis_Data.PID_information.RB_Speed_PID, &New_Chassis_Data.PID_information.RB_Position_PID, 0, New_Chassis_Data.Motor_information.Motor_encoder[RB]->Encode_Record_Val, New_Chassis_Data.Motor_information.RB_Motor->speed_data);
//	}
//	else if(New_Chassis_Data.Set_information.Mode == NO_FORCE)
//	{
//		New_Chassis_Data.Set_information.LF_current_input = 0;
//		New_Chassis_Data.Set_information.RF_current_input = 0;
//		New_Chassis_Data.Set_information.LB_current_input = 0;
//		New_Chassis_Data.Set_information.RB_current_input = 0;
//	}
//	else if(New_Chassis_Data.Set_information.State == SPEED)
//	{		
//        New_Chassis_Data.Set_information.LF_current_input = PidCalculate(&New_Chassis_Data.PID_information.LF_Speed_PID	 ,New_Chassis_Data.Set_information.LF_speed_set, New_Chassis_Data.Motor_information.LF_Motor->speed_data);
//        New_Chassis_Data.Set_information.RF_current_input = PidCalculate(&New_Chassis_Data.PID_information.RF_Speed_PID	 ,New_Chassis_Data.Set_information.RF_speed_set, New_Chassis_Data.Motor_information.RF_Motor->speed_data);	
//        New_Chassis_Data.Set_information.LB_current_input = PidCalculate(&New_Chassis_Data.PID_information.LB_Speed_PID	 ,New_Chassis_Data.Set_information.LB_speed_set, New_Chassis_Data.Motor_information.LB_Motor->speed_data);
//        New_Chassis_Data.Set_information.RB_current_input = PidCalculate(&New_Chassis_Data.PID_information.RB_Speed_PID	 ,New_Chassis_Data.Set_information.RB_speed_set, New_Chassis_Data.Motor_information.RB_Motor->speed_data);
//        if (New_Chassis_Data.Motor_information.motor_state != ALL_OK)
//        {
//            if (New_Chassis_Data.Set_information.LF_speed_set == 0) New_Chassis_Data.Set_information.LF_current_input = 0;
//            if (New_Chassis_Data.Set_information.RF_speed_set == 0) New_Chassis_Data.Set_information.RF_current_input = 0;
//            if (New_Chassis_Data.Set_information.LB_speed_set == 0) New_Chassis_Data.Set_information.LB_current_input = 0;
//            if (New_Chassis_Data.Set_information.RB_speed_set == 0) New_Chassis_Data.Set_information.RB_current_input = 0;
//        }
//    }
//}

///*
// *函数名称：Clear_or_not
// *函数参数：void
// *函数返回：void
// *函数功能：清除底盘电机码盘累积值
// *特别说明：
// */
//void Clear_or_not(void)
//{
//    if (New_Chassis_Data.Motor_information.Motor_encoder_clear_flag == 1)
//    {
//        New_Chassis_Data.Motor_information.Motor_encoder_clear_flag = 0;
//        
//        //用于锁车
//        EncoderValZero(New_Chassis_Data.Motor_information.Motor_encoder[RF]);
//        EncoderValZero(New_Chassis_Data.Motor_information.Motor_encoder[RB]);
//        EncoderValZero(New_Chassis_Data.Motor_information.Motor_encoder[LB]);
//        EncoderValZero(New_Chassis_Data.Motor_information.Motor_encoder[LF]);
//        pid_clear(&New_Chassis_Data.PID_information.RF_Position_PID);
//        pid_clear(&New_Chassis_Data.PID_information.RB_Position_PID);
//        pid_clear(&New_Chassis_Data.PID_information.LB_Position_PID);
//        pid_clear(&New_Chassis_Data.PID_information.LF_Position_PID);
//        
//    }
//        
//}

//void a_limit(void)
//{
//	filted_accel[0] = Recursive_ave_filter(&filter1, New_Chassis_Data.Motor_information.Motor_encoder[RF]->AccSpeed, 100);
//	filted_accel[0] = first_order_filter(&ditong_filter1,	filted_accel[0]);
//    
//	filted_accel[1] = Recursive_ave_filter(&filter1, Chassis_Data.INS->E_Accel[Y], 100);
//	filted_accel[0] = first_order_filter(&ditong_filter1,	filted_accel[0]);
//}

//void Power_Limit(void)
//{
//    #define MAX_Super_capacitor_power 200
//    #define MAX_Super_capacitor_work  200
////    if(Chassis_Data.referee->Power_Heat.chassis_power == 0)
////	{
////		power = 60;
////	}
////	else 
////	{
////		power = Chassis_Data.referee->Robot_Status.chassis_power_limit + Chassis_Data.referee->Power_Heat.chassis_power_buffer - 5.0f;
////	}
//    
//    //读取裁判系统数据获取底盘总功率上限
//    Chassis_Power_Limit = New_Chassis_Data.Necessary_information.referee->Robot_Status.chassis_power_limit;
//    Chassis_Power_Limit+= New_Chassis_Data.Necessary_information.referee->Buff.power_rune_buff;
//    
//    /*---铜铁损耗系数---*/
//    static fp32 P_C620 = 1.25f;//电调功率损耗
//    static fp32 K = 1.99688994e-6f; // (20/16384)  电调电流与真实电流比     *(0.3)*(187/3591)/9.55   M3509扭矩比例参数   电调值转化为力矩
//    static fp32 K_1 = 1.453e-07;			     // k1  已经除 K
//    static fp32 a = 1.23e-07;					 // k2
//    
//    /*---电机运动数据---*/
//    int16_t speed[4];//转子转速
//    int16_t I_out[4];//力矩电流
//    fp32    P_out[4];//电机输出功率
//    fp32     P_in[4];//电机输入功率
//    int16_t F_out[4];//输出力矩
//    int16_t I_max[4];//力矩电流
//    
//    /*---超电补偿做功 积分相关部分---*/
//    static uint8_t work_shortage_flag = 0;
//    static uint8_t over_compensation_power_flag = 0;
//    static uint32_t DWT_Count = 0;
//    static uint32_t last_compensation_power = 0;//上一刻的补偿功率
//    static uint32_t last_charging_power = 0;    //上一刻的充电功率
//    static uint32_t work = 2000;//电容 补偿 功 最高容量2000J
//    static fp64 dt = 0;//时间积分
//    dt = DWT_GetDeltaT64(&DWT_Count);//使用double型积分更准
//    
//    /*---由于电机对称放置，因此有些参数需要取反，默认向前为正值---*/
//    int16_t give_current[4];//PID计算电流值
//    give_current[RF] = -New_Chassis_Data.PID_information.RF_Speed_PID.Dout;
//    give_current[RB] = -New_Chassis_Data.PID_information.RB_Speed_PID.Dout;
//    give_current[LB] =  New_Chassis_Data.PID_information.LB_Speed_PID.Dout;
//    give_current[LF] =  New_Chassis_Data.PID_information.LF_Speed_PID.Dout;

//    Chassis_Power_All = 0;//底盘总功率
//       
//    /*---转子转速---*/
//    //前走 L正 R负
//    speed[RF] = -New_Chassis_Data.Motor_information.RF_Motor->speed_data;
//    speed[RB] = -New_Chassis_Data.Motor_information.RB_Motor->speed_data;
//    speed[LB] =  New_Chassis_Data.Motor_information.LB_Motor->speed_data;
//    speed[LF] =  New_Chassis_Data.Motor_information.LF_Motor->speed_data;
//    /*---当前计算出的力矩电流---*/
//    //前走 L正 R负
//    I_out[RF] = -New_Chassis_Data.Set_information.RF_current_input;
//    I_out[RB] = -New_Chassis_Data.Set_information.RB_current_input;
//    I_out[LB] =  New_Chassis_Data.Set_information.LB_current_input;
//    I_out[LF] =  New_Chassis_Data.Set_information.LF_current_input;
//    /*---输出功率---*/
//    P_out[RF] = K * I_out[RF] * speed[RF];
//    P_out[RB] = K * I_out[RB] * speed[RB];
//    P_out[LB] = K * I_out[LB] * speed[LB];
//    P_out[LF] = K * I_out[LF] * speed[RF];
//    /*---输入功率---*/
//    P_in[RF] = P_out[RF] + K_1 * speed[RF]*speed[RF] + a * I_out[RF]*I_out[RF] + P_C620;
//    P_in[RB] = P_out[RB] + K_1 * speed[RB]*speed[RB] + a * I_out[RB]*I_out[RB] + P_C620;
//    P_in[LB] = P_out[LB] + K_1 * speed[LB]*speed[LB] + a * I_out[LB]*I_out[LB] + P_C620;
//    P_in[LF] = P_out[LF] + K_1 * speed[LF]*speed[LF] + a * I_out[LF]*I_out[LF] + P_C620;
//    /*---底盘输出功率总和---*/
//    for(int i = RF;i <= RB; i++)
//    {
//        if ( P_in[i] > 0 )  Chassis_Power_All += P_in[i];//忽略瞬时负功率
//    }    
//    /*---结算上一刻的超电剩余能量---*/
//    work -= last_compensation_power * dt;
//    work += last_charging_power *dt;
//    if (work > MAX_Super_capacitor_work)    work = MAX_Super_capacitor_work;
//    last_compensation_power = 0;
//    last_charging_power = 0;
//    
//    if ( Chassis_Power_All > Chassis_Power_Limit )//电容补偿功率
//    {
//        //超电能量不足
//        if ( work < 100 )   work_shortage_flag = 1;
//        else                work_shortage_flag = 0;
//        //超电补偿功率不足
//        if ( Chassis_Power_All > Chassis_Power_Limit + MAX_Super_capacitor_power )
//        {
//            over_compensation_power_flag = 1;
//            last_compensation_power = MAX_Super_capacitor_power;
//        }            
//        else
//        {
//            over_compensation_power_flag = 0;
//            last_compensation_power = Chassis_Power_All - Chassis_Power_Limit;
//        }            

//        if ( work_shortage_flag || over_compensation_power_flag)
//        {
//            fp32 divisor = (Chassis_Power_Limit + MAX_Super_capacitor_power*(1-work_shortage_flag)) / Chassis_Power_All;//缩放因子
//            /*---目标输入功率等比例缩小---*/
//            P_in[RF] = P_in[RF] * divisor;
//            P_in[RB] = P_in[RB] * divisor;
//            P_in[LB] = P_in[LB] * divisor;
//            P_in[LF] = P_in[LF] * divisor;
//            /*---计算该输入功率下的输出力矩---*/
//            for (int i = RF; i <= RB; i++)
//            {
//                if ( P_in[i] < 0 )    continue;
//                fp32 b = K * speed[i];
//                fp32 c = K_1 * speed[i] * speed[i] - P_in[i] + P_C620;
//                if ( give_current[i] > 0 )//加速状态
//                {
//                    fp32 temp = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
//                    if (temp > MOTOR_3508_CURRENT_LIMIT)    I_max[i] = MOTOR_3508_CURRENT_LIMIT;
//                    else                                    I_max[i] = temp;
//                }
//                else //减速
//                {
//                    fp32 temp = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
//                    if (temp > MOTOR_3508_CURRENT_LIMIT)    I_max[i] = MOTOR_3508_CURRENT_LIMIT;
//                    else                                    I_max[i] = temp;
//                }
//            }
//            
//            /*---更改发送給电调的数据---*/
//            New_Chassis_Data.Set_information.RF_current_input = -I_max[RF];
//            New_Chassis_Data.Set_information.RB_current_input = -I_max[RB];
//            New_Chassis_Data.Set_information.LB_current_input =  I_max[LB];
//            New_Chassis_Data.Set_information.LF_current_input =  I_max[LF];
//        }
//        else
//        {
//            last_charging_power = Chassis_Power_Limit-Chassis_Power_All;
//        }
//    }
//}

//void chassis_prevent_motion_distortion(void)
//{
//	float divisor = 1.0f;
//	if (New_Chassis_Data.Set_information.LF_current_input > MOTOR_3508_CURRENT_LIMIT ||
//	  	New_Chassis_Data.Set_information.RF_current_input > MOTOR_3508_CURRENT_LIMIT ||
//	  	New_Chassis_Data.Set_information.LB_current_input > MOTOR_3508_CURRENT_LIMIT ||
//	  	New_Chassis_Data.Set_information.RB_current_input > MOTOR_3508_CURRENT_LIMIT)
//	{
//		divisor = user_abs(MOTOR_3508_CURRENT_LIMIT / max(max(New_Chassis_Data.Set_information.LF_current_input, New_Chassis_Data.Set_information.RF_current_input), max(New_Chassis_Data.Set_information.LB_current_input, New_Chassis_Data.Set_information.RB_current_input)));
//	}
//	New_Chassis_Data.Set_information.LF_current_input = New_Chassis_Data.Set_information.LF_current_input * divisor;
//	New_Chassis_Data.Set_information.RF_current_input = New_Chassis_Data.Set_information.RF_current_input * divisor;
//	New_Chassis_Data.Set_information.LB_current_input = New_Chassis_Data.Set_information.LB_current_input * divisor;
//	New_Chassis_Data.Set_information.RB_current_input = New_Chassis_Data.Set_information.RB_current_input * divisor;
//}

///*
// *函数名称：Motion_Calc
// *函数参数：void
// *函数返回：void
// *函数功能：运动麦轮解算
// *特别说明：根据底盘电机的在线情况选择对应情况解算，
//            并且由于电机对称放置，计算出的左侧轮子速度方向已经取反，后续电流无需取反
// */
//void Motion_Calc(void)
//{
//    ALL_OK_Motion_Calc(); 
////    switch(New_Chassis_Data.Motor_information.motor_state)
////    {
////        case ALL_OK:
////            ALL_OK_Motion_Calc();
////            break;
////        case RF_MISS:
////            Miss_RF_Motion_Calc();
////            break;
////        case LF_MISS:
////            Miss_LF_Motion_Calc();
////            break;
////        case LB_MISS:
////            Miss_LB_Motion_Calc();
////            break;
////        case RB_MISS:
////            Miss_RB_Motion_Calc();
////            break;
////        default:
////			break;
////    }
//}

////int is_accelerate(int16_t old_speed,int16_t new_speed)
////{
////    if (old_speed >= 0 && new_speed >= 0 && new_speed > old_speed)  return 1;
////    if (old_speed <= 0 && new_speed <= 0 && new_speed > old_speed)  return 1;
////    if (old_speed <= 0 && new_speed > 0);
////}