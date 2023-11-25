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

static Fire_State_e ON_OR_OFF = NO_Fire_FORCE;//Ħ���ֿ���״̬
void fire_work(void)
{
    static uint8_t Fist_set = 1;//�״�����Ŀ��ֵ
    static uint8_t Block = 0;//��ת��־
    static uint16_t On_Fire_Block_time = 0;//������תʱ��
    
    //����Ŀ��λ��
    if (fire_task.Robot_cmd->Fire_State == On_Fire)
    {
        if (Fist_set == 1)
        {
            Fist_set = 0;
            Block = 0;
            fire_task.set_position += 36864;
            On_Fire_Block_time = 0;//��¼��ʼʱ��
        }       
        On_Fire_Block_time++;        
        if (On_Fire_Block_time > 4000){//��ʱ�䴦�ڽ���״̬����Ϊ�ǿ������˵�����
            Fist_set = 1;
            Block = 1;
            fire_task.Robot_cmd->Fire_State = On_Empty;//�˵�״̬
        }
    }
    else if (fire_task.Robot_cmd->Fire_State == On_Empty)
    {
        if (Fist_set == 1)
        {
            Fist_set = 0;
            //��Ϊ��ת�����˵�ģʽ����ǰ��ת��λ�û��˰����
            if (Block == 1)
                fire_task.set_position = fire_task.Fire_Motor.Motor_encoder->Encode_Record_Val - 20000;
            else                
                fire_task.set_position -= 36864;            //���������˵�ģʽ������ȫ����
        }      
    }
    else    ON_OR_OFF = fire_task.Robot_cmd->Fire_State;//����Ħ���ֿ���״̬�������˵�ģʽ��ɺ���
    
    //�������г�
    if (fire_task.Robot_cmd->Fire_State == On_Fire && fire_task.Fire_Motor.Motor_encoder->Encode_Record_Val > fire_task.set_position-4000)//�������г�
    {
        fire_task.Robot_cmd->Fire_State = Ready_Fire;
        Fist_set = 1;
    }
    //����˵��г�
    if (fire_task.Robot_cmd->Fire_State == On_Empty && fire_task.Fire_Motor.Motor_encoder->Encode_Record_Val < fire_task.set_position+4000)//����˵��г�
    {
        if(Block == 1)   fire_task.Robot_cmd->Fire_State = On_Fire;//��ת����˵������½���
        else                fire_task.Robot_cmd->Fire_State = ON_OR_OFF;//�����˵�ʱ��Ħ����״̬
        Fist_set = 1;
    }
            
    //λ���ٶȻ�����
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
		taskENTER_CRITICAL(); //�����ٽ���
        Fire_Set();
		fire_work();
        Fire_acion();
		taskEXIT_CRITICAL(); //�˳��ٽ���
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
//		taskENTER_CRITICAL(); //�����ٽ���
//		Fire_Work(&Chassis_Data);
//		taskEXIT_CRITICAL(); //�˳��ٽ���
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
//    if(Chassis_Data->RC->rc.s2 == RC_SW_UP || Chassis_Data->RC->rc.s2 == RC_SW_MID)//���̿�ʼ�ж�
//    {

//        Chassis_Data->fire_state = FIRE_ACTION;
//        
//        //�Ӹ����ֻ�����⣬��ֹ����
//        if ( Chassis_Data->fire_on_flag == 0 && Chassis_Data->RC->rc.ch[4] <= 200 && Chassis_Data->RC->mouse.press_l ==0)
//            ready_fire = 1;
//        
//        //�������
//        if((Chassis_Data->RC->rc.ch[4] >= 650 || Chassis_Data->RC->mouse.press_l ==1) && Chassis_Data->fire_on_flag == 0 && ready_fire)
//        {
//            count = 0;//��տ���������
//            ready_fire = 0;
//            Chassis_Data->fire_on_flag = 1; //�����б�־
//            Chassis_Data->fire_detect_flag = 0;//�˵���־
//            last_fire_set_position =  Chassis_Data->fire_set_position;//��¼��ǰλ�ã������˵���
//            Chassis_Data->fire_set_position += 36864;
//        }
//        Chassis_Data->FIRE_current_input = motor_position_speed_control(&Chassis_Data->FIRE_Speed_PID, &Chassis_Data->FIRE_Position_PID, Chassis_Data->fire_set_position, Chassis_Data->Motor_encoder[4]->Encode_Record_Val ,Chassis_Data->FIRE_Motor->speed_data);	
//    }
//    else//ȫ���˵�
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
//    //������Ƿ��������ԭ�� �� ��������� λ�û� I �������ۣ�����ֵ��ʱ�䵽����ֵ �ж�Ϊ�������
//    if (Chassis_Data->fire_on_flag == 1)
//    {
//        if (Chassis_Data->FIRE_Speed_PID.Ierror == 150)    count++;
//        if (count > 3500) //��ת����ת�˵�
//        {
//            Chassis_Data->fire_state = FIRE_LOCK_POSITION;
//            Chassis_Data->fire_detect_flag = 1;//�˵���־
//            count = 0;
//        }
//    }
//        
//    //�˵���
//    if ( Chassis_Data->fire_detect_flag == 1 )
//    {
//        if (Chassis_Data->Motor_encoder[4]->Encode_Record_Val > last_fire_set_position-36084/2)//���˰�����ľ���
//        {
//            //Chassis_Data->FIRE_current_input = 0;//������0�������˵�
//            //���������ת�˵�
//            Chassis_Data->FIRE_current_input = motor_position_speed_control(&Chassis_Data->FIRE_Speed_PID, 
//                                                                            &Chassis_Data->FIRE_Position_PID, 
//                                                                             last_fire_set_position-36084/2, 
//                                                                             Chassis_Data->Motor_encoder[4]->Encode_Record_Val ,
//                                                                             Chassis_Data->FIRE_Motor->speed_data);	
//        }            
//        else 
//        {
//            Chassis_Data->fire_detect_flag = 0;//�˵���ɣ������ϵ�

//            Chassis_Data->FIRE_current_input = motor_position_speed_control(&Chassis_Data->FIRE_Speed_PID, &Chassis_Data->FIRE_Position_PID, Chassis_Data->fire_set_position, Chassis_Data->Motor_encoder[4]->Encode_Record_Val ,Chassis_Data->FIRE_Motor->speed_data);	
//        }            
//    }
//    
//    // �������
//    if(Chassis_Data->Motor_encoder[4]->Encode_Record_Val >= Chassis_Data->fire_set_position)
//    {
//        Chassis_Data->fire_on_flag = 0;
//    }
//}


