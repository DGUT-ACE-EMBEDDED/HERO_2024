#include "fire_task.h"
#include "chassis_task.h"
#include "pid.h"
#include "PID_Data.h"

extern Chassis_Task_t Chassis_Data;

static void Fire_Init(Chassis_Task_t *Chassis_Data);
static void Fire_Work(Chassis_Task_t *Chassis_Data);

void Fire_Task(void const *argument)
{
	Fire_Init(&Chassis_Data);
	while(1)
	{
		taskENTER_CRITICAL(); //�����ٽ���
		Fire_Work(&Chassis_Data);
		taskEXIT_CRITICAL(); //�˳��ٽ���
		vTaskDelay(1);
	}
}

void Fire_Init(Chassis_Task_t *Chassis_Data)
{
	Chassis_Data->fire_state = FIRE_NO_ACTION;
	PidInit(&Chassis_Data->FIRE_Speed_PID, FIRE_MOTOR_S_KP, FIRE_MOTOR_S_KI, FIRE_MOTOR_S_KD, Output_Limit);
	PidInitMode(&Chassis_Data->FIRE_Speed_PID,Output_Limit, FIRE_3508_CURRENT_LIMIT, 0);	
	
	PidInit(&Chassis_Data->FIRE_Position_PID, FIRE_MOTOR_P_KP, FIRE_MOTOR_P_KI, FIRE_MOTOR_P_KD, Integral_Limit | Output_Limit);
	PidInitMode(&Chassis_Data->FIRE_Position_PID,Output_Limit, FIRE_3508_CURRENT_LIMIT, 0);	
	PidInitMode(&Chassis_Data->FIRE_Position_PID,Integral_Limit, 150 , 0);
    
	EncoderValZero(Chassis_Data->Motor_encoder[4]);
	Chassis_Data->fire_set_position = 0;
}

void Fire_Work(Chassis_Task_t *Chassis_Data)
{
    static int count = 0;
    static int ready_fire = 1;
    static int32_t last_fire_set_position = 0;
    
    fire_state_e last_fire_state = Chassis_Data->fire_state;
    
    if(Chassis_Data->RC->rc.s2 == RC_SW_UP || Chassis_Data->RC->rc.s2 == RC_SW_MID)//���̿�ʼ�ж�
    {

        Chassis_Data->fire_state = FIRE_ACTION;
        
        //�Ӹ����ֻ�����⣬��ֹ����
        if ( Chassis_Data->fire_on_flag == 0 && Chassis_Data->RC->rc.ch[4] <= 200 && Chassis_Data->RC->mouse.press_l ==0)
            ready_fire = 1;
        
        //�������
        if((Chassis_Data->RC->rc.ch[4] >= 650 || Chassis_Data->RC->mouse.press_l ==1) && Chassis_Data->fire_on_flag == 0 && ready_fire)
        {
            count = 0;//��տ���������
            ready_fire = 0;
            Chassis_Data->fire_on_flag = 1; //�����б�־
            Chassis_Data->fire_detect_flag = 0;//�˵���־
            last_fire_set_position =  Chassis_Data->fire_set_position;//��¼��ǰλ�ã������˵���
            Chassis_Data->fire_set_position += 36864;
        }
        Chassis_Data->FIRE_current_input = motor_position_speed_control(&Chassis_Data->FIRE_Speed_PID, &Chassis_Data->FIRE_Position_PID, Chassis_Data->fire_set_position, Chassis_Data->Motor_encoder[4]->Encode_Record_Val ,Chassis_Data->FIRE_Motor->speed_data);	
    }
    else//ȫ���˵�
    {
        Chassis_Data->fire_state = FIRE_NO_ACTION;
        Chassis_Data->fire_on_flag = 0;

        if ( last_fire_state == FIRE_ACTION && Chassis_Data->fire_detect_flag == 0 )
        {
            Chassis_Data->fire_detect_flag = 1;
            Chassis_Data->fire_set_position -= 36864;
        }       
        
        if ( Chassis_Data->fire_detect_flag == 1 )
        {
            Chassis_Data->FIRE_current_input = motor_position_speed_control(&Chassis_Data->FIRE_Speed_PID, 
                                                                            &Chassis_Data->FIRE_Position_PID, 
                                                                             Chassis_Data->fire_set_position, 
                                                                             Chassis_Data->Motor_encoder[4]->Encode_Record_Val ,
                                                                             Chassis_Data->FIRE_Motor->speed_data);	
        }
        else
        {
            Chassis_Data->FIRE_current_input = 0;
        }
        
        if ( Chassis_Data->Motor_encoder[4]->Encode_Record_Val <= Chassis_Data->fire_set_position )
            Chassis_Data->fire_detect_flag = 0;
        
    }
    
    
    //������Ƿ��������ԭ�� �� ��������� λ�û� I �������ۣ�����ֵ��ʱ�䵽����ֵ �ж�Ϊ�������
    if (Chassis_Data->fire_on_flag == 1)
    {
        if (Chassis_Data->FIRE_Speed_PID.Ierror == 150)    count++;
        if (count > 3500) //��ת����ת�˵�
        {
            Chassis_Data->fire_state = FIRE_LOCK_POSITION;
            Chassis_Data->fire_detect_flag = 1;//�˵���־
            count = 0;
        }
    }
        
    //�˵���
    if ( Chassis_Data->fire_detect_flag == 1 )
    {
        if (Chassis_Data->Motor_encoder[4]->Encode_Record_Val > last_fire_set_position-36084/2)//���˰�����ľ���
        {
            //Chassis_Data->FIRE_current_input = 0;//������0�������˵�
            //���������ת�˵�
            Chassis_Data->FIRE_current_input = motor_position_speed_control(&Chassis_Data->FIRE_Speed_PID, 
                                                                            &Chassis_Data->FIRE_Position_PID, 
                                                                             last_fire_set_position-36084/2, 
                                                                             Chassis_Data->Motor_encoder[4]->Encode_Record_Val ,
                                                                             Chassis_Data->FIRE_Motor->speed_data);	
        }            
        else 
        {
            Chassis_Data->fire_detect_flag = 0;//�˵���ɣ������ϵ�

            Chassis_Data->FIRE_current_input = motor_position_speed_control(&Chassis_Data->FIRE_Speed_PID, &Chassis_Data->FIRE_Position_PID, Chassis_Data->fire_set_position, Chassis_Data->Motor_encoder[4]->Encode_Record_Val ,Chassis_Data->FIRE_Motor->speed_data);	
        }            
    }
    
    // �������
    if(Chassis_Data->Motor_encoder[4]->Encode_Record_Val >= Chassis_Data->fire_set_position)
    {
        Chassis_Data->fire_on_flag = 0;
    }
}


