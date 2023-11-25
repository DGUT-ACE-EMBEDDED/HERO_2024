#include "protect_task.h"
#include "bsp_dr16.h"

extern DMA_HandleTypeDef hdma_usart3_rx;

/*--------------------����-----------------------*/
//int safecheck_heart = 0; //��ȫ�����������

const RC_ctrl_t *rc_security;
uint8_t safecheck_task_run = 0;

int Offline_time;
/*--------------------����-----------------------*/
uint32_t (*rc_lost_time)(void);   
extern void __set_FAULTMASK(uint32_t faultMask);

void Protect_Task(void *pvParameters)
{
  TickType_t currentTime;
	
	rc_lost_time = chassis_rc_lost_time;
	
	  rc_security = RC_Get_RC_Pointer();
	
	  currentTime = xTaskGetTickCount(); //��ȡ��ǰϵͳʱ��
	
	  while(1)
	  {
				currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
			
//		    //����vTaskSuspend�����ǲ����ۼƵģ���ʹ��ε��� vTaskSuspend ()������һ���������Ҳֻ�����һ��vTaskResume ()��������ʹ���������������״̬��
//        Offline_time = currentTime - rc_lost_time();
//        if (currentTime >= rc_lost_time() || rc_data_is_error() == 1) //ң��ʧ��ʱ��̫�� �� ң�������ݴ���
//        {
//            out_of_control(); //�����������񡢿����쳣��������
//        }
//	    else
//        {
//            normal_control(); //�ָ���������
//        }
//			}
			 //�������
	  	vTaskDelayUntil(&currentTime, 2);//������ʱ//vTaskDelay(2)
		}
}

