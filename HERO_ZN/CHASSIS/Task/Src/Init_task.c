#include "Init_task.h"
#include "cmsis_os.h"
#include "cmsis_armcc.h"

/************************* bsp ************************/
#include "bsp_can.h"
#include "bsp_dr16.h"
#include "bsp_referee.h"
#include "CAN1.h"
#include "CAN2.h"
/************************* Task ************************/
#include "chassis_task.h"
#include "fire_task.h"
#include "imu_task.h"
#include "safe_task.h"
/* ************************freertos******************** */
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

extern osThreadId Init_TASKHandle;
extern osThreadId defaultTaskHandle;

//osThreadId Safe_TASKHandle;
//osThreadId UI_TASKHandle;
osThreadId Chassis_TASKHandle;
osThreadId Fire_TaskHandler;
osThreadId TASK_CHASSISHandle;
osThreadId IMUTask_Handler;

void init_Task(void const *argument)
{
		taskENTER_CRITICAL(); //Ϊ�˱�֤��PORTA�Ĵ����ķ��ʲ����жϣ������ʲ��������ٽ����������ٽ���
	
		// CAN��ʼ��
		CAN1_Init();
		CAN2_Init();
	
		//ң������ʼ��
		ECF_RC_Init();
	
        //����ϵͳ��ʼ��
		ECF_referee_uart_init();
	
		//������������
//		osThreadDef(Chassis_task, Task_Chassis, osPriorityHigh, 0, 256);
//		Chassis_TASKHandle = osThreadCreate(osThread(Chassis_task), NULL);
	
		//�����������
		osThreadDef(FIRE_TASK, Fire_Task, osPriorityHigh, 0, 128);
 		Fire_TaskHandler = osThreadCreate(osThread(FIRE_TASK), NULL);

		// IMU
//		osThreadDef(IMU_TASK, imu_Task, osPriorityNormal, 0, 512);
//		IMUTask_Handler = osThreadCreate(osThread(IMU_TASK), NULL);
        
        //������ȫ����
//        osThreadDef(SAFE_TASK, Safe_Task, osPriorityNormal, 0, 128);
//		Chassis_TASKHandle = osThreadCreate(osThread(SAFE_TASK), NULL);
	
		vTaskDelete(Init_TASKHandle); //ɾ����ʼ����
		vTaskDelete(defaultTaskHandle);
		taskEXIT_CRITICAL();		  //�˳��ٽ���
}

