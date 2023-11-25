#include "protect_task.h"
#include "bsp_dr16.h"

extern DMA_HandleTypeDef hdma_usart3_rx;

/*--------------------变量-----------------------*/
//int safecheck_heart = 0; //安全检测任务心跳

const RC_ctrl_t *rc_security;
uint8_t safecheck_task_run = 0;

int Offline_time;
/*--------------------函数-----------------------*/
uint32_t (*rc_lost_time)(void);   
extern void __set_FAULTMASK(uint32_t faultMask);

void Protect_Task(void *pvParameters)
{
  TickType_t currentTime;
	
	rc_lost_time = chassis_rc_lost_time;
	
	  rc_security = RC_Get_RC_Pointer();
	
	  currentTime = xTaskGetTickCount(); //获取当前系统时间
	
	  while(1)
	  {
				currentTime = xTaskGetTickCount();//当前系统时间
			
//		    //调用vTaskSuspend函数是不会累计的：即使多次调用 vTaskSuspend ()函数将一个任务挂起，也只需调用一次vTaskResume ()函数就能使挂起的任务解除挂起状态。
//        Offline_time = currentTime - rc_lost_time();
//        if (currentTime >= rc_lost_time() || rc_data_is_error() == 1) //遥控失联时间太长 或 遥控器数据错误
//        {
//            out_of_control(); //挂起正常任务、开启异常处理任务
//        }
//	    else
//        {
//            normal_control(); //恢复正常任务
//        }
//			}
			 //检测周期
	  	vTaskDelayUntil(&currentTime, 2);//绝对延时//vTaskDelay(2)
		}
}

