/************************** Dongguan-University of Technology -ACE**************************
 * @file    bsp_usb.c
 * @brief 
 * @author  wuage2335 
 * @version 1.1
 * @date    2023-07-18
 * 
 * @note    侯文辉先生在1.0版本中先写好了对printf函数的重定向, 
 *          wuage2335则是在侯文辉先生写好的文档的基础上根据需求增加了一些内容
 * @history
 * <table>
 * Date       Version Author Description
 * 2022-10-11   1.0     pansyhou侯文辉(1677195845lyb@gmail.com)    
 * 2023-07-18   1.1     wuage2335
 * @verbatim 
 * ==============================================================================
 * ==============================================================================
 * @endverbatim
************************** Dongguan-University of Technology -ACE***************************/
#include "bsp_usb_cdc.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "stdarg.h"

/**
 * 重写printf,通过usbcdc虚拟串口
 * 注意：buff可能要足够大，不然发送乱码或者不全
 * @param format
 * @param ...
 */
void printf_usb(const char *format, ...){
    va_list  args;
    uint32_t length;
    uint8_t buff[APP_TX_DATA_SIZE];

    va_start(args, format);
    length = vsnprintf((char *)buff, APP_TX_DATA_SIZE, (char *)format, args);
    va_end(args);
    CDC_Transmit_FS(buff, length);
}


/**
 * @brief   USB重新枚举函数, 该函数需要放到 USB 初始化之前
 * @note    在每次芯片下载完后需要重新枚举一次串口才能做到对 USB 重新识别, 
 *          具体枚举方法就是将 USB_DP(PA12) 拉低一段时间即可
 */
void USB_Reset(void)
{
  	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
}
