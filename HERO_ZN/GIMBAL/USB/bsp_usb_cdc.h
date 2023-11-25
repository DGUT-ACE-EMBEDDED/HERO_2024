#ifndef __BSP_USB_H
#define __BSP_USB_H
#include "main.h"
#include "ECF_config.h"
#include "usbd_cdc_if.h"

void printf_usb(const char *format, ...);
void USB_Reset(void);


#endif


