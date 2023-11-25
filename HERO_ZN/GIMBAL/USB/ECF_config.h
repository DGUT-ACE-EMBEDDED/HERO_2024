#ifndef __ECF_CONFIG_H
#define __ECF_CONFIG_H

#include "struct_typedef.h"


#define _Use_F4xx_HAL 1
#define _Use_F1xx_HAL 0

#if _Use_F4xx_HAL
#include "stm32f4xx_hal.h"
#endif

#if _Use_F1xx_HAL
#include "stm32f1xx_hal.h"
#endif





#endif

