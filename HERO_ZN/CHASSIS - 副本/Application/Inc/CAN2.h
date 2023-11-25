#ifndef __CAN2_H
#define __CAN2_H

void CAN2_Init(void);
void CAN1_filter_config(void);
void CAN2_RX_Deal(CAN_HandleTypeDef *hcan);
void CAN2_Send(void);
void CAN2_Send_Gambal(void);

#endif
