#include "data_handle.h"

#define PATHFINDER_DATA_BAG_LENTH    0x10
#define PATHFINDER_DATA_BAG_RX_LENTH 0x20

uint8_t PATHFINDER_DATA_BAG_RX_BUFFER[2][PATHFINDER_DATA_BAG_RX_LENTH];
PATHFINDER_DATA_BAG Pathfinder_data_bag;//底盘数据包

GIMBAL_DATA_BAG     gimbal_data_bag;//云台数据包

/****函数声明****/

void Rx_buffer_data_handle(void);
uint8_t CRC_cal(void);

uint8_t CRC_cal(void)
{
    Pathfinder_data_bag.crc_verify = 0;
    Pathfinder_data_bag.crc_verify = PATHFINDER_DATA_BAG_RX_BUFFER[1][2] ^ PATHFINDER_DATA_BAG_RX_BUFFER[1][3];
    for (int i = 4; i < 14; i++)
    {
        Pathfinder_data_bag.crc_verify ^= PATHFINDER_DATA_BAG_RX_BUFFER[1][i];
    }
    if (Pathfinder_data_bag.crc_verify != PATHFINDER_DATA_BAG_RX_BUFFER[1][14])
    {
        return 0;
    }
    else
        return 1;
}

void Rx_buffer_data_handle(void)
{
    int i = 0;
    for (int j = 0; j < PATHFINDER_DATA_BAG_RX_LENTH - 1; j++)
    {
        if ((PATHFINDER_DATA_BAG_RX_BUFFER[0][j] == 0xBB) && (PATHFINDER_DATA_BAG_RX_BUFFER[0][j + 1] == 0xBB))
        {
            for (i = 0; i < 15; i++)
            {
                PATHFINDER_DATA_BAG_RX_BUFFER[1][i] = PATHFINDER_DATA_BAG_RX_BUFFER[0][j + i];
            }
            break;
        }
    }
}

/**
 * @brief 底盘数据包接收处理
 */
void pathfinder_data_handle(uint8_t* Buf, uint32_t *Len)
{
//	uint32_t data_bag_lenth  = PATHFINDER_DATA_BAG_LENTH;


    for(int i = 0; i < *Len; i++)
    {
        PATHFINDER_DATA_BAG_RX_BUFFER[0][i] = Buf[i];
    }

    Rx_buffer_data_handle();
    if (CRC_cal())
    {
        Pathfinder_data_bag.head[0] = PATHFINDER_DATA_BAG_RX_BUFFER[1][0];
        Pathfinder_data_bag.head[1] = PATHFINDER_DATA_BAG_RX_BUFFER[1][1];

        Pathfinder_data_bag.x.ch_linear_x[0] = PATHFINDER_DATA_BAG_RX_BUFFER[1][2];
        Pathfinder_data_bag.x.ch_linear_x[1] = PATHFINDER_DATA_BAG_RX_BUFFER[1][3];
        Pathfinder_data_bag.x.ch_linear_x[2] = PATHFINDER_DATA_BAG_RX_BUFFER[1][4];
        Pathfinder_data_bag.x.ch_linear_x[3] = PATHFINDER_DATA_BAG_RX_BUFFER[1][5];

        Pathfinder_data_bag.y.ch_linear_y[0] = PATHFINDER_DATA_BAG_RX_BUFFER[1][6];
        Pathfinder_data_bag.y.ch_linear_y[1] = PATHFINDER_DATA_BAG_RX_BUFFER[1][7];
        Pathfinder_data_bag.y.ch_linear_y[2] = PATHFINDER_DATA_BAG_RX_BUFFER[1][8];
        Pathfinder_data_bag.y.ch_linear_y[3] = PATHFINDER_DATA_BAG_RX_BUFFER[1][9];

        Pathfinder_data_bag.v.ch_angular_v[0] = PATHFINDER_DATA_BAG_RX_BUFFER[1][10];
        Pathfinder_data_bag.v.ch_angular_v[1] = PATHFINDER_DATA_BAG_RX_BUFFER[1][11];
        Pathfinder_data_bag.v.ch_angular_v[2] = PATHFINDER_DATA_BAG_RX_BUFFER[1][12];
        Pathfinder_data_bag.v.ch_angular_v[3] = PATHFINDER_DATA_BAG_RX_BUFFER[1][13];
    }
}

/**
 * @brief 云台数据包接收处理
 */
void gimbal_data_handle(uint8_t* Buf, uint32_t *Len)
{
    if (*Len != 8)   return;//接收长度错误
    for(int i = 0; i < 7; i++)
    {
        gimbal_data_bag.data[i] = Buf[i];
    }
    if(gimbal_data_bag.data[0] == 0xFF && gimbal_data_bag.data[7] == 0xFE)
	{
			gimbal_data_bag.auto_yaw   = -(((float)((int16_t)(gimbal_data_bag.data[2]<<8 | gimbal_data_bag.data[1]))) / 100.0f);        //自动打击的y轴角度计算/100
			gimbal_data_bag.auto_pitch = -(((float)((int16_t)(gimbal_data_bag.data[4]<<8 | gimbal_data_bag.data[3]))) / 100.0f);      //自动打击的p轴角度计算
//			Virtual_recive_p->auto_pitch_speed = (read_buff[5]<<8 | read_buff[6]) / 10000; //自动打击的p轴角度计算/100
	}
}

/**
 * @brief 返回结构体指针
 */
PATHFINDER_DATA_BAG *Return_PATHFINDER_DATA_BAG_pointer(void) 
{ 
    return &Pathfinder_data_bag; 
}

void PATHFINDER_DATA_BAG_init(void)
{
    Pathfinder_data_bag.head[0] = 0;
    Pathfinder_data_bag.head[1] = 0;

    Pathfinder_data_bag.x.ch_linear_x[0] = 0;
    Pathfinder_data_bag.x.ch_linear_x[1] = 0;
    Pathfinder_data_bag.x.ch_linear_x[2] = 0;
    Pathfinder_data_bag.x.ch_linear_x[3] = 0;

    Pathfinder_data_bag.y.ch_linear_y[0] = 0;
    Pathfinder_data_bag.y.ch_linear_y[1] = 0;
    Pathfinder_data_bag.y.ch_linear_y[2] = 0;
    Pathfinder_data_bag.y.ch_linear_y[3] = 0;

    Pathfinder_data_bag.v.ch_angular_v[0] = 0;
    Pathfinder_data_bag.v.ch_angular_v[1] = 0;
    Pathfinder_data_bag.v.ch_angular_v[2] = 0;
    Pathfinder_data_bag.v.ch_angular_v[3] = 0;

    Pathfinder_data_bag.crc_verify = 0;
}

//返回云台数据包指针
GIMBAL_DATA_BAG *RC_Get_AUTO_Pointer(void)
{
    return &gimbal_data_bag;
}

//void PATHFINDER_DATA_COOMUNICATE_TASK(void)
//{
//    pathfinder_data_handle();
//}

