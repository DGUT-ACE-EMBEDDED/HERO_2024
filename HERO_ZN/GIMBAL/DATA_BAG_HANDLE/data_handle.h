#ifndef __DATA_HANDLE_H
#define __DATA_HANDLE_H
#include "ECF_config.h"


/*
 * @brief 导航组定义的底盘数据包, 使用union快速转型
*/
typedef struct 
{//两个帧头, 初始化时需要将其初始化为 0xbb
    uint8_t head[2];
    //    
    union 
    {
        uint8_t ch_linear_x[4];
        float   f_linear_x;
    }x;
    union 
    {
        uint8_t ch_linear_y[4];
        float   f_linear_y;
    }y;
    union 
    {
        uint8_t ch_angular_v[4];
        float   f_angular_v;
    }v;
    //使用异或校验进行验证, 需要对前面每个字节数据内容进行验证
    uint8_t crc_verify;
}PATHFINDER_DATA_BAG;

/*
 * @brief 视觉组定义的云台数据包
*/
typedef struct 
{
    uint8_t data[8];
    float   auto_yaw;
    float   auto_pitch;
}GIMBAL_DATA_BAG;


//返回云台数据包指针
GIMBAL_DATA_BAG *RC_Get_AUTO_Pointer(void);

void pathfinder_data_handle(uint8_t *Buf, uint32_t *Len);
void gimbal_data_handle(uint8_t *Buf, uint32_t *Len);

PATHFINDER_DATA_BAG *Return_PATHFINDER_DATA_BAG_pointer(void);
void PATHFINDER_DATA_BAG_init(void);
//void PATHFINDER_DATA_COOMUNICATE_TASK(void);

#endif

