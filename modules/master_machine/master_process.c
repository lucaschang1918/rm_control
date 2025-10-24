/**
* @file master_process.c
* @author neozng
* @brief  module for recv&send vision data
* @version beta
* @date 2022-11-03
* @todo 增加对串口调试助手协议的支持,包括vofa和serial debug
* @copyright Copyright (c) 2022
*
*/
#include "master_process.h"
#include "seasky_protocol.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"
#include "crc_ref.h"

#include "bsp_dwt.h"

#include  "usart.h"

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;





//=================================================
//电专通信
static ReceiverPacket rece_packet;
static SendPacket send_packet;

float Vison_Frequency; // 视觉数据频率

static DaemonInstance *vision_daemon_instance;


void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed)
{
 send_data.enemy_color = enemy_color;
 send_data.work_mode = work_mode;
 send_data.bullet_speed = bullet_speed;
}


/**
* @brief 离线回调函数,将在daemon.c中被daemon task调用
* @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
*            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
*
* @param id vision_usart_instance的地址,此处没用.
*/
static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
 USARTServiceInit(vision_usart_instance);
#endif // !VISION_USE_UART
 LOGWARNING("[vision] vision offline, restart communication.");
}

#ifdef VISION_USE_UART

#include "bsp_usart.h"

static USARTInstance *vision_usart_instance;

/**
* @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
* @todo  1.提高可读性,将get_protocol_info的第四个参数增加一个float类型buffer
*        2.添加标志位解码
*/
static void DecodeVision()
{
 uint16_t flag_register;
 DaemonReload(vision_daemon_instance); // 喂狗
 get_protocol_info(vision_usart_instance->recv_buff, &flag_register, (uint8_t *)&recv_data.pitch);
 // TODO: code to resolve flag_register;
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
 USART_Init_Config_s conf;
 conf.module_callback = DecodeVision;
 conf.recv_buff_size = VISION_RECV_SIZE;
 conf.usart_handle = _handle;
 vision_usart_instance = USARTRegister(&conf);

 // 为master process注册daemon,用于判断视觉通信是否离线
 Daemon_Init_Config_s daemon_conf = {
     .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
     .owner_id = vision_usart_instance,
     .reload_count = 10,
 };
 vision_daemon_instance = DaemonRegister(&daemon_conf);

 return &recv_data;
}

/**
* @brief 发送函数
*
* @param send 待发送数据
*
*/
void VisionSend()
{
 // buff和txlen必须为static,才能保证在函数退出后不被释放,使得DMA正确完成发送
 // 析构后的陷阱需要特别注意!
 static uint16_t flag_register;
 static uint8_t send_buff[VISION_SEND_SIZE];
 static uint16_t tx_len;
 // TODO: code to set flag_register
 flag_register = 30 << 8 | 0b00000001;
 // 将数据转化为seasky协议的数据包
 get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
 USARTSend(vision_usart_instance, send_buff, tx_len, USART_TRANSFER_DMA); // 和视觉通信使用IT,防止和接收使用的DMA冲突
                                                                          // 此处为HAL设计的缺陷,DMASTOP会停止发送和接收,导致再也无法进入接收中断.
                                                                          // 也可在发送完成中断中重新启动DMA接收,但较为复杂.因此,此处使用IT发送.
                                                                          // 若使用了daemon,则也可以使用DMA发送.
}

#endif // VISION_USE_UART

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
static uint8_t *vis_recv_buff;

//static void DecodeVision(uint16_t recv_len)
//{
//    uint16_t flag_register;
//    get_protocol_info(vis_recv_buff, &flag_register, (uint8_t *)&recv_data.pitch);
//    // TODO: code to resolve flag_register;
//}

enum ARMOR_ID
{
 ARMOR_OUTPOST = 0,
 ARMOR_HERO = 1,
 ARMOR_ENGINEER = 2,
 ARMOR_INFANTRY3 = 3,
 ARMOR_INFANTRY4 = 4,
 ARMOR_INFANTRY5 = 5,
 ARMOR_GUARD = 6,
 ARMOR_BASE = 7
};

enum ARMOR_NUM
{
 ARMOR_NUM_BALANCE = 2,
 ARMOR_NUM_OUTPOST = 3,
 ARMOR_NUM_NORMAL = 4
};

enum BULLET_TYPE
{
 BULLET_17 = 0,
 BULLET_42 = 1
};

//设置参数
struct SolveTrajectoryParams
{
 float32_t k;             //弹道系数

 //自身参数
 enum BULLET_TYPE bullet_type;  //自身机器人类型 0-步兵 1-英雄
 float32_t current_v;      //当前弹速
 float32_t current_pitch;  //当前pitch
 float32_t current_yaw;    //当前yaw

 //目标参数
 float32_t xw;             //ROS坐标系下的x
 float32_t yw;             //ROS坐标系下的y
 float32_t zw;             //ROS坐标系下的z
 float32_t vxw;            //ROS坐标系下的vx
 float32_t vyw;            //ROS坐标系下的vy
 float32_t vzw;            //ROS坐标系下的vz
 float32_t tar_yaw;        //目标yaw
 float32_t v_yaw;          //目标yaw速度
 float32_t set_yaw;
 float32_t ref_yaw;
 float32_t alo_max;
 float32_t alo_min;
 int shoot_flag;
 float32_t lastv_yaw;      //上一次目标yaw速度
 float32_t r1;             //目标中心到前后装甲板的距离
 float32_t r2;             //目标中心到左右装甲板的距离
 float32_t dz;             //另一对装甲板的相对于被跟踪装甲板的高度差
 int bias_time;        //偏置时间
 float32_t s_bias;         //枪口前推的距离
 float32_t z_bias;         //yaw轴电机到枪口水平面的垂直距离
 enum ARMOR_ID armor_id;     //装甲板类型  0-outpost 6-guard 7-base
                         //1-英雄 2-工程 3-4-5-步兵
 enum ARMOR_NUM armor_num;   //装甲板数字  2-balance 3-outpost 4-normal
};

static struct SolveTrajectoryParams st;
static union
{
 float flo;
 char buf[4];
} HexFloat;
// number1,number2
float write_hex_to_float(uint8_t number1, uint8_t number2, uint8_t number3, uint8_t number4)
{
 HexFloat.buf[0] = number1;
 HexFloat.buf[1] = number2;
 HexFloat.buf[2] = number3;
 HexFloat.buf[3] = number4;
 float fdata = HexFloat.flo;
 return fdata;
}



static void DecodeVision(uint16_t recv_len)
{

 DaemonReload(vision_daemon_instance); // 喂狗,防止离线
 static uint32_t counter;
 float dt = DWT_GetDeltaT(&counter);
 Vison_Frequency = 1.0f / dt; // 计算频率

 //  vis_recv_buff //usb 收到的数据缓冲区
 //recv_data->rec 电专解包之后的数据
 static uint8_t index = 0;
 int16_t  data_length_version = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
 Verify_CRC16_Check_Sum(vis_recv_buff, sizeof(vis_recv_buff) - data_length_version);
 if (vis_recv_buff[0] == 0xA5)
 {
   recv_data.rec->header = vis_recv_buff[index];
   index += 1;
   recv_data.rec->tracking = vis_recv_buff[index] & 0x01;
   recv_data.rec->id = (vis_recv_buff[index] & 0x0e) >> 1;
   recv_data.rec->armors_num = (vis_recv_buff[index] & 0x70) >> 4;
   recv_data.rec->reserved = (vis_recv_buff[index] & 0x80) >> 7;
   index += 1;
   recv_data.rec->x = write_hex_to_float(vis_recv_buff[index], vis_recv_buff[index + 1], vis_recv_buff[index + 2], vis_recv_buff[index + 3]);
   index += 4;
   recv_data.rec->y = write_hex_to_float(vis_recv_buff[index], vis_recv_buff[index + 1], vis_recv_buff[index + 2], vis_recv_buff[index + 3]);
   index += 4;
   recv_data.rec->z = write_hex_to_float(vis_recv_buff[index], vis_recv_buff[index + 1], vis_recv_buff[index + 2], vis_recv_buff[index + 3]);
   index += 4;
   recv_data.rec->yaw = write_hex_to_float(vis_recv_buff[index], vis_recv_buff[index + 1], vis_recv_buff[index + 2], vis_recv_buff[index + 3]);
   index += 4;
   recv_data.rec->vx = write_hex_to_float(vis_recv_buff[index], vis_recv_buff[index + 1], vis_recv_buff[index + 2], vis_recv_buff[index + 3]);
   index += 4;
   recv_data.rec->vy = write_hex_to_float(vis_recv_buff[index], vis_recv_buff[index + 1], vis_recv_buff[index + 2], vis_recv_buff[index + 3]);
   index += 4;
   recv_data.rec->vz = write_hex_to_float(vis_recv_buff[index], vis_recv_buff[index + 1], vis_recv_buff[index + 2], vis_recv_buff[index + 3]);
   index += 4;
   recv_data.rec->v_yaw = write_hex_to_float(vis_recv_buff[index], vis_recv_buff[index + 1], vis_recv_buff[index + 2], vis_recv_buff[index + 3]);
   index += 4;
   recv_data.rec->r1 = write_hex_to_float(vis_recv_buff[index], vis_recv_buff[index + 1], vis_recv_buff[index + 2], vis_recv_buff[index + 3]);
   index += 4;
   recv_data.rec->r2 = write_hex_to_float(vis_recv_buff[index], vis_recv_buff[index + 1], vis_recv_buff[index + 2], vis_recv_buff[index + 3]);
   index += 4;
   recv_data.rec->dz = write_hex_to_float(vis_recv_buff[index], vis_recv_buff[index + 1], vis_recv_buff[index + 2], vis_recv_buff[index + 3]);
   index += 4;

   switch ( recv_data.rec->id)
   {
   case 1:
     st.armor_id = ARMOR_HERO;
     break;
   case 2:
     st.armor_id = ARMOR_ENGINEER;
     break;
   case 3:
     st.armor_id = ARMOR_INFANTRY3;
     break;
   case 4:
     st.armor_id = ARMOR_INFANTRY4;
     break;
   case 5:
     st.armor_id = ARMOR_INFANTRY5;
     break;
   case 6:
     st.armor_id = ARMOR_GUARD;
     break;
   case 7:
     st.armor_id = ARMOR_BASE;
     break;
   default:
     st.armor_id = ARMOR_OUTPOST;
     break;
   }
   switch ( recv_data.rec->armors_num)
   {
   case 2:
     st.armor_num = ARMOR_NUM_BALANCE;
     break;
   case 3:
     st.armor_num = ARMOR_NUM_OUTPOST;
     break;
   case 4:
     st.armor_num = ARMOR_NUM_NORMAL;
     break;
   default:
     break;
   }
   st.xw = - recv_data.rec->x;// recv_data.recx;
   st.yw = - recv_data.rec->y; //  recv_data.recy;
   st.zw =  recv_data.rec->z;
   st.tar_yaw =  recv_data.rec->yaw + PI;
   st.vxw =  recv_data.rec->vx;
   st.vyw =  recv_data.rec->vy;
   st.vzw =  recv_data.rec->vz;
   st.v_yaw =  recv_data.rec->v_yaw;
   st.r1 =  recv_data.rec->r1;
   st.r2 =  recv_data.rec->r2;
   st.dz =  recv_data.rec->dz;
   //这部分在下面的VisionSetAltitude赋值
   //    st.current_pitch = INS_num[1];
   //    st.current_yaw = INS_num[0];

   //     st.xw = 3.0f;
   //     st.yw = 0.0f;
   //     st.zw = 0.2f;
   //     st.tar_yaw = 0.0f;
   //     st.vxw = 0.0f;
   //     st.vyw = 0.0f;
   //     st.vzw = 0.0f;
   //     st.v_yaw = 0.0f;
   //     st.r1 =  recv_data.rec->r1;
   //     st.r2 =  recv_data.rec->r2;
   //     st.dz =  recv_data.rec->dz;
 }
 index = 0;
 memset(vis_recv_buff, 0, 128);
 //  HAL_UART_Receive_DMA(&huart1, vis_recv_buff, BUFLENGTH);

}



/* 视觉通信初始化 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
 UNUSED(_handle); // 仅为了消除警告
 USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
 vis_recv_buff = USBInit(conf);

 // 为master process注册daemon,用于判断视觉通信是否离线
 Daemon_Init_Config_s daemon_conf = {
     .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
     .owner_id = NULL,
     .reload_count = 5, // 50ms
 };
 vision_daemon_instance = DaemonRegister(&daemon_conf);

 return &recv_data;
}

//void VisionSend()
//{
//    static uint16_t flag_register;
//    static uint8_t send_buff[VISION_SEND_SIZE];
//    static uint16_t tx_len;
//    // TODO: code to set flag_register
//    flag_register = 30 << 8 | 0b00000001;
//    // 将数据转化为seasky协议的数据包
//    get_protocol_send_data(0x02, flag_register, &send_data.yaw, 3, send_buff, &tx_len);
//    USBTransmit(send_buff, tx_len);
//}

//imu解算赋值
void VisionSetAltitude(float yaw, float pitch, float roll)
{
 //    send_data.yaw = yaw;
 //    send_data.pitch = pitch;
 //    send_data.roll = roll;
 //===========================
 send_packet.yaw = yaw;
 send_packet.pitch = pitch;
 send_packet.roll = roll;


 st.current_pitch = pitch;
 st.current_yaw = yaw;



}

void VisionSend(){

 send_packet.header = 0x5A;
 send_packet.detect_color = 0;  //0红 1蓝
 send_packet.task_mode = 2;

 send_packet.aim_x = 2.0f;
 send_packet.aim_y = 0.0f;
 send_packet.aim_z = 0.0f;

 Append_CRC16_Check_Sum((uint8_t *)&send_packet,sizeof(send_packet));

 USBTransmit((uint8_t *)&send_packet, sizeof(send_packet));

}




#endif // VISION_USE_VCP
