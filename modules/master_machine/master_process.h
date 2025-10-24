#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "seasky_protocol.h"
#include <stdbool.h>

#define VISION_RECV_SIZE 18u // 当前为固定值,36字节
#define VISION_SEND_SIZE 36u

#pragma pack(1)
typedef enum
{
	NO_FIRE = 0,
	AUTO_FIRE = 1,
	AUTO_AIM = 2
} Fire_Mode_e;

typedef enum
{
	NO_TARGET = 0,
	TARGET_CONVERGING = 1,
	READY_TO_FIRE = 2
} Target_State_e;

typedef enum
{
	NO_TARGET_NUM = 0,
	HERO1 = 1,
	ENGINEER2 = 2,
	INFANTRY3 = 3,
	INFANTRY4 = 4,
	INFANTRY5 = 5,
	OUTPOST = 6,
	SENTRY = 7,
	BASE = 8
} Target_Type_e;

typedef struct
{
	Fire_Mode_e fire_mode;
	Target_State_e target_state;
	Target_Type_e target_type;

	float pitch;
	float yaw;
} Vision_Recv_s;

typedef enum
{
	COLOR_NONE = 0,
	COLOR_BLUE = 1,
	COLOR_RED = 2,
} Enemy_Color_e;

typedef enum
{
	VISION_MODE_AIM = 0,
	VISION_MODE_SMALL_BUFF = 1,
	VISION_MODE_BIG_BUFF = 2
} Work_Mode_e;

typedef enum
{
	BULLET_SPEED_NONE = 0,
	BIG_AMU_10 = 10,
	SMALL_AMU_15 = 15,
	BIG_AMU_16 = 16,
	SMALL_AMU_18 = 18,
	SMALL_AMU_30 = 30,
} Bullet_Speed_e;

typedef struct
{
	Enemy_Color_e enemy_color;
	Work_Mode_e work_mode;
	Bullet_Speed_e bullet_speed;

	float yaw;
	float pitch;
	float roll;
} Vision_Send_s;
#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);

/**
 * @brief 发送视觉数据
 *
 */
void VisionSend();


/**
 * @brief 设置视觉发送标志位
 *
 * @param enemy_color
 * @param work_mode
 * @param bullet_speed
 */
void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed);

/**
 * @brief 设置发送数据的姿态部分
 *
 * @param yaw
 * @param pitch
 */
void VisionSetAltitude(float yaw, float pitch, float roll);


//电专的通信
typedef struct ReceiverPacket_{
  uint8_t header ;    //数据包头
  bool  state : 1 ;   //识别状态 如果是1 就是tracking状态 否则没追踪到
  uint8_t id : 3;     //数字是多少就是多少 哨兵是6  没识别到就是0

  uint8_t armors_num : 3;   //装甲板数量
  float x;
  float y;
  float z;
  float yaw;

  float vx;
  float vy;
  float vz;
  float v_yaw;
  float r1;
  float r2;
  float dz;

  uint16_t checksum;

}__attribute__((packed)) ReceiverPacket;


typedef struct SendPacket_{
  uint8_t header;
  uint8_t detect_color : 1; // 0-red 1-blue
  uint8_t task_mode : 2;
  uint8_t reserved : 5;
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
  uint16_t checksum;
}__attribute((packed)) SendPacket;


#endif // !MASTER_PROCESS_H