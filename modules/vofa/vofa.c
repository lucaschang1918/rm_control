//
// Created by lzx on 2025/10/8.
//

#include "vofa.h"
#include "daemon.h"

static uint8_t  Vofa_revc_flag =0;
/*VOFA浮点协议*/
/**
 *
 * @param data
 * @param num
 * @param huart
 */
void vofa_justfloat_output(float *data, uint8_t num , UART_HandleTypeDef *huart )
{
  static uint8_t i = 0;
  send_float temp[num];			//定义缓冲区数组
  uint8_t send_data[4 * num + 4]; //定义通过串口传出去的数组，数量是所传数据的字节数加上4个字节的尾巴
  for (i = 0; i < num; i++)
  {
    temp[i].float_t = data[i]; //将所传数据移到缓冲区数组
  }
  for (i = 0; i < num; i++)
  {
    send_data[4 * i] = temp[i].uint8_t[0];
    send_data[4 * i + 1] = temp[i].uint8_t[1];
    send_data[4 * i + 2] = temp[i].uint8_t[2];
    send_data[4 * i + 3] = temp[i].uint8_t[3]; //将缓冲区数组内的浮点型数据转成4个字节的无符号整型，之后传到要通过串口传出的数组里
  }
  send_data[4 * num] = 0x00;
  send_data[4 * num + 1] = 0x00;
  send_data[4 * num + 2] = 0x80;
  send_data[4 * num + 3] = 0x7f; //加上协议要求的4个尾巴

  HAL_UART_Transmit(huart, (uint8_t *)send_data, 4 * num + 4,100);
}




static void DecodeVofa()
{
}

static void VisionOfflineCallback(void *owner_id)
{
  Vofa_revc_flag = 0; // 清除接收标志位
}

static float Vofa_Frequency; // Hz
static USARTInstance *vofa_usart_instance;
static DaemonInstance *vofa_daemon_instance;


void Vofa_Uart_Init(UART_HandleTypeDef *_handle)
{
  USART_Init_Config_s conf;
  conf.module_callback = DecodeVofa;
  conf.recv_buff_size = 54;
  conf.usart_handle = _handle;
  vofa_usart_instance = USARTRegister(&conf);

  // 为master process注册daemon,用于判断视觉通信是否离线
  Daemon_Init_Config_s daemon_conf = {
      .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
      .owner_id = NULL,
      .reload_count = 10,
  };
  vofa_daemon_instance = DaemonRegister(&daemon_conf);
}

void Vofa_Send_Data(float *data, uint8_t num)
{
  USARTSend(vofa_usart_instance,(uint8_t *)data, num * sizeof(float),USART_TRANSFER_DMA);
}