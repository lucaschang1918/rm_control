//
// Created by lzx on 2025/10/8.
//

#ifndef BASIC_FRAMEWORK_VOFA_H
#define BASIC_FRAMEWORK_VOFA_H

#include <stdint.h>
#include "bsp_usart.h"
#include "usart.h"

typedef union
{
  float float_t;
  uint8_t uint8_t[4];
} send_float;
void vofa_justfloat_output(float *data, uint8_t num , UART_HandleTypeDef *huart);
void Vofa_Uart_Init(UART_HandleTypeDef *_handle);
void Vofa_Send_Data(float *data, uint8_t num);
#endif // BASIC_FRAMEWORK_VOFA_H
