#include "bsp_log.h"

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
#include "bsp_usart.h"
#include "usart.h"
#include <stdio.h>

void BSPLogInit()
{
    SEGGER_RTT_Init();
}

int PrintLog(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int n = SEGGER_RTT_vprintf(BUFFER_INDEX, fmt, &args); // 一次可以开启多个buffer(多个终端),我们只用一个
    va_end(args);
    return n;
}

void Float2Str(char *str, float va)
{
    int flag = va < 0;
    int head = (int)va;
    int point = (int)((va - head) * 1000);
    head = abs(head);
    point = abs(point);
    if (flag)
        sprintf(str, "-%d.%d", head, point);
    else
        sprintf(str, "%d.%d", head, point);
}



void hhSerial_Printf(char *format, ...)
{
  char String[100];              // 定义字符数组
  va_list arg;                   // 定义可变参数列表数据类型的变量arg
  va_start(arg, format);         // 从format开始，接收参数列表到arg变量
  vsprintf(String, format, arg); // 使用vsprintf打印格式化字符串和参数列表到字符数组中
  va_end(arg);                   // 结束变量arg
  hhSerial_SendString(String);   // 串口发送字符数组（字符串）
}
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, 100);
  return ch;
}

