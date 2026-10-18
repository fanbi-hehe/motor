/**
 * comm.c —— USART1 和 VOFA+ 通信
 *
 * 发送: JustFloat 二进制帧 (20 字节)
 *       4 个 float 小端 + 帧尾 00 00 80 7F
 *       帧尾其实就是一个 +Inf, 上位机靠它判断"一帧到这里结束"
 *
 * 接收: 中断一个字节一个字节收, 攒够一行(遇到 \n 或 \r)就置个标志,
 *       主循环调 Comm_GetCommand() 取走。
 */
#include "comm.h"
#include "main.h"
#include "usart.h"
#include <string.h>

#define CMD_MAX   24          /* 一条命令最长多少字节 */

static uint8_t rx_byte = 0;              /* 中断里收的字节 */
static char    rx_line[CMD_MAX];         /* 正在拼的一行 */
static uint8_t rx_len = 0;

static char             cmd_ready[CMD_MAX];   /* 拼好、等主循环取走的一行 */
static volatile uint8_t cmd_flag = 0;

void Comm_Init(void)
{
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

void Comm_SendFrame(float c0, float c1, float c2, float c3)
{
  uint8_t buf[20];
  float   data[4];

  data[0] = c0;
  data[1] = c1;
  data[2] = c2;
  data[3] = c3;

  /* STM32 是小端, 直接把 4 个 float 拷进去就行 */
  memcpy(buf, data, 16);

  buf[16] = 0x00;                        /* 帧尾 */
  buf[17] = 0x00;
  buf[18] = 0x80;
  buf[19] = 0x7F;

  HAL_UART_Transmit(&huart1, buf, 20, 100);
}

/* 串口每收到 1 个字节, HAL 就会调用这里 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  char c = (char)rx_byte;

  if ((c == '\n') || (c == '\r'))
  {
    if (rx_len > 0)
    {
      rx_line[rx_len] = '\0';
      strncpy(cmd_ready, rx_line, CMD_MAX - 1);
      cmd_ready[CMD_MAX - 1] = '\0';
      cmd_flag = 1;
      rx_len = 0;
    }
  }
  else if (rx_len < (CMD_MAX - 1))
  {
    rx_line[rx_len++] = c;
  }

  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);   /* 重新开接收 */
}

int Comm_GetCommand(char *buf, int size)
{
  uint32_t primask;

  if ((cmd_flag == 0) || (size <= 0))
  {
    return 0;
  }

  /* 拷的时候关一下中断, 免得拷到一半中断又写进来新命令 */
  primask = __get_PRIMASK();
  __disable_irq();

  strncpy(buf, cmd_ready, (size_t)(size - 1));
  buf[size - 1] = '\0';
  cmd_flag = 0;

  __set_PRIMASK(primask);

  return 1;
}
