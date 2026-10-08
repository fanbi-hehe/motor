/**
  ******************************************************************************
  * @file    vofa.c
  * @brief   VOFA+ 上位机通信 (FireWater 协议) + 命令接收
  ******************************************************************************
  */

#include "vofa.h"
#include "fmt.h"
#include "main.h"
#include "usart.h"
#include <string.h>

/* ============================ 配置 ============================ */

#define VOFA_RX_RING_SIZE   256u   /* 环形缓冲区 (2 的幂最佳) */
#define VOFA_LINE_SIZE      64u    /* 单行命令最大长度 */
#define VOFA_TX_BUF_SIZE    128u   /* 单帧发送缓冲 */
#define VOFA_IDLE_FLUSH_MS  40u    /* 无换行时的空闲刷新时间 */
#define VOFA_TX_TIMEOUT_MS  100u   /* 阻塞发送超时 */

/* ============================ 状态 ============================ */

static volatile uint8_t  s_ring[VOFA_RX_RING_SIZE];
static volatile uint16_t s_head = 0;        /* ISR 写入 */
static volatile uint16_t s_tail = 0;        /* 主循环读取 */

static char     s_line[VOFA_LINE_SIZE];
static uint16_t s_line_len = 0;

static volatile uint32_t s_last_rx_tick = 0;
static volatile uint8_t  s_link_alive   = 0;

static char s_tx[VOFA_TX_BUF_SIZE];

/* ============================ 初始化 ============================ */

void VOFA_Init(void)
{
  s_head = 0;
  s_tail = 0;
  s_line_len    = 0;
  s_last_rx_tick = 0;
  s_link_alive   = 0;

  /* 清掉上电期间可能残留的错误标志, 避免一进中断就反复触发 */
  __HAL_UART_CLEAR_PEFLAG(&huart1);
  (void)huart1.Instance->DR;                 /* 读 DR 清 RXNE/ORE */
  (void)huart1.Instance->SR;

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_ClearPendingIRQ(USART1_IRQn);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* ============================ 接收 ============================ */

void VOFA_RxByte(uint8_t b)
{
  uint16_t next = (uint16_t)((s_head + 1u) % VOFA_RX_RING_SIZE);

  if (next != s_tail)          /* 缓冲区未满 */
  {
    s_ring[s_head] = b;
    s_head = next;
  }
  /* 满则丢弃, 保证不会覆盖未处理数据 */

  s_last_rx_tick = HAL_GetTick();
  s_link_alive   = 1u;
}

uint32_t VOFA_GetLastRxTick(void)
{
  return s_last_rx_tick;
}

uint8_t VOFA_IsLinkAlive(void)
{
  uint8_t alive = s_link_alive;
  s_link_alive = 0u;
  return alive;
}

static void VOFA_Dispatch(VOFA_CommandCb cb)
{
  if (s_line_len == 0u)
  {
    return;
  }
  s_line[s_line_len] = '\0';
  if (cb != 0)
  {
    cb(s_line);
  }
  s_line_len = 0u;
}

void VOFA_Poll(VOFA_CommandCb cb)
{
  uint8_t  c;
  uint8_t  got = 0u;

  while (s_tail != s_head)     /* 缓冲区非空 */
  {
    c = s_ring[s_tail];
    s_tail = (uint16_t)((s_tail + 1u) % VOFA_RX_RING_SIZE);
    got = 1u;

    if ((c == '\n') || (c == '\r'))
    {
      VOFA_Dispatch(cb);
    }
    else if (s_line_len < (VOFA_LINE_SIZE - 1u))
    {
      s_line[s_line_len++] = (char)c;
    }
    else
    {
      /* 行缓冲溢出: 丢弃这一行, 重新同步 */
      s_line_len = 0u;
    }
  }

  /* 兼容不发送换行的串口工具: 空闲一段时间后按整行处理 */
  if ((got == 0u) && (s_line_len > 0u) &&
      ((HAL_GetTick() - s_last_rx_tick) > VOFA_IDLE_FLUSH_MS))
  {
    VOFA_Dispatch(cb);
  }
}

/* ============================ 发送 ============================ */

void VOFA_SendText(const char *text)
{
  uint16_t len;

  if ((text == 0) || (*text == '\0'))
  {
    return;
  }

  len = (uint16_t)strlen(text);
  if (len > VOFA_TX_BUF_SIZE)
  {
    len = VOFA_TX_BUF_SIZE;
  }
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)text, len, VOFA_TX_TIMEOUT_MS);
}

void VOFA_SendLine(const char *text)
{
  static const uint8_t nl = (uint8_t)'\n';

  VOFA_SendText(text);
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)&nl, 1u, VOFA_TX_TIMEOUT_MS);
}

void VOFA_SendSamples(const float *v, uint8_t n)
{
  uint16_t len = 0u;
  uint8_t  i;
  int      w;

  if ((v == 0) || (n == 0u))
  {
    return;
  }
  if (n > VOFA_MAX_CHANNELS)
  {
    n = VOFA_MAX_CHANNELS;
  }

  for (i = 0u; i < n; i++)
  {
    if (i != 0u)
    {
      if (len >= (VOFA_TX_BUF_SIZE - 1u)) { break; }
      s_tx[len++] = ',';
    }

    w = FMT_Float(&s_tx[len], (int)(VOFA_TX_BUF_SIZE - len), v[i], VOFA_DECIMALS);
    if (w <= 0)
    {
      break;                            /* 缓冲不足, 本帧截断 */
    }
    len = (uint16_t)(len + (uint16_t)w);
  }

  if (len == 0u)
  {
    return;
  }
  if (len >= VOFA_TX_BUF_SIZE)
  {
    len = VOFA_TX_BUF_SIZE - 1u;
  }

  s_tx[len++] = '\n';                   /* FireWater 帧结束标志 */

  (void)HAL_UART_Transmit(&huart1, (uint8_t *)s_tx, len, VOFA_TX_TIMEOUT_MS);
}
