/**
  ******************************************************************************
  * @file    vofa.c
  * @brief   VOFA+ 上位机通信: JustFloat / FireWater 发送 + 命令接收
  *
  *  发送: 中断 + 环形缓冲 (TXE 中断直写 DR), 完全不阻塞主循环。
  *        这样即使主循环被 I2C 刷 OLED 占用几十毫秒, 数据帧也能照常按
  *        10ms 节拍发出去, 不会丢帧或产生抖动。
  *  接收: RXNE 中断 + 环形缓冲, 主循环按行解析 ASCII 命令。
  ******************************************************************************
  */

#include "vofa.h"
#include "fmt.h"
#include "main.h"
#include "usart.h"

/* ============================ 配置 ============================ */

#define VOFA_RX_RING_SIZE   256u   /* 必须是 2 的幂 */
#define VOFA_RX_RING_MASK   (VOFA_RX_RING_SIZE - 1u)

#define VOFA_TX_RING_SIZE   512u   /* 必须是 2 的幂, 512B ≈ 25 帧 JustFloat */
#define VOFA_TX_RING_MASK   (VOFA_TX_RING_SIZE - 1u)

#define VOFA_LINE_SIZE      64u    /* 单行命令最大长度 */
#define VOFA_IDLE_FLUSH_MS  40u    /* 无换行时的空闲成行时间 */

/* ============================ 状态 ============================ */

/* --- 接收 --- */
static volatile uint8_t  s_ring[VOFA_RX_RING_SIZE];
static volatile uint16_t s_head = 0u;        /* ISR 写入 */
static volatile uint16_t s_tail = 0u;        /* 主循环读取 */

static char     s_line[VOFA_LINE_SIZE];
static uint16_t s_line_len = 0u;

static volatile uint32_t s_last_rx_tick = 0u;

/* --- 发送 --- */
static volatile uint8_t  s_txring[VOFA_TX_RING_SIZE];
static volatile uint16_t s_tx_head = 0u;     /* ISR 读出 */
static volatile uint16_t s_tx_tail = 0u;     /* 主循环写入 */

static uint8_t s_frame[VOFA_FRAME_BUF_SIZE];

/* --- 文本回显 --- */
static volatile uint8_t s_text_reply = VOFA_TEXT_REPLY_DEFAULT;

/* ============================ 发送缓冲 ============================ */

static uint16_t VOFA_TxUsed(void)
{
  return (uint16_t)((s_tx_tail - s_tx_head) & VOFA_TX_RING_MASK);
}

/**
  * @brief  把一整块数据压入发送环形缓冲, 空间不足时整块丢弃
  *
  *  整块丢弃而不是发一半: 半帧会让上位机一直等不到帧尾。
  *  写入 + 开中断这一步用 PRIMASK 保护, 避免与 TXE 中断里
  *  "缓冲空了就关中断" 撞车导致数据卡死。
  */
static void VOFA_TxPush(const uint8_t *p, uint16_t len)
{
  uint16_t i;
  uint32_t primask;

  if ((p == 0) || (len == 0u) || (len > (VOFA_TX_RING_SIZE - 1u)))
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  if (len > (uint16_t)(VOFA_TX_RING_SIZE - 1u - VOFA_TxUsed()))
  {
    __set_PRIMASK(primask);          /* 空间不够: 整帧丢弃 */
    return;
  }

  for (i = 0u; i < len; i++)
  {
    s_txring[s_tx_tail] = p[i];
    s_tx_tail = (uint16_t)((s_tx_tail + 1u) & VOFA_TX_RING_MASK);
  }

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);

  __set_PRIMASK(primask);
}

/* ============================ 中断服务 ============================ */

void VOFA_RxByte(uint8_t b)
{
  uint16_t next = (uint16_t)((s_head + 1u) & VOFA_RX_RING_MASK);

  if (next != s_tail)          /* 缓冲区未满 */
  {
    s_ring[s_head] = b;
    s_head = next;
  }
  /* 满则丢弃, 保证不会覆盖未处理数据 */

  s_last_rx_tick = HAL_GetTick();
}

void VOFA_IrqHandler(void)
{
  uint32_t sr = USART1->SR;

  /* ---------- 接收 ---------- */
  if ((sr & USART_SR_RXNE) != 0u)
  {
    /* 读 DR 同时清除 RXNE 与 ORE */
    VOFA_RxByte((uint8_t)(USART1->DR & 0xFFu));
  }
  else if ((sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0u)
  {
    /* 错误标志必须"先读 SR 再读 DR"才能清掉, 否则会反复进中断 */
    (void)USART1->DR;
  }
  else
  {
    /* 无接收事件 */
  }

  /* ---------- 发送 ---------- */
  if ((sr & USART_SR_TXE) != 0u)
  {
    if (s_tx_head != s_tx_tail)
    {
      USART1->DR = s_txring[s_tx_head];
      s_tx_head = (uint16_t)((s_tx_head + 1u) & VOFA_TX_RING_MASK);
    }
    else
    {
      __HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
    }
  }
}

/* ============================ 初始化 ============================ */

void VOFA_Init(void)
{
  s_head = 0u;
  s_tail = 0u;
  s_line_len = 0u;
  s_last_rx_tick = 0u;

  s_tx_head = 0u;
  s_tx_tail = 0u;

  s_text_reply = VOFA_TEXT_REPLY_DEFAULT;

  /* 清掉上电期间可能残留的接收标志, 避免一进中断就反复触发 */
  __HAL_UART_CLEAR_PEFLAG(&huart1);
  (void)huart1.Instance->DR;                 /* 读 DR 清 RXNE/ORE */
  (void)huart1.Instance->SR;

  __HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_ClearPendingIRQ(USART1_IRQn);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* ============================ 发送接口 ============================ */

void VOFA_SendSamples(const float *v, uint8_t n)
{
  uint16_t len;

  if ((v == 0) || (n == 0u))
  {
    return;
  }
  if (n > VOFA_MAX_CHANNELS)
  {
    n = VOFA_MAX_CHANNELS;
  }

#if (VOFA_USE_JUSTFLOAT != 0)
  /* JustFloat: float[n] 小端 + 4 字节帧尾 */
  if (((uint32_t)n * 4u + 4u) > VOFA_FRAME_BUF_SIZE)
  {
    return;
  }
  len = VOFA_BuildJustFloat(s_frame, v, n);
#else
  /* FireWater: "ch0,ch1,...,chN\n" */
  len = VOFA_BuildFireWater((char *)s_frame, (uint16_t)VOFA_FRAME_BUF_SIZE, v, n);
#endif

  if (len == 0u)
  {
    return;
  }

  VOFA_TxPush(s_frame, len);
}

void VOFA_SendText(const char *text)
{
  uint16_t len;

  if ((text == 0) || (*text == '\0'))
  {
    return;
  }
  if (s_text_reply == 0u)
  {
    return;
  }

  len = (uint16_t)strlen(text);
  if (len > VOFA_TEXT_BUF_SIZE)
  {
    len = VOFA_TEXT_BUF_SIZE;
  }
  VOFA_TxPush((const uint8_t *)text, len);
}

void VOFA_SendLine(const char *text)
{
  static const uint8_t nl = (uint8_t)'\n';

  if (s_text_reply == 0u)
  {
    return;
  }

  VOFA_SendText(text);
  VOFA_TxPush(&nl, 1u);
}

void VOFA_SetTextReply(uint8_t on)
{
  s_text_reply = (on != 0u) ? 1u : 0u;
}

uint8_t VOFA_GetTextReply(void)
{
  return s_text_reply;
}

uint8_t VOFA_GetProtocol(void)
{
#if (VOFA_USE_JUSTFLOAT != 0)
  return 1u;
#else
  return 0u;
#endif
}

/* ============================ 命令接收 ============================ */

uint32_t VOFA_GetLastRxTick(void)
{
  return s_last_rx_tick;
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
  uint8_t c;
  uint8_t got = 0u;

  while (s_tail != s_head)     /* 缓冲区非空 */
  {
    c = s_ring[s_tail];
    s_tail = (uint16_t)((s_tail + 1u) & VOFA_RX_RING_MASK);
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
