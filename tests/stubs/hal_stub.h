/**
  ******************************************************************************
  * @file    hal_stub.h
  * @brief   主机端测试用的极小 USART/HAL 桩
  *
  *  只实现 vofa.c 用到的那几个寄存器与宏, 用来在 PC 上把
  *  "中断 + 环形缓冲" 的收发时序完整跑一遍。
  ******************************************************************************
  */
#ifndef __HAL_STUB_H
#define __HAL_STUB_H

#include <stdint.h>

/* ---------------- USART 寄存器 ---------------- */

typedef struct
{
  volatile uint32_t SR;
  volatile uint32_t DR;
  volatile uint32_t CR1;
} USART_TypeDef;

extern USART_TypeDef g_usart1;
#define USART1 (&g_usart1)

#define USART_SR_PE     (1u << 0)
#define USART_SR_FE     (1u << 1)
#define USART_SR_NE     (1u << 2)
#define USART_SR_ORE    (1u << 3)
#define USART_SR_RXNE   (1u << 5)
#define USART_SR_TXE    (1u << 7)

#define USART_CR1_RXNEIE (1u << 5)
#define USART_CR1_TXEIE  (1u << 7)

/* ---------------- HAL 桩 ---------------- */

typedef struct
{
  USART_TypeDef *Instance;
} UART_HandleTypeDef;

extern UART_HandleTypeDef huart1;

typedef enum { USART1_IRQn = 37 } IRQn_Type;

#define UART_IT_RXNE   USART_CR1_RXNEIE
#define UART_IT_TXE    USART_CR1_TXEIE

static inline void __HAL_UART_ENABLE_IT(UART_HandleTypeDef *h, uint32_t it)
{
  h->Instance->CR1 |= it;
}

static inline void __HAL_UART_DISABLE_IT(UART_HandleTypeDef *h, uint32_t it)
{
  h->Instance->CR1 &= ~it;
}

static inline void __HAL_UART_CLEAR_PEFLAG(UART_HandleTypeDef *h)
{
  (void)h;
}

void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t pre, uint32_t sub);
void HAL_NVIC_ClearPendingIRQ(IRQn_Type irq);
void HAL_NVIC_EnableIRQ(IRQn_Type irq);

uint32_t HAL_GetTick(void);

/* vofa.c 用 PRIMASK 保护"写入缓冲 + 开中断"这段临界区,
   主机端是单线程, 直接空实现即可。 */
static inline uint32_t __get_PRIMASK(void) { return 0u; }
static inline void     __disable_irq(void) { }
static inline void     __set_PRIMASK(uint32_t p) { (void)p; }

#endif /* __HAL_STUB_H */
