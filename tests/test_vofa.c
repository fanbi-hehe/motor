/**
  ******************************************************************************
  * @file    test_vofa.c
  * @brief   VOFA 收发链路的主机端单元测试 (中断 + 环形缓冲时序)
  *
  *  用 tests/stubs 里的寄存器桩代替 STM32 HAL, 在 PC 上完整模拟:
  *    - TXE 中断逐字节发送, 发完自动关中断
  *    - 发送环形缓冲放不下整帧时整帧丢弃 (绝不发半帧)
  *    - RXNE 中断收字节 + 命令行组帧 + 空闲成行
  *
  *    cd motor
  *    gcc -std=c11 -Wall -Wextra -I tests/stubs -I Core/Inc \
  *        tests/test_vofa.c Core/Src/vofa.c Core/Src/fmt.c -lm \
  *        -o build/test_vofa
  *    ./build/test_vofa
  ******************************************************************************
  */

#include <stdio.h>
#include <string.h>
#include "vofa.h"
#include "hal_stub.h"

/* ============================ 桩实现 ============================ */

USART_TypeDef      g_usart1;
UART_HandleTypeDef huart1 = { &g_usart1 };

static uint32_t g_tick = 0u;

uint32_t HAL_GetTick(void) { return g_tick; }

void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t pre, uint32_t sub)
{
  (void)irq; (void)pre; (void)sub;
}
void HAL_NVIC_ClearPendingIRQ(IRQn_Type irq) { (void)irq; }
void HAL_NVIC_EnableIRQ(IRQn_Type irq) { (void)irq; }

/* ============================ 测试工具 ============================ */

static int g_pass = 0;
static int g_fail = 0;

static void check_int(const char *name, int got, int want)
{
  if (got == want) { g_pass++; printf("  PASS  %-38s = %d\n", name, got); }
  else             { g_fail++; printf("  FAIL  %-38s got %d, want %d\n", name, got, want); }
}

static void check_true(const char *name, int cond)
{
  if (cond) { g_pass++; printf("  PASS  %-38s\n", name); }
  else      { g_fail++; printf("  FAIL  %-38s\n", name); }
}

/* 发送捕获 */
#define CAP_SIZE 4096u
static uint8_t  g_cap[CAP_SIZE];
static uint16_t g_cap_len = 0u;

/* 模拟"线路上的字节": TXE 置位 -> 调中断 -> 读 DR */
static void drain_tx(void)
{
  int guard = 0;

  while (((g_usart1.CR1 & USART_CR1_TXEIE) != 0u) && (guard < 4096))
  {
    guard++;
    g_usart1.SR = USART_SR_TXE;
    g_usart1.DR = 0xDEADBEEFu;          /* 哨兵: 用于判断中断是否真的写了 DR */
    VOFA_IrqHandler();
    if (g_usart1.DR != 0xDEADBEEFu)
    {
      if (g_cap_len < CAP_SIZE) { g_cap[g_cap_len++] = (uint8_t)(g_usart1.DR & 0xFFu); }
    }
    else if ((g_usart1.CR1 & USART_CR1_TXEIE) != 0u)
    {
      /* 中断还在等发送, 但没写 DR -> 逻辑有问题 */
      g_fail++;
      printf("  FAIL  TXE 中断已使能却没有写出数据\n");
      return;
    }
  }
}

/* 模拟从串口收到一个字节 */
static void feed_rx(uint8_t b)
{
  g_usart1.DR = b;
  g_usart1.SR = USART_SR_RXNE;
  VOFA_IrqHandler();
}

static void reset_all(void)
{
  memset(&g_usart1, 0, sizeof(g_usart1));
  g_usart1.CR1 = 0u;
  g_cap_len = 0u;
  g_tick = 0u;
  VOFA_Init();
  g_cap_len = 0u;         /* VOFA_Init 不发送, 清一下更保险 */
}

/* 命令捕获 */
static char g_cmd[8][80];
static int  g_cmd_count = 0;

static void on_cmd(const char *cmd)
{
  if (g_cmd_count < 8)
  {
    snprintf(g_cmd[g_cmd_count], sizeof(g_cmd[0]), "%s", cmd);
    g_cmd_count++;
  }
}

/* ============================ 用例 ============================ */

static void test_init_state(void)
{
  printf("\n[1] init\n");

  reset_all();
  check_true("RXNE interrupt enabled", (g_usart1.CR1 & USART_CR1_RXNEIE) != 0u);
  check_true("TXE interrupt idle (off)", (g_usart1.CR1 & USART_CR1_TXEIE) == 0u);
}

static void test_justfloat_frame_on_wire(void)
{
  float v[4];
  const uint8_t tail[4] = {0x00u, 0x00u, 0x80u, 0x7Fu};

  printf("\n[2] JustFloat frame on the wire\n");

  reset_all();

  v[0] = 100.5f; v[1] = -2.5f; v[2] = 0.0f; v[3] = -0.125f;
  VOFA_SendSamples(v, 4u);

  check_true("TXE interrupt armed after push", (g_usart1.CR1 & USART_CR1_TXEIE) != 0u);
  check_int("nothing sent before IRQ", (int)g_cap_len, 0);

  drain_tx();

  check_int("bytes on wire", (int)g_cap_len, 20);
  check_int("TXE interrupt released", (int)((g_usart1.CR1 & USART_CR1_TXEIE) != 0u), 0);
  check_true("tail present", memcmp(&g_cap[16], tail, 4) == 0);

  {
    uint8_t expect[16] = {0x00,0x00,0xC9,0x42, 0x00,0x00,0x20,0xC0,
                          0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0xBE};
    check_true("payload little-endian", memcmp(g_cap, expect, 16) == 0);
  }

  {
    float back[4];
    uint8_t i;
    int ok = 1;
    for (i = 0u; i < 4u; i++) { memcpy(&back[i], &g_cap[(int)i * 4], 4); }
    for (i = 0u; i < 4u; i++) { if (back[i] != v[i]) { ok = 0; } }
    check_true("values recoverable by host", ok);
  }
}

static void test_back_to_back_frames(void)
{
  float v[4];
  uint8_t f;
  int frames;

  printf("\n[3] back-to-back frames (no partial frame)\n");

  reset_all();

  /* 连续压 30 帧再排空: 512B 环形缓冲实际可用 511B, 只放得下 25 帧(500B) */
  v[0] = 1.0f; v[1] = 2.0f; v[2] = 3.0f; v[3] = 4.0f;
  for (f = 0u; f < 30u; f++)
  {
    VOFA_SendSamples(v, 4u);
  }
  drain_tx();

  check_int("total bytes", (int)g_cap_len, 25 * 20);
  check_int("multiple of frame size", (int)(g_cap_len % 20u), 0);

  frames = 0;
  for (f = 0u; f < (uint8_t)(g_cap_len / 20u); f++)
  {
    if ((g_cap[(int)f * 20 + 16] == 0x00u) && (g_cap[(int)f * 20 + 17] == 0x00u) &&
        (g_cap[(int)f * 20 + 18] == 0x80u) && (g_cap[(int)f * 20 + 19] == 0x7Fu))
    {
      frames++;
    }
  }
  check_int("every frame terminated", frames, 25);
}

static void test_ring_recovers_after_drain(void)
{
  float v[4];
  int i;

  printf("\n[4] ring keeps working after a burst\n");

  reset_all();
  v[0] = -1.0f; v[1] = -2.0f; v[2] = -3.0f; v[3] = -4.0f;

  for (i = 0; i < 30; i++) { VOFA_SendSamples(v, 4u); }
  drain_tx();
  g_cap_len = 0u;

  /* 排空后再发一帧, 必须立刻恢复正常 */
  VOFA_SendSamples(v, 4u);
  drain_tx();
  check_int("one frame after burst", (int)g_cap_len, 20);
}

static void test_commands(void)
{
  const char *c1 = "TARGET 1200\r\n";
  const char *c2 = "KP 3.5\n";
  int i;

  printf("\n[5] command reception\n");

  reset_all();
  g_cmd_count = 0;

  for (i = 0; c1[i] != '\0'; i++) { feed_rx((uint8_t)c1[i]); }
  for (i = 0; c2[i] != '\0'; i++) { feed_rx((uint8_t)c2[i]); }
  VOFA_Poll(on_cmd);

  check_int("commands parsed", g_cmd_count, 2);
  if (g_cmd_count >= 2)
  {
    check_true("cmd0 = TARGET 1200", strcmp(g_cmd[0], "TARGET 1200") == 0);
    check_true("cmd1 = KP 3.5", strcmp(g_cmd[1], "KP 3.5") == 0);
  }

  /* 不带换行: 空闲 40ms 后自动成行 */
  reset_all();
  g_cmd_count = 0;
  for (i = 0; "STOP"[i] != '\0'; i++) { feed_rx((uint8_t)"STOP"[i]); }
  VOFA_Poll(on_cmd);
  check_int("no command before idle timeout", g_cmd_count, 0);

  g_tick += 50u;
  VOFA_Poll(on_cmd);
  check_int("command after idle timeout", g_cmd_count, 1);
  if (g_cmd_count >= 1)
  {
    check_true("cmd = STOP", strcmp(g_cmd[0], "STOP") == 0);
  }

  /* last-rx tick 用于通信超时保护: 每收到一个字节就刷新 */
  g_tick = 1234u;
  feed_rx((uint8_t)'A');
  check_int("last rx tick tracks reception", (int)VOFA_GetLastRxTick(), 1234);
}

static void test_text_reply_gate(void)
{
  printf("\n[6] text reply gating (JustFloat stays clean)\n");

  reset_all();
  g_cap_len = 0u;

  /* JustFloat 模式默认静音 */
  check_int("default muted in JustFloat", (int)VOFA_GetTextReply(), 0);
  VOFA_SendLine("HELLO");
  drain_tx();
  check_int("muted text sends nothing", (int)g_cap_len, 0);

  /* 打开后正常发送 */
  VOFA_SetTextReply(1u);
  VOFA_SendLine("HELLO");
  drain_tx();
  check_int("unmuted text length", (int)g_cap_len, 6);
  check_true("text + newline", memcmp(g_cap, "HELLO\n", 6) == 0);

  /* 关掉后立刻静音 */
  VOFA_SetTextReply(0u);
  g_cap_len = 0u;
  VOFA_SendLine("WORLD");
  drain_tx();
  check_int("muted again", (int)g_cap_len, 0);

  /* 数据帧不受文本开关影响 */
  {
    float v[4] = {1.0f, 2.0f, 3.0f, 4.0f};
    VOFA_SendSamples(v, 4u);
    drain_tx();
    check_int("samples still flow", (int)g_cap_len, 20);
  }
}

static void test_overflow_flag_cleared(void)
{
  printf("\n[7] RX overrun / error flag handling\n");

  reset_all();
  g_cmd_count = 0;

  /* ORE: 只置错误标志, 不清掉的话会反复进中断 */
  g_usart1.SR = USART_SR_ORE;
  VOFA_IrqHandler();
  check_true("ORE consumed (no TXE arm)", (g_usart1.CR1 & USART_CR1_TXEIE) == 0u);

  /* 错误之后仍能正常收命令 */
  {
    const char *c = "RESET\n";
    int i;
    for (i = 0; c[i] != '\0'; i++) { feed_rx((uint8_t)c[i]); }
    VOFA_Poll(on_cmd);
    check_int("command after ORE", g_cmd_count, 1);
  }
}

int main(void)
{
  printf("==== VOFA transport unit tests (interrupt + ring buffer) ====\n");
  printf("protocol: JustFloat=%d, text reply default=%u\n",
         (int)VOFA_GetProtocol(), (unsigned)VOFA_TEXT_REPLY_DEFAULT);

  test_init_state();
  test_justfloat_frame_on_wire();
  test_back_to_back_frames();
  test_ring_recovers_after_drain();
  test_commands();
  test_text_reply_gate();
  test_overflow_flag_cleared();

  printf("\n==== result: %d passed, %d failed ====\n", g_pass, g_fail);
  return (g_fail == 0) ? 0 : 1;
}
