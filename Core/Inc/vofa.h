/**
  ******************************************************************************
  * @file    vofa.h
  * @brief   VOFA+ 上位机通信: 数据流可选 JustFloat / FireWater + 命令接收
  *
  * 硬件: USART1, PA9 = TX, PA10 = RX, 默认 115200-8-N-1
  *
  * ---------------------------------------------------------------
  * 数据格式由 VOFA_USE_JUSTFLOAT 选择 (默认 JustFloat):
  *
  * [1] JustFloat (默认, 二进制小端浮点流)
  *       每帧 = float ch[0..N-1] (各 4 字节小端) + 帧尾 00 00 80 7F
  *       4 通道时一帧固定 20 字节。
  *       !! 上位机必须在 VOFA+ 里把"通道数"设成 4, 否则解析会错位。
  *       !! 字节接收区建议勾选十六进制显示。
  *
  * [2] FireWater (ASCII 文本, 把 VOFA_USE_JUSTFLOAT 改成 0 即可切回)
  *       每帧 = "ch0,ch1,...,chN\n", 以 '\n' 结束。
  *
  * ---------------------------------------------------------------
  * 下行 (VOFA+ -> 单片机) 始终是 ASCII 命令, 以 \r 或 \n 结束:
  *     1200            -> 直接发送数字, 设为目标转速 1200 RPM
  *     TARGET 1200     -> 同义 (-1200 即反转)
  *     KP / KI / KD <v>            -> 修改 PID 增益
  *     LIMIT / ILIMIT / SEP / DFILT <v> -> 限幅与非线性参数
  *     START / STOP    -> 使能 / 停机
  *     RESET           -> 复位 PID 与编码器
  *     OPEN / CLOSE    -> 开环 / 闭环
  *     DUTY <v>        -> 开环给定占空比
  *     SAFE <ms>       -> 通信超时保护
  *     TEXT 0|1        -> 关闭/打开文本回显 (JustFloat 下默认关闭, 见下)
  *     SAVE / HELP     -> 打印参数 / 帮助
  *
  * 关于文本回显:
  *   JustFloat 是纯二进制流, 上位机要靠帧尾才能切分数据帧。任何 ASCII 文本
  *   都会滞留在上位机缓冲区里, 直到下一个帧尾到达, 从而被当成一大帧垃圾数据
  *   (官方文档也提示这会造成缓冲区爆满)。因此 JustFloat 模式下文本回显默认
  *   关闭, 需要看 HELP/SAVE 输出时发送一次 TEXT 1 打开 (调试完再 TEXT 0)。
  ******************************************************************************
  */
#ifndef __VOFA_H
#define __VOFA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include "fmt.h"

/* ============================ 协议选择 ============================ */

/** 1 = JustFloat (二进制, 默认)   0 = FireWater (ASCII 文本) */
#define VOFA_USE_JUSTFLOAT      1

/** 文本回显默认开关: JustFloat 下默认关闭, 避免污染二进制流 */
#if (VOFA_USE_JUSTFLOAT != 0)
#define VOFA_TEXT_REPLY_DEFAULT 0u
#else
#define VOFA_TEXT_REPLY_DEFAULT 1u
#endif

/** 一帧最多通道数 */
#define VOFA_MAX_CHANNELS       8u

/** JustFloat 帧尾 (小端 float +Inf: 0x7F800000) */
#define VOFA_JUSTFLOAT_TAIL_0   0x00u
#define VOFA_JUSTFLOAT_TAIL_1   0x00u
#define VOFA_JUSTFLOAT_TAIL_2   0x80u
#define VOFA_JUSTFLOAT_TAIL_3   0x7Fu

/** 单帧最大字节数: 8 通道 ASCII 也够用 */
#define VOFA_FRAME_BUF_SIZE     128u

/** 文本回显最大长度 */
#define VOFA_TEXT_BUF_SIZE      128u

/** FireWater 小数位数 */
#define VOFA_FIREWATER_DECIMALS 2u

/* ============================ 帧构造 ============================ */
/*  下面两个是纯字节/字符串构造函数, 不依赖任何硬件, 因此可以做主机端单元测试。 */

/**
  * @brief  构造一帧 JustFloat 二进制数据
  * @param  out  输出缓冲区, 至少需要 n*4+4 字节
  * @param  v    通道数组
  * @param  n    通道数 (1 ~ VOFA_MAX_CHANNELS)
  * @retval 写入的字节数 (n*4+4)
  */
static inline uint16_t VOFA_BuildJustFloat(uint8_t *out, const float *v, uint8_t n)
{
  uint16_t idx = 0u;
  uint8_t  i;
  uint32_t raw;

  for (i = 0u; i < n; i++)
  {
    /* 用 memcpy 取位模式, 避免联合体类型双关; 再显式小端展开 */
    memcpy(&raw, &v[i], sizeof(raw));
    out[idx++] = (uint8_t)(raw & 0xFFu);
    out[idx++] = (uint8_t)((raw >> 8) & 0xFFu);
    out[idx++] = (uint8_t)((raw >> 16) & 0xFFu);
    out[idx++] = (uint8_t)((raw >> 24) & 0xFFu);
  }

  out[idx++] = (uint8_t)VOFA_JUSTFLOAT_TAIL_0;
  out[idx++] = (uint8_t)VOFA_JUSTFLOAT_TAIL_1;
  out[idx++] = (uint8_t)VOFA_JUSTFLOAT_TAIL_2;
  out[idx++] = (uint8_t)VOFA_JUSTFLOAT_TAIL_3;

  return idx;
}

/**
  * @brief  构造一帧 FireWater ASCII 数据
  * @param  out  输出缓冲区
  * @param  size 缓冲区大小
  * @param  v    通道数组
  * @param  n    通道数 (1 ~ VOFA_MAX_CHANNELS)
  * @retval 写入的字节数 (含结尾 '\n'), 缓冲不足时返回 0
  */
static inline uint16_t VOFA_BuildFireWater(char *out, uint16_t size,
                                           const float *v, uint8_t n)
{
  uint16_t idx = 0u;
  uint8_t  i;
  int      w;

  if ((out == 0) || (size < 4u))
  {
    return 0u;
  }

  for (i = 0u; i < n; i++)
  {
    if (i != 0u)
    {
      if (idx >= (size - 1u)) { return 0u; }
      out[idx++] = ',';
    }
    w = FMT_Float(&out[idx], (int)(size - idx - 1u), v[i], VOFA_FIREWATER_DECIMALS);
    if (w <= 0)
    {
      return 0u;
    }
    idx = (uint16_t)(idx + (uint16_t)w);
  }

  out[idx++] = '\n';
  return idx;
}

/* ============================ 对外接口 ============================ */

/** 收到一整行命令时的回调 (字符串不含 \r\n) */
typedef void (*VOFA_CommandCb)(const char *cmd);

/**
  * @brief  初始化: 使能 USART1 收发中断 (接收用 RXNE, 发送用 TXE)
  */
void VOFA_Init(void);

/**
  * @brief  按当前协议发送一帧数据
  * @param  v 通道数组
  * @param  n 通道数 (1 ~ VOFA_MAX_CHANNELS)
  *
  *  发送走中断 + 环形缓冲, 不阻塞主循环; 缓冲区放不下整帧时整帧丢弃,
  *  绝不发出半帧 (否则上位机会一直等帧尾)。
  */
void VOFA_SendSamples(const float *v, uint8_t n);

/** 发送一段文本 (不自动换行); 文本回显关闭时为空操作 */
void VOFA_SendText(const char *text);

/** 发送一行文本 (自动补 '\n'); 文本回显关闭时为空操作 */
void VOFA_SendLine(const char *text);

/** 打开/关闭文本回显 */
void VOFA_SetTextReply(uint8_t on);

/** 查询文本回显开关 */
uint8_t VOFA_GetTextReply(void);

/** 当前协议: 1 = JustFloat, 0 = FireWater */
uint8_t VOFA_GetProtocol(void);

/** 把收到的字节压入接收环形缓冲 (由 VOFA_IrqHandler 调用) */
void VOFA_RxByte(uint8_t b);

/** USART1 中断服务: 收发一体, 由 USART1_IRQHandler 调用 */
void VOFA_IrqHandler(void);

/** 主循环调用: 解析完整命令行并回调 cb */
void VOFA_Poll(VOFA_CommandCb cb);

/** 最近一次收到字节的 tick (ms), 用于判断链路是否活跃 */
uint32_t VOFA_GetLastRxTick(void);

#ifdef __cplusplus
}
#endif

#endif /* __VOFA_H */
