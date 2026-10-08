/**
  ******************************************************************************
  * @file    vofa.h
  * @brief   VOFA+ 上位机通信 (FireWater 协议) + 命令接收
  *
  * 硬件: USART1, PA9 = TX, PA10 = RX, 默认 115200-8-N-1
  *
  * ---------------------------------------------------------------
  * 上行 (单片机 -> VOFA+) : FireWater 文本协议
  *   帧格式: "<ch0>,<ch1>,...,<chN>\n"
  *   数值为 ASCII 十进制文本, 以 ',' 分隔, 以 '\n' 结束一帧。
  *   注意 FireWater 必须遇到换行才会解析, 缺少 '\n' 会导致
  *   VOFA+ 缓冲区只进不出。
  *
  * ---------------------------------------------------------------
  * 下行 (VOFA+ -> 单片机) : ASCII 命令, 以 \r 或 \n 结束
  *   命令列表 (大小写敏感, 也可用数字直接设定转速):
  *     1200            -> 直接发送数字, 设为目标转速 1200 RPM
  *     TARGET 1200     -> 同义
  *     KP 1.2          -> 修改比例增益
  *     KI 0.05         -> 修改积分增益
  *     KD 0.002        -> 修改微分增益
  *     START           -> 使能闭环
  *     STOP            -> 停机并清零目标值
  *     RESET           -> 复位 PID 内部状态
  *     LIMIT 4000      -> 修改输出限幅
  *     DUTY 800        -> 开环测试: 直接给定占空比
  *     OPEN / CLOSE    -> 开环 / 闭环切换
  *     DIR 0|1         -> 方向取反
  *     SAVE            -> 打印当前所有参数
  *     HELP            -> 打印命令帮助
  ******************************************************************************
  */
#ifndef __VOFA_H
#define __VOFA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** 上行小数位数 */
#define VOFA_DECIMALS     2u

/** 一帧最多通道数 */
#define VOFA_MAX_CHANNELS 8u

/* 收到一整行命令时的回调 (字符串不含 \r\n) */
typedef void (*VOFA_CommandCb)(const char *cmd);

/** 初始化: 使能 USART1 接收中断 */
void VOFA_Init(void);

/**
  * @brief 按 FireWater 格式发送一帧浮点数据
  * @param v 通道数组
  * @param n 通道数 (1 ~ VOFA_MAX_CHANNELS)
  */
void VOFA_SendSamples(const float *v, uint8_t n);

/** 发送一段原始文本 (不自动换行) */
void VOFA_SendText(const char *text);

/** 发送一行文本 (自动补 '\n') */
void VOFA_SendLine(const char *text);

/** 在 USART1 接收中断中调用, 把收到的字节压入环形缓冲 */
void VOFA_RxByte(uint8_t b);

/** 主循环调用: 解析完整命令行并回调 cb */
void VOFA_Poll(VOFA_CommandCb cb);

/** 最近一次收到字节的 tick (ms), 用于判断链路是否活跃 */
uint32_t VOFA_GetLastRxTick(void);

/** 自上次调用以来是否收到过数据 */
uint8_t VOFA_IsLinkAlive(void);

#ifdef __cplusplus
}
#endif

#endif /* __VOFA_H */
