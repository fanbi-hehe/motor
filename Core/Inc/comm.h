/**
 * comm.h —— 和 VOFA+ 通信
 *
 * 上行: JustFloat 格式, 一帧 = 4 个 float(小端) + 帧尾 00 00 80 7F = 20 字节
 * 下行: ASCII 命令, 一行一条 (以 \n 或 \r 结尾)
 */
#ifndef __COMM_H
#define __COMM_H

/** 初始化: 开串口接收中断 */
void Comm_Init(void);

/** 发一帧 4 通道数据给 VOFA+ */
void Comm_SendFrame(float c0, float c1, float c2, float c3);

/**
 * 取一条收到的命令 (主循环里轮询)
 * @param buf  存放命令的缓冲区
 * @param size 缓冲区大小
 * @retval 1 = 取到一条完整命令, 0 = 还没有
 */
int Comm_GetCommand(char *buf, int size);

#endif
