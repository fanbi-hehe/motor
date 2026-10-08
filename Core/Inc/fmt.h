/**
  ******************************************************************************
  * @file    fmt.h
  * @brief   轻量级数值格式化工具 (不依赖 printf 浮点支持)
  *
  *  链接脚本使用 --specs=nano.specs 时, newlib-nano 默认不链接浮点
  *  printf 代码 (需额外加 -u _printf_float, 会显著增大固件)。这里用纯
  *  整数运算 + 少量浮点运算自行实现, 供 OLED 显示与 VOFA+ 上报共用。
  ******************************************************************************
  */
#ifndef __FMT_H
#define __FMT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
  * @brief  有符号整数转十进制字符串
  * @param  buf    输出缓冲区
  * @param  size   缓冲区大小 (含结束符)
  * @param  v      待转换的值
  * @retval 写入的字符数 (不含结束符), 缓冲区不足时写入 0
  */
int FMT_Int(char *buf, int size, int32_t v);

/**
  * @brief  浮点数转定点小数字符串 (四舍五入)
  * @param  buf       输出缓冲区
  * @param  size      缓冲区大小 (含结束符)
  * @param  v         待转换的值
  * @param  decimals  小数位数 (0 ~ 6)
  * @retval 写入的字符数 (不含结束符), 缓冲区不足时写入 0
  */
int FMT_Float(char *buf, int size, float v, uint8_t decimals);

#ifdef __cplusplus
}
#endif

#endif /* __FMT_H */
