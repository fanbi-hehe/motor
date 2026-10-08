/**
  ******************************************************************************
  * @file    fmt.c
  * @brief   轻量级数值格式化工具
  ******************************************************************************
  */

#include "fmt.h"

/* 10^0 .. 10^6 */
static const uint32_t s_pow10[7] = {1u, 10u, 100u, 1000u, 10000u, 100000u, 1000000u};

/* 单次转换允许的最大标量值 (uint32 可精确表示的上限附近) */
#define FMT_SCALAR_MAX   4290000000.0f

/* 无符号整数 -> 十进制, 返回长度 */
static int u32_to_dec(char *p, uint32_t v)
{
  char tmp[10];
  int  n = 0;
  int  i;

  if (v == 0u)
  {
    p[0] = '0';
    return 1;
  }

  while ((v > 0u) && (n < (int)sizeof(tmp)))
  {
    tmp[n++] = (char)('0' + (v % 10u));
    v /= 10u;
  }

  for (i = 0; i < n; i++)
  {
    p[i] = tmp[n - 1 - i];
  }
  return n;
}

int FMT_Int(char *buf, int size, int32_t v)
{
  int      n = 0;
  uint32_t mag;

  if ((buf == 0) || (size <= 1))
  {
    return 0;
  }

  if (v < 0)
  {
    if (size < 3) { buf[0] = '\0'; return 0; }
    buf[n++] = '-';
    /* 注意 INT32_MIN: 先取反会溢出, 用无符号取绝对值 */
    mag = (uint32_t)(-(int64_t)v);
  }
  else
  {
    mag = (uint32_t)v;
  }

  /* 最多 10 位数字 + 结束符 */
  if ((size - n) < 11)
  {
    buf[0] = '\0';
    return 0;
  }

  n += u32_to_dec(&buf[n], mag);
  buf[n] = '\0';
  return n;
}

int FMT_Float(char *buf, int size, float v, uint8_t decimals)
{
  int      n   = 0;
  int      neg = 0;
  uint32_t scale;
  uint32_t whole;
  uint32_t frac;
  float    av;
  float    scaled;

  if ((buf == 0) || (size <= 2))
  {
    return 0;
  }
  if (decimals > 6u)
  {
    decimals = 6u;
  }

  /* NaN / Inf 保护: x != x 判断 NaN, 超出量程按饱和处理 */
  if (v != v)
  {
    const char *s = "nan";
    int i = 0;
    while ((s[i] != '\0') && ((n + 1) < size)) { buf[n++] = s[i++]; }
    buf[n] = '\0';
    return n;
  }

  if (v < 0.0f)
  {
    neg = 1;
    av  = -v;
  }
  else
  {
    av = v;
  }
  if (av > FMT_SCALAR_MAX)
  {
    av = FMT_SCALAR_MAX;
  }

  scale  = s_pow10[decimals];
  scaled = (av * (float)scale) + 0.5f;      /* 四舍五入 */
  if (scaled > FMT_SCALAR_MAX)
  {
    scaled = FMT_SCALAR_MAX;
  }

  whole = (uint32_t)scaled / scale;
  frac  = (uint32_t)scaled % scale;

  /* 负号 */
  if (neg && (whole != 0u || frac != 0u))
  {
    if (size < 3) { buf[0] = '\0'; return 0; }
    buf[n++] = '-';
  }

  if ((size - n) < 12)
  {
    buf[0] = '\0';
    return 0;
  }

  n += u32_to_dec(&buf[n], whole);

  if (decimals > 0u)
  {
    uint8_t d;
    buf[n++] = '.';
    for (d = decimals; d > 0u; d--)
    {
      scale /= 10u;
      buf[n++] = (char)('0' + ((frac / scale) % 10u));
    }
  }

  buf[n] = '\0';
  return n;
}
