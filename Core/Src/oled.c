/**
  ******************************************************************************
  * @file    oled.c
  * @brief   0.98 寸 I2C OLED 显示驱动 (SSD1306 / SSD1315 兼容)
  ******************************************************************************
  */

#include "oled.h"
#include "oled_font.h"
#include "fmt.h"
#include "main.h"
#include "i2c.h"
#include <string.h>

/* ============================ 内部常量 ============================ */

#define OLED_CMD_PREFIX   0x00u    /* 控制字节: 后续为命令 */
#define OLED_DAT_PREFIX   0x40u    /* 控制字节: 后续为数据 */

#define OLED_I2C_TIMEOUT  100u     /* ms */

#define OLED_MAX_FAILS    3u       /* 连续失败多少次后复位 I2C */

/* ============================ 内部状态 ============================ */

static uint8_t  s_fb[OLED_PAGES][OLED_WIDTH];
static uint8_t  s_dirty[OLED_PAGES];
static uint8_t  s_present = 0u;
static uint16_t s_addr    = (uint16_t)(OLED_I2C_ADDR_PRIMARY << 1);

/* I2C 发送缓冲: 1 字节控制字 + 一页数据 (放在静态区, 避免占用栈) */
static uint8_t  s_i2cbuf[1u + OLED_WIDTH];

/* ============================ 底层 I2C ============================ */

static HAL_StatusTypeDef _i2c_write(uint8_t prefix, const uint8_t *data, uint16_t len)
{
  static uint8_t s_fail = 0u;
  HAL_StatusTypeDef st;

  if ((data == 0) || (len > OLED_WIDTH))
  {
    return HAL_ERROR;
  }

  s_i2cbuf[0] = prefix;
  if (len > 0u)
  {
    memcpy(&s_i2cbuf[1], data, len);
  }

  st = HAL_I2C_Master_Transmit(&hi2c1, s_addr, s_i2cbuf, (uint16_t)(len + 1u),
                               OLED_I2C_TIMEOUT);
  if (st != HAL_OK)
  {
    s_fail++;
    if (s_fail >= OLED_MAX_FAILS)
    {
      /* 连续失败: 复位 I2C 外设, 处理总线被拉死的情况 */
      s_fail = 0u;
      (void)HAL_I2C_DeInit(&hi2c1);
      MX_I2C1_Init();
    }
  }
  else
  {
    s_fail = 0u;
  }
  return st;
}

static HAL_StatusTypeDef _write_cmd(uint8_t cmd)
{
  uint8_t tmp = cmd;
  return _i2c_write(OLED_CMD_PREFIX, &tmp, 1u);
}

static HAL_StatusTypeDef _write_cmd2(uint8_t cmd, uint8_t arg)
{
  uint8_t tmp[2];
  tmp[0] = cmd;
  tmp[1] = arg;
  return _i2c_write(OLED_CMD_PREFIX, tmp, 2u);
}

/* ============================ 初始化 ============================ */

#if (OLED_CTRL_SH1106 != 0)
/* SH1106: 不支持 0x20/0x21/0x22 寻址命令, 用页 + 列地址设置 */
static void _set_page_window(uint8_t page)
{
  _write_cmd((uint8_t)(0xB0u | page));
  _write_cmd(0x00);                 /* 低列地址 (含 SH1106 的 2 列偏移) */
  _write_cmd(0x10);                 /* 高列地址 */
}
#else
/* SSD1306: 水平寻址, 限定为当前页 (col 0..127, page = page) */
static void _set_page_window(uint8_t page)
{
  (void)_write_cmd2(0x21, 0x00);    /* 列起始 */
  (void)_write_cmd(0x7Fu);          /* 列结束 = 127 */
  (void)_write_cmd2(0x22, page);    /* 页起始 */
  (void)_write_cmd(page);           /* 页结束 */
}
#endif

#define OLED_START_LINE  0x40u

static void _init_sequence(void)
{
  (void)_write_cmd(0xAEu);                      /* 关显示 */

  (void)_write_cmd2(0xD5u, 0x80u);              /* 时钟分频 */
  (void)_write_cmd2(0xA8u, (uint8_t)(OLED_HEIGHT - 1u)); /* 复用率 */
  (void)_write_cmd2(0xD3u, 0x00u);              /* 显示偏移 */
  (void)_write_cmd(OLED_START_LINE);            /* 起始行 */

  (void)_write_cmd2(0x8Du, 0x14u);              /* 电荷泵开 */

#if (OLED_CTRL_SH1106 == 0)
  (void)_write_cmd2(0x20u, 0x00u);              /* 水平寻址模式 */
#endif

  (void)_write_cmd(0xA1u);                      /* 段重映射 (左右镜像) */
  (void)_write_cmd(0xC8u);                      /* COM 扫描方向 (上下镜像) */

  if (OLED_HEIGHT == 32u)
  {
    (void)_write_cmd2(0xDAu, 0x02u);            /* COM 引脚配置: 32 行 */
    (void)_write_cmd2(0x81u, 0x8Fu);            /* 对比度 */
  }
  else
  {
    (void)_write_cmd2(0xDAu, 0x12u);            /* COM 引脚配置: 64 行 */
    (void)_write_cmd2(0x81u, 0xCFu);            /* 对比度 */
  }

  (void)_write_cmd2(0xD9u, 0xF1u);              /* 预充电周期 */
  (void)_write_cmd2(0xDBu, 0x40u);              /* VCOMH 反选电平 */

  (void)_write_cmd(0xA4u);                      /* 显示跟随 RAM */
  (void)_write_cmd(0xA6u);                      /* 正常显示 (非反色) */
  (void)_write_cmd(0x2Eu);                      /* 关闭滚动 */
}

static uint8_t _probe(uint16_t addr7)
{
  s_addr = (uint16_t)(addr7 << 1);
  return (HAL_I2C_IsDeviceReady(&hi2c1, s_addr, 2u, 50u) == HAL_OK) ? 1u : 0u;
}

uint8_t OLED_Init(void)
{
  uint8_t found;

  s_present = 0u;

  found = _probe(OLED_I2C_ADDR_PRIMARY);
  if (found == 0u)
  {
    found = _probe(OLED_I2C_ADDR_ALT);
  }
  if (found == 0u)
  {
    s_addr = (uint16_t)(OLED_I2C_ADDR_PRIMARY << 1);
    return 0u;                    /* 未接屏幕: 不影响电机控制 */
  }

  _init_sequence();

  memset(s_fb, 0x00, sizeof(s_fb));
  memset(s_dirty, 0x01, sizeof(s_dirty));

  s_present = 1u;
  OLED_Refresh();                 /* 推一帧全黑 */
  (void)_write_cmd(0xAFu);        /* 开显示 */

  return 1u;
}

uint8_t OLED_IsPresent(void)
{
  return s_present;
}

void OLED_DisplayOn(uint8_t on)
{
  (void)_write_cmd((on != 0u) ? 0xAFu : 0xAEu);
}

/* ============================ 显存刷新 ============================ */

void OLED_Clear(void)
{
  memset(s_fb, 0x00, sizeof(s_fb));
  memset(s_dirty, 0x01, sizeof(s_dirty));
}

void OLED_ClearLine(uint8_t page)
{
  if (page >= OLED_PAGES)
  {
    return;
  }
  memset(s_fb[page], 0x00, OLED_WIDTH);
  s_dirty[page] = 1u;
}

void OLED_RefreshAll(void)
{
  memset(s_dirty, 0x01, sizeof(s_dirty));
  OLED_Refresh();
}

void OLED_Refresh(void)
{
  uint8_t page;

  if (s_present == 0u)
  {
    return;
  }

  for (page = 0u; page < OLED_PAGES; page++)
  {
    if (s_dirty[page] == 0u)
    {
      continue;
    }

    _set_page_window(page);
    if (_i2c_write(OLED_DAT_PREFIX, s_fb[page], OLED_WIDTH) == HAL_OK)
    {
      s_dirty[page] = 0u;
    }
    else
    {
      break;                      /* I2C 异常: 保留脏标记, 下次重试 */
    }
  }
}

/* ============================ 绘制 ============================ */

void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t on)
{
  uint8_t page;
  uint8_t mask;

  if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT))
  {
    return;
  }

  page = (uint8_t)(y >> 3);
  mask = (uint8_t)(1u << (y & 7u));

  if (on != 0u)
  {
    s_fb[page][x] |= mask;
  }
  else
  {
    s_fb[page][x] &= (uint8_t)~mask;
  }
  s_dirty[page] = 1u;
}

static void _draw_char(uint8_t x, uint8_t y, char c, uint8_t scale)
{
  const uint8_t *glyph;
  uint8_t col;
  uint8_t row;
  uint8_t sx;
  uint8_t sy;

  if ((c < (char)OLED_FONT_FIRST_CHAR) || (c > (char)OLED_FONT_LAST_CHAR))
  {
    c = ' ';
  }

  if (scale == 0u)
  {
    scale = 1u;
  }

  glyph = s_oled_font6x8[(uint8_t)c - OLED_FONT_FIRST_CHAR];

  for (col = 0u; col < OLED_FONT_WIDTH; col++)
  {
    uint8_t bits = glyph[col];
    for (row = 0u; row < OLED_FONT_HEIGHT; row++)
    {
      if ((bits & (uint8_t)(1u << row)) == 0u)
      {
        continue;
      }
      for (sy = 0u; sy < scale; sy++)
      {
        for (sx = 0u; sx < scale; sx++)
        {
          OLED_DrawPixel((uint8_t)(x + (col * scale) + sx),
                         (uint8_t)(y + (row * scale) + sy), 1u);
        }
      }
    }
  }
}

uint8_t OLED_DrawStringScaled(uint8_t x, uint8_t y, const char *str, uint8_t scale)
{
  uint8_t cx = x;

  if (str == 0)
  {
    return x;
  }
  if (scale == 0u)
  {
    scale = 1u;
  }

  while (*str != '\0')
  {
    if (cx >= OLED_WIDTH)
    {
      break;
    }
    _draw_char(cx, y, *str, scale);
    cx = (uint8_t)(cx + (OLED_FONT_WIDTH * scale));
    str++;
  }
  return cx;
}

uint8_t OLED_DrawString(uint8_t x, uint8_t y, const char *str)
{
  return OLED_DrawStringScaled(x, y, str, 1u);
}

void OLED_DrawStringScaledPad(uint8_t x, uint8_t y, const char *str,
                              uint8_t scale, uint8_t chars)
{
  uint8_t i;
  uint8_t adv;
  uint8_t cx = x;

  if (scale == 0u)
  {
    scale = 1u;
  }
  adv = (uint8_t)(OLED_FONT_WIDTH * scale);

  /* 先擦除整段区域, 保证不会留下上一次更长的字符串残影 */
  for (i = 0u; i < chars; i++)
  {
    if (cx >= OLED_WIDTH)
    {
      break;
    }
    _draw_char(cx, y, ' ', scale);
    cx = (uint8_t)(cx + adv);
  }

  (void)OLED_DrawStringScaled(x, y, str, scale);
}

void OLED_DrawStringPad(uint8_t x, uint8_t y, const char *str, uint8_t chars)
{
  OLED_DrawStringScaledPad(x, y, str, 1u, chars);
}

uint8_t OLED_DrawInt(uint8_t x, uint8_t y, int32_t v)
{
  char buf[12];
  (void)FMT_Int(buf, (int)sizeof(buf), v);
  return OLED_DrawString(x, y, buf);
}

uint8_t OLED_DrawFloat(uint8_t x, uint8_t y, float v, uint8_t decimals)
{
  char buf[16];
  (void)FMT_Float(buf, (int)sizeof(buf), v, decimals);
  return OLED_DrawString(x, y, buf);
}
