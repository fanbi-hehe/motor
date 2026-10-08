/**
  ******************************************************************************
  * @file    oled.h
  * @brief   0.98 寸 I2C OLED 显示驱动 (SSD1306 / SSD1315 兼容)
  *
  * 硬件: I2C1, PB6 = SCL, PB7 = SDA, 7 位地址 0x3C (自动兼容 0x3D)
  *       分辨率默认 128 x 64
  *
  * 结构:
  *   - 内部 1KB 显存 (按页组织), 绘制操作只改显存并标脏页
  *   - OLED_Refresh() 只通过 I2C 推送被修改过的页, 避免 100 kHz 下
  *     每帧推送 1KB 造成的长时间阻塞
  ******************************************************************************
  */
#ifndef __OLED_H
#define __OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================ 参数配置 ============================ */

/** 屏幕宽度 (像素) */
#define OLED_WIDTH            128u

/** 屏幕高度 (像素): 0.98/0.96 寸模块通常为 64, 0.91 寸模块为 32 */
#define OLED_HEIGHT           64u

/** 控制器类型: 0 = SSD1306/SSD1315, 1 = SH1106 (1.3 寸常见) */
#define OLED_CTRL_SH1106      0

/** 主 I2C 从机地址 (7 位) */
#define OLED_I2C_ADDR_PRIMARY 0x3C
/** 备用 I2C 从机地址 (部分模块焊为 0x3D) */
#define OLED_I2C_ADDR_ALT     0x3D

/* ================================================================== */

/** 显存页数 (OLED_HEIGHT / 8) */
#define OLED_PAGES            (OLED_HEIGHT / 8u)

/**
  * @brief  初始化 OLED
  * @retval 1 = 检测到屏幕并初始化成功, 0 = 未检测到 (后续绘制静默丢弃)
  */
uint8_t OLED_Init(void);

/** 是否检测到屏幕 */
uint8_t OLED_IsPresent(void);

/** 清除整个显存 (只改显存, 需调用 OLED_Refresh 才会生效) */
void OLED_Clear(void);

/** 清除一行 (行号 0 ~ OLED_PAGES-1) */
void OLED_ClearLine(uint8_t page);

/** 全屏重绘 (先标脏, 再推送) */
void OLED_RefreshAll(void);

/** 推送所有脏页到屏幕, 并清除脏标记 */
void OLED_Refresh(void);

/** 打开/关闭屏幕显示 (SSD1306 0xAE/0xAF) */
void OLED_DisplayOn(uint8_t on);

/* ------------------------- 绘制 API ------------------------- */

/** 画一个像素 */
void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t on);

/** 画一个字符串, 6x8 字体, 返回结束位置的下一个 x 坐标 */
uint8_t OLED_DrawString(uint8_t x, uint8_t y, const char *str);

/**
  * @brief 画一个字符串并补齐到指定字符数 (不足补空格)
  * @param  x      起始 x
  * @param  y      起始 y
  * @param  str    字符串
  * @param  chars  需要占用的字符格数 (用于覆盖上一次显示的残影)
  */
void OLED_DrawStringPad(uint8_t x, uint8_t y, const char *str, uint8_t chars);

/**
  * @brief 画放大字体 (scale = 1 -> 6x8, scale = 2 -> 12x16)
  * @retval 结束位置的下一个 x 坐标
  */
uint8_t OLED_DrawStringScaled(uint8_t x, uint8_t y, const char *str, uint8_t scale);

/** 同上, 并补齐到 chars 个字符宽 */
void OLED_DrawStringScaledPad(uint8_t x, uint8_t y, const char *str,
                              uint8_t scale, uint8_t chars);

/** 在 (x,y) 显示有符号整数, 返回结束 x */
uint8_t OLED_DrawInt(uint8_t x, uint8_t y, int32_t v);

/** 在 (x,y) 显示浮点数 (decimals 位小数), 返回结束 x */
uint8_t OLED_DrawFloat(uint8_t x, uint8_t y, float v, uint8_t decimals);

#ifdef __cplusplus
}
#endif

#endif /* __OLED_H */
