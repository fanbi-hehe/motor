/**
 * oled.h —— 0.98 寸 I2C OLED 显示 (128x64, SSD1306 / SH1106 通用)
 * ------------------------------------------------------------------
 * 只有两个函数, 够用就行:
 *
 *   OLED_Init()                初始化
 *   OLED_ShowLine(行, 文本)     在第 0~7 行写一行字 (6x8 字体, 每行最多 21 字)
 *
 * 用页寻址, 每次只写一行, 不需要显存。
 */
#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>

/** 初始化屏幕 (7 位地址 0x3C, 不通会再试 0x3D) */
void OLED_Init(void);

/** 在第 line 行(0~7)显示一行文字, 不足 21 字会自动补空格擦掉旧内容 */
void OLED_ShowLine(uint8_t line, const char *text);

#endif
