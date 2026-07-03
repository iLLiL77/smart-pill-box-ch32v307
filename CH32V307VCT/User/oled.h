#ifndef __OLED_H
#define __OLED_H
#include "config.h"

void OLED_Init(void);
void OLED_Select(uint8_t id);   /* 0=OLED1(药盒UI), 1=OLED2(称重) */
void OLED_Clear(void);
void OLED_Show_Char(uint8_t x, uint8_t y, uint8_t ch);
void OLED_Show_Str(uint8_t x, uint8_t y, const char *str);
void OLED_Show_Num(uint8_t x, uint8_t y, uint8_t num, uint8_t digits);
void OLED_Show_Time(uint8_t x, uint8_t y, uint8_t h, uint8_t m);
void OLED_Show_Weight(uint8_t x, uint8_t y, float w);
void OLED_Show_Bar(uint8_t x, uint8_t y, uint8_t percent);
void OLED_Show_Float(uint8_t x, uint8_t y, float val, uint8_t int_digits, uint8_t dec_digits);

void Refresh_Setup(void);
void Refresh_Run(void);
#endif
