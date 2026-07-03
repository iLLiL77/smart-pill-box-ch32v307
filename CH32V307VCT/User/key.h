#ifndef __KEY_H
#define __KEY_H
#include "config.h"

uint8_t Key_Mode_Scan(void);       // PA8: 短按=切换菜单项
uint8_t Key_Add_Scan(void);        // PA9: 按一次+1, 长按连发
uint8_t Key_OK_Scan(void);         // PA10: 按一下=确认
uint8_t Key_Max30102_Scan(void);   // PA6: 短按=MAX30102开关切换
#endif
