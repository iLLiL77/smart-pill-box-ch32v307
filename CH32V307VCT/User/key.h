/**********************************************
 * 按键扫描 — 头文件
 *
 * 所有函数非阻塞, 由主循环 tick (~100ms) 驱动消抖
 * 消抖: 2次确认 (~200ms)
 * 长按: ~1秒 (ADD键专用)
 **********************************************/
#ifndef __KEY_H
#define __KEY_H
#include "config.h"

uint8_t Key_Mode_Scan(void);       /* PA8: 短按=切换菜单项 */
uint8_t Key_Mode_Hold(void);       /* PA8: 长按=药盒1去皮 */
uint8_t Key_Add_Scan(void);        /* PA9: 短按+长按连发都有效 */
uint8_t Key_Add_Hold(void);        /* PA9: 仅长按触发 (用于药盒2去皮) */
uint8_t Key_OK_Scan(void);         /* PA10: 短按=确认 */
uint8_t Key_Max30102_Scan(void);   /* PA6: 短按=MAX30102开关切换 */
#endif
