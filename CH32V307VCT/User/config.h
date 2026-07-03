#ifndef __CONFIG_H
#define __CONFIG_H

#include "debug.h"
#include <stdio.h>
#include <string.h>

/*==========================================================================
 * 引脚定义 — 智能药盒 2024
 *==========================================================================*/
/* LED (高电平亮) */
#define LED_PIN     GPIO_Pin_5       // PA5
#define GPIO_PORT   GPIOA

/* 蜂鸣器 (低电平响) */
#define BEEP_PIN    GPIO_Pin_4       // PA4
#define BEEP_PORT   GPIOA

/* HX711-A 称重1 (PE8=DT, PE9=SCK) */
#define HXA_DT      GPIO_Pin_8       // PE8
#define HXA_SCK     GPIO_Pin_9       // PE9
#define HXA_PORT    GPIOE

/* HX711-B 称重2 (PE10=DT, PE11=SCK) */
#define HXB_DT      GPIO_Pin_10      // PE10
#define HXB_SCK     GPIO_Pin_11      // PE11
#define HXB_PORT    GPIOE

/* 槽型红外对射 — 共用一路 (遮挡=低电平) */
#define PHOTO_PIN   GPIO_Pin_7       // PA7

/* 按键 (上拉输入, 按下=低) */
#define KEY_MODE    GPIO_Pin_8       // PA8  切换/选择
#define KEY_ADD     GPIO_Pin_9       // PA9  加值
#define KEY_OK      GPIO_Pin_10      // PA10 确认

/* OLED1 I2C — 药盒UI (PA0=SCL, PA1=SDA) */
#define OLED_SCL    GPIO_Pin_0       // PA0
#define OLED_SDA    GPIO_Pin_1       // PA1
#define OLED_PORT   GPIOA

/* OLED2 I2C — 称重+心率血氧 (PA2=SCL, PA3=SDA) */
#define OLED2_SCL   GPIO_Pin_2       // PA2
#define OLED2_SDA   GPIO_Pin_3       // PA3
#define OLED2_PORT  GPIOA

/* MAX30102 软件I2C (PE12=SCL, PE13=SDA) */
#define MAX_SCL     GPIO_Pin_12      // PE12
#define MAX_SDA     GPIO_Pin_13      // PE13
#define MAX_PORT    GPIOE

/* MAX30102 INT (PB1, 上拉输入, FIFO数据就绪=低) */
#define MAX_INT_PIN     GPIO_Pin_1       // PB1
#define MAX_INT_PORT    GPIOB

/* Button4 (PA6) — MAX30102 开启/关闭 */
#define KEY_MAX30102    GPIO_Pin_6       // PA6

/* 语音模块 UART — 待定, 稍后接入 */
/* #define VOICE_TX    GPIO_Pin_x */
/* #define VOICE_RX    GPIO_Pin_x */

/*==========================================================================
 * 常量
 *==========================================================================*/
#define MED_COUNT   2              // 药盒数量
#define MED_A       0              // 药盒A索引
#define MED_B       1              // 药盒B索引
#define MAX_SLOTS   3              // 每个药盒最多几个时间段
#define MAX_PILL    9
#define MAX_HOUR    23
#define MAX_MIN     59
#define MAX_TIMES   3              // 一天最多几次

#define TAKE_WEIGHT 0.0025f
#define SCALE_DEFAULT 220.0f
extern float scale;

/*==========================================================================
 * 系统状态
 *==========================================================================*/
#define STATE_SETUP  0
#define STATE_RUN    1
#define STATE_ALERT  2

/*==========================================================================
 * 药盒配置结构体
 *==========================================================================*/
typedef struct {
    uint8_t pills_per_dose;          // 一次几粒
    uint8_t times_per_day;           // 一天几次
    uint8_t slot_h[MAX_SLOTS];       // 时间段-时
    uint8_t slot_m[MAX_SLOTS];       // 时间段-分
} MedConfig;

/*==========================================================================
 * 全局变量
 *==========================================================================*/
extern MedConfig med[MED_COUNT];
extern uint8_t cur_h, cur_m;

extern uint8_t  sel_item;
extern uint8_t  sys_state;
extern volatile uint8_t  refresh_flag;
extern volatile uint32_t rtc_seconds;

extern uint8_t  alert_flag[MED_COUNT];
extern uint8_t  voice_ok[MED_COUNT];
extern float    origin_w[MED_COUNT];
extern uint32_t hx_buf[MED_COUNT];
extern uint8_t  max30102_enabled;   /* MAX30102 检测开关: 1=开启 0=关闭 */

/*==========================================================================
 * 函数声明
 *==========================================================================*/
void Delay_ms(uint32_t ms);
extern const uint8_t font_6x8[][6];

#endif
