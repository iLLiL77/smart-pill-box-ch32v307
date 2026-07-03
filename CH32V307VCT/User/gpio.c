#include "gpio.h"

void GPIO_All_Init(void)
{
    GPIO_InitTypeDef gpio_cfg;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

    /* LED PA5 推挽输出 (高电平亮) */
    gpio_cfg.GPIO_Pin   = LED_PIN;
    gpio_cfg.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio_cfg.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIO_PORT, &gpio_cfg);

    /* 蜂鸣器 PA4 推挽输出 (低电平响) */
    gpio_cfg.GPIO_Pin   = BEEP_PIN;
    GPIO_Init(BEEP_PORT, &gpio_cfg);

    /* 红外对射 PA7 上拉输入 (遮挡=低) */
    gpio_cfg.GPIO_Pin   = PHOTO_PIN;
    gpio_cfg.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio_cfg);

    /* 按键 PA8/PA9/PA10 上拉输入 (按下=低) */
    gpio_cfg.GPIO_Pin   = KEY_MODE | KEY_ADD | KEY_OK;
    gpio_cfg.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio_cfg);

    /* Button4 PA6 上拉输入 (按下=低) — MAX30102 开关 */
    gpio_cfg.GPIO_Pin   = KEY_MAX30102;
    gpio_cfg.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio_cfg);

    /* MAX30102 INT PB1 上拉输入 (FIFO就绪=低) */
    gpio_cfg.GPIO_Pin   = MAX_INT_PIN;
    gpio_cfg.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(MAX_INT_PORT, &gpio_cfg);

    /* HX711-A DT(PE8)输入, SCK(PE9)输出 */
    gpio_cfg.GPIO_Pin   = HXA_DT;
    gpio_cfg.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(HXA_PORT, &gpio_cfg);

    gpio_cfg.GPIO_Pin   = HXA_SCK;
    gpio_cfg.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio_cfg.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(HXA_PORT, &gpio_cfg);

    /* HX711-B DT(PE10)输入, SCK(PE11)输出 */
    gpio_cfg.GPIO_Pin   = HXB_DT;
    gpio_cfg.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(HXB_PORT, &gpio_cfg);

    gpio_cfg.GPIO_Pin   = HXB_SCK;
    gpio_cfg.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio_cfg.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(HXB_PORT, &gpio_cfg);

    /* OLED1 I2C (PA0=SCL推挽, PA1=SDA开漏) */
    gpio_cfg.GPIO_Pin   = OLED_SCL;
    gpio_cfg.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio_cfg.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(OLED_PORT, &gpio_cfg);

    gpio_cfg.GPIO_Pin   = OLED_SDA;
    gpio_cfg.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_Init(OLED_PORT, &gpio_cfg);

    /* OLED2 I2C (PA2=SCL推挽, PA3=SDA开漏) */
    gpio_cfg.GPIO_Pin   = OLED2_SCL;
    gpio_cfg.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio_cfg.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(OLED2_PORT, &gpio_cfg);

    gpio_cfg.GPIO_Pin   = OLED2_SDA;
    gpio_cfg.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_Init(OLED2_PORT, &gpio_cfg);

    /* MAX30102 软件I2C (PE12=SCL推挽, PE13=SDA推挽 — 驱动动态切换SDA方向) */
    gpio_cfg.GPIO_Pin   = MAX_SCL;
    gpio_cfg.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio_cfg.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MAX_PORT, &gpio_cfg);

    gpio_cfg.GPIO_Pin   = MAX_SDA;
    gpio_cfg.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(MAX_PORT, &gpio_cfg);

    /* 初始: LED灭(低电平), 蜂鸣器关(高电平) */
    GPIO_ResetBits(GPIO_PORT, LED_PIN);
    GPIO_SetBits(BEEP_PORT, BEEP_PIN);

    /* HX711 SCK 初始低 */
    GPIO_ResetBits(HXA_PORT, HXA_SCK);
    GPIO_ResetBits(HXB_PORT, HXB_SCK);
}
