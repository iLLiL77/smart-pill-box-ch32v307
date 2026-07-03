#include "key.h"

/*==========================================================================
 * 按键1: 模式切换 (PA8) — 纯短按, 按一下切换一次
 *==========================================================================*/
uint8_t Key_Mode_Scan(void)
{
    static uint8_t last = 1;
    uint8_t now = (GPIO_ReadInputDataBit(GPIOA, KEY_MODE) == 0) ? 0 : 1;
    if(now == 0 && last == 1)        /* 下降沿 */
    {
        Delay_ms(20);
        if(GPIO_ReadInputDataBit(GPIOA, KEY_MODE) == 0)
        {
            last = 0;
            return 1;
        }
    }
    if(now == 1) last = 1;
    return 0;
}

/*==========================================================================
 * 按键2: 加值 (PA9) — 按一次+1, 长按0.5s后连发(150ms)
 *==========================================================================*/
uint8_t Key_Add_Scan(void)
{
    static uint8_t  state = 0;    /* 0=空闲, 1=首次, 2=连发 */
    static uint16_t timer = 0;

    if(GPIO_ReadInputDataBit(GPIOA, KEY_ADD) == 0)
    {
        if(state == 0)
        {
            Delay_ms(20);
            if(GPIO_ReadInputDataBit(GPIOA, KEY_ADD) == 0)
            { state = 1; timer = 0; return 1; }
        }
        else if(state == 1)
        {
            Delay_ms(10); timer += 10;
            if(timer >= 500) { state = 2; timer = 0; return 1; }
        }
        else /* state==2 */
        {
            Delay_ms(50); timer += 50;
            if(timer >= 150) { timer = 0; return 1; }
        }
    }
    else { state = 0; timer = 0; }
    return 0;
}

/*==========================================================================
 * 按键3: 确认 (PA10) — 按下松开触发一次
 *==========================================================================*/
uint8_t Key_OK_Scan(void)
{
    static uint8_t last = 1;
    uint8_t now = (GPIO_ReadInputDataBit(GPIOA, KEY_OK) == 0) ? 0 : 1;
    if(now == 0 && last == 1)
    {
        Delay_ms(20);
        if(GPIO_ReadInputDataBit(GPIOA, KEY_OK) == 0)
        { last = 0; return 1; }
    }
    if(now == 1) last = 1;
    return 0;
}

/*==========================================================================
 * 按键4: MAX30102开关 (PA6) — 纯短按, 按一下切换一次
 *==========================================================================*/
uint8_t Key_Max30102_Scan(void)
{
    static uint8_t last = 1;
    uint8_t now = (GPIO_ReadInputDataBit(GPIOA, KEY_MAX30102) == 0) ? 0 : 1;
    if(now == 0 && last == 1)        /* 下降沿 */
    {
        Delay_ms(20);
        if(GPIO_ReadInputDataBit(GPIOA, KEY_MAX30102) == 0)
        {
            last = 0;
            return 1;
        }
    }
    if(now == 1) last = 1;
    return 0;
}
