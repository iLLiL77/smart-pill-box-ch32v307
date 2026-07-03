#include "hx711.h"

/* 根据药盒索引获取引脚和端口 */
static void hx_pins(uint8_t med, GPIO_TypeDef **port, uint16_t *dt, uint16_t *sck)
{
    if(med == MED_A) { *port = HXA_PORT;  *dt = HXA_DT; *sck = HXA_SCK; }
    else             { *port = HXB_PORT;  *dt = HXB_DT; *sck = HXB_SCK; }
}

/*==========================================================================
 * 读取 HX711 原始24位ADC值 (带超时)
 *==========================================================================*/
uint32_t HX_Read(uint8_t med)
{
    uint32_t dat = 0, timeout = 0;
    uint8_t  i;
    uint16_t dt_pin, sck_pin;
    GPIO_TypeDef *port;
    hx_pins(med, &port, &dt_pin, &sck_pin);

    while(GPIO_ReadInputDataBit(port, dt_pin))
    {
        if(++timeout > 500000) return 0xFFFFFFFF;
    }

    for(i = 0; i < 24; i++)
    {
        GPIO_SetBits(port, sck_pin);
        Delay_Us(1);
        dat <<= 1;
        GPIO_ResetBits(port, sck_pin);
        if(GPIO_ReadInputDataBit(port, dt_pin)) dat++;
        Delay_Us(1);
    }

    GPIO_SetBits(port, sck_pin);
    Delay_Us(1);
    GPIO_ResetBits(port, sck_pin);
    Delay_Us(1);

    dat ^= 0x800000;
    return dat;
}

/*==========================================================================
 * 获取重量(克) — EMA滤波 + 零区快照
 *==========================================================================*/
float Get_Weight(uint8_t med)
{
    static float  ema[MED_COUNT]     = {0, 0};
    static uint8_t ema_ok[MED_COUNT] = {0, 0};

    uint32_t raw = HX_Read(med);
    if(raw == 0xFFFFFFFF)
    {
        return ema_ok[med] ? ema[med] : 0.0f;
    }

    int32_t val = (int32_t)(raw - hx_buf[med]);
    float   now = (float)val / scale;

    /* 原始值接近0 → 直接归零, 避免滤波惯性拖尾 */
    if(now < 0.5f && now > -0.5f)
    {
        ema[med]    = 0.0f;
        ema_ok[med] = 1;
        return 0.0f;
    }

    if(!ema_ok[med])
    {
        ema[med]    = now;
        ema_ok[med] = 1;
    }
    else
    {
        /* EMA: 50%旧值 + 50%新值 → 响应更快 */
        ema[med] = ema[med] * 0.5f + now * 0.5f;
    }

    return ema[med];
}

/*==========================================================================
 * 去皮
 *==========================================================================*/
void HX_Tare(uint8_t med)
{
    uint8_t  i, valid = 0;
    uint32_t sum = 0;
    for(i = 0; i < 12; i++)
    {
        uint32_t r = HX_Read(med);
        if(r != 0xFFFFFFFF) { sum += r; valid++; }
        Delay_ms(20);
    }
    if(valid > 0) hx_buf[med] = sum / valid;
}

/*==========================================================================
 * 双重检测取药: 重量下降 > TAKE_WEIGHT + 红外遮挡
 * PA7 红外: 遮挡=低电平
 *==========================================================================*/
uint8_t Check_Take(uint8_t med)
{
    float now = Get_Weight(med);
    uint8_t ir = GPIO_ReadInputDataBit(GPIOA, PHOTO_PIN);
    if((origin_w[med] - now) >= TAKE_WEIGHT && ir == 0)
        return 1;
    return 0;
}
