/**********************************************
 * 第1步: MAX30102 I2C 极简测试
 *
 * 依赖原始 oled.c / gpio.c — 只替换 main()
 * 启动自检 → 双OLED初始化 → 循环读PART_ID
 * INT悬空不用
 **********************************************/

#include "config.h"
#include "gpio.h"
#include "oled.h"

/* LED + BEEP 宏 (与原始main.c一致) */
#define LED_ON()    GPIO_SetBits(GPIOA, GPIO_Pin_5)
#define LED_OFF()   GPIO_ResetBits(GPIOA, GPIO_Pin_5)
#define BEEP_ON()   GPIO_ResetBits(GPIOA, GPIO_Pin_4)
#define BEEP_OFF()  GPIO_SetBits(GPIOA, GPIO_Pin_4)

/*==========================================================================
 * MAX30102 软件I2C — PE12=SCL, PE13=SDA, ~100kHz
 * 独立实现，不依赖max30102.c。复用config.h中的引脚定义
 *==========================================================================*/
#define MAX_ADDR_W   0xAE      /* 0x57<<1 | W */
#define MAX_ADDR_R   0xAF      /* 0x57<<1 | R */
#define M_DELAY      Delay_Us(5)

static void M_SDA_H(void) { GPIO_SetBits(MAX_PORT, MAX_SDA); }
static void M_SDA_L(void) { GPIO_ResetBits(MAX_PORT, MAX_SDA); }
static void M_SCL_H(void) { GPIO_SetBits(MAX_PORT, MAX_SCL); }
static void M_SCL_L(void) { GPIO_ResetBits(MAX_PORT, MAX_SCL); }
static uint8_t M_RDSDA(void) { return GPIO_ReadInputDataBit(MAX_PORT, MAX_SDA); }

/* SDA方向: 输出(推挽) / 输入(上拉) */
static void M_SDAOUT(void) {
    GPIO_InitTypeDef c; c.GPIO_Pin=MAX_SDA; c.GPIO_Mode=GPIO_Mode_Out_PP; c.GPIO_Speed=GPIO_Speed_50MHz; GPIO_Init(MAX_PORT,&c);
}
static void M_SDAIN(void) {
    GPIO_InitTypeDef c; c.GPIO_Pin=MAX_SDA; c.GPIO_Mode=GPIO_Mode_IPU; c.GPIO_Speed=GPIO_Speed_50MHz; GPIO_Init(MAX_PORT,&c);
}

static void M_Start(void)  { M_SDAOUT(); M_SDA_H(); M_SCL_H(); M_DELAY; M_SDA_L(); M_DELAY; M_SCL_L(); M_DELAY; }
static void M_Stop(void)   { M_SDAOUT(); M_SCL_L(); M_SDA_L(); M_DELAY; M_SCL_H(); M_SDA_H(); M_DELAY; }
static uint8_t M_WaitAck(void) {
    uint8_t t=0; M_SDAIN(); M_SDA_H(); M_DELAY; M_SCL_H(); M_DELAY;
    while(M_RDSDA()){if(++t>250){M_Stop();return 1;}}
    M_SCL_L(); M_DELAY; return 0;
}
static void M_SendByte(uint8_t d) {
    uint8_t t; M_SDAOUT(); M_SCL_L(); M_DELAY;
    for(t=0;t<8;t++){ if(d&0x80)M_SDA_H();else M_SDA_L(); d<<=1; M_DELAY; M_SCL_H(); M_DELAY; M_SCL_L(); M_DELAY; }
}
static uint8_t M_ReadByte(uint8_t ack) {
    uint8_t i,r=0; M_SDAIN();
    for(i=0;i<8;i++){ M_SCL_L();M_DELAY; M_SCL_H();M_DELAY; r<<=1; if(M_RDSDA())r++; }
    M_SCL_L(); M_DELAY; M_SDAOUT();
    if(ack) M_SDA_L(); else M_SDA_H();
    M_DELAY; M_SCL_H(); M_DELAY; M_SCL_L(); M_DELAY;
    return r;
}

/* 读单寄存器: START→写addr→写reg→ReSTART→读addr→读data→STOP */
static uint8_t MAX_ReadReg(uint8_t reg)
{
    uint8_t v;
    M_Start(); M_SendByte(MAX_ADDR_W); if(M_WaitAck()){M_Stop();return 0xFF;}
    M_SendByte(reg);                if(M_WaitAck()){M_Stop();return 0xFE;}
    M_Start(); M_SendByte(MAX_ADDR_R); if(M_WaitAck()){M_Stop();return 0xFD;}
    v=M_ReadByte(0); M_Stop();
    return v;
}

/*==========================================================================
 * 主函数
 *==========================================================================*/
int main(void)
{
    uint8_t  part_id;
    uint16_t count = 0;
    uint8_t  pass  = 0;

    /*---- 初始化 ----*/
    Delay_Init();
    GPIO_All_Init();
    LED_OFF();
    BEEP_OFF();

    /*---- 启动自检: LED闪3次 + 蜂鸣器短鸣 ----*/
    { int i; for(i=0;i<3;i++){ LED_ON(); Delay_Ms(200); LED_OFF(); Delay_Ms(200); } }
    BEEP_ON(); Delay_Ms(100); BEEP_OFF();

    /*---- 双OLED初始化 ----*/
    OLED_Select(1); OLED_Init(); OLED_Clear();
    OLED_Show_Str(0,0,"OLED2: OK");
    OLED_Show_Str(0,1,"PA2/PA3 working");

    OLED_Select(0); OLED_Init(); OLED_Clear();
    OLED_Show_Str(0,0,"MAX30102 I2C TEST");
    OLED_Show_Str(0,2,"PART_ID: 0x");

    Delay_Ms(100);

    /*---- 循环读取 ----*/
    while(1)
    {
        part_id = MAX_ReadReg(0xFF);
        count++;

        OLED_Select(0);

        /* 显示读到的值 */
        {
            char hex[3];
            hex[0] = (part_id>>4)>=10 ? 'A'+(part_id>>4)-10 : '0'+(part_id>>4);
            hex[1] = (part_id&0xF)>=10 ? 'A'+(part_id&0xF)-10 : '0'+(part_id&0xF);
            hex[2] = 0;
            OLED_Show_Str(60,2,hex);
        }

        /* 结果 */
        if(part_id == 0x15) {
            OLED_Show_Str(0,4,">> PASS!! <<     ");
            pass++;
        } else if(part_id == 0xFF) {
            OLED_Show_Str(0,4,"FAIL: NACK(addr) ");
        } else if(part_id == 0xFE) {
            OLED_Show_Str(0,4,"FAIL: NACK(reg)  ");
        } else if(part_id == 0xFD) {
            OLED_Show_Str(0,4,"FAIL: NACK(read) ");
        } else {
            OLED_Show_Str(0,4,"FAIL: wrong ID  ");
        }

        /* 重试 */
        OLED_Show_Str(0,6,"Try:");
        OLED_Show_Num(30,6,count,4);
        if(pass > 0) {
            OLED_Show_Str(60,6,"PASS:");
            OLED_Show_Num(96,6,pass,2);
        }

        /* OLED2 心跳 */
        OLED_Select(1);
        OLED_Show_Str(0,3,(count%2)?"Reading .":"Reading ..");

        Delay_Ms(300);
    }
}
