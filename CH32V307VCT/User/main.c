/**********************************************
 * 第1步: MAX30102 I2C 极简测试 — 只读 PART_ID (0xFF)
 *
 * 预期: 返回 0x15 → 硬件OK, 进入第2步
 *       返回 0x00/0xFF/乱码 → 检查:
 *          1. 3.3V供电
 *          2. PE12=SCL, PE13=SDA 是否接反
 *          3. GND 共地
 *          4. I2C延时不够 (已设 Delay_Us(5) = ~100kHz)
 *          5. 芯片是否损坏
 *
 * OLED 显示:
 *   Row0: "MAX30102 I2C TEST"
 *   Row1: "PART_ID: 0x??"
 *   Row2: "PASS!!" or "FAIL!!"
 *   Row3: 重读次数
 * INT 引脚悬空不接 (本测试用FIFO指针, 不用中断)
 *
 * 编译: MounRiver Studio → Build → Download
 **********************************************/

#include "debug.h"
#include "ch32v30x_gpio.h"
#include "ch32v30x_rcc.h"

/*==========================================================================
 * OLED 简化驱动 (只用到 OLED1: PA0=SCL, PA1=SDA)
 *==========================================================================*/
#define OLED_ADDR  0x78
#define OLED_SCL_PIN    GPIO_Pin_0   /* PA0 */
#define OLED_SDA_PIN    GPIO_Pin_1   /* PA1 */
#define OLED_PORT_GPIO  GPIOA

/* LED + 蜂鸣器 */
#define LED_PIN    GPIO_Pin_5   /* PA5, 高电平亮 */
#define BEEP_PIN   GPIO_Pin_4   /* PA4, 低电平响 */

/* ---- OLED I2C 微秒延时 (96MHz下 Delay_Us(5)≈标准模式) ---- */
#define O_Delay()  Delay_Us(5)

static void O_SDA_H(void) { GPIO_SetBits(OLED_PORT_GPIO, OLED_SDA_PIN); }
static void O_SDA_L(void) { GPIO_ResetBits(OLED_PORT_GPIO, OLED_SDA_PIN); }
static void O_SCL_H(void) { GPIO_SetBits(OLED_PORT_GPIO, OLED_SCL_PIN); }
static void O_SCL_L(void) { GPIO_ResetBits(OLED_PORT_GPIO, OLED_SCL_PIN); }

static void O_SDA_OUT(void)
{
    GPIO_InitTypeDef c;
    c.GPIO_Pin   = OLED_SDA_PIN;
    c.GPIO_Mode  = GPIO_Mode_Out_PP;
    c.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(OLED_PORT_GPIO, &c);
}
static void O_SDA_IN(void)
{
    GPIO_InitTypeDef c;
    c.GPIO_Pin   = OLED_SDA_PIN;
    c.GPIO_Mode  = GPIO_Mode_IPU;
    c.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(OLED_PORT_GPIO, &c);
}
static uint8_t O_READ_SDA(void) { return GPIO_ReadInputDataBit(OLED_PORT_GPIO, OLED_SDA_PIN); }

static void O_Start(void)
{
    O_SDA_OUT(); O_SDA_H(); O_SCL_H(); O_Delay();
    O_SDA_L();   O_Delay(); O_SCL_L(); O_Delay();
}
static void O_Stop(void)
{
    O_SDA_OUT(); O_SCL_L(); O_SDA_L(); O_Delay();
    O_SCL_H();   O_SDA_H(); O_Delay();
}
static uint8_t O_WaitAck(void)
{
    uint8_t t=0; O_SDA_IN(); O_SDA_H(); O_Delay();
    O_SCL_H(); O_Delay();
    while(O_READ_SDA()){if(++t>200){O_SCL_L();return 1;}}
    O_SCL_L(); O_Delay(); return 0;
}
static void O_Ack(void)   { O_SCL_L(); O_Delay(); O_SDA_OUT(); O_SDA_L(); O_Delay(); O_SCL_H(); O_Delay(); O_SCL_L(); O_Delay(); }
static void O_NAck(void)  { O_SCL_L(); O_Delay(); O_SDA_OUT(); O_SDA_H(); O_Delay(); O_SCL_H(); O_Delay(); O_SCL_L(); O_Delay(); }
static void O_SendByte(uint8_t d)
{
    uint8_t t; O_SDA_OUT(); O_SCL_L(); O_Delay();
    for(t=0;t<8;t++){if(d&0x80)O_SDA_H();else O_SDA_L(); d<<=1; O_Delay(); O_SCL_H(); O_Delay(); O_SCL_L(); O_Delay();}
}
static uint8_t O_ReadByte(uint8_t ack)
{
    uint8_t i,r=0; O_SDA_IN();
    for(i=0;i<8;i++){O_SCL_L();O_Delay();O_SCL_H();r<<=1;if(O_READ_SDA())r++;O_Delay();}
    O_SCL_L(); O_Delay(); if(ack)O_Ack();else O_NAck(); return r;
}

/* ---- OLED 写命令/数据 ---- */
static void O_WriteCmd(uint8_t cmd)
{
    O_Start(); O_SendByte(OLED_ADDR<<1); O_WaitAck();
    O_SendByte(0x00); O_WaitAck();
    O_SendByte(cmd);  O_WaitAck();
    O_Stop();
}
static void O_WriteData(uint8_t dat)
{
    O_Start(); O_SendByte(OLED_ADDR<<1); O_WaitAck();
    O_SendByte(0x40); O_WaitAck();
    O_SendByte(dat);  O_WaitAck();
    O_Stop();
}
static void O_Init(void)
{
    Delay_Ms(100);
    O_WriteCmd(0xAE); O_WriteCmd(0x20); O_WriteCmd(0x00); /* 水平寻址 */
    O_WriteCmd(0xB0); O_WriteCmd(0xC8); /* 从上到下 */
    O_WriteCmd(0x00); O_WriteCmd(0x10); /* 低/高列起始 */
    O_WriteCmd(0x40); /* 显示起始行 */
    O_WriteCmd(0x8D); O_WriteCmd(0x14); /* 电荷泵 */
    O_WriteCmd(0xA1); O_WriteCmd(0xA6); /* 正常显示 */
    O_WriteCmd(0xA8); O_WriteCmd(0x3F); /* MUX=64 */
    O_WriteCmd(0xD3); O_WriteCmd(0x00);
    O_WriteCmd(0xD5); O_WriteCmd(0xF0);
    O_WriteCmd(0xD9); O_WriteCmd(0x22);
    O_WriteCmd(0xDA); O_WriteCmd(0x12);
    O_WriteCmd(0xDB); O_WriteCmd(0x20);
    O_WriteCmd(0x81); O_WriteCmd(0xCF);
    O_WriteCmd(0xA4); O_WriteCmd(0xAF); /* 开显示 */
}
static void O_SetPos(uint8_t x, uint8_t y)
{
    O_WriteCmd(0xB0+y);
    O_WriteCmd(((x&0xF0)>>4)|0x10);
    O_WriteCmd((x&0x0F));
}
static void O_Clear(void)
{
    uint8_t i,j;
    for(i=0;i<8;i++){O_SetPos(0,i);for(j=0;j<128;j++)O_WriteData(0x00);}
}

/* 6x8字体 */
static const uint8_t f6x8[][6] = {
    {0x00,0x00,0x00,0x00,0x00,0x00}, /* ' ' */
    {0x00,0x00,0x00,0x4F,0x00,0x00}, /* ! */
    {0x00,0x3E,0x51,0x49,0x45,0x3E}, /* 0 */
    {0x00,0x00,0x42,0x7F,0x40,0x00}, /* 1 */
    {0x00,0x42,0x61,0x51,0x49,0x46}, /* 2 */
    {0x00,0x21,0x41,0x45,0x4B,0x31}, /* 3 */
    {0x00,0x18,0x14,0x12,0x7F,0x10}, /* 4 */
    {0x00,0x27,0x45,0x45,0x45,0x39}, /* 5 */
    {0x00,0x3C,0x4A,0x49,0x49,0x30}, /* 6 */
    {0x00,0x01,0x71,0x09,0x05,0x03}, /* 7 */
    {0x00,0x36,0x49,0x49,0x49,0x36}, /* 8 */
    {0x00,0x06,0x49,0x49,0x29,0x1E}, /* 9 */
    {0x00,0x7E,0x11,0x11,0x11,0x7E}, /* A */
    {0x00,0x7F,0x49,0x49,0x49,0x36}, /* B */
    {0x00,0x3E,0x41,0x41,0x41,0x22}, /* C */
    {0x00,0x7F,0x41,0x41,0x22,0x1C}, /* D */
    {0x00,0x7F,0x49,0x49,0x49,0x41}, /* E */
    {0x00,0x7F,0x09,0x09,0x01,0x01}, /* F */
    {0x00,0x3E,0x41,0x51,0x32,0x00}, /* G */   /* pass的简写 */
    {0x00,0x7F,0x08,0x08,0x08,0x7F}, /* H */
    {0x00,0x00,0x41,0x7F,0x41,0x00}, /* I */
    {0x00,0x30,0x40,0x41,0x3F,0x01}, /* J */
    {0x00,0x00,0x00,0x00,0x00,0x00}, /* K (用空格) */
    {0x00,0x7F,0x40,0x40,0x40,0x40}, /* L */
    {0x00,0x7F,0x02,0x0C,0x02,0x7F}, /* M */
    {0x00,0x7F,0x04,0x08,0x10,0x7F}, /* N */
    {0x00,0x3E,0x41,0x41,0x41,0x3E}, /* O */
    {0x00,0x7F,0x09,0x09,0x09,0x06}, /* P */
    {0x00,0x3E,0x41,0x51,0x21,0x5E}, /* Q */
    {0x00,0x7F,0x09,0x19,0x29,0x46}, /* R */
    {0x00,0x46,0x49,0x49,0x49,0x31}, /* S */
    {0x00,0x01,0x01,0x7F,0x01,0x01}, /* T */
    {0x00,0x3F,0x40,0x40,0x40,0x3F}, /* U */
    {0x00,0x1F,0x20,0x40,0x20,0x1F}, /* V */
    {0x00,0x7F,0x20,0x18,0x20,0x7F}, /* W */
    {0x00,0x63,0x14,0x08,0x14,0x63}, /* X */
    {0x00,0x03,0x04,0x78,0x04,0x03}, /* Y */
    {0x00,0x61,0x51,0x49,0x45,0x43}, /* Z */
};

static void O_ShowChar(uint8_t x, uint8_t y, char ch)
{
    uint8_t i, idx;
    if(ch == ' ') idx = 0;
    else if(ch >= '0' && ch <= '9') idx = 1 + (ch - '0');  /* 1-11 */
    else if(ch >= 'A' && ch <= 'F') idx = 12 + (ch - 'A'); /* 12-17 */
    else if(ch == 'x') idx = 29;  /* X */
    else return;
    O_SetPos(x, y);
    for(i = 0; i < 6; i++) O_WriteData(f6x8[idx][i]);
}
static void O_ShowStr(uint8_t x, uint8_t y, const char *s)
{
    while(*s) { O_ShowChar(x, y, *s); x += 6; s++; }
}
static void O_ShowHex(uint8_t x, uint8_t y, uint8_t val)
{
    char nib = (val >> 4) & 0x0F;
    char c0 = (nib < 10) ? ('0' + nib) : ('A' + nib - 10);
    nib = val & 0x0F;
    char c1 = (nib < 10) ? ('0' + nib) : ('A' + nib - 10);
    O_ShowChar(x, y, c0);
    O_ShowChar(x+6, y, c1);
}

/*==========================================================================
 * MAX30102 软件I2C — PE12=SCL, PE13=SDA, 地址 0xAE(W) / 0xAF(R)
 *==========================================================================*/
#define MAX_SCL_PIN  GPIO_Pin_12
#define MAX_SDA_PIN  GPIO_Pin_13
#define MAX_PORT     GPIOE
#define MAX_ADDR_W   0xAE   /* 0x57<<1 | 0 */
#define MAX_ADDR_R   0xAF   /* 0x57<<1 | 1 */

static void M_SDA_H(void) { GPIO_SetBits(MAX_PORT, MAX_SDA_PIN); }
static void M_SDA_L(void) { GPIO_ResetBits(MAX_PORT, MAX_SDA_PIN); }
static void M_SCL_H(void) { GPIO_SetBits(MAX_PORT, MAX_SCL_PIN); }
static void M_SCL_L(void) { GPIO_ResetBits(MAX_PORT, MAX_SCL_PIN); }
static uint8_t M_READ_SDA(void) { return GPIO_ReadInputDataBit(MAX_PORT, MAX_SDA_PIN); }

static void M_SDA_OUT(void)
{
    GPIO_InitTypeDef c;
    c.GPIO_Pin   = MAX_SDA_PIN;
    c.GPIO_Mode  = GPIO_Mode_Out_PP;
    c.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MAX_PORT, &c);
}
static void M_SDA_IN(void)
{
    GPIO_InitTypeDef c;
    c.GPIO_Pin   = MAX_SDA_PIN;
    c.GPIO_Mode  = GPIO_Mode_IPU;
    c.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MAX_PORT, &c);
}

/* ---- MAX30102 I2C 时序 (Delay_Us(5)=~100kHz标准模式) ---- */
#define M_DELAY  Delay_Us(5)

static void M_Start(void)
{
    M_SDA_OUT(); M_SDA_H(); M_SCL_H(); M_DELAY;
    M_SDA_L();   M_DELAY; M_SCL_L(); M_DELAY;
}
static void M_Stop(void)
{
    M_SDA_OUT(); M_SCL_L(); M_SDA_L(); M_DELAY;
    M_SCL_H(); M_SDA_H(); M_DELAY;
}
static uint8_t M_WaitAck(void)
{
    uint8_t t = 0;
    M_SDA_IN(); M_SDA_H(); M_DELAY;
    M_SCL_H();  M_DELAY;
    while(M_READ_SDA()) { if(++t > 250) { M_SCL_L(); return 1; } }
    M_SCL_L(); M_DELAY;
    return 0;
}
static void M_SendByte(uint8_t d)
{
    uint8_t t;
    M_SDA_OUT(); M_SCL_L(); M_DELAY;
    for(t = 0; t < 8; t++) {
        if(d & 0x80) M_SDA_H(); else M_SDA_L();
        d <<= 1;
        M_DELAY; M_SCL_H(); M_DELAY; M_SCL_L(); M_DELAY;
    }
}
static uint8_t M_ReadByte(uint8_t ack)
{
    uint8_t i, r = 0;
    M_SDA_IN();
    for(i = 0; i < 8; i++) {
        M_SCL_L(); M_DELAY; M_SCL_H(); M_DELAY;
        r <<= 1; if(M_READ_SDA()) r++;
    }
    M_SCL_L(); M_DELAY;
    if(ack) { M_SDA_OUT(); M_SDA_L(); }  /* ACK */
    else    { M_SDA_OUT(); M_SDA_H(); }  /* NACK */
    M_DELAY; M_SCL_H(); M_DELAY; M_SCL_L(); M_DELAY;
    return r;
}

/* ---- 核心函数: 读 MAX30102 单寄存器 ---- */
static uint8_t MAX_ReadReg(uint8_t reg)
{
    uint8_t val;

    /* 第1帧: START + 写地址 + 寄存器号 */
    M_Start();
    M_SendByte(MAX_ADDR_W);
    if(M_WaitAck()) { M_Stop(); return 0xFF; }  /* NACK=器件不存在 */

    M_SendByte(reg);
    if(M_WaitAck()) { M_Stop(); return 0xFE; }

    /* 第2帧: Repeated START + 读地址 + 读数据 */
    M_Start();
    M_SendByte(MAX_ADDR_R);
    if(M_WaitAck()) { M_Stop(); return 0xFD; }

    val = M_ReadByte(0);  /* NACK after last byte */
    M_Stop();

    return val;
}

/* ---- 写单寄存器 (测试用) ---- */
static uint8_t MAX_WriteReg(uint8_t reg, uint8_t val)
{
    M_Start();
    M_SendByte(MAX_ADDR_W); if(M_WaitAck()) { M_Stop(); return 0; }
    M_SendByte(reg);        if(M_WaitAck()) { M_Stop(); return 0; }
    M_SendByte(val);        if(M_WaitAck()) { M_Stop(); return 0; }
    M_Stop();
    return 1;
}

/*==========================================================================
 * GPIO 初始化
 *==========================================================================*/
static void GPIO_Init_All(void)
{
    GPIO_InitTypeDef c;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  /* OLED */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);  /* MAX30102 */

    /* OLED SCL(PA0推挽) + SDA(PA1推挽, OLED I2C用推挽+方向切换) */
    c.GPIO_Pin   = OLED_SCL_PIN;
    c.GPIO_Mode  = GPIO_Mode_Out_PP;
    c.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(OLED_PORT_GPIO, &c);

    c.GPIO_Pin   = OLED_SDA_PIN;
    GPIO_Init(OLED_PORT_GPIO, &c);

    O_SCL_H();
    O_SDA_H();

    /* LED PA5 推挽输出 (高电平亮) */
    c.GPIO_Pin   = LED_PIN;
    c.GPIO_Mode  = GPIO_Mode_Out_PP;
    c.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(OLED_PORT_GPIO, &c);
    GPIO_ResetBits(OLED_PORT_GPIO, LED_PIN);   /* 初始灭 */

    /* 蜂鸣器 PA4 推挽输出 (低电平响) */
    c.GPIO_Pin   = BEEP_PIN;
    c.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(OLED_PORT_GPIO, &c);
    GPIO_SetBits(OLED_PORT_GPIO, BEEP_PIN);    /* 初始关 */

    /* MAX30102 SCL(PE12推挽) + SDA(PE13推挽, 方向动态切换) */
    c.GPIO_Pin   = MAX_SCL_PIN;
    c.GPIO_Mode  = GPIO_Mode_Out_PP;
    c.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MAX_PORT, &c);

    c.GPIO_Pin   = MAX_SDA_PIN;
    GPIO_Init(MAX_PORT, &c);

    M_SCL_H();
    M_SDA_H();
}

/*==========================================================================
 * 主函数
 *==========================================================================*/
int main(void)
{
    uint8_t  part_id;
    uint16_t count = 0;
    uint8_t  pass  = 0;
    char     buf[21];

    /*---- 初始化 ----*/
    Delay_Init();
    GPIO_Init_All();
    O_Init();
    O_Clear();

    /*---- 启动自检: LED 慢闪3次 + 蜂鸣器短鸣1次 ----*/
    {
        int bl;
        for(bl = 0; bl < 3; bl++) {
            GPIO_SetBits(OLED_PORT_GPIO, LED_PIN);   Delay_Ms(200);
            GPIO_ResetBits(OLED_PORT_GPIO, LED_PIN);  Delay_Ms(200);
        }
        GPIO_ResetBits(OLED_PORT_GPIO, BEEP_PIN);     Delay_Ms(100);
        GPIO_SetBits(OLED_PORT_GPIO, BEEP_PIN);
    }

    /*---- 屏显标题 ----*/
    O_ShowStr(0, 0, "MAX30102 I2C TEST");
    O_ShowStr(0, 2, "PART_ID: 0x");
    O_ShowStr(0, 7, "Reading...");

    Delay_Ms(100);  /* 等传感器上电稳定 */

    /*---- 循环读取 PART_ID ----*/
    while(1)
    {
        part_id = MAX_ReadReg(0xFF);
        count++;

        /* 显示: 及时显示进屏 */
        // Row2: 显示读到的值
        O_ShowStr(0, 2, "PART_ID: 0x");
        O_ShowHex(66, 2, part_id);

        /* 判断 */
        if(part_id == 0x15) {
            O_ShowStr(0, 4, ">> PASS!! <<");
            pass++;
        } else if(part_id == 0xFF) {
            O_ShowStr(0, 4, "FAIL: NACK (no dev)");   /*器件不应答*/
        } else if(part_id == 0xFE) {
            O_ShowStr(0, 4, "FAIL: NACK (reg)");        /*寄存器地址不应答*/
        } else if(part_id == 0xFD) {
            O_ShowStr(0, 4, "FAIL: NACK (read)");       /*读操作不应答*/
        } else {
            O_ShowStr(0, 4, "FAIL: wrong ID");
        }

        /* 重试信息 */
        O_ShowStr(0, 6, "Retry: ");
        {
            uint8_t i;
            for(i=0;i<16;i++) buf[i]=' '; buf[16]=0;
        }
        O_ShowStr(42, 6, buf);  /* 清旧数字 */
        {
            char ns[6];
            uint8_t pos = 54;
            ns[0] = '0' + (count / 1000 % 10);
            ns[1] = '0' + (count / 100  % 10);
            ns[2] = '0' + (count / 10   % 10);
            ns[3] = '0' + (count / 1    % 10);
            ns[4] = 0;
            O_ShowStr(pos, 6, ns);
        }
        O_ShowStr(0, 7, (pass > 0) ? "PASS x" : "       ");
        if(pass > 0) {
            O_ShowStr(48, 7, "");
            O_ShowHex(48, 7, (uint8_t)pass);
        }

        Delay_Ms(1000);  /* 每秒读一次 */
    }
}
