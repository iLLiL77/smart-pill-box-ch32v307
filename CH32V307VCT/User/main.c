/**********************************************
 * 智能药盒 — 双药盒独立配置、多时段提醒、取药检测
 *
 * LED:  PA5 高电平亮 (SetBits=ON, ResetBits=OFF)
 * BEEP: PA4 低电平响 (ResetBits=ON, SetBits=OFF)
 *
 * 按键: K1(PA8 MODE)=切换选项  K2(PA9 ADD)=加值  K3(PA10 OK)=确认/下一页
 *
 * 三态系统:
 *   STATE_SETUP(0) — 三页菜单设置两个药盒+当前时间
 *   STATE_RUN  (1) — 走时+闹钟检测
 *   STATE_ALERT(2) — 声光提醒+取药检测(重量+红外双重判定)
 **********************************************/

#include "config.h"
#include "gpio.h"
#include "key.h"
#include "hx711.h"
#include "usart_voice.h"
#include "oled.h"
#include "max30102.h"

/*==========================================================================
 * LED/BEEP 快捷宏
 *==========================================================================*/
#define LED_ON()    GPIO_SetBits(GPIOA, LED_PIN)
#define LED_OFF()   GPIO_ResetBits(GPIOA, LED_PIN)
#define BEEP_ON()   GPIO_ResetBits(BEEP_PORT, BEEP_PIN)
#define BEEP_OFF()  GPIO_SetBits(BEEP_PORT, BEEP_PIN)

/* 红外传感器 (PA7, 遮挡=低=开盖) */
#define IR_BLOCKED()  (GPIO_ReadInputDataBit(GPIOA, PHOTO_PIN) == 0)

void Delay_ms(uint32_t ms) { Delay_Ms(ms); }

/*==========================================================================
 * 全局变量定义
 *==========================================================================*/
MedConfig med[MED_COUNT] = {
    {1, 1, {8,  12, 18}, {0, 0, 0}},   /* 药盒1: 1粒,1次/天,默认8:00 */
    {1, 1, {12, 18, 20}, {0, 0, 0}}    /* 药盒2: 1粒,1次/天,默认12:00 */
};
uint8_t  cur_h = 8, cur_m = 0;          /* 当前时间 */
uint8_t  sel_page  = 0;                 /* 设置页: 0=药盒1, 1=药盒2, 2=当前时间 */
uint8_t  sel_item  = 0;                 /* 页内选项索引 */
uint8_t  sys_state = STATE_SETUP;       /* 系统状态 */
volatile uint8_t  refresh_flag = 1;
volatile uint32_t rtc_seconds  = 0;

uint8_t  alert_flag  [MED_COUNT];       /* 各药盒触发标记(本次) */
uint8_t  voice_ok    [MED_COUNT];       /* 语音已播标记 */
float    origin_w    [MED_COUNT];       /* 触发时初始重量 */
uint32_t hx_buf      [MED_COUNT];       /* 去皮值 */
float    scale = SCALE_DEFAULT;         /* HX711 标定系数 */

uint8_t  triggered[MED_COUNT][MAX_SLOTS]; /* 今天各时段是否已触发 */
uint8_t  alert_med = 0;                   /* 当前告警的药盒 */
uint32_t entry_sec = 0;                   /* 进入运行时的秒数(防误触发) */
uint8_t  last_min  = 0xFF;                /* 上一分钟(分钟沿检测) */

/*==========================================================================
 * TIM2: 1Hz 时间基准
 * 系统时钟=96MHz, APB1=48MHz, TIM2_CLK=96MHz(APB1*2)
 * Prescaler=9600-1, Period=10000-1 → 1Hz
 *==========================================================================*/
static void TIM2_Init(void)
{
    TIM_TimeBaseInitTypeDef tim;
    NVIC_InitTypeDef nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseStructInit(&tim);
    tim.TIM_Prescaler     = 9600 - 1;
    tim.TIM_Period        = 10000 - 1;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &tim);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM2_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        rtc_seconds++;
        refresh_flag = 1;
    }
}

/*==========================================================================
 * 获取当前页的选项数量
 *==========================================================================*/
static uint8_t PageItems(uint8_t page)
{
    if(page == 0 || page == 1)
        return 2 + med[page].times_per_day * 2; /* pills+times + H/M各一 */
    else
        return 2;                                /* cur_h, cur_m */
}

/*==========================================================================
 * OLED2 显示初始化 — 全屏绘制(重量+心率血氧)
 * Row 0: 心率血氧  Row 1: 分隔线
 * Row 2-3: 重量    Row 6-7: 去皮提示
 *==========================================================================*/
static void Show_OLED2_Init(void)
{
    OLED_Select(1);
    OLED_Clear();
    OLED_Show_Str(0, 0, "HR:---bpm SpO2:--%");
    OLED_Show_Str(0, 1, "-------------------");
    OLED_Show_Str(0, 2, "W1:");
    OLED_Show_Str(0, 3, "W2:");
    OLED_Show_Str(0, 6, "Tare:");
    OLED_Show_Str(0, 7, "M=Box1  A=Box2");
    OLED_Select(0);
}

/*==========================================================================
 * OLED2 显示 — 更新重量+心率血氧(不清全屏,不闪屏)
 *==========================================================================*/
static void Show_OLED2(void)
{
    float w1 = Get_Weight(MED_A);
    float w2 = Get_Weight(MED_B);
    int16_t hr   = MAX30102_GetHR();
    uint8_t spo2 = MAX30102_GetSpO2();

    if(w1 < 0) w1 = 0;
    if(w2 < 0) w2 = 0;

    OLED_Select(1);

    /* Row 0: 心率+血氧 */
    {
        char buf[21];
        uint8_t i;
        if(!max30102_enabled)
        {
            /* MAX30102 已关闭 */
            char *s = "MAX:OFF             ";
            for(i=0;i<20;i++) buf[i]=s[i]; buf[20]=0;
        }
        else if(MAX30102_IsValid())
        {
            /* HR:XXXbpm SpO2:XX% */
            buf[0]='H'; buf[1]='R'; buf[2]=':';
            buf[3]='0'+hr/100; buf[4]='0'+(hr/10)%10; buf[5]='0'+hr%10;
            buf[6]='b'; buf[7]='p'; buf[8]='m'; buf[9]=' ';
            buf[10]='S'; buf[11]='p'; buf[12]='O'; buf[13]='2'; buf[14]=':';
            buf[15]='0'+spo2/10; buf[16]='0'+spo2%10; buf[17]='%';
            for(i=18;i<21;i++) buf[i]=' ';
        }
        else
        {
            /* HR:---bpm SpO2:--% */
            char *s = "HR:---bpm SpO2:--%";
            for(i=0;i<20;i++) buf[i]=s[i]; buf[20]=0;
        }
        OLED_Show_Str(0, 0, buf);
    }

    /* Row 2: 药盒1重量 */
    OLED_Show_Str(0, 2, "W1:       ");
    OLED_Show_Weight(18, 2, w1);

    /* Row 3: 药盒2重量 */
    OLED_Show_Str(0, 3, "W2:       ");
    OLED_Show_Weight(18, 3, w2);

    OLED_Select(0);
}

/*==========================================================================
 * 显示 — 设置模式(分页)
 *==========================================================================*/
static void Show_Setup(void)
{
    uint8_t med_idx = (sel_page <= 1) ? sel_page : 0;
    uint8_t cursor  = 0;  /* 当前页内的游标 */
    uint8_t active_slots;
    uint8_t row;          /* 显示行号 */

    OLED_Clear();

    /* 标题行 */
    if(sel_page == 0)
        OLED_Show_Str(0,0,"== Med1 Setup ==");
    else if(sel_page == 1)
        OLED_Show_Str(0,0,"== Med2 Setup ==");
    else
        OLED_Show_Str(0,0,"== Set Clock  ==");

    /* 右上角页码 */
    {
        char pg[4];
        pg[0] = '0' + sel_page; pg[1] = '/'; pg[2] = '2'; pg[3] = 0;
        OLED_Show_Str(90,0,pg);
    }

    if(sel_page <= 1)
    {
        active_slots = med[med_idx].times_per_day;

        /* ---- 药盒设置页 ---- */
        /* Row1: 每次粒数 (cursor=0) */
        row = 1;
        OLED_Show_Str(0,row,(sel_item == cursor) ? ">" : " ");
        OLED_Show_Str(6,row,"Pills/dose:");
        OLED_Show_Num(78,row,med[med_idx].pills_per_dose,2);
        cursor++;

        /* Row2: 每天次数 (cursor=1) */
        row = 2;
        OLED_Show_Str(0,row,(sel_item == cursor) ? ">" : " ");
        OLED_Show_Str(6,row,"Times/day:");
        OLED_Show_Num(78,row,med[med_idx].times_per_day,2);
        cursor++;

        /* Row3~Row8: 时间段H/M */
        for(uint8_t s = 0; s < active_slots; s++)
        {
            char lbl[4];

            /* 时段-时: row = 3 + s*2 */
            row = 3 + s * 2;
            OLED_Show_Str(0,row,(sel_item == cursor) ? ">" : " ");
            lbl[0] = 'T'; lbl[1] = '1'+s; lbl[2] = 'H'; lbl[3] = 0;
            OLED_Show_Str(6,row,lbl);
            OLED_Show_Num(30,row,med[med_idx].slot_h[s],2);
            cursor++;

            /* 时段-分: row = 4 + s*2 */
            row = 4 + s * 2;
            OLED_Show_Str(0,row,(sel_item == cursor) ? ">" : " ");
            lbl[0] = 'T'; lbl[1] = '1'+s; lbl[2] = 'M'; lbl[3] = 0;
            OLED_Show_Str(6,row,lbl);
            OLED_Show_Num(30,row,med[med_idx].slot_m[s],2);
            cursor++;
        }

        /* 防止切换次数后 sel_item 越界 */
        {
            uint8_t max_item = 2 + active_slots * 2;
            if(sel_item >= max_item) sel_item = max_item > 0 ? max_item - 1 : 0;
        }

        OLED_Show_Str(0,7,"K1:sel K2:+  K3:nxt");
    }
    else
    {
        /* ---- 当前时间页 ---- */
        OLED_Show_Str(0,1,(sel_item == 0) ? ">" : " ");
        OLED_Show_Str(6,1,"Hour:");
        OLED_Show_Num(54,1,cur_h,2);

        OLED_Show_Str(0,2,(sel_item == 1) ? ">" : " ");
        OLED_Show_Str(6,2,"Min :");
        OLED_Show_Num(54,2,cur_m,2);

        OLED_Show_Str(0,7,"K1:sel K2:+  K3:OK ");
    }
}

/*==========================================================================
 * 显示 — 运行模式 (含双通道实时重量)
 *==========================================================================*/
static void Show_Run(void)
{
    uint32_t sod = rtc_seconds % 86400;
    uint8_t h = sod / 3600;
    uint8_t m = (sod % 3600) / 60;
    uint8_t s = sod % 60;
    float w1 = Get_Weight(MED_A);
    float w2 = Get_Weight(MED_B);
    uint8_t row = 3;

    OLED_Clear();

    /* Row 0: 当前时间 + 状态 */
    {
        char t[9];
        t[0]='0'+h/10; t[1]='0'+h%10; t[2]=':';
        t[3]='0'+m/10; t[4]='0'+m%10; t[5]=':';
        t[6]='0'+s/10; t[7]='0'+s%10; t[8]=0;
        OLED_Show_Str(0,0,t);
        OLED_Show_Str(54,0,"Running");
    }

    /* Row 1: 药盒1 实时重量 */
    OLED_Show_Str(0,1,"W1:");
    OLED_Show_Weight(18,1,w1);
    OLED_Show_Str(60,1,"       ");
    OLED_Show_Char(60,1,'[');
    {
        uint8_t stable1 = (w1 >= 0) ? 8 : 0;
        uint8_t i;
        for(i=0;i<10;i++)
        {
            if(i < stable1)
                OLED_Show_Char(66+i*6,1,'|');
            else
                OLED_Show_Char(66+i*6,1,' ');
        }
    }
    OLED_Show_Char(126,1,']');

    /* Row 2: 药盒2 实时重量 */
    OLED_Show_Str(0,2,"W2:");
    OLED_Show_Weight(18,2,w2);
    OLED_Show_Str(60,2,"       ");
    OLED_Show_Char(60,2,'[');
    {
        uint8_t stable2 = (w2 >= 0) ? 8 : 0;
        uint8_t i;
        for(i=0;i<10;i++)
        {
            if(i < stable2)
                OLED_Show_Char(66+i*6,2,'|');
            else
                OLED_Show_Char(66+i*6,2,' ');
        }
    }
    OLED_Show_Char(126,2,']');

    /* Row 3-6: 闹钟时段 (紧凑排列) */
    for(uint8_t idx = 0; idx < MED_COUNT && row <= 6; idx++)
    {
        uint8_t ts = med[idx].times_per_day;

        /* 第一行: Mx + 前2个时段 */
        OLED_Show_Char(0,row,'M');
        OLED_Show_Char(6,row,'1'+idx);
        {
            uint8_t cx = 18;
            for(uint8_t sl = 0; sl < ts && sl < 2; sl++)
            {
                OLED_Show_Char(cx,row,'T'); cx += 6;
                OLED_Show_Char(cx,row,'1'+sl); cx += 6;
                OLED_Show_Time(cx,row,med[idx].slot_h[sl],med[idx].slot_m[sl]);
                cx += 30;
                if(triggered[idx][sl])
                {
                    OLED_Show_Char(cx,row,'['); cx += 6;
                    OLED_Show_Char(cx,row,'O'); cx += 6;
                    OLED_Show_Char(cx,row,'K'); cx += 6;
                    OLED_Show_Char(cx,row,']');
                }
                cx += 6;
            }
        }
        row++;

        /* 第二行: 第3个时段 (如果存在) */
        if(ts > 2 && row <= 6)
        {
            uint8_t sl = 2;
            uint8_t cx = 18;
            OLED_Show_Char(cx,row,'T'); cx += 6;
            OLED_Show_Char(cx,row,'1'+sl); cx += 6;
            OLED_Show_Time(cx,row,med[idx].slot_h[sl],med[idx].slot_m[sl]);
            if(triggered[idx][sl])
            {
                cx += 30;
                OLED_Show_Char(cx,row,'['); cx += 6;
                OLED_Show_Char(cx,row,'O'); cx += 6;
                OLED_Show_Char(cx,row,'K'); cx += 6;
                OLED_Show_Char(cx,row,']');
            }
            row++;
        }
    }

    /* Row 7: 按键提示 */
    OLED_Show_Str(0,7,"K3=back M:tare1 A:tare2");
}

/*==========================================================================
 * 显示 — 提醒模式 (含取药重量变化)
 *==========================================================================*/
static void Show_Alert(void)
{
    float now_w = Get_Weight(alert_med);
    float removed = origin_w[alert_med] - now_w;
    if(now_w < 0) now_w = 0;
    if(removed < 0) removed = 0;

    OLED_Clear();

    /* Row 0: 标题 */
    OLED_Show_Str(0,0,"!!! TAKE MED !!!");

    /* Row 1: 哪个药盒 + 吃几粒 */
    OLED_Show_Str(0,1,"Box:");
    OLED_Show_Char(30,1,'1'+alert_med);
    OLED_Show_Str(48,1,"Take:");
    OLED_Show_Num(78,1,med[alert_med].pills_per_dose,2);
    OLED_Show_Str(96,1,"pills");

    /* Row 2: 红外状态 */
    {
        uint8_t ir = GPIO_ReadInputDataBit(GPIOA, PHOTO_PIN);
        OLED_Show_Str(0,2,"IR:");
        OLED_Show_Str(24,2,(ir == 0) ? "Blocked(open)" : "Clear(closed)");
    }

    /* Row 3: 取药前重量 */
    OLED_Show_Str(0,3,"Before:");
    OLED_Show_Weight(42,3,origin_w[alert_med]);

    /* Row 4: 当前重量 */
    OLED_Show_Str(0,4,"Now:  ");
    OLED_Show_Weight(42,4,now_w);

    /* Row 5: 已取走重量 */
    OLED_Show_Str(0,5,"Taken:");
    OLED_Show_Weight(42,5,removed);

    /* Row 6: 状态提示 */
    if(removed >= TAKE_WEIGHT && IR_BLOCKED())
        OLED_Show_Str(0,6,"Taken -> dismiss!");
    else if(removed >= TAKE_WEIGHT)
        OLED_Show_Str(0,6,"Open lid to confirm");
    else
        OLED_Show_Str(0,6,"Remove pills + open");

    /* Row 7: 按键 */
    OLED_Show_Str(0,7,"K3:force dismiss");
}

/*==========================================================================
 * 主函数
 *==========================================================================*/
int main(void)
{
    uint8_t blink_tick = 0;    /* LED/BEEP 闪烁计数 */

    /*---- 初始化 ----*/
    Delay_Init();
    GPIO_All_Init();
    LED_OFF();
    BEEP_OFF();

    /*---- 启动自检: LED 慢闪3次 + 蜂鸣器短鸣1次 ----*/
    for(int bl = 0; bl < 3; bl++)
    {
        LED_ON();  Delay_ms(200);
        LED_OFF(); Delay_ms(200);
    }
    BEEP_ON();  Delay_ms(100);
    BEEP_OFF();

    /*---- HX711 去皮 ----*/
    HX_Tare(MED_A);
    HX_Tare(MED_B);

    /*---- TIM2 + OLED ----*/
    TIM2_Init();                    /* 1Hz 时间基准 */
    OLED_Init();                    /* OLED1 — 药盒UI (PA0/PA1) */
    OLED_Select(1); OLED_Init();    /* OLED2 — 称重+心率 (PA2/PA3) */
    OLED_Select(0);                 /* 切回 OLED1 */
    Show_OLED2_Init();              /* OLED2 画静态界面(只一次) */
    Show_Setup();                   /* OLED1 显示设置菜单 */

    /*---- MAX30102 心率血氧初始化 ----*/
    MAX30102_Init();

    /*---- 清零触发标记 ----*/
    for(uint8_t i = 0; i < MED_COUNT; i++)
    {
        for(uint8_t j = 0; j < MAX_SLOTS; j++)
            triggered[i][j] = 0;
        alert_flag[i]  = 0;
        voice_ok[i]    = 0;
        origin_w[i]    = 0;
    }

    /*---- 主循环 ----*/
    while(1)
    {
        /* OLED2 称重显示: 每 ~500ms 刷新数字 (不闪屏) */
        {
            static uint8_t oled2_tick = 0;
            oled2_tick++;
            if(oled2_tick >= 5)
            {
                Show_OLED2();
                oled2_tick = 0;
            }
        }

        /* MAX30102 心率血氧: 每 ~100ms 从FIFO读取样本 */

        /* Button4 (PA6): 切换 MAX30102 开关 */
        if(Key_Max30102_Scan())
        {
            max30102_enabled = !max30102_enabled;
            MAX30102_Enable(max30102_enabled);
            Show_OLED2();  /* 立即刷新OLED2显示状态 */
        }

        MAX30102_Update();

        /*==========================*/
        /*  设置模式                */
        /*==========================*/
        if(sys_state == STATE_SETUP)
        {
            LED_OFF();
            BEEP_OFF();

            /* K1: 切换选项 */
            if(Key_Mode_Scan())
            {
                sel_item++;
                if(sel_item >= PageItems(sel_page))
                    sel_item = 0;
                Show_Setup();
            }

            /* K2: 加值 */
            if(Key_Add_Scan())
            {
                uint8_t med_idx = (sel_page <= 1) ? sel_page : 0;

                if(sel_page <= 1)
                {
                    uint8_t active = med[med_idx].times_per_day;

                    switch(sel_item)
                    {
                        case 0: /* pills_per_dose */
                            med[med_idx].pills_per_dose++;
                            if(med[med_idx].pills_per_dose > MAX_PILL)
                                med[med_idx].pills_per_dose = 1;
                            break;

                        case 1: /* times_per_day */
                            med[med_idx].times_per_day++;
                            if(med[med_idx].times_per_day > MAX_TIMES)
                                med[med_idx].times_per_day = 1;
                            /* 如果光标落在不可见的 slot 上, 回退 */
                            if(sel_item >= PageItems(sel_page))
                                sel_item = PageItems(sel_page) - 1;
                            break;

                        default:
                        {
                            /* slot H/M */
                            uint8_t rel = sel_item - 2; /* 相对 pills+times 后的偏移 */
                            uint8_t slot = rel / 2;
                            uint8_t hm   = rel % 2;  /* 0=H, 1=M */

                            if(slot < active)
                            {
                                if(hm == 0)
                                {
                                    med[med_idx].slot_h[slot]++;
                                    if(med[med_idx].slot_h[slot] > MAX_HOUR)
                                        med[med_idx].slot_h[slot] = 0;
                                }
                                else
                                {
                                    med[med_idx].slot_m[slot]++;
                                    if(med[med_idx].slot_m[slot] > MAX_MIN)
                                        med[med_idx].slot_m[slot] = 0;
                                }
                            }
                            break;
                        }
                    }
                }
                else /* 当前时间页 */
                {
                    if(sel_item == 0)
                    {
                        cur_h++;
                        if(cur_h > MAX_HOUR) cur_h = 0;
                    }
                    else
                    {
                        cur_m++;
                        if(cur_m > MAX_MIN) cur_m = 0;
                    }
                }
                Show_Setup();
            }

            /* K3: 下一页 / 确认 */
            if(Key_OK_Scan())
            {
                if(sel_page < 2)
                {
                    sel_page++;
                    sel_item = 0;
                    Show_Setup();
                }
                else
                {
                    /* 确认 → 进入运行模式 */
                    rtc_seconds = (uint32_t)cur_h * 3600 + (uint32_t)cur_m * 60;
                    entry_sec   = rtc_seconds;
                    last_min    = 0xFF;

                    /* 清零触发标记 */
                    for(uint8_t i = 0; i < MED_COUNT; i++)
                    {
                        alert_flag[i] = 0;
                        voice_ok[i]   = 0;
                        for(uint8_t j = 0; j < MAX_SLOTS; j++)
                            triggered[i][j] = 0;
                    }

                    sys_state = STATE_RUN;
                    LED_OFF();
                    BEEP_OFF();
                    Show_Run();       /* OLED1 */
                    Show_OLED2();     /* OLED2 */
                }
            }
        }
        /*==========================*/
        /*  运行模式                */
        /*==========================*/
        else if(sys_state == STATE_RUN)
        {
            uint32_t sod   = rtc_seconds % 86400;
            uint8_t  h_now = sod / 3600;
            uint8_t  m_now = (sod % 3600) / 60;
            uint8_t  s_now = sod % 60;

            /* LED 心跳: 每10秒闪一下, 显示系统在运行 */
            if((s_now % 10) == 0 && (rtc_seconds & 1) == 0)
                LED_ON();
            else if((s_now % 10) == 1)
                LED_OFF();

            /* K3: 返回设置 */
            if(Key_OK_Scan())
            {
                LED_OFF();
                BEEP_OFF();
                sys_state = STATE_SETUP;
                sel_page  = 0;
                sel_item  = 0;
                cur_h = h_now;
                cur_m = m_now;
                Show_Setup();
                continue;
            }

            /* 长按 MODE(PA8) >1秒: 药盒1去皮 */
            {
                static uint8_t  mode_hold = 0;
                static uint8_t  mode_done = 0;
                if(GPIO_ReadInputDataBit(GPIOA, KEY_MODE) == 0)
                {
                    if(!mode_done)
                    {
                        mode_hold++;
                        if(mode_hold > 10)  /* ~1秒 */
                        {
                            HX_Tare(MED_A);
                            mode_done = 1;
                            /* LED 快闪3次确认 */
                            for(int i=0;i<3;i++)
                            { LED_ON(); Delay_ms(80); LED_OFF(); Delay_ms(80); }
                        }
                    }
                }
                else { mode_hold = 0; mode_done = 0; }
            }

            /* 长按 ADD(PA9) >1秒: 药盒2去皮 */
            {
                static uint8_t  add_hold = 0;
                static uint8_t  add_done = 0;
                if(GPIO_ReadInputDataBit(GPIOA, KEY_ADD) == 0)
                {
                    if(!add_done)
                    {
                        add_hold++;
                        if(add_hold > 10)  /* ~1秒 */
                        {
                            HX_Tare(MED_B);
                            add_done = 1;
                            /* LED 快闪3次确认 */
                            for(int i=0;i<3;i++)
                            { LED_ON(); Delay_ms(80); LED_OFF(); Delay_ms(80); }
                        }
                    }
                }
                else { add_hold = 0; add_done = 0; }
            }

            /* 分钟沿 → 检查所有药盒所有时段 */
            if(m_now != last_min)
            {
                last_min = m_now;

                for(uint8_t idx = 0; idx < MED_COUNT; idx++)
                {
                    uint8_t ts = med[idx].times_per_day;

                    for(uint8_t sl = 0; sl < ts; sl++)
                    {
                        if(h_now == med[idx].slot_h[sl] &&
                           m_now == med[idx].slot_m[sl] &&
                           !triggered[idx][sl] &&
                           rtc_seconds > entry_sec + 10)
                        {
                            /* ✅ 触发! */
                            triggered[idx][sl] = 1;
                            alert_med          = idx;
                            alert_flag[idx]    = 1;
                            voice_ok[idx]      = 0;

                            /* 记录当前重量用于取药检测 */
                            origin_w[idx] = Get_Weight(idx);

                            sys_state = STATE_ALERT;
                            LED_ON();
                            BEEP_ON();
                            blink_tick = 0;

                            /* 播报语音 (语音模块稍后接入时自动生效) */
                            Voice_Play_Alert();
                            voice_ok[idx] = 1;

                            Show_Alert();
                            goto break_outer;
                        }
                    }
                }
            }
            break_outer:

            /* 00:00 重置当天触发标记 */
            if(h_now == 0 && m_now == 0 && s_now < 5)
            {
                for(uint8_t i = 0; i < MED_COUNT; i++)
                    for(uint8_t j = 0; j < MAX_SLOTS; j++)
                        triggered[i][j] = 0;
            }

            /* 屏幕刷新 */
            if(refresh_flag)
            {
                Show_Run();       /* OLED1: 药盒UI */
                Show_OLED2();     /* OLED2: 称重显示 */
                refresh_flag = 0;
            }
        }
        /*==========================*/
        /*  提醒模式                */
        /*==========================*/
        else if(sys_state == STATE_ALERT)
        {
            /* 闪烁控制: 500ms ON / 500ms OFF */
            blink_tick++;
            if(blink_tick < 5)       /* ~500ms ON */
            {
                LED_ON();
                BEEP_ON();
            }
            else if(blink_tick < 10) /* ~500ms OFF */
            {
                LED_OFF();
                BEEP_OFF();
            }
            else
            {
                blink_tick = 0;
            }

            /* K3: 手动解除 */
            if(Key_OK_Scan())
            {
                LED_OFF();
                BEEP_OFF();
                alert_flag[alert_med] = 0;
                last_min  = 0xFF;  /* 强制重新检查: 同分钟可能还有别的闹钟 */
                entry_sec = 0;     /* 移除10秒保护(triggered标记已防重触发) */
                sys_state = STATE_RUN;
                Show_Run();        /* OLED1 */
                Show_OLED2();      /* OLED2 */
                continue;
            }

            /* 取药检测: 重量下降 > TAKE_WEIGHT + 红外遮挡(双重判定) */
            {
                float now_w = Get_Weight(alert_med);
                uint8_t ir_blocked = IR_BLOCKED();

                if((origin_w[alert_med] - now_w >= TAKE_WEIGHT) && ir_blocked)
                {
                    /* 已取药 → 播报粒数 → 解除告警 */
                    Voice_Play_Pill(med[alert_med].pills_per_dose);

                    LED_OFF();
                    BEEP_OFF();
                    alert_flag[alert_med] = 0;
                    last_min  = 0xFF;  /* 强制重新检查同分钟其他闹钟 */
                    entry_sec = 0;
                    sys_state = STATE_RUN;
                    Show_Run();        /* OLED1 */
                    Show_OLED2();      /* OLED2 */
                    continue;
                }
            }

            /* 刷新显示屏 */
            if((blink_tick % 10) == 0)
            {
                Show_Alert();    /* OLED1: 告警信息 */
                Show_OLED2();    /* OLED2: 称重显示 */
            }
        }

        Delay_ms(100); /* 主循环节拍 ~100ms */
    }
}
