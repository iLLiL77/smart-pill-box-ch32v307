/**********************************************
 * 语音模块 UART 驱动 — 待引脚分配后启用
 *
 * 当前状态: 桩函数 (编译通过, 功能空操作)
 * 接入步骤:
 *   1. 在 config.h 中定义 VOICE_TX / VOICE_RX 引脚
 *   2. 取消下面 USART1_Init 中的注释
 *   3. 取消 Voice_Play_Pill / Voice_Play_Alert 中的发送逻辑
 **********************************************/
#include "usart_voice.h"

/*==========================================================================
 * USART1 初始化 (连接语音模块, 默认9600bps)
 * 当前为桩 — 等待引脚分配
 *==========================================================================*/
void USART1_Init(uint32_t baud)
{
    /* ---- 待引脚确定后启用 ----
    GPIO_InitTypeDef  GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOx, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_InitStruct.GPIO_Pin   = VOICE_TX;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOx, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin   = VOICE_RX;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GPIOx, &GPIO_InitStruct);

    USART_InitStruct.USART_BaudRate            = baud;
    USART_InitStruct.USART_WordLength          = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits            = USART_StopBits_1;
    USART_InitStruct.USART_Parity              = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStruct);
    USART_Cmd(USART1, ENABLE);
    ---- */
    (void)baud;  /* 消除未使用警告 */
}

/*==========================================================================
 * 发送一个字节到语音模块 (桩)
 *==========================================================================*/
void USART1_Send_Byte(uint8_t dat)
{
    /* ---- 待引脚确定后启用 ----
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, dat);
    ---- */
    (void)dat;
}

/*==========================================================================
 * 播报"本次吃X粒"
 * 协议帧: 0xAA 0x07 0x01 <音轨号> <校验和>
 * 音轨001="请", 002="吃", 003="药", 004="一"~"九", 005="粒"
 *==========================================================================*/
void Voice_Play_Pill(uint8_t num)
{
    /* ---- 桩: 串口未初始化, 静默返回 ----
    uint8_t buf[5] = {0xAA, 0x07, 0x01, num, 0};
    if(num < 1)  num = 1;
    if(num > 9)  num = 9;
    buf[3] = num + 3;
    buf[4] = buf[0] + buf[1] + buf[2] + buf[3];
    for(uint8_t i = 0; i < 5; i++)
        USART1_Send_Byte(buf[i]);
    ---- */
    (void)num;
}

/*==========================================================================
 * 播报"请吃药"提醒音
 * 音轨000 = "请吃药"
 *==========================================================================*/
void Voice_Play_Alert(void)
{
    /* ---- 桩: 串口未初始化, 静默返回 ----
    uint8_t buf[5] = {0xAA, 0x07, 0x01, 0x00, 0};
    buf[4] = buf[0] + buf[1] + buf[2] + buf[3];
    for(uint8_t i = 0; i < 5; i++)
        USART1_Send_Byte(buf[i]);
    ---- */
}
