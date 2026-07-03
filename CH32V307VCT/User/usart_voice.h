#ifndef __USART_VOICE_H
#define __USART_VOICE_H

#include "config.h"

void USART1_Init(uint32_t baud);
void USART1_Send_Byte(uint8_t dat);
void Voice_Play_Pill(uint8_t num);        // 播报"本次吃X粒"
void Voice_Play_Alert(void);              // 播报"请吃药"提醒

#endif
