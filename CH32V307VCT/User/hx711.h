#ifndef __HX711_H
#define __HX711_H
#include "config.h"

/* 药盒索引: MED_A(0) 或 MED_B(1) */
uint32_t HX_Read(uint8_t med);
float    Get_Weight(uint8_t med);
void     HX_Tare(uint8_t med);
uint8_t  Check_Take(uint8_t med);
#endif
