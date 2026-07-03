/**********************************************
 * MAX30102 心率血氧传感器驱动 — 头文件
 *
 * I2C 地址: 0x57 (写0xAE, 读0xAF)
 * 引脚:    PE12(SCL) PE13(SDA)  软件I2C
 * INT:     PB1 (上拉输入, FIFO数据就绪=低电平)
 *
 * 算法:    Maxim Integrated AN6409 官方算法
 *          汉明窗滤波 + 峰值检测 + SpO2查表
 *
 * 移植自:  STM32F103C8T6 例程 (云帆电子)
 **********************************************/
#ifndef __MAX30102_H
#define __MAX30102_H

#include "config.h"

/*==========================================================================
 * MAX30102 I2C 地址
 *==========================================================================*/
#define MAX30102_ADDR       0x57

#define max30102_WR_address  0xAE
#define I2C_WR               0
#define I2C_RD               1

/*==========================================================================
 * 寄存器地址
 *==========================================================================*/
#define REG_INTR_STATUS_1    0x00
#define REG_INTR_STATUS_2    0x01
#define REG_INTR_ENABLE_1    0x02
#define REG_INTR_ENABLE_2    0x03
#define REG_FIFO_WR_PTR      0x04
#define REG_OVF_COUNTER      0x05
#define REG_FIFO_RD_PTR      0x06
#define REG_FIFO_DATA        0x07
#define REG_FIFO_CONFIG      0x08
#define REG_MODE_CONFIG      0x09
#define REG_SPO2_CONFIG      0x0A
#define REG_LED1_PA          0x0C
#define REG_LED2_PA          0x0D
#define REG_PILOT_PA         0x10
#define REG_MULTI_LED_CTRL1  0x11
#define REG_MULTI_LED_CTRL2  0x12
#define REG_TEMP_INTR        0x1F
#define REG_TEMP_FRAC        0x20
#define REG_TEMP_CONFIG      0x21
#define REG_PROX_INT_THRESH  0x30
#define REG_REV_ID           0xFE
#define REG_PART_ID          0xFF

/*==========================================================================
 * 采样参数
 *==========================================================================*/
#define MAX_SAMPLE_RATE      100        /* 100Hz */
#define MAX_FIFO_DEPTH       32         /* FIFO深度 */
#define BUFFER_SIZE          500        /* 5秒 @100Hz */
#define MA4_SIZE             4          /* 4点移动平均窗宽 (不可改) */
#define HAMMING_SIZE         5          /* 汉明窗宽 (不可改) */
#define FINGER_THRESHOLD     2000       /* IR DC均值>2000 → 有手指 */

/*==========================================================================
 * 数据结构
 *==========================================================================*/
typedef struct {
    uint32_t ir;                        /* IR通道 18-bit原始值 */
    uint32_t red;                       /* Red通道 18-bit原始值 */
} MAX_Sample;

typedef struct {
    int16_t  hr;                        /* 心率 BPM */
    uint8_t  spo2;                      /* 血氧 % */
    uint8_t  valid;                     /* 心率+血氧均有效? 1=是 */
    uint8_t  finger_on;                 /* 手指检测 1=有手指 (保留) */
    uint8_t  signal_quality;            /* 信号质量 0~100 */
    float    temperature;               /* 温度 ℃ */
} MAX_Result;

/*==========================================================================
 * I2C 底层函数 (软件I2C, PE12/PE13)
 *==========================================================================*/
void MAX30102_IIC_Init(void);
void MAX30102_IIC_Start(void);
void MAX30102_IIC_Stop(void);
void MAX30102_IIC_Send_Byte(uint8_t txd);
uint8_t MAX30102_IIC_Read_Byte(unsigned char ack);
uint8_t MAX30102_IIC_Wait_Ack(void);
void MAX30102_IIC_Ack(void);
void MAX30102_IIC_NAck(void);
void MAX30102_IIC_SDA_OUT(void);
void MAX30102_IIC_SDA_IN(void);

void MAX30102_IIC_Write_One_Byte(uint8_t daddr, uint8_t addr, uint8_t data);
void MAX30102_IIC_Read_One_Byte(uint8_t daddr, uint8_t addr, uint8_t *data);
void MAX30102_IIC_WriteBytes(uint8_t WriteAddr, uint8_t *data, uint8_t dataLength);
void MAX30102_IIC_ReadBytes(uint8_t deviceAddr, uint8_t writeAddr, uint8_t *data, uint8_t dataLength);

/*==========================================================================
 * 寄存器级操作
 *==========================================================================*/
uint8_t max30102_Bus_Write(uint8_t Register_Address, uint8_t Word_Data);
uint8_t max30102_Bus_Read(uint8_t Register_Address);
void max30102_FIFO_ReadWords(uint8_t Register_Address, uint16_t Word_Data[][2], uint8_t count);
void max30102_FIFO_ReadBytes(uint8_t Register_Address, uint8_t *Data);

/*==========================================================================
 * 传感器配置
 *==========================================================================*/
void MAX30102_Reset(void);
void MAX30102_Init(void);

/*==========================================================================
 * 高层API
 *==========================================================================*/
uint8_t     MAX30102_IsPresent(void);       /* I2C器件检测: 返回1=OK */
void        MAX30102_Enable(uint8_t on);    /* 1=启动SpO2模式  0=关机(~0.7μA) */
uint8_t     MAX30102_IsEnabled(void);       /* 返回当前开关状态 */
void        MAX30102_Update(void);          /* 主循环每~100ms调用 */

int16_t     MAX30102_GetHR(void);           /* 心率 BPM, 0=无效 */
uint8_t     MAX30102_GetSpO2(void);         /* 血氧 %, 0=无效 */
uint8_t     MAX30102_IsValid(void);         /* 双方均有效? */
uint8_t     MAX30102_FingerOn(void);        /* 手指在传感器上? */
uint8_t     MAX30102_GetSignalQuality(void);/* 信号质量 0~100 */
float       MAX30102_ReadTemperature(void); /* 温度 ℃ */
MAX_Result  MAX30102_GetResult(void);       /* 取完整结果 */

/*==========================================================================
 * Maxim Integrated 官方心率+血氧算法 (AN6409)
 *==========================================================================*/
void maxim_heart_rate_and_oxygen_saturation(
    uint32_t *pun_ir_buffer,  int32_t n_ir_buffer_length,
    uint32_t *pun_red_buffer,
    int32_t *pn_spo2, int8_t *pch_spo2_valid,
    int32_t *pn_heart_rate, int8_t *pch_hr_valid);

void maxim_find_peaks(int32_t *pn_locs, int32_t *pn_npks,
    int32_t *pn_x, int32_t n_size,
    int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num);

void maxim_peaks_above_min_height(int32_t *pn_locs, int32_t *pn_npks,
    int32_t *pn_x, int32_t n_size, int32_t n_min_height);

void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks,
    int32_t *pn_x, int32_t n_min_distance);

void maxim_sort_ascend(int32_t *pn_x, int32_t n_size);
void maxim_sort_indices_descend(int32_t *pn_x, int32_t *pn_indx, int32_t n_size);

/*==========================================================================
 * 全局变量 (外部引用)
 *==========================================================================*/
extern MAX_Result  max_result;
extern uint8_t     max30102_ok;             /* 传感器初始化成功标志 */

#endif
