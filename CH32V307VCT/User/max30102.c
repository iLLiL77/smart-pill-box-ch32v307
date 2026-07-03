/**********************************************
 * MAX30102 心率血氧传感器驱动
 *
 * 软件I2C: PE12(SCL) + PE13(SDA), ~400kHz
 * INT引脚: PB1 (上拉输入, FIFO数据就绪=低电平)
 * 采样:     100Hz, IR + Red 双通道, 18-bit ADC
 * 算法:     Maxim Integrated AN6409 官方算法
 *           汉明窗滤波 + 峰值检测 + SpO2查表
 *
 * 数据流:
 *   用FIFO指针(WP-RP)计算可用样本数, 每次Update读光FIFO
 *   初始化 → 采集500样本(5秒 @100Hz) → Maxim算法计算HR+SpO2
 *   主循环 → 滑动窗口(丢弃最旧100, 采集100新样本) → 重新计算
 *
 * 移植自:  STM32F103C8T6 例程 (云帆电子 www.yfcdz.cn)
 * 适配:    CH32V307VCT (RISC-V) + WCH StdPeriph
 **********************************************/

#include "max30102.h"

/*==========================================================================
 * 全局变量
 *==========================================================================*/
MAX_Result  max_result = {0, 0, 0, 0, 0, 0.0f};
uint8_t     max30102_ok = 0;
uint8_t     max30102_enabled = 1;   /* 默认开启 */

/*==========================================================================
 * 内部状态
 *==========================================================================*/
static uint32_t aun_ir_buffer[BUFFER_SIZE];   /* IR 原始数据缓冲 */
static uint32_t aun_red_buffer[BUFFER_SIZE];  /* Red 原始数据缓冲 */
static int32_t  n_buf_index   = 0;            /* 当前缓冲写入位置 */
static uint8_t  n_buf_ready   = 0;            /* 缓冲已填满500? 0=采集阶段 1=计算阶段 */

/* 复用的算法全局缓冲 (Maxim算法内部使用) */
static int32_t  an_x[BUFFER_SIZE];
static int32_t  an_y[BUFFER_SIZE];
static int32_t  an_dx[BUFFER_SIZE - MA4_SIZE];

/*==========================================================================
 * 软件I2C — PE12(SCL推挽) + PE13(SDA开漏)
 *==========================================================================*/
#define MAX_I2C_DELAY  Delay_Us(2)

#define MAX_SDA_H()   GPIO_SetBits(MAX_PORT, MAX_SDA)
#define MAX_SDA_L()   GPIO_ResetBits(MAX_PORT, MAX_SDA)
#define MAX_SCL_H()   GPIO_SetBits(MAX_PORT, MAX_SCL)
#define MAX_SCL_L()   GPIO_ResetBits(MAX_PORT, MAX_SCL)
#define MAX_READ_SDA  GPIO_ReadInputDataBit(MAX_PORT, MAX_SDA)

#define MAX_INT_READ  GPIO_ReadInputDataBit(MAX_INT_PORT, MAX_INT_PIN)

/*==========================================================================
 * SDA方向切换
 *==========================================================================*/
void MAX30102_IIC_SDA_OUT(void)
{
    GPIO_InitTypeDef cfg;
    cfg.GPIO_Pin   = MAX_SDA;
    cfg.GPIO_Speed = GPIO_Speed_50MHz;
    cfg.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(MAX_PORT, &cfg);
}

void MAX30102_IIC_SDA_IN(void)
{
    GPIO_InitTypeDef cfg;
    cfg.GPIO_Pin   = MAX_SDA;
    cfg.GPIO_Speed = GPIO_Speed_50MHz;
    cfg.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(MAX_PORT, &cfg);
}

/*==========================================================================
 * I2C 初始化
 *==========================================================================*/
void MAX30102_IIC_Init(void)
{
    GPIO_InitTypeDef cfg;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    cfg.GPIO_Pin   = MAX_SCL | MAX_SDA;
    cfg.GPIO_Mode  = GPIO_Mode_Out_PP;
    cfg.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MAX_PORT, &cfg);
    MAX_SCL_H();
    MAX_SDA_H();
}

/*==========================================================================
 * I2C 基本操作
 *==========================================================================*/
void MAX30102_IIC_Start(void)
{
    MAX30102_IIC_SDA_OUT();
    MAX_SDA_H(); MAX_SCL_H(); Delay_Us(4);
    MAX_SDA_L(); Delay_Us(4); MAX_SCL_L();
}

void MAX30102_IIC_Stop(void)
{
    MAX30102_IIC_SDA_OUT();
    MAX_SCL_L(); MAX_SDA_L(); Delay_Us(4);
    MAX_SCL_H(); MAX_SDA_H(); Delay_Us(4);
}

uint8_t MAX30102_IIC_Wait_Ack(void)
{
    uint8_t t = 0;
    MAX30102_IIC_SDA_IN();
    MAX_SDA_H(); Delay_Us(1);
    MAX_SCL_H(); Delay_Us(1);
    while (MAX_READ_SDA) {
        if (++t > 250) { MAX30102_IIC_Stop(); return 1; }
    }
    MAX_SCL_L();
    return 0;
}

void MAX30102_IIC_Ack(void)
{
    MAX_SCL_L();
    MAX30102_IIC_SDA_OUT();
    MAX_SDA_L(); Delay_Us(2);
    MAX_SCL_H(); Delay_Us(2);
    MAX_SCL_L();
}

void MAX30102_IIC_NAck(void)
{
    MAX_SCL_L();
    MAX30102_IIC_SDA_OUT();
    MAX_SDA_H(); Delay_Us(2);
    MAX_SCL_H(); Delay_Us(2);
    MAX_SCL_L();
}

void MAX30102_IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
    MAX30102_IIC_SDA_OUT();
    MAX_SCL_L();
    for (t = 0; t < 8; t++) {
        if (txd & 0x80) MAX_SDA_H(); else MAX_SDA_L();
        txd <<= 1;
        Delay_Us(2); MAX_SCL_H(); Delay_Us(2); MAX_SCL_L(); Delay_Us(2);
    }
}

uint8_t MAX30102_IIC_Read_Byte(unsigned char ack)
{
    unsigned char i, recv = 0;
    MAX30102_IIC_SDA_IN();
    for (i = 0; i < 8; i++) {
        MAX_SCL_L(); Delay_Us(2);
        MAX_SCL_H();
        recv <<= 1;
        if (MAX_READ_SDA) recv++;
        Delay_Us(1);
    }
    if (!ack) MAX30102_IIC_NAck(); else MAX30102_IIC_Ack();
    return recv;
}

/*==========================================================================
 * 批量 I2C 读写 — 注意: ReadBytes末尾有Delay_Ms(10), 批量采FIFO时别用它
 *==========================================================================*/
void MAX30102_IIC_WriteBytes(uint8_t WriteAddr, uint8_t *data, uint8_t dataLength)
{
    uint8_t i;
    MAX30102_IIC_Start();
    MAX30102_IIC_Send_Byte(WriteAddr); MAX30102_IIC_Wait_Ack();
    for (i = 0; i < dataLength; i++) {
        MAX30102_IIC_Send_Byte(data[i]); MAX30102_IIC_Wait_Ack();
    }
    MAX30102_IIC_Stop();
    Delay_Ms(10);
}

void MAX30102_IIC_ReadBytes(uint8_t deviceAddr, uint8_t writeAddr, uint8_t *data, uint8_t dataLength)
{
    uint8_t i;
    MAX30102_IIC_Start();
    MAX30102_IIC_Send_Byte(deviceAddr); MAX30102_IIC_Wait_Ack();
    MAX30102_IIC_Send_Byte(writeAddr);  MAX30102_IIC_Wait_Ack();
    MAX30102_IIC_Send_Byte(deviceAddr | 0x01); MAX30102_IIC_Wait_Ack();
    for (i = 0; i < dataLength - 1; i++) data[i] = MAX30102_IIC_Read_Byte(1);
    data[dataLength - 1] = MAX30102_IIC_Read_Byte(0);
    MAX30102_IIC_Stop();
    Delay_Ms(10);
}

void MAX30102_IIC_Read_One_Byte(uint8_t daddr, uint8_t addr, uint8_t *data)
{
    MAX30102_IIC_Start();
    MAX30102_IIC_Send_Byte(daddr); MAX30102_IIC_Wait_Ack();
    MAX30102_IIC_Send_Byte(addr);  MAX30102_IIC_Wait_Ack();
    MAX30102_IIC_Start();
    MAX30102_IIC_Send_Byte(daddr | 0x01); MAX30102_IIC_Wait_Ack();
    *data = MAX30102_IIC_Read_Byte(0);
    MAX30102_IIC_Stop();
}

void MAX30102_IIC_Write_One_Byte(uint8_t daddr, uint8_t addr, uint8_t data)
{
    MAX30102_IIC_Start();
    MAX30102_IIC_Send_Byte(daddr); MAX30102_IIC_Wait_Ack();
    MAX30102_IIC_Send_Byte(addr);  MAX30102_IIC_Wait_Ack();
    MAX30102_IIC_Send_Byte(data);  MAX30102_IIC_Wait_Ack();
    MAX30102_IIC_Stop();
    Delay_Ms(10);
}

/*==========================================================================
 * 寄存器级操作 (带ACK检测)
 *==========================================================================*/
uint8_t max30102_Bus_Write(uint8_t reg, uint8_t val)
{
    MAX30102_IIC_Start();
    MAX30102_IIC_Send_Byte(max30102_WR_address | I2C_WR);
    if (MAX30102_IIC_Wait_Ack()) goto fail;
    MAX30102_IIC_Send_Byte(reg);
    if (MAX30102_IIC_Wait_Ack()) goto fail;
    MAX30102_IIC_Send_Byte(val);
    if (MAX30102_IIC_Wait_Ack()) goto fail;
    MAX30102_IIC_Stop();
    return 1;
fail:
    MAX30102_IIC_Stop();
    return 0;
}

uint8_t max30102_Bus_Read(uint8_t reg)
{
    uint8_t data;
    MAX30102_IIC_Start();
    MAX30102_IIC_Send_Byte(max30102_WR_address | I2C_WR);
    if (MAX30102_IIC_Wait_Ack()) goto fail;
    MAX30102_IIC_Send_Byte(reg);
    if (MAX30102_IIC_Wait_Ack()) goto fail;
    MAX30102_IIC_Start();
    MAX30102_IIC_Send_Byte(max30102_WR_address | I2C_RD);
    if (MAX30102_IIC_Wait_Ack()) goto fail;
    data = MAX30102_IIC_Read_Byte(0);
    MAX30102_IIC_NAck();
    MAX30102_IIC_Stop();
    return data;
fail:
    MAX30102_IIC_Stop();
    return 0;
}

/*==========================================================================
 * FIFO 读取6字节 (1个样本: 3字节IR + 3字节Red, 各18-bit)
 * 注意: 内部使用, 不在批量采集中使用
 *==========================================================================*/
void max30102_FIFO_ReadBytes(uint8_t reg, uint8_t *Data)
{
    max30102_Bus_Read(REG_INTR_STATUS_1);
    max30102_Bus_Read(REG_INTR_STATUS_2);
    MAX30102_IIC_Start();
    MAX30102_IIC_Send_Byte(max30102_WR_address | I2C_WR);
    if (MAX30102_IIC_Wait_Ack()) goto fail;
    MAX30102_IIC_Send_Byte(reg);
    if (MAX30102_IIC_Wait_Ack()) goto fail;
    MAX30102_IIC_Start();
    MAX30102_IIC_Send_Byte(max30102_WR_address | I2C_RD);
    if (MAX30102_IIC_Wait_Ack()) goto fail;
    Data[0] = MAX30102_IIC_Read_Byte(1);
    Data[1] = MAX30102_IIC_Read_Byte(1);
    Data[2] = MAX30102_IIC_Read_Byte(1);
    Data[3] = MAX30102_IIC_Read_Byte(1);
    Data[4] = MAX30102_IIC_Read_Byte(1);
    Data[5] = MAX30102_IIC_Read_Byte(0);
    MAX30102_IIC_Stop();
    return;
fail:
    MAX30102_IIC_Stop();
}

/*==========================================================================
 * Maxim 风格包装函数 (算法兼容)
 *==========================================================================*/
void maxim_max30102_write_reg(uint8_t uch_addr, uint8_t uch_data)
{
    MAX30102_IIC_Write_One_Byte(max30102_WR_address, uch_addr, uch_data);
}

void maxim_max30102_read_reg(uint8_t uch_addr, uint8_t *puch_data)
{
    MAX30102_IIC_Read_One_Byte(max30102_WR_address, uch_addr, puch_data);
}

void maxim_max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)
{
    uint32_t un_temp;
    unsigned char uch_temp;
    char ach_i2c_data[6];

    *pun_red_led = 0;
    *pun_ir_led  = 0;

    maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_temp);
    maxim_max30102_read_reg(REG_INTR_STATUS_2, &uch_temp);
    MAX30102_IIC_ReadBytes(max30102_WR_address, REG_FIFO_DATA, (uint8_t *)ach_i2c_data, 6);

    un_temp = (unsigned char)ach_i2c_data[0]; un_temp <<= 16; *pun_red_led += un_temp;
    un_temp = (unsigned char)ach_i2c_data[1]; un_temp <<= 8;  *pun_red_led += un_temp;
    un_temp = (unsigned char)ach_i2c_data[2];                  *pun_red_led += un_temp;
    un_temp = (unsigned char)ach_i2c_data[3]; un_temp <<= 16; *pun_ir_led += un_temp;
    un_temp = (unsigned char)ach_i2c_data[4]; un_temp <<= 8;  *pun_ir_led += un_temp;
    un_temp = (unsigned char)ach_i2c_data[5];                  *pun_ir_led += un_temp;

    *pun_red_led &= 0x03FFFF;
    *pun_ir_led  &= 0x03FFFF;
}

/*==========================================================================
 * 传感器复位 — 写入复位位 + 轮询等待复位完成
 *==========================================================================*/
void MAX30102_Reset(void)
{
    uint8_t val;
    uint16_t timeout;

    max30102_Bus_Write(REG_MODE_CONFIG, 0x40);   /* 设置 RESET 位 */

    /* 等待复位完成 (RESET bit self-clears, 典型~100μs) */
    timeout = 0;
    do {
        Delay_Us(50);
        val = max30102_Bus_Read(REG_MODE_CONFIG);
        timeout++;
    } while ((val & 0x40) && timeout < 1000);    /* 最多等50ms */

    Delay_Ms(1);  /* 额外等待传感器稳定 */
}

/*==========================================================================
 * 传感器初始化 (带重试)
 *==========================================================================*/
void MAX30102_Init(void)
{
    uint8_t id, retry;
    uint8_t ok;

    MAX30102_IIC_Init();

    /* 检查器件是否存在 (最多重试3次) */
    for (retry = 0; retry < 3; retry++)
    {
        MAX30102_Reset();
        Delay_Ms(5);

        id = max30102_Bus_Read(REG_PART_ID);
        if (id == 0x15) break;
        Delay_Ms(10);
    }

    if (id != 0x15) {
        max30102_ok = 0;
        return;
    }

    /* 寄存器配置 — 逐个写入并检查ACK */
    #define WR_REG(r, v) do { \
        ok = max30102_Bus_Write((r), (v)); \
        if (!ok) { max30102_ok = 0; return; } \
    } while(0)

    WR_REG(REG_INTR_ENABLE_1, 0xc0);   /* FIFO almost full + new FIFO data ready */
    WR_REG(REG_INTR_ENABLE_2, 0x00);   /* 禁用内部测温就绪中断 */
    WR_REG(REG_FIFO_WR_PTR,   0x00);   /* 清零写指针 */
    WR_REG(REG_OVF_COUNTER,   0x00);   /* 清零溢出计数 */
    WR_REG(REG_FIFO_RD_PTR,   0x00);   /* 清零读指针 */
    WR_REG(REG_FIFO_CONFIG,   0x0f);   /* avg=1, rollover=yes, A_FULL=15 */
    WR_REG(REG_MODE_CONFIG,   0x03);   /* SpO2 mode (Red+IR) */
    WR_REG(REG_SPO2_CONFIG,   0x27);   /* 4096nA, 100Hz, 400μs pulse, 18-bit */
    WR_REG(REG_LED1_PA,      0x24);   /* IR LED ~7mA */
    WR_REG(REG_LED2_PA,      0x24);   /* Red LED ~7mA */
    WR_REG(REG_PILOT_PA,     0x7f);   /* Pilot LED ~25mA */

    #undef WR_REG

    /* 预热: 传感器LED需要稳定时间 */
    Delay_Ms(100);

    n_buf_index = 0;
    n_buf_ready = 0;
    max30102_ok = 1;
}

/*==========================================================================
 * 开启/关闭 MAX30102
 *==========================================================================*/
void MAX30102_Enable(uint8_t on)
{
    if (!max30102_ok) return;
    if (on) {
        max30102_Bus_Write(REG_MODE_CONFIG, 0x03);
        n_buf_index = 0;
        n_buf_ready = 0;
        max30102_enabled = 1;
    } else {
        max30102_Bus_Write(REG_MODE_CONFIG, 0x00);
        max_result.hr    = 0;
        max_result.spo2  = 0;
        max_result.valid = 0;
        max30102_enabled = 0;
    }
}

uint8_t MAX30102_IsEnabled(void) { return max30102_enabled; }

/*==========================================================================
 * 从 FIFO 读取1个样本 (6字节) — 无延迟版本, 批量采集专用
 * 正确实现: START→写器件地址+W→写REG_FIFO_DATA→Restart→读器件地址+R→读6字节→STOP
 *==========================================================================*/
static uint8_t fifo_read_one(uint32_t *ir, uint32_t *red)
{
    uint8_t data[6], j;

    /* Step 1: START + 写器件地址 (W) */
    MAX30102_IIC_Start();
    MAX30102_IIC_Send_Byte(max30102_WR_address | I2C_WR);
    if (MAX30102_IIC_Wait_Ack()) { MAX30102_IIC_Stop(); return 0; }

    /* Step 2: 写 REG_FIFO_DATA (0x07) — 关键! 告诉MAX30102要读FIFO */
    MAX30102_IIC_Send_Byte(REG_FIFO_DATA);
    if (MAX30102_IIC_Wait_Ack()) { MAX30102_IIC_Stop(); return 0; }

    /* Step 3: Repeated START + 读器件地址 (R) */
    MAX30102_IIC_Start();
    MAX30102_IIC_Send_Byte(max30102_WR_address | I2C_RD);
    if (MAX30102_IIC_Wait_Ack()) { MAX30102_IIC_Stop(); return 0; }

    /* Step 4: 读6字节 (前5字节ACK, 末字节NACK) */
    for (j = 0; j < 6; j++) {
        data[j] = MAX30102_IIC_Read_Byte(j < 5 ? 1 : 0);
    }
    MAX30102_IIC_Stop();

    /* Step 5: 拼合18-bit原始值 */
    *ir  = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | (uint32_t)data[2];
    *ir &= 0x0003FFFF;
    *red = ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | (uint32_t)data[5];
    *red &= 0x0003FFFF;
    return 1;
}

/*==========================================================================
 * 主更新函数 — 从主循环调用 (~100ms间隔)
 *
 * 数据流:
 *   读FIFO指针 → 计算可用样本数 → 批量读光 → 写入环形缓冲
 *   采集满500样本 → 手指检测 → 调用Maxim算法 → 有效性过滤 → 滑动窗口
 *
 * 防抖策略:
 *   - IR DC均值 < FINGER_THRESHOLD → 不计算, 清零结果
 *   - 心率范围过滤: 30~220 BPM
 *   - 血氧范围过滤: 70~100%
 *   - 信号幅度检查: IR AC峰峰值 > MIN_AC_AMPLITUDE
 *   - 尖峰保护: 单样本 > 上次均值*3 则丢弃
 *==========================================================================*/
void MAX30102_Update(void)
{
    uint8_t  i, wp, rp, count;
    uint32_t ir_val, red_val;
    static uint32_t last_ir_mean = 50000;  /* 上次IR均值 (抗尖峰) */

    if (!max30102_ok || !max30102_enabled) return;

    /* 读 FIFO 指针, 计算可用样本数 */
    wp = max30102_Bus_Read(REG_FIFO_WR_PTR);
    rp = max30102_Bus_Read(REG_FIFO_RD_PTR);

    if (wp >= rp)
        count = wp - rp;
    else
        count = MAX_FIFO_DEPTH + wp - rp;

    if (count == 0) return;
    if (count > 32) count = 32;    /* 防止异常值 */

    /* 清除中断状态 */
    max30102_Bus_Read(REG_INTR_STATUS_1);
    max30102_Bus_Read(REG_INTR_STATUS_2);

    /* 批量读取 FIFO 样本 */
    for (i = 0; i < count; i++)
    {
        if (!fifo_read_one(&ir_val, &red_val)) break;

        /* 尖峰保护: 如果单样本是上次均值的3倍以上, 丢弃 */
        if (last_ir_mean > 0 && ir_val > last_ir_mean * 3) {
            continue;
        }

        aun_ir_buffer[n_buf_index]  = ir_val;
        aun_red_buffer[n_buf_index] = red_val;
        n_buf_index++;

        if (n_buf_index >= BUFFER_SIZE) {
            n_buf_index = BUFFER_SIZE - 1;
            n_buf_ready = 1;
        }
    }

    /* FIFO溢出检测 */
    {
        uint8_t ovf = max30102_Bus_Read(REG_OVF_COUNTER);
        if (ovf > 0) {
            /* FIFO溢出了 → 丢弃当前缓冲, 从头采集 */
            n_buf_index = 0;
            n_buf_ready = 0;
            max30102_Bus_Write(REG_OVF_COUNTER, 0x00);
        }
    }

    /*---------------- 计算阶段 ----------------*/
    if (n_buf_ready)
    {
        int32_t  n_spo2, n_heart_rate;
        int8_t   ch_spo2_valid, ch_hr_valid;
        uint32_t ir_dc_sum;
        uint8_t  finger_detected;

        /* 手指检测: IR DC均值 */
        ir_dc_sum = 0;
        for (i = 0; i < BUFFER_SIZE; i++)
            ir_dc_sum += aun_ir_buffer[i];
        ir_dc_sum /= BUFFER_SIZE;
        last_ir_mean = ir_dc_sum;

        finger_detected = (ir_dc_sum > FINGER_THRESHOLD) ? 1 : 0;
        max_result.finger_on = finger_detected;

        if (!finger_detected)
        {
            /* 无手指 → 清零, 但仍维持滑动窗口 */
            max_result.hr    = 0;
            max_result.spo2  = 0;
            max_result.valid = 0;
            max_result.signal_quality = 0;
        }
        else
        {
            /* 信号质量: 基于IR AC幅度 (峰峰值) */
            {
                uint32_t ir_min = 0x3FFFF, ir_max = 0;
                for (i = 0; i < BUFFER_SIZE; i++) {
                    if (aun_ir_buffer[i] < ir_min) ir_min = aun_ir_buffer[i];
                    if (aun_ir_buffer[i] > ir_max) ir_max = aun_ir_buffer[i];
                }
                {
                    uint32_t ac_amp = ir_max - ir_min;
                    if (ac_amp > 50000)      max_result.signal_quality = 100;
                    else if (ac_amp > 20000) max_result.signal_quality = 80;
                    else if (ac_amp > 10000) max_result.signal_quality = 60;
                    else if (ac_amp > 5000)  max_result.signal_quality = 40;
                    else if (ac_amp > 2000)  max_result.signal_quality = 20;
                    else                     max_result.signal_quality = 0;
                }
            }

            /* 调用 Maxim 官方算法 */
            maxim_heart_rate_and_oxygen_saturation(
                aun_ir_buffer, BUFFER_SIZE,
                aun_red_buffer,
                &n_spo2, &ch_spo2_valid,
                &n_heart_rate, &ch_hr_valid);

            /* 有效性过滤 */
            max_result.valid = 0;

            if (ch_hr_valid && n_heart_rate > 30 && n_heart_rate < 220) {
                max_result.hr = (int16_t)n_heart_rate;
                max_result.valid = 1;
            } else {
                max_result.hr = 0;
            }

            if (ch_spo2_valid && n_spo2 >= 70 && n_spo2 <= 100) {
                max_result.spo2 = (uint8_t)n_spo2;
            } else {
                max_result.spo2 = 0;
                /* 心率有效但血氧无效 → 仍标记有效(心率) */
            }
        }

        /* 滑动窗口: 丢弃最旧100样本, [100..499] → [0..399] */
        for (i = 100; i < BUFFER_SIZE; i++) {
            aun_ir_buffer[i - 100]  = aun_ir_buffer[i];
            aun_red_buffer[i - 100] = aun_red_buffer[i];
        }
        n_buf_index = BUFFER_SIZE - 100;
        n_buf_ready = 0;
    }
}

/*==========================================================================
 * 对外查询接口
 *==========================================================================*/
int16_t  MAX30102_GetHR(void)    { return (max30102_ok && max30102_enabled) ? max_result.hr   : 0; }
uint8_t  MAX30102_GetSpO2(void)  { return (max30102_ok && max30102_enabled) ? max_result.spo2 : 0; }
uint8_t  MAX30102_IsValid(void)  { return (max30102_ok && max30102_enabled) ? max_result.valid : 0; }
uint8_t  MAX30102_FingerOn(void) { return (max30102_ok && max30102_enabled) ? max_result.finger_on : 0; }
uint8_t  MAX30102_GetSignalQuality(void) { return (max30102_ok && max30102_enabled) ? max_result.signal_quality : 0; }
MAX_Result MAX30102_GetResult(void)      { return max_result; }

/*==========================================================================
 * 器件存在检测 — 读 PART_ID 应为 0x15
 *==========================================================================*/
uint8_t MAX30102_IsPresent(void)
{
    uint8_t id;
    MAX30102_IIC_Init();
    MAX30102_Reset();
    Delay_Ms(5);
    id = max30102_Bus_Read(REG_PART_ID);
    return (id == 0x15) ? 1 : 0;
}

/*==========================================================================
 * 读取温度 — 用于传感器内部温度补偿
 *==========================================================================*/
float MAX30102_ReadTemperature(void)
{
    uint8_t temp_int, temp_frac;
    float temperature;

    if (!max30102_ok) return 0.0f;

    /* 启动温度测量 */
    max30102_Bus_Write(REG_TEMP_CONFIG, 0x01);

    /* 等待测量完成 (典型~30ms) */
    Delay_Ms(30);

    temp_int  = max30102_Bus_Read(REG_TEMP_INTR);
    temp_frac = max30102_Bus_Read(REG_TEMP_FRAC);

    temperature = (float)temp_int + (float)temp_frac * 0.0625f;

    /* 恢复 SpO2 模式 */
    max30102_Bus_Write(REG_MODE_CONFIG, 0x03);

    max_result.temperature = temperature;
    return temperature;
}

/*==========================================================================
 * ============  Maxim Integrated AN6409 官方算法 ============
 *==========================================================================*/

const uint16_t auw_hamm[5] = { 41, 276, 512, 276, 41 };

const uint8_t uch_spo2_table[184] = {
     95,  95,  95,  96,  96,  96,  97,  97,  97,  97,
     97,  98,  98,  98,  98,  98,  99,  99,  99,  99,
     99,  99,  99,  99, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100,  99,  99,  99,  99,  99,  99,
     99,  99,  98,  98,  98,  98,  98,  98,  97,  97,
     97,  97,  96,  96,  96,  96,  95,  95,  95,  94,
     94,  94,  93,  93,  93,  92,  92,  92,  91,  91,
     90,  90,  89,  89,  89,  88,  88,  87,  87,  86,
     86,  85,  85,  84,  84,  83,  82,  82,  81,  81,
     80,  80,  79,  78,  78,  77,  76,  76,  75,  74,
     74,  73,  72,  72,  71,  70,  69,  69,  68,  67,
     66,  66,  65,  64,  63,  62,  62,  61,  60,  59,
     58,  57,  56,  56,  55,  54,  53,  52,  51,  50,
     49,  48,  47,  46,  45,  44,  43,  42,  41,  40,
     39,  38,  37,  36,  35,  34,  33,  31,  30,  29,
     28,  27,  26,  25,  23,  22,  21,  20,  19,  17,
     16,  15,  14,  12,  11,  10,   9,   7,   6,   5,
      3,   2,   1
};

/*==========================================================================
 * 心率+血氧核心算法 — Maxim AN6409
 *==========================================================================*/
void maxim_heart_rate_and_oxygen_saturation(
    uint32_t *pun_ir_buffer,  int32_t n_ir_buffer_length,
    uint32_t *pun_red_buffer,
    int32_t *pn_spo2, int8_t *pch_spo2_valid,
    int32_t *pn_heart_rate, int8_t *pch_hr_valid)
{
    uint32_t un_ir_mean, un_only_once;
    int32_t k, n_i_ratio_count;
    int32_t i, s, m, n_exact_ir_valley_locs_count, n_middle_idx;
    int32_t n_th1, n_npks, n_c_min;
    int32_t an_ir_valley_locs[15];
    int32_t an_exact_ir_valley_locs[15];
    int32_t an_dx_peak_locs[15];
    int32_t n_peak_interval_sum;

    int32_t n_y_ac, n_x_ac;
    int32_t n_spo2_calc;
    int32_t n_y_dc_max, n_x_dc_max;
    int32_t n_y_dc_max_idx = 0, n_x_dc_max_idx = 0;
    int32_t an_ratio[5], n_ratio_average;
    int32_t n_nume, n_denom;

    /* 1. 去除IR直流分量 */
    un_ir_mean = 0;
    for (k = 0; k < n_ir_buffer_length; k++)
        un_ir_mean += pun_ir_buffer[k];
    un_ir_mean = un_ir_mean / n_ir_buffer_length;
    for (k = 0; k < n_ir_buffer_length; k++)
        an_x[k] = pun_ir_buffer[k] - un_ir_mean;

    /* 2. 4点移动平均 */
    for (k = 0; k < BUFFER_SIZE - MA4_SIZE; k++) {
        n_denom = an_x[k] + an_x[k+1] + an_x[k+2] + an_x[k+3];
        an_x[k] = n_denom / 4;
    }

    /* 3. 差分信号 */
    for (k = 0; k < BUFFER_SIZE - MA4_SIZE - 1; k++)
        an_dx[k] = an_x[k+1] - an_x[k];

    /* 4. 2点移动平均 */
    for (k = 0; k < BUFFER_SIZE - MA4_SIZE - 2; k++)
        an_dx[k] = (an_dx[k] + an_dx[k+1]) / 2;

    /* 5. 汉明窗滤波 (波形翻转, 检测谷值) */
    for (i = 0; i < BUFFER_SIZE - HAMMING_SIZE - MA4_SIZE - 2; i++) {
        s = 0;
        for (k = i; k < i + HAMMING_SIZE; k++)
            s -= an_dx[k] * (int32_t)auw_hamm[k - i];
        an_dx[i] = s / 1146;
    }

    /* 6. 阈值 */
    n_th1 = 0;
    for (k = 0; k < BUFFER_SIZE - HAMMING_SIZE; k++)
        n_th1 += (an_dx[k] > 0) ? an_dx[k] : -an_dx[k];
    n_th1 = n_th1 / (BUFFER_SIZE - HAMMING_SIZE);

    /* 7. 峰值检测 → 心率 */
    maxim_find_peaks(an_dx_peak_locs, &n_npks,
        an_dx, BUFFER_SIZE - HAMMING_SIZE, n_th1, 8, 5);

    n_peak_interval_sum = 0;
    if (n_npks >= 2) {
        for (k = 1; k < n_npks; k++)
            n_peak_interval_sum += (an_dx_peak_locs[k] - an_dx_peak_locs[k - 1]);
        n_peak_interval_sum = n_peak_interval_sum / (n_npks - 1);
        *pn_heart_rate = 6000 / n_peak_interval_sum;
        *pch_hr_valid  = 1;
    } else {
        *pn_heart_rate = -999;
        *pch_hr_valid  = 0;
    }

    /* 8. 谷值位置 */
    for (k = 0; k < n_npks; k++)
        an_ir_valley_locs[k] = an_dx_peak_locs[k] + HAMMING_SIZE / 2;

    /* 9. 恢复原始值 */
    for (k = 0; k < n_ir_buffer_length; k++) {
        an_x[k] = pun_ir_buffer[k];
        an_y[k] = pun_red_buffer[k];
    }

    /* 10. 找精确谷值最小值 */
    n_exact_ir_valley_locs_count = 0;
    for (k = 0; k < n_npks; k++) {
        un_only_once = 1;
        m = an_ir_valley_locs[k];
        n_c_min = 16777216;
        if (m + 5 < BUFFER_SIZE - HAMMING_SIZE && m - 5 > 0) {
            for (i = m - 5; i < m + 5; i++) {
                if (an_x[i] < n_c_min) {
                    if (un_only_once > 0) un_only_once = 0;
                    n_c_min = an_x[i];
                    an_exact_ir_valley_locs[k] = i;
                }
            }
            if (un_only_once == 0) n_exact_ir_valley_locs_count++;
        }
    }
    if (n_exact_ir_valley_locs_count < 2) {
        *pn_spo2 = -999; *pch_spo2_valid = 0;
        return;
    }

    /* 11. 4点MA */
    for (k = 0; k < BUFFER_SIZE - MA4_SIZE; k++) {
        an_x[k] = (an_x[k] + an_x[k+1] + an_x[k+2] + an_x[k+3]) / 4;
        an_y[k] = (an_y[k] + an_y[k+1] + an_y[k+2] + an_y[k+3]) / 4;
    }

    /* 12. SpO2比值 */
    for (k = 0; k < n_exact_ir_valley_locs_count; k++) {
        if (an_exact_ir_valley_locs[k] > BUFFER_SIZE) {
            *pn_spo2 = -999; *pch_spo2_valid = 0; return;
        }
    }

    n_ratio_average = 0; n_i_ratio_count = 0;
    for (k = 0; k < 5; k++) an_ratio[k] = 0;

    for (k = 0; k < n_exact_ir_valley_locs_count - 1; k++) {
        n_y_dc_max = -16777216; n_x_dc_max = -16777216;
        if (an_exact_ir_valley_locs[k+1] - an_exact_ir_valley_locs[k] > 10) {
            for (i = an_exact_ir_valley_locs[k]; i < an_exact_ir_valley_locs[k+1]; i++) {
                if (an_x[i] > n_x_dc_max) { n_x_dc_max = an_x[i]; n_x_dc_max_idx = i; }
                if (an_y[i] > n_y_dc_max) { n_y_dc_max = an_y[i]; n_y_dc_max_idx = i; }
            }
            n_y_ac = (an_y[an_exact_ir_valley_locs[k+1]] - an_y[an_exact_ir_valley_locs[k]])
                   * (n_y_dc_max_idx - an_exact_ir_valley_locs[k]);
            n_y_ac = an_y[an_exact_ir_valley_locs[k]]
                   + n_y_ac / (an_exact_ir_valley_locs[k+1] - an_exact_ir_valley_locs[k]);
            n_y_ac = an_y[n_y_dc_max_idx] - n_y_ac;

            n_x_ac = (an_x[an_exact_ir_valley_locs[k+1]] - an_x[an_exact_ir_valley_locs[k]])
                   * (n_x_dc_max_idx - an_exact_ir_valley_locs[k]);
            n_x_ac = an_x[an_exact_ir_valley_locs[k]]
                   + n_x_ac / (an_exact_ir_valley_locs[k+1] - an_exact_ir_valley_locs[k]);
            n_x_ac = an_x[n_y_dc_max_idx] - n_x_ac;

            n_nume = (n_y_ac * n_x_dc_max) >> 7;
            n_denom = (n_x_ac * n_y_dc_max) >> 7;
            if (n_denom > 0 && n_i_ratio_count < 5 && n_nume != 0)
                an_ratio[n_i_ratio_count++] = (n_nume * 20) / n_denom;
        }
    }

    /* 13. 中位数 */
    maxim_sort_ascend(an_ratio, n_i_ratio_count);
    n_middle_idx = n_i_ratio_count / 2;
    if (n_middle_idx > 1)
        n_ratio_average = (an_ratio[n_middle_idx - 1] + an_ratio[n_middle_idx]) / 2;
    else
        n_ratio_average = an_ratio[n_middle_idx];

    /* 14. 查表 */
    if (n_ratio_average > 2 && n_ratio_average < 184) {
        *pn_spo2 = uch_spo2_table[n_ratio_average];
        *pch_spo2_valid = 1;
    } else {
        *pn_spo2 = -999; *pch_spo2_valid = 0;
    }
}

/*==========================================================================
 * 峰值检测器
 *==========================================================================*/
void maxim_find_peaks(int32_t *pn_locs, int32_t *pn_npks,
    int32_t *pn_x, int32_t n_size,
    int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num)
{
    maxim_peaks_above_min_height(pn_locs, pn_npks, pn_x, n_size, n_min_height);
    maxim_remove_close_peaks(pn_locs, pn_npks, pn_x, n_min_distance);
    if (*pn_npks > n_max_num) *pn_npks = n_max_num;
}

void maxim_peaks_above_min_height(int32_t *pn_locs, int32_t *pn_npks,
    int32_t *pn_x, int32_t n_size, int32_t n_min_height)
{
    int32_t i = 1, n_width;
    *pn_npks = 0;
    while (i < n_size - 1) {
        if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i - 1]) {
            n_width = 1;
            while (i + n_width < n_size && pn_x[i] == pn_x[i + n_width]) n_width++;
            if (pn_x[i] > pn_x[i + n_width] && (*pn_npks) < 15) {
                pn_locs[(*pn_npks)++] = i; i += n_width + 1;
            } else i += n_width;
        } else i++;
    }
}

void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks,
    int32_t *pn_x, int32_t n_min_distance)
{
    int32_t i, j, n_old_npks, n_dist;
    maxim_sort_indices_descend(pn_x, pn_locs, *pn_npks);
    for (i = -1; i < *pn_npks; i++) {
        n_old_npks = *pn_npks; *pn_npks = i + 1;
        for (j = i + 1; j < n_old_npks; j++) {
            n_dist = pn_locs[j] - (i == -1 ? -1 : pn_locs[i]);
            if (n_dist > n_min_distance || n_dist < -n_min_distance)
                pn_locs[(*pn_npks)++] = pn_locs[j];
        }
    }
    maxim_sort_ascend(pn_locs, *pn_npks);
}

void maxim_sort_ascend(int32_t *pn_x, int32_t n_size)
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++) {
        n_temp = pn_x[i];
        for (j = i; j > 0 && n_temp < pn_x[j - 1]; j--)
            pn_x[j] = pn_x[j - 1];
        pn_x[j] = n_temp;
    }
}

void maxim_sort_indices_descend(int32_t *pn_x, int32_t *pn_indx, int32_t n_size)
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++) {
        n_temp = pn_indx[i];
        for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j - 1]]; j--)
            pn_indx[j] = pn_indx[j - 1];
        pn_indx[j] = n_temp;
    }
}
