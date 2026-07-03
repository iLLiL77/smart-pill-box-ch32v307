#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "OLED.h"
#include "string.h"
#include "max30102.h"

/*****************云帆单片机程序******************
											STM32
 * 项目		:	MAX30102心率血氧传感器实验 (修正版)
 * 版本		: V1.1
 * 日期		: 2024.8.18
 * MCU		:	STM32F103C8T6
 * 接口		:	见max30102.h
 * IP账号	:	云帆单片机编程 同BILIBILI|抖音|快手|小红书|CSDN|公众号|视频号等
 * 作者		:	云帆
 * 版权方	: 云帆电子科技官方
 * 教学视频	:	https://www.bilibili.com/video/BV1Ew4m1k7Nv/?share_source=copy_web
 * 官方网站	:	www.yfcdz.cn

**********************BEGIN***********************/

/* ==================================================================
 * 算法说明 (Maxim Integrated 官方算法 AN6409)
 *
 * [数据流]
 *   阶段0: 采集500个样本(5秒@100Hz) → 确定信号范围 → 首次计算
 *   阶段1: 丢弃前100个 → [100..499]移到[0..399] →
 *          采集100个新样本填入[400..499] → 手指检测 → 计算
 *
 * [心率计算]
 *   去DC → 4点MA → 一阶导数 → 2点MA → 汉明窗 →
 *   自适应阈值 → 峰值检测 → HR = 6000/平均峰间隔
 *
 * [血氧计算]
 *   波谷精确定位 → 4点MA → 逐段AC/DC比 →
 *   中位数滤波 → 预计算查表
 *
 * [防抖策略]
 *   - 手指检测: IR DC均值 > 2000 才计算
 *   - 心率范围: 30~250 BPM 有效
 *   - 血氧范围: 0~100% 有效
 *   - 峰值数<2 → 心率无效(-999)
 *   - 波谷数<2 → 血氧无效(-999)
 *   - R比值超出[2,184] → 血氧无效
 * ================================================================== */

/* 硬件连接:
	VCC <-> 3.3V
	GND <-> GND
	SCL <-> PB7
	SDA <-> PB8
	INT <-> PB9
*/

/* ---- 数据缓冲区 ---- */
uint32_t aun_ir_buffer[500];    // IR LED 传感器数据, 用于计算血氧
uint32_t aun_red_buffer[500];   // Red LED 传感器数据, 用于计算心率及血氧
int32_t  n_ir_buffer_length;    // 数据长度 = 500

/* ---- 计算结果 ---- */
int32_t  n_sp02;                // SpO2值
int8_t   ch_spo2_valid;         // SpO2有效性标志
int32_t  n_heart_rate;          // 心率值
int8_t   ch_hr_valid;           // 心率有效性标志

/* ---- 辅助变量 ---- */
uint32_t un_min, un_max;        // 信号范围(Red LED)
uint8_t  temp[6];               // FIFO读取临时缓冲
uint8_t  str[100];              // 显示字符串缓冲
uint8_t  dis_hr   = 0;          // 显示心率
uint8_t  dis_spo2 = 0;          // 显示血氧
uint8_t  finger_on = 0;         // 手指检测标志

/* ---- 手指检测阈值 ---- */
#define FINGER_THRESHOLD  2000  // IR DC均值 > 2000 → 有手指
#define HR_MIN            30    // 最低有效心率
#define HR_MAX            250   // 最高有效心率
#define SPO2_MIN          0     // 最低有效血氧
#define SPO2_MAX          100   // 最高有效血氧

int main(void)
{
	int i;

	/* ---- 初始化 ---- */
	delay_init(72);
	LED_Init();
	OLED_Init();
	delay_ms(50);
	OLED_Clear();

	MAX30102_Init();
	USART1_Config();

	un_min = 0x3FFFF;
	un_max = 0;

	/* ---- OLED显示框架 ---- */
	OLED_ShowChinese(0,  0, 0, 16, 1);   // 心
	OLED_ShowChinese(16, 0, 1, 16, 1);   // 率
	OLED_ShowChar   (40, 0, ':', 16, 1);
	OLED_ShowString (80, 0, "BPM", 16, 1);

	OLED_ShowChinese(0,  16, 2, 16, 1);  // 血
	OLED_ShowChinese(16, 16, 3, 16, 1);  // 氧
	OLED_ShowChar   (40, 16, ':', 16, 1);
	OLED_ShowChar   (80, 16, '%', 16, 1);

	n_ir_buffer_length = 500;  // 100Hz × 5秒

	/* ================================================================
	 * 阶段0: 初始填充500个样本 (~5秒), 确定信号动态范围
	 * ================================================================ */
	printf("MAX30102 初始化完成, 开始采集前500个样本...\r\n");

	for(i = 0; i < n_ir_buffer_length; i++)
	{
		while(MAX30102_INT == 1);   // 等待FIFO中断(INT变低表示数据就绪)

		max30102_FIFO_ReadBytes(REG_FIFO_DATA, temp);

		/* 拼合18-bit原始值 (每通道3字节, 高2位掩码) */
		aun_red_buffer[i] = ((long)(temp[0] & 0x03) << 16)
		                  | ((long)temp[1] << 8)
		                  |  (long)temp[2];
		aun_ir_buffer[i]  = ((long)(temp[3] & 0x03) << 16)
		                  | ((long)temp[4] << 8)
		                  |  (long)temp[5];

		/* 记录Red通道的min/max (用于信号范围评估) */
		if(un_min > aun_red_buffer[i])
			un_min = aun_red_buffer[i];
		if(un_max < aun_red_buffer[i])
			un_max = aun_red_buffer[i];
	}

	printf("初始采集完成. Red范围: %lu ~ %lu\r\n", un_min, un_max);

	/* ================================================================
	 * 阶段0: 首次计算 (用前500个样本)
	 * ================================================================ */
	maxim_heart_rate_and_oxygen_saturation(
		aun_ir_buffer,  n_ir_buffer_length,
		aun_red_buffer,
		&n_sp02,        &ch_spo2_valid,
		&n_heart_rate,  &ch_hr_valid);

	if(ch_hr_valid == 1 && n_heart_rate >= HR_MIN && n_heart_rate <= HR_MAX)
		dis_hr = (uint8_t)n_heart_rate;
	else
		dis_hr = 0;

	if(ch_spo2_valid == 1 && n_sp02 >= SPO2_MIN && n_sp02 <= SPO2_MAX)
		dis_spo2 = (uint8_t)n_sp02;
	else
		dis_spo2 = 0;

	printf("首次计算: HR=%d BPM, SpO2=%d%%\r\n", dis_hr, dis_spo2);

	/* ================================================================
	 * 阶段1: 滑动窗口循环
	 *
	 * 每轮流程:
	 *   ① 丢弃最老的100个样本 [0..99]
	 *   ② 将 [100..499] 移动到 [0..399]
	 *   ③ 采集100个新样本填入 [400..499]
	 *   ④ 手指检测 (IR DC均值 > FINGER_THRESHOLD)
	 *   ⑤ 调用Maxim算法计算心率+血氧
	 *   ⑥ 有效性校验 + 范围过滤 → 更新显示
	 * ================================================================ */
	while(1)
	{
		uint32_t ir_dc_sum;

		/* ---- ①+② 移窗: 丢弃前100个, [100..499] → [0..399] ---- */
		for(i = 100; i < 500; i++)
		{
			aun_red_buffer[i - 100] = aun_red_buffer[i];
			aun_ir_buffer[i - 100]  = aun_ir_buffer[i];
		}

		/* ---- ③ 采集100个新样本填入 [400..499] ---- */
		for(i = 400; i < 500; i++)
		{
			while(MAX30102_INT == 1);   // 等待FIFO数据就绪

			max30102_FIFO_ReadBytes(REG_FIFO_DATA, temp);

			aun_red_buffer[i] = ((long)(temp[0] & 0x03) << 16)
			                  | ((long)temp[1] << 8)
			                  |  (long)temp[2];
			aun_ir_buffer[i]  = ((long)(temp[3] & 0x03) << 16)
			                  | ((long)temp[4] << 8)
			                  |  (long)temp[5];
		}

		/* ---- ④ 手指检测: 计算IR通道DC均值 ---- */
		ir_dc_sum = 0;
		for(i = 0; i < 500; i++)
		{
			ir_dc_sum += aun_ir_buffer[i];
		}
		ir_dc_sum /= 500;

		if(ir_dc_sum > FINGER_THRESHOLD)
		{
			finger_on = 1;

			/* ---- ⑤ 调用官方Maxim算法 (AN6409) ---- */
			maxim_heart_rate_and_oxygen_saturation(
				aun_ir_buffer,  n_ir_buffer_length,
				aun_red_buffer,
				&n_sp02,        &ch_spo2_valid,
				&n_heart_rate,  &ch_hr_valid);

			/* ---- ⑥ 有效性校验 + 范围过滤 ---- */
			if(ch_hr_valid == 1 && n_heart_rate >= HR_MIN && n_heart_rate <= HR_MAX)
			{
				dis_hr = (uint8_t)n_heart_rate;
			}
			else
			{
				dis_hr = 0;  // 心率无效, 显示0
			}

			if(ch_spo2_valid == 1 && n_sp02 >= SPO2_MIN && n_sp02 <= SPO2_MAX)
			{
				dis_spo2 = (uint8_t)n_sp02;
			}
			else
			{
				dis_spo2 = 0;  // 血氧无效, 显示0
			}
		}
		else
		{
			/* 无手指 → 清零显示 */
			finger_on = 0;
			dis_hr    = 0;
			dis_spo2  = 0;
		}

		/* ---- 更新OLED显示 ---- */
		if(dis_hr == 0)
			OLED_ShowNum(47, 0, 0, 3, 16, 1);       // 显示"  0"
		else
			OLED_ShowNum(47, 0, dis_hr, 3, 16, 1);   // 显示实际心率(不再减20)

		if(dis_spo2 == 0)
			OLED_ShowNum(47, 16, 0, 2, 16, 1);       // 显示" 0"
		else
			OLED_ShowNum(47, 16, dis_spo2, 2, 16, 1); // 显示实际血氧

		/* ---- 串口输出 (调试用) ---- */
		if(finger_on)
			printf("HR=%d BPM  SpO2=%d%%  IR_DC=%lu\r\n", dis_hr, dis_spo2, ir_dc_sum);
		else
			printf("无手指 (IR_DC=%lu)\r\n", ir_dc_sum);

	} /* while(1) */
}
