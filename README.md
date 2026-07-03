# 智能药盒 — Smart Pill Box

基于 **CH32V307VCT (RISC-V)** 的智能药盒系统，集成双称重传感器 + MAX30102 心率血氧检测。

## 功能特性

- **定时服药提醒** — 双药盒独立配置（粒数/次数/时段），到达预设时间 LED+蜂鸣器提醒
- **双重取药判定** — HX711 称重传感器 + 槽型红外对射传感器协同校验
- **MAX30102 心率血氧** — 软件I2C驱动 + Maxim Integrated AN6409 官方算法
- **双 OLED 显示** — OLED1(药盒UI) + OLED2(称重+心率血氧)
- **按键交互** — 4按键操作，支持短按/长按，设置药盒参数

## 硬件

| 模块 | 型号/引脚 |
|------|----------|
| 主控 | CH32V307VCT (RISC-V, 96MHz) |
| 心率血氧 | MAX30102 (PE12=SCL, PE13=SDA, PB1=INT) |
| 称重×2 | HX711 (PE8/PE9, PE10/PE11) |
| 显示屏×2 | SSD1306 OLED I2C (PA0/PA1, PA2/PA3) |
| 红外 | 槽型对射 (PA7) |
| 按键×4 | PA8/K1, PA9/K2, PA10/K3, PA6/K4 |
| LED | PA5 |
| 蜂鸣器 | PA4 |
| IDE | MounRiver Studio |

## 引脚分配

```
PA0 — OLED1-SCL      PA8 — Button1 (MODE)
PA1 — OLED1-SDA      PA9 — Button2 (ADD)
PA2 — OLED2-SCL      PA10 — Button3 (OK)
PA3 — OLED2-SDA      PA6 — Button4 (MAX30102开关)
PA4 — Buzzer
PA5 — LED            PE8 — HX711-A DT
PA7 — 红外对射        PE9 — HX711-A SCK
                      PE10 — HX711-B DT
PB1 — MAX30102-INT    PE11 — HX711-B SCK
                      PE12 — MAX30102-SCL
                      PE13 — MAX30102-SDA
```

## 项目结构

```
├── Core/           # RISC-V 内核支持
├── Debug/          # 调试串口 + Delay
├── Ld/             # 链接脚本
├── Peripheral/     # CH32V30x 标准外设库
├── Startup/        # 启动文件
├── User/
│   ├── main.c          # 主程序 (三态系统)
│   ├── max30102.c/h    # MAX30102 驱动 + Maxim算法
│   ├── hx711.c/h       # HX711 称重驱动
│   ├── oled.c/h        # SSD1306 OLED 驱动
│   ├── key.c/h         # 按键扫描
│   ├── gpio.c/h        # GPIO 初始化
│   ├── config.h        # 引脚+常量定义
│   ├── usart_voice.c/h # 语音模块
│   └── system_ch32v30x.c/h
└── doc/            # 芯片文档
```

## MAX30102 驱动说明

- **通信**: 软件 I2C (PE12=SCL, PE13=SDA), 地址 0x57 (写 0xAE / 读 0xAF)
- **采样**: 100Hz, IR + Red 双通道, 18-bit ADC
- **算法**: Maxim Integrated AN6409 — 汉明窗滤波 → 峰值检测 → 心率; AC/DC比 → SpO2 查表
- **防抖**: 手指检测(IR DC > 2000), 尖峰保护, 信号质量评估, FIFO 溢出检测

```c
// 使用示例
MAX30102_Init();                  // 初始化 (自动检测传感器)
while (1) {
    MAX30102_Update();            // ~100ms 调用一次
    if (MAX30102_IsValid()) {
        hr   = MAX30102_GetHR();     // 心率 BPM
        spo2 = MAX30102_GetSpO2();   // 血氧 %
    }
    Delay_Ms(100);
}
```

## 构建

1. 使用 MounRiver Studio 打开 `CH32V307VCT/` 目录
2. 编译 (Project → Build All)
3. 烧录 (WCH-Link 或 DAP-Link)

## 致谢

- MAX30102 例程参考: 云帆电子科技 (www.yfcdz.cn)
- Maxim Integrated AN6409 算法
- WCH CH32V30x 标准外设库
