# 硬件连接

以下引脚来自当前归档代码。实际接线可能因车模版本或转接板而不同，烧录前应以最终原理图和实物核对。

## 主控与姿态传感器

| 项目 | 配置 |
|---|---|
| 主控 | AI32G12K128 Series |
| 工作时钟 | 30 MHz |
| 姿态传感器 | IMU660RB |
| 姿态更新周期 | 10 ms |

IMU660RB 的 SPI/IIC 引脚由逐飞设备配置层选择，使用前应检查 `Libraries/zf_device/zf_device_config.h` 中的模块配置。

## 电磁传感器

| 逻辑名称 | 主控引脚 | ADC 通道 | 工程内数组下标 |
|---|---|---|---|
| INDUCTOR_1 | P0.0 | ADC_CH8 | `adc_raw[0]` |
| INDUCTOR_2 | P0.1 | ADC_CH9 | `adc_raw[1]` |
| INDUCTOR_3 | P0.5 | ADC_CH13 | `adc_raw[2]` |
| INDUCTOR_4 | P0.6 | ADC_CH14 | `adc_raw[3]` |

偏差计算按 `l, lb, rb, r` 的顺序传入：

```c
deviation(norm_val[2], norm_val[3], norm_val[0], norm_val[1]);
```

因此代码注释中的通道顺序可理解为 `rb, r, l, lb`。更换电感布局后必须重新标定 `adc_min` 和 `adc_max`，并核对左右方向。

## 电机与风扇

| 功能 | 引脚/外设 | 说明 |
|---|---|---|
| 左电机 PWM | PWMA_CH3P_P64 | 17 kHz |
| 左电机方向 | P6.6 | GPIO |
| 右电机 PWM | PWMA_CH1P_P60 | 17 kHz |
| 右电机方向 | P6.2 | GPIO |
| 吸附风扇 PWM | PWMB_CH3_P33 | 10 kHz |

电机控制函数按 10000 作为最大占空比，再按驱动库的 PWM 分辨率换算。

## 编码器

| 车轮 | 定时器 | 方向引脚 | 脉冲引脚 |
|---|---|---|---|
| 左轮 | TIM0 | P3.5 | P3.4 |
| 右轮 | TIM3 | P5.3 | P0.4 |

左轮读取值在代码中取反：

```c
encoder_data_dir_L = -encoder_get_count(ENCODER_DIR_1);
encoder_data_dir_R = encoder_get_count(ENCODER_DIR_2);
```

如果更换编码器接线，需相应调整正负号。

## 人机接口

| 功能 | 引脚 | 当前用途 |
|---|---|---|
| 启动按键 | P7.3 | 低电平有效，内部上拉 |
| LED1 | P5.2 | 推挽输出，主流程暂未使用 |
| 蜂鸣器 | P6.7 | 预留模块，主流程暂未使用 |
| 蓝牙 | BLE6A20 | 初始化和周期调试输出 |
