# iFOC 智能驱动系统

<center class ='img'>
<img title="整体架构图" src=".\docs\zh\structure_map_cn.png" width="100%">
</center>
### 使用说明



#### 附件和工具

- 改进版 DroneCAN GUI Tool 上位机：https://github.com/Matrixchung/dronecan_gui_tool
- 配套的 Bootloader 工程：https://github.com/Matrixchung/iBL
- ROS2 中间件（可独立运行）和配套的 Python 库：https://github.com/Matrixchung/ifoc_control

#### 已通过测试的平台

|    CPU 架构    |  MCU 型号 @ 设定主频   | PWM 频率 | 速度环频率 |    采样方式     |      成品平台       |      支持功能      |
| :------------: | :--------------------: | :------: | :--------: | :-------------: | :-----------------: | :----------------: |
| ARM Cortex-M4  | STM32G474RET6 @ 170MHz |  20KHz   |    5KHz    |  三相低侧采样   |     AxDr-L v1.3     |   有感/无感/开环   |
| ARM Cortex-M4  | AT32F403ACGT7 @ 240MHz |  20KHz   |    5KHz    |  两相低侧采样   |   iFOC-GIM6010-v4   |     有感/双编      |
| ARM Cortex-M4  | AT32F435CGU7 @ 288MHz  |  20KHz   |    5KHz    |  两相相线采样   | iFOC-EL05（开发中） |     有感/双编      |
| ARM Cortex-M33 | GD32G553CEU7 @ 216MHz  |  20KHz   |    5KHz    |  两相低侧采样   |   iFOC-DM4310-v1    |     有感/双编      |
| ARM Cortex-M7  | STM32F722RET6 @ 216MHz | 20KHz x2 |  5KHz x2   | 两相相线采样 x2 |   手术机器人平台    | 双驱/有感/增强安全 |
|     RISC-V     |      HPM5E31IPB1       | 20KHz x2 |  5KHz x2   |        /        |       开发中        | 双驱/有感/EtherCAT |

### 设计文档

1. [设计目标](https://github.com/Matrixchung/iFOC/blob/v2.0/docs/zh/%E8%AE%BE%E8%AE%A1%E7%9B%AE%E6%A0%87.md)
1. [IAP & Bootloader 设计](https://github.com/Matrixchung/iFOC/blob/v2.0/docs/zh/IAP%20%26%20Bootloader%20%E8%AE%BE%E8%AE%A1.md)
1. 各模块设计
   - [自适应缺相检测算法](https://github.com/Matrixchung/iFOC/blob/v2.0/docs/zh/modules/自适应缺相检测算法.md)


### 开发文档

施工中...



中文 | [English](https://github.com/Matrixchung/iFOC/blob/v2.0/readme_en.md)