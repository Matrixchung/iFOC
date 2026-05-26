# iFOC 智能驱动系统

<center class ='img'>
<img title="整体架构图" src=".\docs\zh\structure_map_cn.png" width="100%">
</center>


### 使用说明

#### 快速开始

##### 基本约定：

- 本项目基于 FreeRTOS 架构，以 heap_4 重写了项目全局的 `new/delete` 机制，并尽可能保证本体组件遵循 **RAII** 原则，因此可以有限度的使用动态内存（为确保实时性，实时中断中不可分配动态内存）
- 需要调用系统 C 库中的回调的外设，例如串口、CAN 等，需要声明为全局变量或指针，若声明为指针，则必须在主函数开头进行 `new`，将对象安全建立在堆空间中。
- 系统包含三层循环：
  1. 电流环为实时循环（RT_LOOP），一般在 ADC 注入回调中调用，运行频率为 PWM 频率（例子中取 20KHz）。
  2. 速度/位置环为中速循环（MID_LOOP），一般由一个独立的定时器中断调用，优先级低于 ADC 中断，运行频率一般为实时循环的四分之一或更低，但不推荐低于 2KHz。（例子中取 5KHz）
  3. 非实时循环（NORMAL），在内部注册到 FreeRTOS 的任务链表中，由操作系统按需进行调度。
- 运行起 FOC 控制的最小结构：PWM 驱动（`motor->LinkDriver(ptr)`），电流感测器（`motor->LinkCurrSense(ptr)`），母线电压检测（`motor->LinkBusSense(ptr)`）

采用全局指针配合 `new` 的代码示例（可参考 example 文件夹下的工程，各工程下有详细的 readme）

```c++
// 以 STM32 为例，先引入对应的头文件合集（路径在 iFOC/HAL/STM32）
#include "main.h"
#include "stm32_include.hpp" // 包含了所有 STM32 外设的抽象层

using namespace iFOC; // 为简化代码，引入 iFOC 命名空间（节约类型名前面的 iFOC::）

FOCMotor* motor = nullptr; // 新建一个 FOC 电机指针，若存在双路或更多路驱动，可以新建更多

// 在 HAL 库生成的 main() 函数里面的用户部分调用，请注意，app_main() 需要接管 main() 循环，即：放在 main() 函数的启动 RTOS 内核和空 while(1) 循环前面，可参考 example 文件夹下的工程
void app_main() 
{
    motor = new FOCMotor(); // 所有的 new/delete 均在 FreeRTOS 内存堆内部操作
    
    // 链接 PWM 驱动模块（所有模块都设计成虚函数指针，方便未来根据配置在运行时决定分支）
    // void LinkDriver(Driver::FOCDriverImpl auto *drv)
    // Driver::FOCDriver6PWM(TIM_TypeDef *htim); (STM32 分支下，采用 LL 库的定时器指针)
    motor->LinkDriver(new Driver::FOCDriver6PWM(TIM1));
    
    // 链接电流采样模块
    // void LinkCurrSense(Sense::FOCCurrSenseImpl auto *curr)
    // Sense::CurrSenseThreeShunts(HAL::ADCPortBase* _a, 
    //							HAL::ADCPortBase* _b, 
    //							HAL::ADCPortBase* _c, 
    //							bool rev_a, bool rev_b, bool rev_c);
    // 三相低侧注入采样，支持其他采样方式
    // ADCPortBase* 指针为 ADC 端口包装类，例子中传入的参数依次为注入寄存器的地址和参考电压 Vrefint 的原始值地址（Vrefint 可以通过 ADC DMA 持续获取）
    motor->LinkCurrSense(new Sense::CurrSenseThreeShunts(
        new HAL::ADCPort(&ADC1->JDR1, m_pVrefint),
        new HAL::ADCPort(&ADC1->JDR2, m_pVrefint),
        new HAL::ADCPort(&ADC1->JDR3, m_pVrefint),
        true, true, true // 控制三路的符号是否反相
    ));
    
    // 链接母线电压电流检测模块（可以只检测电压，电流置 0，但会失去母线过流防护）
    // void LinkBusSense(Sense::BusSenseBase *bus)
    // Sense::BusSenseADC(HAL::ADCPortBase* _vbus,
    //                 real_t _vbus_gain,
    //                 HAL::ADCPortBase* _ibus,
    //                 real_t _ibus_gain);
    // 最简单的例子，通过 ADC 分别检测母线电流采样电阻上的分压和母线电压分压后的结果，原始值 * gain = 实际值
    // 支持使用总线采样芯片进行采样，如 INA226、INA237 等（见 iFOC/Sense）
    motor->LinkBusSense(new Sense::BusSenseADC(
        new HAL::ADCPort(m_pVbus_raw, m_pVrefint), 21.0f,
        new HAL::ADCPort(m_pIbus_raw, m_pVrefint). 20.0f
    ));
    
    // 初始化电机类
    // FuncRetCode Init(const bool initTIM);
    // initTIM 参数控制是否初始化定时器，最常用的例子是双路驱动，为了节省资源，常将双路定时器设置为同时触发，以便在一个 ADC 注入回调中断中同时检测多路电机的电流，这里就可以先置 false，后续在启用 ADC 注入中断时采用原子操作同时启动多路定时器。
    motor->Init(true);
    
    ADC_Start_Injected(); // 设置 ADC JEOS 标志，启用注入中断（在外部实现）
    LL_TIM_EnableIT_UPDATE(TIM16); // 例子中，我们设置中速循环运行在 TIM16 更新中断中
    LL_TIM_EnableCounter(TIM16); // 启用 TIM16 计数
    
    vTaskStartScheduler(); // 启用 FreeRTOS 调度器，接管主循环
    
    while(1);
}

void ADC_1_2_IRQHandler()
{
    // void FOCMotor::DispatchRTTasks(const float Ts);
    // Ts 代表循环运行周期，可给定值 1/20KHz = 0.00005s
    motor->DispatchRTTasks(0.00005f);
}

void TIM1_UP_TIM16_IRQHandler()
{
    // void FOCMotor::DispatchMidTasks(const float Ts);
    // Ts 代表循环运行周期，可给定值 1/5KHz = 0.0002s
    motor->DispatchMidTasks(0.0002f);
}
```



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