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

#### 常量定义

##### 一、`iFOC::Motion`

为了遵循“输入输出均基于参考系”这一目标，iFOC 系统使用 `iFOC::Motion` 结构体来表达输入输出命令，可参见：[`motion.hpp`](https://github.com/Matrixchung/iFOC/blob/v2.0/src/DataType/Headers/Base/motion.hpp)

```c++
namespace iFOC
{
    struct Motion // size: 40 bytes, alignof(4)
    {
        enum class Ref :        uint8_t { ELEC, BASE, OUTPUT };
    	enum class TorqueUnit : uint8_t { AMP,  NM };
    	enum class SpeedUnit :  uint8_t { RADS, DEGS, REVS, RPM, HZ };
    	enum class PosUnit :    uint8_t { RAD,  DEG,  REV };
    	Ref ref{Ref::ELEC};
    	triple<real_t, TorqueUnit> torque{0.0f, 0.0f, TorqueUnit::AMP};
    	triple<real_t, SpeedUnit> speed{0.0f, 0.0f, SpeedUnit::RADS};
    	triple<real_t, PosUnit> pos{0.0f, 0.0f, PosUnit::RAD};
    }
}
```

首先，需要确定 `Motion` 结构体的参考系 `Ref`，这里有三种选择：

* `ELEC` 代表**电角度**参考系，一般不用于控制
* `BASE` 代表**转子**参考系
* `OUTPUT` 代表**输出轴**参考系，通常与 `BASE` 之间通过减速比参数 `deduction_ratio` 联系，`BASE / deduction_ratio = OUTPUT`。

使用三元组 `triple<real_t, Type>` 依次表示 `value`, `limit`, `unit`。`limit` 表示对该量的限制，一般用在输入控制指令中，强制为正值。若没有设置限制，则 `limit = 0`。

##### 二、`iFOC::DataType::Base::MotorControlMode`

由 `motor->SetControlMode()` 切换 [`motor_base.hpp`](https://github.com/Matrixchung/iFOC/blob/v2.0/src/Motor/motor_base.hpp)

控制逻辑在 [`foc_speed_loop_pi.cpp`](https://github.com/Matrixchung/iFOC/blob/v2.0/src/Motor/FOC/Controller/foc_speed_loop_pi.cpp) 中

```c++
namespace iFOC {
namespace DataType {
namespace Base {

enum class MotorControlMode : uint32_t
{
  // 位置控制模式，此模式下接收 TargetMotion 中的 pos，并把 torque 和 speed 分别作为各自环路的前馈 + 限幅
  /* 控制公式：pos_error = target.pos.value - current.pos.value (位置误差)
   *         target_speed = limit(pos_pi.GetOutput(pos_error) + target.speed.value, target.speed.limit)（位置误差经过位置环 P 控制器，加上速度前馈得到目标速度，并限幅）
   *         speed_error = target_speed - current.speed.value (速度误差)
   *         Iq = limit(speed_pi.GetOutput(speed_error) + target.torque.value, target.torque.limit) (速度误差经过速度环 PI 控制器，加上扭矩前馈得到目标扭矩，并限幅)
   */
  CTRL_MODE_POSITION = 0, 
    
  // 速度控制模式，此模式下接收 TargetMotion 中的 speed，并把 torque 作为前馈 + 限幅，忽略 pos
  /* 控制公式：speed_error = limit(target_speed, target.speed.value) - current.speed.value (速度误差)
   *         Iq = limit(speed_pi.GetOutput(speed_error) + target.torque.value, target.torque.limit) (速度误差经过速度环 PI 控制器，加上扭矩前馈得到目标扭矩，并限幅)
   */
  CTRL_MODE_VELOCITY = 1, 
    
  // 力矩控制模式，此模式下接收 TargetMotion 中的 torque，忽略 pos 和 speed
  // 控制公式：Iq = limit(target.torque.value, target.torque.limit)
  CTRL_MODE_CURRENT = 2,  
    
  // MIT 混合控制模式，此模式下将 TargetMotion 中的 pos.limit 作为 Kp，speed.limit 作为 Kd，torque.limit 作为扭矩限幅
  // 控制公式：Iq = limit( Kp * pos_error + Kd * speed_error + target.torque.value, target.torque.limit )
  CTRL_MODE_HYBRID = 3    
};

} // End of namespace Base
} // End of namespace DataType
} // End of namespace iFOC
```

##### 三、`iFOC::DataType::Base::MotorState`

由 `motor->state_machine->RequestState()` 切换 [`foc_task_state_machine.cpp`](https://github.com/Matrixchung/iFOC/blob/v2.0/src/Motor/FOC/Task/foc_task_state_machine.cpp)。在切换入新状态前，会校验当前条件是否满足状态切入条件，若不满足则停留在现有状态。

```c++
namespace iFOC {
namespace DataType {
namespace Base {

enum class MotorState : uint32_t
{
  // 空闲状态，此状态为初始状态，三相整流桥与相线断开
  IDLE = 0,
  
  // 启动序列，在配置中激活 startup_sequence_enabled 后有效。
  // 进入启动序列后，会依次搜寻配置中 startup_ 前缀的设置，依次运行所有为 true 的状态
  STARTUP_SEQUENCE = 1,
  
  // 电机基础参数校准状态（foc_task_basic_param_calib.cpp/hpp）
  // 依次测量：相电阻、d/q 轴电感、相电感、磁链（请保证电机能够自由转动）
  BASIC_PARAM_CALIBRATION = 2,
    
  // 编码器零点搜索状态（适用于 ABZ 增量编码器）
  ENCODER_INDEX_SEARCH = 3,
    
  // 编码器校准状态（foc_task_encoder_calib.cpp/hpp）
  // 依次测量编码器方向、电机极对数、编码器零位误差（同时生成编码器非线性校准表）
  ENCODER_CALIBRATION = 4,
    
  // 扩展参数校准状态（foc_task_extend_param_calib.cpp/hpp）
  // 支持测量离轴编码器的零位误差、生成齿槽转矩查找表
  EXTEND_PARAM_CALIBRATION = 5,
    
  // 有感闭环控制状态
  SENSORED_CLOSED_LOOP_CONTROL = 6,
    
  // 无感闭环控制状态
  SENSORLESS_CLOSED_LOOP_CONTROL = 7,
    
  // 开环速度控制状态
  OPEN_LOOP_VELOCITY_CONTROL = 8
};

} // End of namespace Base
} // End of namespace DataType
} // End of namespace iFOC
```

#### 与电机交互

[`motor_base.hpp`](https://github.com/Matrixchung/iFOC/blob/v2.0/src/Motor/motor_base.hpp) 中定义了以下函数（所有输入输出量均为绝对位置，相对位置可自行通过 `GetTargetMotion()` 相加）：

```c++
/// Get current motion info, with transformation to different references and unit agreements.
/// \param dest destination to store the transformed Motion info
/// \param ref_frame reference frame selection, options: {ELEC, BASE, OUTPUT}
/// \param torque_unit torque unit selection, options: {AMP, NM}
/// \param speed_unit speed unit selection, options: {RADS, DEGS, REVS, RPM}
/// \param pos_unit pos unit selection, options: {RAD, DEG, REV}
virtual void GetCurrentMotion(Motion& dest, Motion::Ref ref_frame, Motion::TorqueUnit torque_unit, Motion::SpeedUnit speed_unit, Motion::PosUnit pos_unit) = 0;

/// Get target motion info, with transformation to different references and unit agreements. \n
/// Target motion are handled as ref == BASE internally. \n
/// Note that a ref_frame == ELEC will result in the same output as BASE.
/// \param dest destination to store the transformed Motion info
/// \param ref_frame reference frame selection, options: {BASE, OUTPUT}
/// \param torque_unit torque unit selection, options: {AMP, NM}
/// \param speed_unit speed unit selection, options: {RADS, DEGS, REVS, RPM}
/// \param pos_unit pos unit selection, options: {RAD, DEG, REV}
virtual void GetTargetMotion(Motion& dest, Motion::Ref ref_frame, Motion::TorqueUnit torque_unit, Motion::SpeedUnit speed_unit, Motion::PosUnit pos_unit) = 0;

/// Set target motion, with transformation to different references and unit agreements. \n
/// Target motion with ref == ELEC will be IGNORED!
/// \param motion target Motion struct, with reference frame and units stored in std::pair.second
virtual FuncRetCode SetTargetMotion(Motion& motion) = 0;
```

除此之外，还有一些辅助函数，其内部均是通过调用这三个函数实现的。

获取当前反馈状态，可以使用 `GetCurrentMotion()`；获取当前设置的目标状态，可以使用 `GetTargetMotion()`；设置目标状态，对应 `SetTargetMotion()`。

**请注意，为了保证安全，需要校验 `SetTargetMotion()` 的返回值，通过后需要调用 `motor->SetControlMode()` 设置期望的控制模式**。

##### 在驱动板上本地调用的示例：

```c++
... // 需要在 motor->Init() 并启用定时器、RTOS 调度器之后调用，推荐由下一段介绍的 Protocol 调用，或运行在单独的低优先级 FreeRTOS 任务中
// 上电后，电机默认为 IDLE 状态。
motor->state_machine.RequestState(MotorState::BASIC_PARAM_CALIBRATION); // 按需调用，若当前有待校准的参数，或上一次校准后未存入非易失存储，则进入基础参数测量状态。若基础参数全部测量完成，则无作用。系统会自动决定是否跳转进入该状态。
while(foc->GetTaskByName("BasicParam")) sleep(10); // 可以等待基础参数校准完成。
motor->state_machine.RequestState(MotorState::ENCODER_CALIBRATION); // 编码器校准。
while(foc->GetTaskByName("EncCalib")) sleep(10);   // 可以等待编码器校准完成。
motor->state_machine.RequestState(MotorState::SENSORED_CLOSED_LOOP_CONTROL); // 进入有感闭环控制模式
sleep(100);
// 设置位置模式，转到转子角度的 90 度（绝对位置），限速 100 RPM，限力矩 2.0A。
Motion target_motion
{
    .ref = Motion::Ref::BASE,
    .torque = {0.0, 2.0, Motion::TorqueUnit::AMP},
    .speed = {0.0, 100.0, Motion::SpeedUnit::RPM},
    .pos = {90.0, 0.0, Motion::PosUnit::DEG}
};
if(motor->SetTargetMotion(target_motion) == FuncRetCode::OK) motor->SetControlMode(MotorControlMode::CTRL_MODE_POSITION);
```

#### 使用通信协议

除了本地调用的方式之外，关节电机更多的是受总线主机或者外部控制，因此我们设计了一套通信协议接口（可见 [`protocol_base.hpp`](https://github.com/Matrixchung/iFOC/blob/v2.0/src/Protocol/protocol_base.hpp) ）。该接口类内部持有 void 类型的电机类指针，因而得以支持三相 FOC 电机或者是直流 DC 电机、步进电机等变体。

目前已经得到完整支持的有：[`ASCII 可读协议`](https://github.com/Matrixchung/iFOC/blob/v2.0/src/Protocol/ascii_protocol.hpp)、[`DroneCAN 协议`](https://github.com/Matrixchung/iFOC/blob/v2.0/src/Protocol/dronecan_protocol.hpp) 等。一个电机类可以注册多种协议，注册的方式也非常简单：

```c++
// ASCII 可读协议（ascii_protocol.cpp），需要传入一个串口 HAL 类指针
auto* uart1 = new iFOC::HAL::UART(USART1, DMA2_CHANNEL1, DMA2_CHANNEL2);
motor->RegisterProtocol(new iFOC::Protocol::ASCIIProtocol<iFOC::FOCMotor>(uart1));

// DroneCAN 协议（dronecan_protocol.cpp），需要传入一个 CAN HAL 类指针
auto* can1 = new iFOC::HAL::CAN(can1);
motor->RegisterProtocol(new iFOC::Protocol::DroneCANProtocol(can1));
```

各协议的使用方法可到源码中查阅。

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