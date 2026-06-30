## 高速串口接口（UART-HS）

### 设计背景

iFOC 系统中原有的 `UARTBase` 接口采用**生产者-消费者**模型，其核心依赖如下：

- UART 空闲中断（IDLE IRQ）检测帧边界
- DMA HDT / FDT 中断触发数据搬运
- `RxEventHandlerTask`：一个运行于最高优先级的 FreeRTOS 任务，通过任务通知（Task Notification）接收来自中断的唤醒信号，并在任务上下文中依次调用已注册的接收回调
- `tx_sem` / `tx_fifo_mutex`：用于保护发送路径的 FreeRTOS 二值信号量与互斥量

这套设计对通用场景（多个协议层共用同一串口、多线程同时写入）效果良好，但在以下场景下存在明显局限：

1. **实时任务中不允许阻塞**：`WriteBytes()` 在 FIFO 满时会尝试获取互斥量（最多等待 100 ms），这在 RT/Mid 控制循环中是不可接受的延迟。
2. **中断带来不可预测的调度抖动**：IDLE 中断和 DMA 中断会打断正在运行的任务，在高速通信（≥ 1 Mbps）时中断频率显著上升，影响 FOC 控制环路的时序确定性。
3. **RS485 半双工的特殊性**：RS485 总线在同一时刻只允许一个设备驱动，收发切换需要精确感知总线是否空闲，而 FreeRTOS 任务调度引入的不确定延迟使这一判断更加困难。
4. **单消费者场景下互斥开销不必要**：若确定只有一个任务消费串口数据，信号量和互斥量的开销纯属浪费。

基于上述原因，`UARTHSBase` / `UARTHS` 采用**混合驱动模型**：利用系统中已有的高频定时器中断（`UpdateRxFIFO()`）承担 RX 缓冲区的中途搬运职责，以 UART IDLE 中断（`OnUARTIRQ()`）实现帧边界检测、快速回包与 TX DMA 启动。全程不使用任何 FreeRTOS 同步原语，不依赖额外的 DMA 中断通道。

---

### 设计目标

| 目标 | 方案 |
|---|---|
| 无 RTOS 阻塞 | 关键路径不调用任何 FreeRTOS 同步原语 |
| 不增加中断通道 | RX 中途复制复用已有高频定时器中断，无需开启 DMA HDT / FDT 中断 |
| RS485 半双工防碰撞 | 硬件 UART IDLE 中断检测总线空闲，TX DMA 仅在 IDLE 触发后启动 |
| 最低回包延迟 | IDLE 中断触发后立即执行 `IdleCallback`，可在同一中断内完成接收处理并启动 TX DMA |
| 单消费者无锁 | `tx_fifo` / `rx_fifo` 均为 SPSC 无锁环形队列；RX 复制路径通过 `volatile bool rx_copy_busy` 标志位实现互斥 |

---

### 实现原理

#### 总体架构

`UARTHS` 由两条独立的执行路径共同驱动：

```
TMR2 ISR（高优先级）──→ UpdateRxFIFO()
                           ├─ RX DTERR 检测与恢复
                           └─ RX 中途复制（rx_copy_busy 保护，busy 则跳过）

USART1 ISR（低优先级）─→ OnUARTIRQ()
                           ├─ RX 尾段复制（rx_copy_busy 保护）
                           ├─ 调用 IdleCallback（快速回包入口）
                           └─ TX DMA 启动（tx_pending && chen==0）
```

**优先级约束**：USART1 IRQ 优先级必须**低于**调用 `UpdateRxFIFO()` 的定时器中断优先级（数值更大）。定时器 ISR 可抢占 `OnUARTIRQ()`，但反向不成立。

#### 接收路径（RX）

接收 DMA 工作在**循环（Circular）模式**，持续将 UART 数据寄存器中的字节搬运到固定的 `rx_buffer` 环形缓冲区，不会因软件处理延迟而丢失字节。

RX 数据复制分两种情形触发：

**中途复制（`UpdateRxFIFO()` 负责）**：由高频定时器中断周期性调用，读取 DMA NDTR 计算当前写入位置，将新到字节追加进 `rx_fifo`。此路径防止超长数据流或两次 IDLE 之间的数据导致 `rx_buffer` 被 DMA 覆盖。若 `rx_copy_busy` 被 IDLE ISR 置位，则跳过本次复制。

**帧尾复制（`OnUARTIRQ()` 负责）**：IDLE 中断触发时执行，捕获自上次复制以来 DMA 写入的剩余字节，确保 `rx_fifo` 在 `IdleCallback` 被调用前已完整包含本帧全部字节。执行前置 `rx_copy_busy = true`，结束后清除。

两路复制使用相同的绕圈检测逻辑：

```
curr_rx_pos = rx_buffer.size - rx_dma->dtcnt

正常情况（未绕圈）：
  rx_fifo.put(rx_buffer + last_dma_rx_size, curr_rx_pos - last_dma_rx_size)

绕圈情况（curr_rx_pos < last_dma_rx_size）：
  rx_fifo.put(rx_buffer + last_dma_rx_size, rx_buffer.size - last_dma_rx_size)  // 尾段
  rx_fifo.put(rx_buffer, curr_rx_pos)                                            // 头段
```

> **DMA 错误（DTERR）处理**：`UpdateRxFIFO()` 在复制逻辑之前优先检查 DTERR 标志。检测到 DTERR 后清除 GL 标志，重置地址寄存器和 DTCNT，重新使能 DMA，并将 `last_dma_rx_size` 归零。

#### 发送路径（TX）

TX 路径采用**单阶段直传**设计，在 IDLE 中断触发时一次性将 `tx_fifo` 中全部数据直接搬入 `tx_buffer` 并启动 DMA：

```
           调用方
             │ WriteBytes() / StartTransmit()
             ▼
         [ tx_fifo ]  ← 随时可写入，积累回包数据
             │
             │ IDLE 触发时：tx_pending && tx_dma->chen == 0
             │ tx_fifo.get() → tx_buffer（一次性全量搬运）
             ▼
          TX DMA → UART → 总线
```

`StartTransmit()` 将 `tx_pending` 置为 `true`（前提是 `tx_fifo` 非空）。`OnUARTIRQ()` 检测到 `tx_pending` 且 TX DMA 空闲（`chen == 0`）时，将 `tx_fifo` 全部数据取出写入 `tx_buffer`，配置 DMA 并启动。`tx_fifo` 清空后清除 `tx_pending`。

若 TX DMA 仍在传输上一帧（`chen != 0`），本次跳过，数据留在 `tx_fifo` 中等待下一次 IDLE。

#### RS485 总线空闲检测

RS485 半双工总线在 IDLE 中断触发时总线已确认空闲（硬件保证），因此 TX DMA 可在同一中断内立即启动，无需额外的软件空闲标志。`Init()` 会根据波特率自动配置 TSDT / TCDT（DE 信号的断言 / 解断时间），确保收发器切换方向时有足够建立时间。

#### IDLE 中断处理（OnUARTIRQ()）

`OnUARTIRQ()` 在 USART 全局中断服务函数中调用，内部按以下步骤执行：

1. **检查 IDLEF 标志**：若非 IDLE 中断则立即返回。
2. **RX 尾段复制**（`rx_copy_busy` 保护）：置 `rx_copy_busy = true`，执行帧尾字节复制，完成后清除标志。
3. **调用 IdleCallback**：此时 `rx_fifo` 已完整包含本帧全部字节，回调可直接调用 `ReadBytes()` / `WriteBytes()` / `StartTransmit()`。
4. **TX DMA 启动**：若 `tx_pending && tx_dma->ctrl_bit.chen == 0`，一次性将 `tx_fifo` 全部数据转入 `tx_buffer` 并启动 DMA；TX DMA 忙则跳过。
5. **清除 IDLE 标志**：AT32 要求依次读 STS 和 DT 寄存器（`UNUSED(huart->sts); UNUSED(huart->dt);`）。

#### UpdateRxFIFO() 与 IDLE ISR 的互斥

定时器 ISR（高优先级）调用 `UpdateRxFIFO()`，可在 `OnUARTIRQ()` 执行中途抢占它。两者都写 `rx_fifo` 和 `last_dma_rx_size`，需要互斥。

| 竞争区域 | 保护机制 |
|---|---|
| `rx_fifo` 写入 + `last_dma_rx_size` 更新 | `volatile bool rx_copy_busy`：`OnUARTIRQ()` 复制前置位；`UpdateRxFIFO()` 检测到后跳过 |

在单核 Cortex-M4 上，`volatile bool` 的读写（LDRB / STRB）是原子的，处理器为顺序执行，无硬件内存重排序，此机制足以保证互斥，无需 BASEPRI / PRIMASK。

#### IDLE 回调（IdleCallback）

```cpp
using IdleCallback = std::function<void(UARTHSBase*)>;
```

通过 `RegisterIdleCallback()` 注册，在每次 IDLE 中断触发、`rx_fifo` 更新完成后被 `OnUARTIRQ()` 调用。

**约束**：
- 回调在 ISR 上下文中执行，**禁止**调用任何 FreeRTOS 阻塞 API（`vTaskDelay`、`xSemaphoreTake` 等）。
- 可安全调用 `ReadBytes()`、`WriteBytes()`、`StartTransmit()`。
- 回调中通过 `WriteBytes()` + `StartTransmit()` 写入回复数据后，`OnUARTIRQ()` 的后续步骤会在同一中断内完成 TX DMA 启动，无需等待下一次 IDLE。

---

### 缓冲区层次与尺寸约束

```
             ┌─────────────────────────────────────────────────┐
接收方向     │  总线 → UART → [rx_buffer 循环DMA] → [rx_fifo] │
             └─────────────────────────────────────────────────┘
                                                       ↑ ReadBytes()

             ┌─────────────────────────────────────────────────┐
发送方向     │  [tx_fifo] ──────────────→ TX DMA → 总线       │
             └─────────────────────────────────────────────────┘
               ↑ WriteBytes() / StartTransmit()
```

| 缓冲区 | 大小 | 说明 |
|---|---|---|
| `rx_buffer` | 与 `rx_fifo` 相同 | 循环 DMA 目标，不可被软件直接读取 |
| `rx_fifo` | 512 / 256 B ¹ | 消费者调用 `ReadBytes()` 的来源；**决定单帧最大可接收长度** |
| `tx_fifo` | 512 / 256 B ¹ | 调用方调用 `WriteBytes()` 的目标 |
| `tx_buffer` | 与 `tx_fifo` 相同 | TX DMA 的实际数据源，IDLE 时由 `tx_fifo` 一次性填充 |

¹ 缓冲区大小根据 `configTOTAL_HEAP_SIZE` 在编译期自动选择：堆 ≥ 16 KB 时为 512 B，否则为 256 B。

**rx_buffer 尺寸要求**：`rx_buffer.size > 波特率(字节/秒) × UpdateRxFIFO() 最大调用间隔(秒)`

以 6 Mbps、200 µs（5 kHz）为例：750000 × 0.0002 = 150 字节，512 字节有约 3 倍余量，满足要求。

**单帧最大长度**：`rx_fifo` 的容量。若单帧数据量超过 `rx_fifo` 容量，先到达的字节将在消费者读取前被覆盖。

**单次 TX 最大长度**：`tx_buffer` 的容量。若 `tx_fifo` 中数据超过 `tx_buffer` 大小，超出部分留待下次 IDLE 发送。

---

### 适用范围

**适合使用 UART-HS 的场景：**

- 通信任务运行在 RT 或 Mid 控制循环中，对执行时间有严格要求
- 波特率较高（≥ 921600 bps），需要避免额外 DMA 中断频繁触发
- RS485 半双工总线，需要精确的总线空闲检测与快速回包
- 系统中已有高频定时器中断，可复用为 `UpdateRxFIFO()` 的驱动源
- **单一消费者**：只有一个任务负责读取和回复串口数据

**不适合使用 UART-HS 的场景：**

- 多个任务同时需要向同一串口写入（需要 `UARTBase` 的互斥量保护）
- 通信逻辑不运行在固定周期任务中，而是依赖事件触发
- MCU 需要主动发起传输（无入帧即可开始发送），TX 必须在 IDLE 触发后才能启动
- 需要在 `IdleCallback` 中调用 FreeRTOS 阻塞 API

---

### 性能测试

在 AT32F435CGU7 @ 288 MHz 下，5 kHz 定时器中断中调用 `UpdateRxFIFO()`，IDLE 中断中通过 `IdleCallback` 原样回传数据：

```cpp
// 注册回调（初始化时调用一次）
uart1->RegisterIdleCallback([](iFOC::HAL::UARTHSBase* uart) {
    if(uart->rx_fifo.move_to(uart->tx_fifo, uart->GetRxLen()) > 0)
        uart->StartTransmit();
});

// UpdateMid 负责驱动 RX 中途复制
void RS485Protocol::WorkerTask::UpdateMid(float Ts)
{
    parent->uart->UpdateRxFIFO();
}
```

测试环境：

* RS485 总线波特率 6 Mbps，单主机，单从机，从机 120Ω 终端电阻接入
* USB 转高速 RS485 转换器（沁恒 CH9111L）
* 使用沁恒 COMTransmit 软件按 1 ms 周期性发送长度 > 900 字节的数据包，软件上显示发送速度和接收速度均为约 `60 KB/s`

测试结果：

持续测试收发字节数达到 1200000 字节，发送计数 = 接收计数，任务 `max_elapsed_time_us` 结果为 20 µs，多次实验结果稳定，占用 5 kHz 循环约 10% 时间。

---

### 使用方法

#### 1. 实例化与初始化

```cpp
#include "at32wk_uart_hs.hpp"

// RS485 模式（第四个参数 true）
iFOC::HAL::UARTHS* uart = new iFOC::HAL::UARTHS(USART1, DMA1_CHANNEL1, DMA1_CHANNEL2, true);
uart->Init(iFOC::DataType::Comm::UARTBaudrate::BAUD_6000000);

// 普通全双工 UART 模式
iFOC::HAL::UARTHS* uart = new iFOC::HAL::UARTHS(USART1, DMA1_CHANNEL1, DMA1_CHANNEL2);
uart->Init(iFOC::DataType::Comm::UARTBaudrate::BAUD_921600);
```

`Init()` 在 RS485 模式下会自动根据波特率配置 TSDT / TCDT，无需手动计算。`Init()` 不启用 RX DMA 中断（HDT / FDT / DTERR），DTERR 恢复由 `UpdateRxFIFO()` 轮询处理。

#### 2. 配置中断优先级并接入 ISR

**USART 全局中断优先级必须低于调用 `UpdateRxFIFO()` 的定时器中断优先级**（数值更大），否则 IDLE ISR 可能在 `UpdateRxFIFO()` 执行时抢占它，导致 `rx_fifo` 并发写。

在 NVIC 配置处（通常为 `wk_nvic_config()`）：

```c
nvic_irq_enable(TMR2_GLOBAL_IRQn, 2, 0);   // UpdateRxFIFO() 所在定时器：高优先级
nvic_irq_enable(USART1_IRQn,      3, 0);   // OnUARTIRQ()：低优先级（数值更大）
```

在 USART 全局中断服务函数中调用 `OnUARTIRQ()`：

```cpp
// isr.cpp
void usart1_irq(void)
{
    uart1->OnUARTIRQ();
}
```

`OnUARTIRQ()` 内部首先检查 IDLEF 标志，若非 IDLE 中断则立即返回，因此与其他 USART 中断源共存时无副作用。

#### 3. 注册 IDLE 回调（可选）

若需要在帧接收完成后立即处理并回包，通过 `RegisterIdleCallback()` 注册：

```cpp
uart1->RegisterIdleCallback([](iFOC::HAL::UARTHSBase* uart) {
    // 此处在 IDLE ISR 中执行，rx_fifo 已包含本帧全部字节
    uint8_t buf[256];
    uint16_t len = uart->ReadBytes(buf, sizeof(buf), false);
    // 处理 buf[0..len-1]，构造回复...
    uart->WriteBytes(reply, reply_len);
    uart->StartTransmit();  // OnUARTIRQ() 会在同一中断内完成 DMA 启动
});

// 如需注销：
uart1->RemoveIdleCallback();
```

**禁止**在回调中调用任何 FreeRTOS 阻塞 API。

#### 4. 在控制任务中调用 UpdateRxFIFO()

`UpdateRxFIFO()` 必须由高频定时器中断周期性驱动，推荐注册为 Mid 任务（约 5 kHz）：

```cpp
void RS485Protocol::WorkerTask::UpdateMid(float Ts)
{
    parent->uart->UpdateRxFIFO();
}
```

**重要**：`ReadBytes()` 只应在 `IdleCallback` 中消费 `rx_fifo`，不得同时在 `UpdateMid` 中消费（双消费者会破坏 SPSC kfifo 的 `out` 索引）。

#### 5. 发送数据

```cpp
// WriteBytes + StartTransmit（通常在 IdleCallback 中调用）
uint8_t response[] = {0x01, 0x02, 0x03};
uart->WriteBytes(response, sizeof(response));
uart->StartTransmit();
// 实际发送在当前或下一次 IDLE 中断触发时，tx_dma->chen == 0 时启动
// 若 TX DMA 仍忙（上一帧未发完），本次跳过，数据留在 tx_fifo 等待下次 IDLE
```

#### 6. 接收数据

```cpp
// 通常在 IdleCallback 中调用
if(uart->GetRxLen() > 0)
{
    uint8_t buf[256];
    uint16_t len = uart->ReadBytes(buf, sizeof(buf), false); // false = 消费性读取
    // 处理 buf[0..len-1]...
}
```

#### 7. RS485 模式注意事项

- `Init()` 传入 `rs485_mode = true` 后，硬件自动管理 DE 引脚（发送时高电平，接收时低电平），无需软件干预。
- IDLE 中断触发时总线已确认空闲，TX DMA 可立即启动，无需额外的软件空闲标志。
- `Init()` 根据波特率自动配置 TSDT / TCDT，确保收发器切换方向时有足够建立时间。

---

### 状态变量说明

`UARTHS` 内部维护以下状态变量：

| 变量 | 类型 | 含义 |
|---|---|---|
| `last_dma_rx_size` | `uint16_t` | 上次 RX 复制处理到的 DMA 写入位置（相对于 `rx_buffer` 起点） |
| `tx_pending` | `bool` | `tx_fifo` 中有数据待发送；由 `StartTransmit()` 置位，`OnUARTIRQ()` 清除 |
| `rx_copy_busy` | `volatile bool` | `OnUARTIRQ()` RX 复制临界区保护；置位期间 `UpdateRxFIFO()` 跳过复制 |
