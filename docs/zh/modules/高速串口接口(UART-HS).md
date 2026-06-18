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

基于上述原因，`UARTHSBase` / `UARTHS` 选择完全抛弃中断与 FreeRTOS 原语，转而采用**轮询驱动的单消费者模型**。

---

### 设计目标

| 目标 | 方案 |
|---|---|
| 无中断，无 RTOS 依赖 | 全部标志位在 `Update()` 中轮询处理 |
| 有界执行时间 | 每次 `Update()` 处理完当前周期到达的字节即返回，不循环等待 |
| RS485 半双工防碰撞 | 通过 NDTR 稳定性检测总线空闲，不读 DT 寄存器（避免干扰 DMA） |
| 最大化有效缓冲深度 | TX 路径采用两阶段流水线，tx\_fifo + tx\_buffer 同时持有两批数据 |
| 单消费者无锁 | tx\_fifo / rx\_fifo 均为 SPSC 无锁环形队列，不需要任何互斥保护 |

---

### 实现原理

#### 接收路径（RX）

接收 DMA 工作在**循环（Circular）模式**，持续将 UART 数据寄存器中的字节搬运到固定的 `rx_buffer` 环形缓冲区中。这保证了不会因为软件处理延迟而丢失字节（相比于普通模式，后者在 DMA 停止到重新使能之间存在数据丢失窗口）。

每次调用 `Update()` 时，通过读取 DMA 的 **NDTR（Number of Data To Receive）** 寄存器计算出当前 DMA 已写入的位置，与上次保存的位置做差，将新到的字节追加复制进 `rx_fifo`：

```
curr_rx_pos = rx_buffer.size - rx_dma->dtcnt

正常情况（未绕圈）：
  new_bytes = curr_rx_pos - last_pos
  rx_fifo.put(rx_buffer + last_pos, new_bytes)

绕圈情况（curr_rx_pos < last_pos）：
  rx_fifo.put(rx_buffer + last_pos, rx_buffer.size - last_pos)  // 尾段
  rx_fifo.put(rx_buffer, curr_rx_pos)                           // 头段
```

> **DMA 错误（DTERR）处理**：DTERR 发生时硬件自动清零 CHEN，停止 DMA。`Update()` 检测到 DTERR 后清除所有子标志（通过清 GL 标志），重置地址寄存器和 DTCNT，重新使能 DMA，并将 `last_dma_rx_size` 归零。当前接收位置自动重置为缓冲区起点。

#### 发送路径（TX）

TX 路径采用**两阶段流水线**设计，将"准备数据"与"等待总线空闲后发送"解耦：

```
           调用方
             │ WriteBytes()
             ▼
         [ tx_fifo ]  ← 最大 512 字节，随时可写入
             │
             │ 阶段一：DMA 空闲时即可执行（无需总线空闲）
             ▼
         [ tx_buffer ] ← 最大 512 字节，预填充等待发送
             │
             │ 阶段二：检测到总线空闲后才启动 DMA
             ▼
          TX DMA → UART → 总线
```

**阶段一（Pre-copy）**：只要 TX DMA 不忙（`!is_tx_busy`）且 `tx_buffer` 还有剩余空间（`tx_buffer_len < tx_buffer.max_size()`），就将 `tx_fifo` 中的数据追加复制进 `tx_buffer`。此步骤不等待总线空闲，因此在 RS485 对方仍在发送时即可持续填充 `tx_buffer`，充分利用等待时间。

**阶段二（DMA 启动）**：`tx_buffer` 有数据（`tx_buffer_len > 0`）且总线被判定为空闲时，配置 TX DMA 并使能，开始真正的发送。

> **有效缓冲深度**：`tx_fifo`（512 字节）与 `tx_buffer`（512 字节）可以同时各持有一批数据，等效 TX 缓冲深度为 **1024 字节**。当一批数据正在等待总线空闲（已在 `tx_buffer` 中）时，`tx_fifo` 可以继续接收下一批写入。

#### RS485 总线空闲检测

RS485 是半双工总线，从机必须等对方完全停止发送（DE 引脚拉低）后才能启动发送，否则会产生总线竞争。

本实现通过 **NDTR 稳定性检测**判断总线空闲，而非读取 UART 的 IDLEF 标志位（后者需要读 STS 和 DT 寄存器，在 DMA 运行时存在干扰数据流的风险）：

```
在本次 Update() 的 RX 段处理完成后，快照 curr_rx_dtcnt = rx_dma->dtcnt

判断条件：curr_rx_dtcnt == last_rx_dtcnt（上次 Update() 末尾保存的值）

相等  → 自上次 Update() 以来 NDTR 没有变化 → 没有新字节到达 → 总线空闲 → 允许发送
不等  → 对方仍在发送 → tx_buffer 继续等待
```

在 5 kHz 的 Mid 循环下，每个检测周期约 200 μs。在 6 Mbps 波特率下，200 μs 内最多到达 150 字节；若 NDTR 在两次调用之间保持不变，可以安全确认总线空闲。

此外，`Init()` 会根据波特率自动计算并配置 TSDT / TCDT（DE 信号的断言/解断时间），确保收发器在切换方向时有足够的建立时间。

---

### 缓冲区层次与尺寸约束

```
             ┌─────────────────────────────────────────────────┐
接收方向     │  总线 → UART → [rx_buffer 循环DMA] → [rx_fifo] │
             └─────────────────────────────────────────────────┘
                                                       ↑ ReadBytes() / MoveRxToTx()

             ┌─────────────────────────────────────────────────┐
发送方向     │  [tx_fifo] → [tx_buffer 预填充] → TX DMA → 总线│
             └─────────────────────────────────────────────────┘
               ↑ WriteBytes() / StartTransmit()
```

| 缓冲区 | 大小 | 说明 |
|---|---|---|
| `rx_buffer` | 512 B | 循环 DMA 目标，不可被软件直接读取 |
| `rx_fifo` | 512 B | 消费者调用 `ReadBytes()` 的来源 |
| `tx_fifo` | 512 B | 调用方调用 `WriteBytes()` 的目标 |
| `tx_buffer` | 512 B | TX DMA 的实际数据源，由 Pre-copy 阶段填充 |

**RX buffer 尺寸要求**：`rx_buffer.size > baud_rate_bytes_per_sec × max_update_interval_sec`

以 6 Mbps、200 μs 更新间隔为例：750000 × 0.0002 = 150 字节，512 字节有约 3 倍余量，满足要求。

---

### 适用范围

**适合使用 UART-HS 的场景：**

- 通信任务运行在 RT 或 Mid 控制循环中，对执行时间有严格要求
- 波特率较高（≥ 921600 bps），中断频率过高会显著影响控制环路
- RS485 半双工总线，需要精确的总线空闲检测
- **单一消费者**：只有一个任务负责读取和回复串口数据

**不适合使用 UART-HS 的场景：**

- 多个任务同时需要向同一串口写入（需要 `UARTBase` 的互斥量保护）
- 通信逻辑不运行在固定周期任务中，而是依赖事件触发（使用 `UARTBase` 的 `RegisterRxHandler()` 回调机制更合适）
- 需要精确的帧边界检测且响应延迟要求小于一个 `Update()` 周期

---

### 性能测试

在 AT32F435CGU7 @ 288MHz 下，在 5KHz 定时器中断中运行以下逻辑：

```c++
void RS485Protocol::WorkerTask::UpdateMid(float Ts)
{
    parent->uart->Update();
    if(parent->uart->rx_fifo.move_to(parent->uart->tx_fifo, parent->uart->GetRxLen()) > 0)
    {
        parent->uart->StartTransmit();
    }
}
```

代表以 5KHz 频率轮询总线，若总线上主机发来新的数据，则原样转存到 Tx FIFO 中，并等待总线空闲时发送出去。

中断内采用 DWT 计数器进行性能统计，代码为：

```c++
void tmr2_irq(void)
{
    tmr_flag_clear(TMR2, TMR_OVF_FLAG);
    wdt_counter_reload();
    MEASURE_TIME(motor_1->task_times.mid_interval_task)
    {
        motor_1->DispatchMidTasks(iFOC::MID_LOOP_TS);
    }
}
```

测试环境：

* RS485 总线波特率 6Mbps，单主机，单从机，从机 120Ω 终端电阻接入

* USB 转高速 RS485 转换器（沁恒 CH9111L）
* 使用沁恒 COMTransmit 软件按 1ms 周期性发送长度 > 900 字节的数据包，软件上显示发送速度和接收速度均为约 `60KB/s`。

测试结果：

持续测试收发字节数达到 1200000 字节，发送计数 = 接收计数，任务 `max_elapsed_time_us` 结果为 20 μs，多次实验结果稳定，占用 5KHz 循环约 10% 时间。

------

### 使用方法

#### 1. 实例化与初始化

```cpp
#include "at32wk_uart_hs.hpp"

// RS485 模式（第四个参数 true）
iFOC::HAL::UARTHS* uart = new iFOC::HAL::UARTHS(USART1, DMA1_CHANNEL1, DMA1_CHANNEL2, true);
uart->Init(iFOC::DataType::Comm::UARTBaudrate::BAUD_6000000);

// 普通 UART 模式
iFOC::HAL::UARTHS* uart = new iFOC::HAL::UARTHS(USART1, DMA1_CHANNEL1, DMA1_CHANNEL2);
uart->Init(iFOC::DataType::Comm::UARTBaudrate::BAUD_921600);
```

`Init()` 在 RS485 模式下会自动根据波特率配置 TSDT / TCDT，无需手动计算。

#### 2. 在控制任务中定期轮询

`Update()` 必须被周期性调用，推荐注册为 Mid 任务（约 5 kHz）：

```cpp
// 在 Protocol 的 WorkerTask::UpdateMid() 中：
void UpdateMid(float Ts) override
{
    uart->Update();
    // 处理接收到的数据...
}
```

**重要**：`Update()` 和 `ReadBytes()` / `WriteBytes()` / `StartTransmit()` 必须全部在同一个任务上下文中调用，不得跨任务或在中断中调用（kfifo 为 SPSC 无锁设计，不支持并发）。

#### 3. 发送数据

```cpp
// 方式一：WriteBytes + StartTransmit
uint8_t response[] = {0x01, 0x02, 0x03};
uart->WriteBytes(response, sizeof(response));
uart->StartTransmit();  // 标记 tx_pending，实际发送在下次 Update() 中触发

// 方式二：通过 Print 格式化输出（需要 ascii_tiny_printf 支持）
// 注意：UARTHSBase 未内置 Print，如需此功能请在协议层自行封装
```

`StartTransmit()` 只设置标志位并立即返回，实际的 DMA 配置和启动在下一次 `Update()` 中完成。

#### 4. 接收数据

```cpp
if(uart->GetRxLen() > 0)
{
    uint8_t buf[256];
    uint16_t len = uart->ReadBytes(buf, sizeof(buf), false); // false = 消费性读取
    // 或 peek 模式：uart->ReadBytes(buf, sizeof(buf), true);
    // 处理 buf[0..len-1]...
}
```

#### 5. FIFO 间直接搬运（无中间缓冲区）

配合 `kfifo_t::move_to()` 可以在不分配临时缓冲区的情况下将接收数据直接转发到发送队列：

```cpp
// 回声示例（在 RS485 协议中）
uart->Update();
if(uart->MoveRxToTx(uart->GetRxLen()) > 0)
{
    uart->StartTransmit();
}
```

`MoveRxToTx()` 内部调用 `rx_fifo.move_to(tx_fifo, len)`，处理环形缓冲区的绕圈情况，最多只需两次 `memcpy`。

#### 6. RS485 模式注意事项

- `Init()` 传入 `rs485_mode = true` 后，硬件会自动管理 DE 引脚（发送时高电平，接收时低电平），无需软件干预。
- 发送逻辑内置总线空闲检测，`StartTransmit()` 被调用后不会立即发送，而是等待 NDTR 稳定（即至少一个 `Update()` 周期内没有新字节到达）后才真正启动 TX DMA。
- 若需要极短的响应延迟，可提高 `UpdateMid` 的调用频率（增大 Mid 循环频率），但需确保 `rx_buffer` 足够大以容纳两次调用之间到达的全部字节。

---

### 状态变量说明

`UARTHS` 内部维护以下状态变量：

| 变量 | 类型 | 含义 |
|---|---|---|
| `last_dma_rx_size` | `uint16_t` | 上次 `Update()` 处理到的 DMA 写入位置（相对于 `rx_buffer` 起点） |
| `last_rx_dtcnt` | `uint16_t` | 上次 `Update()` 末尾保存的 RX NDTR 值，用于 RS485 总线空闲判断 |
| `tx_pending` | `bool` | `tx_fifo` 中有数据待搬入 `tx_buffer` |
| `tx_buffer_len` | `uint16_t` | `tx_buffer` 中已就绪、等待 DMA 发送的字节数 |
| `is_tx_busy` | `bool` | TX DMA 正在发送中 |
