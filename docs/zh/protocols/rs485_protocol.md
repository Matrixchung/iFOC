### iFOC - 自定义 RS485 协议

#### 基本约定

* 一主多从，一发一收式通信
* 采用小端设计（LSB），每个字节低位在前，高位在后
* 采用高低电平均衡的包头设计（`0xAA = 10101010b`，`0x55 = 01010101b`）
* 所有数据包均不考虑字节对齐的填充空位，所有数据采用紧密排列。
* 类**时分多址（TDMA）通信**，通信频率完全由主机决定。设总线上有 n 个从机（n ≤ 14），主机在每个循环中，将循环时间等分为 n 份，每一份时间内首先发送出本次循环中主机的控制命令，其余时间等待对应 ID 的从机回复；若分配的这份时间片耗尽，但总线上仍然未收到至少一个字节（完全空闲）则不再继续等待，转而去控制下一个节点。

  ​	例：控制频率 2KHz，10 个节点，则每个节点能够分配到的双向通讯时间片（后文简称时间片或槽位）为 0.00005s，即为 50us。（典型 RS485 收发器的 TSDT / TCDT 均取 120ns，可忽略不计）取波特率为 9Mbps，单字节理论传输时间：1.111us，从上位机发包结束到下位机开始发包的固定硬件开销取 3us。则双向总最大字节数取 42 字节。减去双向固定的 6x2 = 12 字节开销，则双向数据段总字节数为 30 字节。

- 主机需要按以上例子类似的方法，根据：1）波特率；2）控制频率；3）当前已发现的节点数量 这三个因素动态计算出理论上每个槽位的双向数据段总字节数 Nmax，若每个槽位上可预见的通信数据量大于该数量，则可以对用户进行可选的警告，警告有 OVERRUN / OVERFLOW 的风险，实时性无法保证。
- 若某节点没有被分配 ID（处于广播 ID），则禁止响应除了 FRAME_ID = 0（节点发现帧）之外的任何帧（包括广播帧）。
- 总线存在可设定时长的看门狗（watchdog）机制，具体时长由从机在各自配置中设定。当从机处于激活状态（ARMED）时，若超过看门狗超时这段时间仍然未收到任何来自主机的指令，则认为是主机离线，从机自动失活（DISARM）。
- 鉴于大部分 USB 转 RS485 转换器存在输入 / 输出的 FIFO，且 Windows 系统并非实时，因此很有可能出现“嵌入式节点物理发包在槽位时间内，但上位机收包是一整个粘包收取”的情况。因此，无论何时主机收到了包，都应该执行完整的分包解析流程，完整的流程甚至可能包括解析出其他节点的回包，而不止是去解析先前一次发包所期望的回包格式。对于明显要求一对一的回包如 DISCOVERY 包，主机在收到之后应该按携带的 UUID 重新进行一次模数和桶的校验，以确保该回包是针对主机的 DISCOVERY 包的回包。

#### 设计思路

- 主机在实现时，可以维护一个状态机：在每轮循环均按实时频率运行的前提下，按照预先定义的频率（例如每隔 1s 一次新节点发现帧和已发现节点状态轮询帧，剩下的时间片均为实时控制帧）决定当前这轮循环需要运行怎样的逻辑。
- 对每个节点，主机也需要维护各自的状态机；例如正在下发文件给节点或要求节点上传文件时，文件操作优先于控制操作，此时分配到属于该节点的槽位就应当用来传输文件，而非发送实时控制命令。
- 主机需要实现超时离线机制，在持续多少秒（可配置，默认可设为 2s）后都未收到来自某节点的状态回复帧（ID=1）或者实时响应帧，则判定该节点离线。
- 主机需要维护一个 ID - UUID 的 Key - Value 数据库（以文本形式），并在 API 中提供增删改查的接口。主机上线时，应将存在数据库中的节点均认为已知，直接开始轮询状态。若：1）节点超时未在线，则不改动数据库，而是将该 ID 标记为空闲，回到分配池中；2）轮询到该 ID 的节点与数据库中 UUID 不匹配，则更改数据库中该 ID 对应的 UUID 为新 UUID，然后在下一个循环中发送 FID = 0 的分配帧，将原 UUID 的节点分配回广播 ID。

#### 包体结构

除了变长的数据段（最长 255 字节）以外，有固定的 6 Bytes 开销。

|       类型/Type        |                     位/Bits                     |   符号   |                             说明                             |
| :--------------------: | :---------------------------------------------: | :------: | :----------------------------------------------------------: |
|     包头 (2Bytes)      |                  0-15 (16bits)                  |   HEAD   |       主机 -> 从机：0xAA 0x55，从机 -> 主机：0xAA 0x54       |
|   目标信息（1Byte）    |                 16-19（4bits）                  |    ID    | 目标节点 ID，可用范围从 1 到 14<br />0 为主机 ID，一般不使用；15 为广播 ID。 |
|                        |                 20-23（4bits）                  | FRAME_ID | 帧类型 ID，从 0 到 15，可以有 16 种不同的帧，各帧 ID 和格式在后文约定 |
|  数据段长度（1Byte）   |                 24-31（1Byte）                  | DATA_LEN |                  数据段长度 n，n ∈ [0, 255]                  |
|   数据段（n Bytes）    | 32-(31+n * 8)<br />（n > 0）<br />或空（n = 0） |   DATA   |    数据段，格式和长度由各帧类型决定，长度需要与 n 匹配。     |
| CRC16 校验码（2Bytes） |              (32+n * 8)-(47+n * 8)              |   CRC    |            CRC-16-IBM (CRC-16/MODBUS)（不含帧头）            |

#### 帧类型

- ##### FRAME_ID = 0（新节点发现帧）

  由主机按一个长间隔时间（默认 1s）周期性发送，对于实时控制的循环（例如 2KHz），则隔 1s 抽出一次循环用于执行新节点发现逻辑。执行时，为保证充足的余量，将一次循环划分为例如 3 - 5 个长时间片。首先可以明确：对于最大节点数固定（14）的情况，已发现的节点越多，则剩余未被发现的节点 UUID 余数发生碰撞的概率就越小。目标节点 ID 为广播 ID 15，已分配节点 ID 的节点不允许响应探测帧，只允许响应 ID 分配帧（常出现在需要重新分配 ID 的情况），未分配节点 ID 的节点回复时以广播 ID 进行回复。

  主机发送格式：

  |    位/Bits     |  符号   | 范围/类型 |              说明              |
  | :------------: | :-----: | :-------: | :----------------------------: |
  |  0-7（1Byte）  | COMMAND |    0-1    | 0 代表探测帧，1 代表 ID 分配帧 |
  | 8-47（5Bytes） | PAYLOAD |     /     |     视 COMMAND 而定，如下      |

  ​	COMMAND = 0（探测帧）的 PAYLOAD 格式：

  |     位/Bits     |     符号     | 范围/类型 |      说明      |
  | :-------------: | :----------: | :-------: | :------------: |
  | 0-15（2Bytes）  |   MODULUS    | uint16_t  | 见下文探测逻辑 |
  | 16-31（2Bytes） | BUCKET_INDEX | uint16_t  | 见下文探测逻辑 |

  ​	探测逻辑：

  ```
  // ===== 参数（主机侧）=====
  //   K     = 16     细分因子：每个碰撞桶裂为 K 个子桶
  //   M0    = 16     初始模数
  //   M_MAX = 65535  模数上限（MODULUS 为 uint16_t）
  //
  // ===== 节点侧规则（收到探测帧 CMD=0 时）=====
  //   if 已分配 ID:       不回复（已分配节点只响应 ID 分配帧 CMD=1）
  //   if 未分配 ID:
  //       if (UUID % MODULUS == BUCKET_INDEX)  立即以广播 ID=15 回复 12B 状态回复帧，主机通过回复帧的 UUID 进行再次校验，
  //                                            通过后认为节点回复有效
  //       else                               不回复
  //   节点侧无随机延迟、无定时，只有"回 / 不回"二元选择；
  //   总线错开责任完全由主机的分桶细分树承担。
  //
  // ===== 主机侧：分桶细分树 =====
  //   维护待探测桶栈 pending（LIFO，优先深挖最新碰撞桶）：
  //     初始 push (M0,0), (M0,1), …, (M0,15)
  //
  //   对每个待探测桶 (M, b)：
  //     1. 发送探测帧 [广播ID=15, FID=0, CMD=0, MODULUS=M, BUCKET_INDEX=b]
  //     2. 在该时间片内收包（最长等至 2 个时间片，见"基本约定"）
  //     3. 三态判据：
  //          空   : 总线完全空闲（0 字节）            → 桶内 0 节点，跳过
  //          命中 : 恰好 1 个有效 12B 状态回复帧        → 桶内 1 节点，取 UUID 入待分配队列
  //                 （帧头 0xAA 0x54、CRC 通过、ID=15）
  //          碰撞 : 有字节但有效帧数 ≠ 1              → 桶内 ≥2 节点，细分：
  //                 （CRC 失败 / 帧头错乱 / ≥2 有效帧）
  //                 M' = (M*K ≤ M_MAX) ? M*K : (≤ M_MAX 的最大 M 的倍数)
  //                 if M' > M:  push (M', b*(M'/M)) … (M', b*(M'/M)+(M'/M)-1)
  //                 else:       已达叶节点，留待下个发现轮重试同一桶
  //
  //   模数序列典型为 16 → 256 → 4096 → 61440（最后一级因 uint16 上限，
  //   取 4096 的最大可用倍数 61440）。叶节点处残余碰撞概率约
  //   C(14,2)/61440 ≈ 0.15%，仅靠下个发现轮重试，不做额外兜底。
  //
  // ===== 主机侧：ID 分配与确认（配合 CMD=1）=====
  //   对待分配队列中的 UUID = X：
  //     1. 从空闲 ID 池 {1..14} 取 new_id
  //     2. 发送分配帧 [广播ID=15, FID=0, CMD=1, UUID=X, ID=new_id]
  //     3. UUID 匹配的节点立即切换到 new_id，并以【新 ID=new_id】回复 12B 状态回复帧
  //     4. 主机收到"来自 new_id 且 UUID=X"的回复 → 确认成功：
  //          assigned{X} = new_id，纳入 FID=1 轮询表，该节点退出探测池
  //     5. 未收到确认 → 下个发现轮重发分配帧
  //        （幂等：已切换的节点收到重复分配帧仍会以 new_id 再回复）
  //   每分配一个节点，探测池缩小，剩余碰撞概率单调下降。
  //
  // ===== 上电加速窗口（主机可选实现）=====
  //   主机可在启动后前 N 秒（如 N=2 s）将发现轮间隔由默认 1 s 临时缩短
  //   （如 100 ms），以尽快扫完初始 16 桶、缩短冷启动发现延迟；
  //   窗口结束后恢复 1 s，进入"巡检有无新节点上线"的低频节奏。
  //   此为可选优化；不实现时直接以 1 s 节奏运行亦完全正确。
  //
  // ===== 已知 UUID 的重新发现（节点重启 / 掉线恢复）=====
  //   UUID 由硬件决定、重启不变。对已知 UUID 的节点重新上线，
  //   主机可跳过整棵分桶树，直接发分配帧（CMD=1）恢复其 ID，无需重新探测；
  //   若连续数次无回复则判定物理离线。
  ```

  ​	COMMAND = 1（ID 分配帧）的 PAYLOAD 格式：

  |    位/Bits     | 符号 | 范围/类型 |                        说明                        |
  | :------------: | :--: | :-------: | :------------------------------------------------: |
  | 0-31（4Bytes） | UUID | uint32_t  |   将要分配新 ID 的目标节点的 UUID，必须完全匹配    |
  | 32-35（4bits） |  ID  |   1-15    | 将要为目标节点分配的新 ID，15 代表回到未分配状态。 |
  | 36-39（4bits） | 保留 |     /     |                         /                          |

  收到 ID 分配帧且 UUID 完全匹配的节点应当立即切换到新 ID 进行回复。

  从机统一回复格式（12 Bytes）：

  |     位/Bits     |    符号    | 范围/类型 |                             说明                             |
  | :-------------: | :--------: | :-------: | :----------------------------------------------------------: |
  | 0-31（4Bytes）  | UPTIME_SEC | uint32_t  | 代表节点自启动以来经过的秒数，不允许溢出，如果回滚则证明该节点发生过重启。 |
  | 32-39（1Byte）  |   HEALTH   |  uint8_t  | 0 代表 OK，其余可自定。可使用 error_count 填充该字段，代表有多少个错误。 |
  | 40-47（1Byte）  |    MODE    |  uint8_t  | OPERATIONAL = 0，INITIALIZATION = 1，MAINTENANCE = 2，SOFTWARE_UPDATE = 3，其余保留 |
  | 48-55（1Byte）  |  SUB_MODE  |  uint8_t  | 对于电机节点，与 STATE 相同。[IDLE, ..., OPEN_LOOP_VELOCITY_CONTROL]，共 9 个状态 |
  | 56-63（1Byte）  |    VSSC    |  uint8_t  |  Vendor Specific Status Code (VSSC)，可用于指示固件更新进度  |
  | 64-95（4Bytes） |    UUID    | uint32_t  | 从机的 32 位唯一 ID，是从机的唯一身份标识，在整个生命周期内不允许重复、不允许变化。 |

- **FRAME_ID = 1（已发现节点状态轮询帧）**

  由主机按一个长间隔时间（默认 1s）周期性发送，设主机已发现的节点有 n 个，则执行时将整个循环时间分为 n 份，对每个槽位，主机应按照已发现的节点 ID 进行发送，数据段长度为 0。各节点应当以与 FRAME_ID = 0 从机统一回复格式相同的格式进行回复。

- **FRAME_ID = 2（节点信息获取帧）**

  主机应当在发现已分配 ID 的新节点（非广播节点）或者已发现的节点下线再重新上线时尽快发送此消息询问节点信息。若主机注册表中某节点一直无回复（或主机注册表中暂无该节点对应的信息），则主机需要在每个实时控制循环中都询问该信息。主机发送的消息数据段长度为 0。

  在从机的实际实现中，由于该帧的回复涉及到大量的 memcpy 和字符串操作，因此通常回复速度较慢，容易拖慢实时进程。并且，这些数据在运行中不允许更改，因此可以牺牲一部分内存空间，在程序初始化时就把这些信息填写好，然后一次性发送 buffer。

  从机回复格式（17-30 Bytes）：

  |               位/Bits                |     符号     | 范围/类型 |                             说明                             |
  | :----------------------------------: | :----------: | :-------: | :----------------------------------------------------------: |
  |            0-31（4Bytes）            |     UUID     | uint32_t  |    根据 MCU 芯片内部的唯一码计算出的硬件 UUID，32 位格式     |
  |            32-39（1Byte）            | SW_VER_MAJOR |  uint8_t  |     软件版本（大版本），以 iFOC 的版本命名方式来说是年份     |
  |           40-71（4Bytes）            | SW_VER_MINOR | uint32_t  | 软件版本（小版本），四个字节（按小端序）分别为十六进制的 MM-DD-HH-mm。<br />整体版本表示方法例：26.06181700 代表 2026 年 6 月 18 日 17:00 编译的二进制文件。 |
  |           72-135（8Bytes）           |   SW_CRC64   | uint64_t  | 按 CRC64-ECMA-182 格式计算出的固件文件校验值（APP 计算的只包括 APP 区域，BL 亦然） |
  | 136-(136+n*8-1)<br />（13Bytes MAX） |  NODE_NAME   |  string   | 节点名，最小为空，最大 13 字节。若小于 13 字节，则可以在字符串末尾加入 '\0' 终止。 |

- ##### FRAME_ID = 3（执行操作码帧，Execute OpCode）

  主机/从机固定发包和收包均为 9 Bytes，主机发包：

  |    位/Bits     |   符号   | 范围/类型 |              说明              |
  | :------------: | :------: | :-------: | :----------------------------: |
  |  0-7（1Byte）  |  OPCODE  |  uint8_t  |     操作码，具体定义见后续     |
  | 8-71（8Bytes） | ARGUMENT | uint64_t  | 参数，其定义随不同操作码而不同 |

  从机回包：

  |    位/Bits     |   符号   | 范围/类型 |              说明              |
  | :------------: | :------: | :-------: | :----------------------------: |
  |  0-7（1Byte）  |  STATUS  |  uint8_t  |            执行状态            |
  | 8-71（8Bytes） | ARGUMENT | uint64_t  | 参数，其定义随不同操作码而不同 |

  执行状态：0x00 不支持此操作码，0x01 失败，0x02 成功

  **操作码定义**：

  |                 操作码                 |                   下发参数                    | 返回参数（若 STATUS = 0x02 有效） |
  | :------------------------------------: | :-------------------------------------------: | :-------------------------------: |
  |                0 / 保留                |                       /                       |                 /                 |
  |              1 / 保存参数              |                   domain_id                   |     表示成功保存的 domain_id      |
  |              2 / 擦除参数              |                   domain_id                   |     表示成功擦除的 domain_id      |
  |              3 / 重启节点              | 0 - 正常重启，1 - 重启到 Bootloader（如果有） |                 /                 |
  |          4 / 切换电机蜂鸣状态          |                       /                       |                 /                 |
  |         5 / 获取电机当前错误值         |                       /                       |    (uint64_t)motor->GetError()    |
  |   6 / 按给定 mask 清除电机当前错误值   |             (uint64_t)error_mask              |    (uint64_t)motor->GetError()    |
  | 7 / 设置电机当前运行状态（MotorState） |                 request_state                 |          response_state           |

- ##### FRAME_ID = 4（Get/Set 参数服务帧，类似 DroneCAN GetSet / CANopen SDO）

  适用：主机读取、修改、枚举和持久化指定节点的参数。FID=4 面向配置与诊断，不是实时控制帧；由于 RS485 总线采用主机驱动的实时 TDMA 时间片，参数服务不得假设任意一次请求 / 回复都能够塞进单个时间片。

  一次参数服务由两层组成：

  - **参数语义层**：定义 GET / SET / SAVE / ERASE / LOAD_DEFAULT、枚举、查询元信息等参数操作；
  - **传输会话层**：负责小请求内联传输，以及长请求 / 长回复的分片、重传、幂等和超时。

  一次 FID=4 事务的完整生命周期为：**生成参数请求对象 → 选择 INLINE 或长请求上传 → 从机执行参数语义 → INLINE_RSP 或 RSP_READY → 必要时读取长回复 → DONE / ERROR / ABORT / 超时释放会话**。会话以主机为每个 `(bus_id, node_id)` 独立递增生成的 `SESSION_ID` 标识；从机用 `SESSION_ID` 区分当前活动事务、重复包和错误包。

  FID=4 的主体功能围绕以下五类操作展开，后文所有包结构和状态机均服务于这五条主流程：

  1. 上位机初始轮询指定节点的所有参数名、类型、访问属性和当前值；
  2. 上位机按 index 查询指定参数值；
  3. 上位机按 name 查询指定参数值；
  4. 上位机按 index 修改指定参数值；
  5. 上位机按 name 修改指定参数值。

  其中按 index 查询 / 修改是实时性最好的路径；按 name 查询 / 修改主要用于参数导入、缓存失效或跨固件版本校验；初始轮询只应在节点上线、`SW_CRC64` 变化或本地缓存缺失时执行。参数服务对某个节点的优先级低于文件传输，高于调试示波器和普通低频状态查询。除非主机明确配置，否则参数事务不应以实时控制频率执行，建议默认以 10Hz - 50Hz 的低频节奏推进。

  **时序预算**

  主机在决定 FID=4 单次可发送 / 接收的数据长度前，必须按当前总线状态计算该节点时间片的双向数据段预算：

  ```
  slot_us          = 1e6 / loop_hz / online_node_count
  slot_total_bytes = floor((slot_us - turn_us) * baud_rate / bits_per_byte / 1e6)
  nmax_data        = slot_total_bytes - 6 * 2
  ```

  其中：

  - `loop_hz` 为实时循环频率；
  - `online_node_count` 为当前参与轮询的在线节点数量；
  - `baud_rate` 为串口波特率；
  - `bits_per_byte` 对 8N1 串口通常取 10；
  - `turn_us` 为主机发送结束到从机开始回复之间的固定延迟，建议默认取 3us，并允许配置；
  - `6 * 2` 为主机请求包和从机回复包各自 6 Bytes 固定包开销。

  因此任何一个 FID=4 时间片都应满足：

  ```
  host_data_len + device_data_len <= nmax_data
  ```

  若该条件无法满足最小请求 / 回复长度，主机应延后参数事务，并向上层提示 OVERRUN / OVERFLOW 风险，而不是强行发送。

  例：`loop_hz = 2000Hz`、`online_node_count = 10`、`baud_rate = 9Mbps`、`turn_us = 3us`：

  ```
  slot_us          = 50us
  slot_total_bytes = floor((50 - 3) * 9e6 / 10 / 1e6) = 42 Bytes
  nmax_data        = 42 - 12 = 30 Bytes
  ```

  即该节点在一个时间片内，主机 DATA 与从机 DATA 相加建议不超过 30 Bytes。

  **参数模型**

  本协议以 **domain.index.name.value** 的形式定义参数：

  - `domain`：参数作用域，`uint8_t`，范围 0-255。例如板级参数可定义为 `domain_id = 0`，电机参数可定义为 `domain_id = 1`。
  - `index`：参数在该 domain 下的有序索引，`uint8_t`，范围 0-255。
  - `name`：参数名，ASCII string，最大 31 Bytes，用于上位机显示和跨版本校验。由于参数已由 `domain_id` 区分作用域，`NAME` 不应携带 `board.`、`motor.` 等作用域前缀。
  - `value`：参数值，采用本节定义的 `TypedValue` 编码。

  下位机应维护一个参数注册表。每个参数建议包含：

  | 字段 | 类型 | 说明 |
  | :--- | :--- | :--- |
  | DOMAIN_ID | uint8 | 参数作用域 |
  | INDEX | uint8 | 作用域内索引 |
  | NAME | string[<=31] | 参数名，建议同一 domain 内唯一 |
  | TYPE | uint3 | 参数值类型 |
  | ACCESS_FLAGS | uint8 | 读写、只读、易失、持久化等标志 |
  | VALUE | TypedValue | 当前值 |
  | DEFAULT_VALUE | TypedValue | 默认值，可选 |
  | MIN_VALUE | TypedValue | 最小值，仅数值类型适用，可选 |
  | MAX_VALUE | TypedValue | 最大值，仅数值类型适用，可选 |

  同一 `SW_CRC64` 下，`domain.index` 到参数的映射必须稳定。上位机可以缓存 `(UUID, SW_CRC64, DOMAIN_ID) -> name/index/type`，从而在后续读写中优先使用更短的 `domain.index` 访问。若固件修改了参数表的排列、名称、类型、访问属性或默认 / 限幅信息，则该固件的 `SW_CRC64` 必须变化；若 `SW_CRC64` 未变化但参数表结构变化，视为违反协议。若上位机使用 name 访问且拥有可靠缓存，建议置位 `FLAGS.INDEX_HINT_VALID` 并同时携带 index hint；下位机应优先校验 `domain + name`，若 name 与有效 index hint 不一致，应返回错误。

  **TypedValue 编码**

  `TypedValue` 由 1 Byte 类型长度头和后续值组成：

  | 位/Bits | 符号 | 范围/类型 | 说明 |
  | :---: | :---: | :---: | :--- |
  | 0-2 | VALUE_TYPE | 0-7 | 值类型 |
  | 3-7 | VALUE_LEN | 0-31 | 值长度，单位 Byte |

  VALUE_TYPE 定义：

  | VALUE_TYPE | 名称 | VALUE_LEN | 说明 |
  | :---: | :--- | :---: | :--- |
  | 0 | EMPTY | 0 | 空值，表示未设置或不存在 |
  | 1 | BOOLEAN | 1 | 0=false，非 0=true |
  | 2 | FLOAT32 | 4 | IEEE-754 float32，小端 |
  | 3 | UINT64 | 8 | uint64，小端 |
  | 4 | INT64 | 8 | int64，小端 |
  | 5 | STRING | 0-31 | ASCII string，不强制 `\0` 结尾 |
  | 6 | BYTES | 0-31 | 原始字节，保留给后续扩展 |
  | 7 | RESERVED | / | 保留 |

  下位机收到 SET 请求时必须校验目标参数类型、长度、读写权限和取值范围。SET 的回复必须返回实际写入后的值；若下位机进行了限幅或拒绝写入，上位机必须能从回复中看到实际值或错误码。

  **包布局总览**

  FID=4 与文件下载帧一样，DATA 段从一个固定公共头开始；公共头之后的字段由 `COMMAND` 决定。主机负责在发送前计算本槽位预算，并在请求头中填入 `MAX_RSP_LEN`。从机只根据当前收到的 `MAX_RSP_LEN` 判断能否直接回复：若完整 `PARAM_RESPONSE` 加上 5 Bytes 回复公共头可以放入该长度，则回 `INLINE_RSP`；否则缓存完整回复并回 `RSP_READY`，等待主机分片读取。

  **FID=4 请求公共头**

  所有主机到从机的 FID=4 DATA 段均以 4 Bytes 公共头起始：

  | 位/Bits | 符号 | 类型 | 说明 |
  | :---: | :---: | :---: | :--- |
  | 0-7 | COMMAND | uint8 | 传输层命令 |
  | 8-15 | SESSION_ID | uint8 | 参数事务会话 ID，0 保留 |
  | 16-23 | FLAGS | uint8 | 命令标志，未使用位填 0 |
  | 24-31 | MAX_RSP_LEN | uint8 | 本次允许从机直接回复的最大 DATA_LEN |

  规则如下：

  - `SESSION_ID = 0` 保留，不得用于活动事务。
  - 主机按 `(bus_id, node_id)` 独立递增，范围 1-255，回绕后继续从 1 开始。
  - 同一节点同一时刻只允许一个活动 FID=4 参数事务。
  - 下位机应为活动参数事务设置超时，建议 200ms - 1000ms；超时后释放会话缓存。
  - 主机不得在旧事务未 DONE / ABORT / 超时前复用同一节点的 SESSION_ID。
  - `MAX_RSP_LEN` 由主机按 `nmax_data - host_data_len` 计算，并限制到 255。若计算值小于从机回复公共头长度 5 Bytes，主机不应发送本次参数事务。
  - 从机生成回复对象后，若 `5 + param_response_len <= MAX_RSP_LEN`，则可直接回复 INLINE_RSP；否则回复 RSP_READY，并等待主机后续用 READ_RSP_CHUNK 拉取长回复。

  FLAGS 定义：

  | Bit | 名称 | 说明 |
  | :---: | :--- | :--- |
  | 0 | INDEX_HINT_VALID | 仅 GET_BY_NAME / SET_BY_NAME 使用。置 1 表示请求对象中的 INDEX 是有效 hint，从机必须校验 name 与 index 一致；置 0 表示 INDEX 字段忽略 |
  | 1-7 | RESERVED | 保留，发送方填 0，接收方忽略 |

  COMMAND 定义：

  | COMMAND | 名称 | 典型用途 | 说明 |
  | :---: | :--- | :--- | :--- |
  | 0x00 | INLINE_REQ | 短 GET / SET / GET_COUNT | 参数请求对象直接跟在公共头之后；请求和回复预计可在一个槽位内完成 |
  | 0x01 | BEGIN_REQ | 长 SET_BY_NAME、长字符串写入 | 开始上传长请求，声明完整请求长度和 `PAYLOAD_CRC16` |
  | 0x02 | WRITE_REQ_CHUNK | 长请求分片 | 上传完整参数请求对象的一段数据 |
  | 0x03 | EXEC_REQ | 长请求执行 | 长请求上传完成后请求从机校验 CRC 并执行 |
  | 0x04 | READ_RSP_CHUNK | 枚举元信息、长字符串读取 | 读取从机缓存的长回复分片 |
  | 0x05 | ABORT | 用户取消或超时恢复 | 中止当前参数会话并释放从机会话缓存 |
  | 0x06-0xFF | RESERVED | / | 保留 |

  `PAYLOAD_CRC16` 仅用于长请求上传流程，校验对象是完整的 `PARAM_REQUEST` 字节序列，用于防止长请求在分片接收、重复包处理或缓存覆盖后被错误重组成另一个合法参数请求。算法为 CRC-16-IBM (CRC-16/MODBUS)。INLINE_REQ 不额外携带 `PAYLOAD_CRC16`。

  **FID=4 回复公共头**

  所有从机到主机的 FID=4 DATA 段均以 5 Bytes 公共头起始：

  | 位/Bits | 符号 | 类型 | 说明 |
  | :---: | :---: | :---: | :--- |
  | 0-7 | STATUS | uint8 | 传输层状态 |
  | 8-15 | SESSION_ID | uint8 | 回显主机 SESSION_ID |
  | 16-31 | INFO | uint16 | 含义随 STATUS 而定 |
  | 32-39 | INFO2 | uint8 | 含义随 STATUS 而定 |

  STATUS 定义：

  | STATUS | 名称 | INFO / INFO2 说明 |
  | :---: | :--- | :--- |
  | 0x00 | BUSY | 从机正在处理；主机下一次该节点参数槽位重试或轮询 |
  | 0x01 | ACK | 分片已接收；`INFO = NEXT_OFFSET` |
  | 0x02 | INLINE_RSP | 本帧携带完整参数回复；后续跟随回复对象 |
  | 0x03 | RSP_READY | 回复已生成但过长；`INFO + (INFO2 << 16) = RSP_TOTAL_LEN` |
  | 0x04 | RSP_CHUNK | 本帧携带长回复分片；后续跟随 OFFSET / TOTAL_LEN / DATA |
  | 0x05 | DONE | 事务完成 |
  | 0x06 | ERROR | `INFO = ERROR_CODE` |
  | 0x07-0xFF | RESERVED | 保留 |

  ERROR_CODE 定义：

  | ERROR_CODE | 名称 | 说明 |
  | :---: | :--- | :--- |
  | 0x01 | INVALID_COMMAND | COMMAND 非法 |
  | 0x02 | INVALID_SESSION | SESSION_ID 非法或与当前活动会话不匹配 |
  | 0x03 | BAD_OFFSET | 分片 OFFSET 不等于期望值 |
  | 0x04 | BAD_CRC | 组包 CRC 校验失败 |
  | 0x05 | PAYLOAD_TOO_LONG | 请求或回复长度超出实现上限 |
  | 0x06 | PARAM_NOT_FOUND | 参数不存在 |
  | 0x07 | TYPE_MISMATCH | 参数类型或长度不匹配 |
  | 0x08 | RANGE_ERROR | 参数值超出范围且不可限幅 |
  | 0x09 | READ_ONLY | 目标参数只读 |
  | 0x0A | BUSY_NVM | NVM 正在保存或擦除 |
  | 0x0B | NOT_ALLOWED_WHILE_ARMED | 当前 ARMED 状态下不允许执行 |
  | 0x0C | DOMAIN_NOT_FOUND | DOMAIN_ID 不存在 |
  | 0x0D | NAME_INDEX_MISMATCH | NAME 与 INDEX hint 不一致 |

  **参数请求对象**

  INLINE_REQ 或长请求组包后的完整请求对象格式如下：

  | 偏移 | 符号 | 类型 | 说明 |
  | :---: | :---: | :---: | :--- |
  | 0 | OP | uint8 | 参数语义操作 |
  | 1 | DOMAIN_ID | uint8 | 参数作用域 |
  | 2 | INDEX | uint8 | 参数索引；按 name 访问时作为 index hint |
  | 3 | FIELD_MASK | uint8 | 请求从机在回复中包含哪些字段 |
  | 4 | VALUE_TYPE_LEN | uint8 | TypedValue 类型长度头；GET / EXEC 可为 EMPTY |
  | 5.. | VALUE | 0-31 Bytes | TypedValue 的值数据 |
  | 5+VALUE_LEN | NAME_LEN | uint8，可选 | 仅 GET_BY_NAME / SET_BY_NAME 或需要 name 校验时存在 |
  | 6+VALUE_LEN.. | NAME | 0-31 Bytes，可选 | ASCII 参数名 |

  OP 定义：

  | OP | 名称 | 说明 |
  | :---: | :--- | :--- |
  | 0x00 | GET_COUNT | 查询某 domain 的参数数量；INDEX / VALUE / NAME 忽略 |
  | 0x01 | GET_BY_INDEX | 按 domain.index 读取参数 |
  | 0x02 | GET_BY_NAME | 按 domain.name 读取参数，INDEX 为 hint |
  | 0x03 | SET_BY_INDEX | 按 domain.index 写参数 |
  | 0x04 | SET_BY_NAME | 按 domain.name 写参数，INDEX 为 hint |
  | 0x05 | EXEC_SAVE | 将当前参数保存到非易失存储 |
  | 0x06 | EXEC_ERASE | 擦除参数存储并恢复默认值 |
  | 0x07 | EXEC_LOAD_DEFAULT | 仅在 RAM 中加载默认值，不立即保存 |
  | 0x08-0xFF | RESERVED | 保留 |

  FIELD_MASK 定义：

  | Bit | 名称 | 说明 |
  | :---: | :--- | :--- |
  | 0 | CURRENT_VALUE | 回复包含当前值 |
  | 1 | NAME | 回复包含参数名 |
  | 2 | DEFAULT_VALUE | 回复包含默认值 |
  | 3 | MIN_VALUE | 回复包含最小值，仅数值类型适用 |
  | 4 | MAX_VALUE | 回复包含最大值，仅数值类型适用 |
  | 5 | ACCESS_FLAGS | 回复包含访问标志 |
  | 6 | PARAM_TYPE | 回复包含参数声明类型 |
  | 7 | RESERVED | 保留 |

  ACCESS_FLAGS 定义：

  | Bit | 名称 | 说明 |
  | :---: | :--- | :--- |
  | 0 | READABLE | 参数可读取 |
  | 1 | WRITABLE | 参数可写入 |
  | 2 | VOLATILE | 参数为易失参数，掉电不保存 |
  | 3 | PERSISTENT | 参数可通过 EXEC_SAVE 保存到非易失存储 |
  | 4 | WRITE_REQUIRES_DISARMED | 仅允许在非 ARMED 状态下写入 |
  | 5 | RESTART_REQUIRED | 修改后需要重启或重新初始化相关模块才完全生效 |
  | 6-7 | RESERVED | 保留 |

  **参数回复对象**

  INLINE_RSP 或长回复组包后的完整回复对象格式如下：

  | 偏移 | 符号 | 类型 | 说明 |
  | :---: | :---: | :---: | :--- |
  | 0 | OP_ECHO | uint8 | 回显请求 OP |
  | 1 | RESULT_CODE | uint8 | 参数语义层结果，0 表示成功 |
  | 2 | DOMAIN_ID | uint8 | 回显 DOMAIN_ID |
  | 3 | INDEX | uint8 | 实际参数索引 |
  | 4 | FIELD_MASK_PRESENT | uint8 | 本回复实际包含的字段 |
  | 5 | ACCESS_FLAGS | uint8，可选 | FIELD_MASK_PRESENT bit5 置位时存在 |
  | 后续 | PARAM_TYPE | uint8，可选 | FIELD_MASK_PRESENT bit6 置位时存在，低 3 bits 为 VALUE_TYPE，其余保留 |
  | 后续 | CURRENT_VALUE | TypedValue，可选 | FIELD_MASK_PRESENT bit0 置位时存在 |
  | 后续 | DEFAULT_VALUE | TypedValue，可选 | FIELD_MASK_PRESENT bit2 置位时存在 |
  | 后续 | MIN_VALUE | TypedValue，可选 | FIELD_MASK_PRESENT bit3 置位时存在 |
  | 后续 | MAX_VALUE | TypedValue，可选 | FIELD_MASK_PRESENT bit4 置位时存在 |
  | 后续 | NAME_LEN + NAME | uint8 + string，可选 | FIELD_MASK_PRESENT bit1 置位时存在 |

  RESULT_CODE 定义：

  | RESULT_CODE | 名称 | 说明 |
  | :---: | :--- | :--- |
  | 0x00 | OK | 成功 |
  | 0x01 | NOT_FOUND | 参数不存在 |
  | 0x02 | TYPE_MISMATCH | 类型不匹配 |
  | 0x03 | RANGE_CLAMPED | 已限幅并写入，回复中返回实际值 |
  | 0x04 | RANGE_REJECTED | 超范围且拒绝写入 |
  | 0x05 | READ_ONLY | 只读 |
  | 0x06 | NAME_INDEX_MISMATCH | NAME 与 INDEX hint 不一致 |
  | 0x07 | NOT_ALLOWED | 当前状态不允许 |
  | 0x08 | NVM_ERROR | NVM 保存 / 擦除失败 |

  对 SET 操作，从机在成功或限幅时必须返回实际参数值。若写入失败，建议返回原值或 EMPTY。

  **五类主体功能流程**

  下面以 `REQ_HDR = COMMAND + SESSION_ID + FLAGS + MAX_RSP_LEN`、`RSP_HDR = STATUS + SESSION_ID + INFO + INFO2` 表示 FID=4 传输层公共头。所有请求均不得发送给广播 ID；未分配 ID 的节点不得响应。五类主体功能均优先尝试 INLINE_REQ；只有当请求对象或回复对象无法满足本槽位 `MAX_RSP_LEN` 时，才进入长请求上传或长回复读取。

  1. **初始轮询指定节点的所有参数名、类型和值**

     主机应先通过 `FRAME_ID = 2` 获取该节点 `UUID`、`SW_CRC64` 和节点名等信息。若 `(UUID, SW_CRC64, DOMAIN_ID)` 命中本地参数表缓存，则可跳过参数名和类型的完整枚举，只按 index 读取当前值；若未命中缓存，则执行完整轮询。完整轮询以 domain 为单位进行：主机若已知该产品的 domain 列表，则只轮询这些 domain；若未知，则可在后台低频扫描 `DOMAIN_ID = 0..255`，对返回 `DOMAIN_NOT_FOUND` 的 domain 直接跳过，不得在实时高负载路径中一次性硬扫全部 domain。

     主机首先查询某个 domain 的参数数量：

     | 方向 | FID=4 DATA | 说明 |
     | :--- | :--- | :--- |
     | 主机 -> 从机 | `INLINE_REQ, sid, flags, max_rsp_len` + `GET_COUNT, domain, 0, CURRENT_VALUE, EMPTY` | 查询参数数量 |
     | 从机 -> 主机 | `INLINE_RSP, sid, len, 0` + `GET_COUNT, OK, domain, 0, CURRENT_VALUE, UINT64(count)` | 返回参数数量 |

     若 `count = 0`，该 domain 没有参数，主机继续下一个 domain。若 `count > 0`，主机对 `index = 0 .. count - 1` 逐个查询参数名、声明类型、访问标志和当前值：

     | 方向 | FID=4 DATA | 说明 |
     | :--- | :--- | :--- |
     | 主机 -> 从机 | `INLINE_REQ, sid, flags, max_rsp_len` + `GET_BY_INDEX, domain, index, NAME + PARAM_TYPE + ACCESS_FLAGS + CURRENT_VALUE, EMPTY` | 按 index 枚举 |
     | 从机 -> 主机 | `INLINE_RSP, sid, len, 0` + `GET_BY_INDEX, OK, domain, index, present_mask, access_flags, param_type, current_value, name_len, name` | 返回该参数元信息和当前值 |

     若 `5 + PARAM_RESPONSE_LEN > MAX_RSP_LEN`，从机不得直接回 INLINE_RSP，而应回复：

     | 方向 | FID=4 DATA | 说明 |
     | :--- | :--- | :--- |
     | 从机 -> 主机 | `RSP_READY, sid, total_len_low16, total_len_high8` | 完整回复已缓存 |
     | 主机 -> 从机 | `READ_RSP_CHUNK, sid, flags, max_rsp_len, offset, max_len` | 拉取回复分片 |
     | 从机 -> 主机 | `RSP_CHUNK, sid, offset, 0, total_len, chunk_data` | 返回回复分片 |

     主机按 offset 递增读取，直到获得完整 `PARAM_RESPONSE`。当所有目标 domain 均完成 `GET_COUNT` 和 index 枚举后，主机以 `(UUID, SW_CRC64, DOMAIN_ID)` 缓存 `index/name/type/access_flags` 映射；后续刷新当前值时优先使用按 index 查询。该流程适合初始建表，不应在实时控制高负载时频繁执行。

  2. **按 index 查询指定参数值**

     该流程是最短的参数读取路径，适合 UI 刷新、脚本读取和缓存命中后的周期性状态确认。主机应先确认本地缓存中的 `SW_CRC64` 与节点当前 `SW_CRC64` 一致；若不一致，应回到初始轮询流程重建映射。

     | 方向 | FID=4 DATA | 说明 |
     | :--- | :--- | :--- |
     | 主机 -> 从机 | `INLINE_REQ, sid, flags, max_rsp_len` + `GET_BY_INDEX, domain, index, CURRENT_VALUE, EMPTY` | 查询当前值 |
     | 从机 -> 主机 | `INLINE_RSP, sid, len, 0` + `GET_BY_INDEX, OK, domain, index, CURRENT_VALUE, current_value` | 返回当前值 |

     若参数不存在，从机回复 `RESULT_CODE = NOT_FOUND`，`CURRENT_VALUE = EMPTY`。若回复值为长 string 且超过 `MAX_RSP_LEN`，从机改回 RSP_READY，主机使用 READ_RSP_CHUNK 读取完整回复。收到 `INLINE_RSP` 且 `RESULT_CODE = OK` 后，本次查询结束；收到 `ERROR` 或超时后，主机应向上层报告失败，不应自动改用 name 猜测其它参数。

  3. **按 name 查询指定参数值**

     该流程用于参数缓存失效、用户手动输入参数名或导入参数文件时的安全匹配。主机若有可靠缓存，应置位 `FLAGS.INDEX_HINT_VALID` 并携带 `INDEX` 作为 hint；从机优先按 `DOMAIN_ID + NAME` 查找，并校验 name 与 index hint 是否匹配。若主机没有 index hint，应清除 `FLAGS.INDEX_HINT_VALID`，此时从机忽略 `INDEX` 字段，只按 name 查找并在回复中返回实际 index。

     请求较短时使用 INLINE_REQ：

     | 方向 | FID=4 DATA | 说明 |
     | :--- | :--- | :--- |
     | 主机 -> 从机 | `INLINE_REQ, sid, flags, max_rsp_len` + `GET_BY_NAME, domain, index_hint, CURRENT_VALUE, EMPTY, name_len, name` | 按 name 查询 |
     | 从机 -> 主机 | `INLINE_RSP, sid, len, 0` + `GET_BY_NAME, OK, domain, actual_index, CURRENT_VALUE, current_value` | 返回实际 index 和当前值 |

     若 `NAME` 使请求对象超过单槽位预算，主机使用长请求上传：

     | 阶段 | 主机 -> 从机 | 从机 -> 主机 |
     | :--- | :--- | :--- |
     | BEGIN | `BEGIN_REQ, sid, flags, max_rsp_len, total_len, payload_crc16` | `ACK, sid, 0, 0` |
     | WRITE | `WRITE_REQ_CHUNK, sid, flags, max_rsp_len, offset, chunk_data` | `ACK, sid, next_offset, 0` |
     | EXEC | `EXEC_REQ, sid, flags, max_rsp_len` | `INLINE_RSP` 或 `RSP_READY` |

     从机在 EXEC_REQ 阶段校验 `received_len == total_len` 和 `CRC16(received_payload) == PAYLOAD_CRC16`。若 name 不存在，回复 `RESULT_CODE = NOT_FOUND`；若 name 与 index hint 不一致，回复 `RESULT_CODE = NAME_INDEX_MISMATCH`。

  4. **按 index 修改指定参数值**

     该流程是最短的参数写入路径，适合缓存命中后的常规配置写入。SET 默认只修改 RAM 中当前值，不应隐式保存到非易失存储；持久化需由主机另行发送 `EXEC_SAVE`。

     | 方向 | FID=4 DATA | 说明 |
     | :--- | :--- | :--- |
     | 主机 -> 从机 | `INLINE_REQ, sid, flags, max_rsp_len` + `SET_BY_INDEX, domain, index, CURRENT_VALUE, value_type_len, value` | 写入新值 |
     | 从机 -> 主机 | `INLINE_RSP, sid, len, 0` + `SET_BY_INDEX, result, domain, index, CURRENT_VALUE, actual_value` | 返回实际值 |

     从机必须校验 domain、index、参数读写权限、`VALUE_TYPE / VALUE_LEN`、当前节点状态和 min/max 范围。若参数只读，回复 `READ_ONLY`；若类型不匹配，回复 `TYPE_MISMATCH`；若超范围但允许限幅，回复 `RANGE_CLAMPED` 并返回限幅后的 `actual_value`；若超范围且拒绝写入，回复 `RANGE_REJECTED` 并建议返回原值。主机收到成功或限幅回复后，应立即以 `actual_value` 更新 UI 和本地缓存中的当前值；若需要掉电保存，必须后续显式发送 `EXEC_SAVE`。

  5. **按 name 修改指定参数值**

     该流程用于跨版本参数导入或必须用名称确认目标参数的写入。主机若有可靠缓存，应置位 `FLAGS.INDEX_HINT_VALID` 并同时携带 index hint；从机按 `DOMAIN_ID + NAME` 查找目标参数，再在 hint 有效时校验 index、类型、权限、范围和节点状态。若 name 与有效 index hint 不一致，从机不得按 index 写入，应返回 `NAME_INDEX_MISMATCH`。

     请求较短时使用 INLINE_REQ：

     | 方向 | FID=4 DATA | 说明 |
     | :--- | :--- | :--- |
     | 主机 -> 从机 | `INLINE_REQ, sid, flags, max_rsp_len` + `SET_BY_NAME, domain, index_hint, CURRENT_VALUE, value_type_len, value, name_len, name` | 按 name 写入 |
     | 从机 -> 主机 | `INLINE_RSP, sid, len, 0` + `SET_BY_NAME, result, domain, actual_index, CURRENT_VALUE, actual_value` | 返回实际 index 和实际值 |

     若请求较长，例如 `value` 为长 string 且 name 较长，主机使用长请求上传：

     | 阶段 | 主机 -> 从机 | 从机 -> 主机 |
     | :--- | :--- | :--- |
     | BEGIN | `BEGIN_REQ, sid, flags, max_rsp_len, total_len, payload_crc16` | `ACK, sid, 0, 0` |
     | WRITE | `WRITE_REQ_CHUNK, sid, flags, max_rsp_len, offset, chunk_data` | `ACK, sid, next_offset, 0` |
     | EXEC | `EXEC_REQ, sid, flags, max_rsp_len` | `INLINE_RSP` 或 `RSP_READY` |

     若从机回复 RSP_READY，主机继续使用 READ_RSP_CHUNK 拉取完整 `PARAM_RESPONSE`。SET_BY_NAME 成功或限幅后必须返回实际值，UI 应以 `actual_value` 刷新显示。

  **INLINE_REQ 单包事务**

  当主机计算出：

  ```
  request_data_len + worst_case_response_data_len <= nmax_data
  ```

  则可以使用 INLINE_REQ。主机 DATA：

  | 偏移 | 符号 | 类型 | 说明 |
  | :---: | :---: | :---: | :--- |
  | 0 | COMMAND | uint8 | 固定 0x00 |
  | 1 | SESSION_ID | uint8 | 非 0 |
  | 2 | FLAGS | uint8 | 见 FLAGS 定义；无语义标志时填 0 |
  | 3 | MAX_RSP_LEN | uint8 | 本次允许从机直接回复的最大 DATA_LEN |
  | 4.. | PARAM_REQUEST | / | 参数请求对象 |

  从机若能在本时间片生成完整回复，则回 INLINE_RSP：

  | 偏移 | 符号 | 类型 | 说明 |
  | :---: | :---: | :---: | :--- |
  | 0 | STATUS | uint8 | 固定 0x02 |
  | 1 | SESSION_ID | uint8 | 回显 |
  | 2-3 | INFO | uint16 | 回复对象长度 |
  | 4 | INFO2 | uint8 | 保留，填 0 |
  | 5.. | PARAM_RESPONSE | / | 参数回复对象 |

  若从机需要较长处理时间，例如 NVM 正在保存，则回 BUSY。主机下一次该节点参数槽位重发同一个 INLINE_REQ 或发送轮询请求，直到收到 INLINE_RSP / ERROR。

  典型小包 GET 当前值：

  ```
  host_data_len = 4(FID4请求公共头) + 5(请求对象最小头) = 9 Bytes
  device_data_len = 5(回复公共头) + 约 7(float32 当前值回复) = 12 Bytes
  total = 21 Bytes
  ```

  在前述 `nmax_data = 30 Bytes` 的 10 节点 2kHz 示例中，该事务可以单包完成。

  **长请求上传**

  当请求对象过长，例如 SET_BY_NAME 携带长字符串和长参数名，主机应使用长请求流程。

  BEGIN_REQ 的 DATA：

  | 偏移 | 符号 | 类型 | 说明 |
  | :---: | :---: | :---: | :--- |
  | 0 | COMMAND | uint8 | 固定 0x01 |
  | 1 | SESSION_ID | uint8 | 非 0 |
  | 2 | FLAGS | uint8 | 见 FLAGS 定义；无语义标志时填 0 |
  | 3 | MAX_RSP_LEN | uint8 | 本次允许从机直接回复的最大 DATA_LEN |
  | 4-5 | TOTAL_LEN | uint16 | 完整参数请求对象长度 |
  | 6-7 | PAYLOAD_CRC16 | uint16 | 完整参数请求对象 CRC-16-IBM |

  从机回复 ACK，`INFO = 0` 表示期望从 offset 0 开始接收。

  WRITE_REQ_CHUNK 的 DATA：

  | 偏移 | 符号 | 类型 | 说明 |
  | :---: | :---: | :---: | :--- |
  | 0 | COMMAND | uint8 | 固定 0x02 |
  | 1 | SESSION_ID | uint8 | 回显活动会话 |
  | 2 | FLAGS | uint8 | 分片命令保留，填 0 |
  | 3 | MAX_RSP_LEN | uint8 | 本次允许从机直接回复的最大 DATA_LEN |
  | 4-5 | OFFSET | uint16 | 本分片在完整请求对象中的偏移 |
  | 6.. | CHUNK_DATA | bytes | 请求对象分片 |

  从机回复 ACK，`INFO = NEXT_OFFSET`。

  WRITE_REQ_CHUNK 的幂等规则：

  - `OFFSET == NEXT_OFFSET`：接收并追加，回复 ACK + 新 NEXT_OFFSET。
  - `OFFSET < NEXT_OFFSET`：认为是 ACK 丢失后的重发，不重复写入，直接回复 ACK + 当前 NEXT_OFFSET。
  - `OFFSET > NEXT_OFFSET`：回复 ERROR / BAD_OFFSET。

  EXEC_REQ 的 DATA：

  | 偏移 | 符号 | 类型 | 说明 |
  | :---: | :---: | :---: | :--- |
  | 0 | COMMAND | uint8 | 固定 0x03 |
  | 1 | SESSION_ID | uint8 | 活动会话 |
  | 2 | FLAGS | uint8 | 见 FLAGS 定义；应与完整请求语义一致 |
  | 3 | MAX_RSP_LEN | uint8 | 本次允许从机直接回复的最大 DATA_LEN |

  从机在 EXEC_REQ 时校验 `TOTAL_LEN` 和 `PAYLOAD_CRC16`。校验通过后执行参数请求；若完整回复可放入本时间片，则回 INLINE_RSP；若回复过长，则回 RSP_READY。

  **长回复读取**

  若从机回复 RSP_READY，主机通过 READ_RSP_CHUNK 拉取完整回复对象。

  READ_RSP_CHUNK 的 DATA：

  | 偏移 | 符号 | 类型 | 说明 |
  | :---: | :---: | :---: | :--- |
  | 0 | COMMAND | uint8 | 固定 0x04 |
  | 1 | SESSION_ID | uint8 | 活动会话 |
  | 2 | FLAGS | uint8 | 读取回复分片时保留，填 0 |
  | 3 | MAX_RSP_LEN | uint8 | 本次允许从机回复的最大 DATA_LEN |
  | 4-5 | OFFSET | uint16 | 需要读取的回复偏移 |
  | 6 | MAX_LEN | uint8 | 主机按 nmax_data 计算出的本次最大可接收分片 |

  RSP_CHUNK 回复 DATA：

  | 偏移 | 符号 | 类型 | 说明 |
  | :---: | :---: | :---: | :--- |
  | 0 | STATUS | uint8 | 固定 0x04 |
  | 1 | SESSION_ID | uint8 | 回显 |
  | 2-3 | INFO | uint16 | 当前 OFFSET |
  | 4 | INFO2 | uint8 | 保留，填 0 |
  | 5-6 | TOTAL_LEN | uint16 | 完整回复对象长度 |
  | 7.. | CHUNK_DATA | bytes | 回复对象分片 |

  主机按 offset 顺序读取直到 `OFFSET + len(CHUNK_DATA) == TOTAL_LEN`。若某个 RSP_CHUNK 丢失，主机重发同一个 READ_RSP_CHUNK；从机必须返回相同 offset 的数据，直到会话超时或主机 ABORT。

  **ABORT**

  主机可随时发送 ABORT 取消当前会话。

  | 偏移 | 符号 | 类型 | 说明 |
  | :---: | :---: | :---: | :--- |
  | 0 | COMMAND | uint8 | 固定 0x05 |
  | 1 | SESSION_ID | uint8 | 活动会话 |
  | 2 | FLAGS | uint8 | 保留，填 0 |
  | 3 | MAX_RSP_LEN | uint8 | 本次允许从机直接回复的最大 DATA_LEN |

  从机释放该节点 FID=4 会话缓存，回复 DONE。若会话不存在，也可以幂等回复 DONE。

  **最坏情况计算**

  一个完整参数元信息回复若包含 `param_type`、`current_value`、`default_value`、`min_value`、`max_value` 和 `name`，数值型最坏情况约为：

  ```
  response_base       = 6 Bytes    // OP_ECHO, RESULT, DOMAIN, INDEX, FIELD_MASK, ACCESS_FLAGS
  param_type          = 1 Byte
  typed_value_uint64  = 1 + 8 = 9 Bytes
  name_field          = 1 + 31 = 32 Bytes
  total = 6 + 1 + 9 * 4 + 32 = 75 Bytes
  ```

  若 string 类型同时返回 current/default/name，最坏情况约为：

  ```
  response_base       = 6 Bytes
  param_type          = 1 Byte
  typed_value_string  = 1 + 31 = 32 Bytes
  name_field          = 1 + 31 = 32 Bytes
  total = 6 + 1 + 32 * 2 + 32 = 103 Bytes
  ```

  以上均不适合在高节点数、高实时频率下强行单包回复。

  仍以前述 `nmax_data = 30 Bytes` 为例，READ_RSP_CHUNK 请求 DATA 长度为：

  ```
  4(FID4请求公共头) + 2(OFFSET) + 1(MAX_LEN) = 7 Bytes
  ```

  RSP_CHUNK 回复除 CHUNK_DATA 外固定长度为：

  ```
  5(FID4回复公共头) + 2(TOTAL_LEN) = 7 Bytes
  ```

  因此本时间片最大回复分片长度：

  ```
  chunk_len = nmax_data - 7 - 7 = 16 Bytes
  ```

  103 Bytes 最坏字符串元信息回复需要：

  ```
  ceil(103 / 16) = 7 个参数服务时间片
  ```

  若当前在线节点较少，例如 `nmax_data = 120 Bytes`，则：

  ```
  chunk_len = 120 - 7 - 7 = 106 Bytes
  ```

  该最坏回复可单个 RSP_CHUNK 读完。

  **调度规则**

  每个节点建议维护独立操作队列，优先级如下：

  ```
  文件传输 > 参数事务 > 示波器 > 低频状态反馈 > 实时控制
  ```

  参数事务应进一步区分：

  - `GET_BY_INDEX + CURRENT_VALUE` 等轻量读取可按低频插入；
  - 参数枚举、完整元信息读取、长字符串 SET 应作为后台任务分片推进；
  - `EXEC_SAVE`、`EXEC_ERASE`、`EXEC_LOAD_DEFAULT` 必须受节点状态约束，建议只允许在非 ARMED 或 MAINTENANCE 模式下执行；
  - 下位机执行 NVM 保存 / 擦除时，不应在 IRQ 内阻塞操作，应置位后台状态机并在后续 FID=4 请求中回复 BUSY，完成后回复 DONE 或 ERROR。

  主机侧推荐流程：

  1. 新节点上线后，先通过 `FRAME_ID = 2` 读取节点信息，获得 `UUID` 和 `SW_CRC64`。
  2. 若 `(UUID, SW_CRC64, DOMAIN_ID)` 命中缓存，则直接使用 index 快速读写。
  3. 若未命中缓存，则先 `GET_COUNT(domain)`，再按 index 枚举参数名、类型、访问标志和当前值。
  4. UI 展开单个参数详情时，再读取 default/min/max 等重字段。
  5. SET 后从机返回实际值；UI 以实际值刷新显示。
  6. 用户点击保存时发送 `EXEC_SAVE`，主机低频轮询直到 DONE / ERROR。

  **会话生命周期**

  - INLINE_REQ：收到 INLINE_RSP / ERROR 后事务结束；收到 BUSY 时主机重发同一请求或按同一 SESSION_ID 轮询。
  - BEGIN_REQ：创建长请求会话；相同 SESSION_ID 的 BEGIN_REQ 重发应幂等回复 ACK + 0。
  - WRITE_REQ_CHUNK：按 OFFSET 顺序推进；ACK 丢失时支持重发。
  - EXEC_REQ：执行组包后的请求；若回复很短则 INLINE_RSP，若回复很长则 RSP_READY。
  - READ_RSP_CHUNK：按 OFFSET 顺序或重复读取回复分片。
  - DONE / ERROR / ABORT / 超时：释放会话缓存。

  未分配 ID 的节点不得响应 FID=4。主机不得向广播 ID 发送 FID=4 参数服务帧。

- ##### FRAME_ID = 5（经典 MIT 控制帧）

- ##### FRAME_ID = 6（扩展 MIT 控制帧）

- **FRAME_ID = 7（位置控制帧）**

  主机下发格式（共 14Bytes）：

  第一字节（控制指令共有的信息头）

  |   位/Bits    |        符号        |    范围/类型    |                             说明                             |
  | :----------: | :----------------: | :-------------: | :----------------------------------------------------------: |
  |  0（1Bit）   |     MOTION_REF     |       0-1       |          0：BASE，1：OUTPUT（不采用 ELEC 进行下发）          |
  |  1（1Bit）   | MOTION_TORQUE_UNIT |       0-1       |                        0：AMP，1：NM                         |
  | 2-4（3Bits） | MOTION_SPEED_UNIT  | 0-4（5-7 保留） |           0：RADS，1：DEGS，2：REVS，3：RPM，4：HZ           |
  | 5-6（2Bits） |  MOTION_POS_UNIT   |  0-2（3 保留）  |                    0：RAD，1：DEG，2：REV                    |
  |  7（1Bit）   |       LATCH        |       0-1       | 0：不锁存指令，直接即时执行；1：锁存指令，等待 FID = 13 同步触发 |

  第二字节（位置控制帧命令字）

  |   位/Bits   |        符号         | 范围/类型 |                             说明                             |
  | :---------: | :-----------------: | :-------: | :----------------------------------------------------------: |
  |  0（1Bit）  |      RELATIVE       |    0-1    |               0：绝对位置控制，1：相对位置控制               |
  |  1（1Bit）  | RELATIVE_CURR_BASED |    0-1    | （仅 RELATIVE = 1 时有效）<br />0：相对的是当前真实位置（若锁存则为同步帧触发时的真实位置）<br />1：相对的是当前目标位置（若锁存则为同步帧触发时的目标位置） |
  |  2（1Bit）  |     TRAJECTORY      |    0-1    | 0：直接位置控制模式；1：梯形轨迹规划模式。<br />若启用梯形轨迹模式，则用户必须在电机配置中配置期望的加减速度和最大速度<br />（traj_output_accel/decel/speed_limit_rpm），全不为 0 时梯形轨迹方可生效。 |
  |  3（1Bit）  | TRAJECTORY_S_CURVE  |    0-1    | （仅 TRAJECTORY = 1 时有效）<br />0：梯形轨迹规划模式（匀加匀减，加速度恒定，急动度无穷大）<br />1：S 形轨迹规划模式（加速度斜坡，急动度恒定，动作更平滑） |
  | 4-7（4Bit） |        保留         |     /     |                              /                               |

  后续（真实包体）

  |     位/Bits     |      符号      |             范围/类型              |                             说明                             |
  | :-------------: | :------------: | :--------------------------------: | :----------------------------------------------------------: |
  | 0-31（4Bytes）  |    POSITION    |          float32（小端）           |         位置给定值，受 RELATIVE 控制，决定绝对/相对          |
  | 32-47（2Bytes） |  VELOCITY_FF   | float16（小端 IEEE 754 binary 16） |              速度前馈值，为 0 代表不需要加前馈               |
  | 48-63（2Bytes） | VELOCITY_LIMIT |          float16（小端）           | 速度限幅值，为 0 代表不限最大速度，以位置、速度环 PI 参数为准 |
  | 64-79（2Bytes） |   TORQUE_FF    |          float16（小端）           |              扭矩前馈值，为 0 代表不需要加前馈               |
  | 80-95（2Bytes） |  TORQUE_LIMIT  |          float16（小端）           |  扭矩限幅值，为 0 代表不限制扭矩，但受配置 max_current 影响  |

  从机回复：FID = 15，在此期间，从机也可能间歇性回复 FID = 14，需要同时做好处理。

- **FRAME_ID = 8（速度/力矩控制帧）**

  主机下发格式（共 10Bytes）：

  第一字节（控制指令共有的信息头）

  |   位/Bits    |        符号        |    范围/类型    |                             说明                             |
  | :----------: | :----------------: | :-------------: | :----------------------------------------------------------: |
  |  0（1Bit）   |     MOTION_REF     |       0-1       |          0：BASE，1：OUTPUT（不采用 ELEC 进行下发）          |
  |  1（1Bit）   | MOTION_TORQUE_UNIT |       0-1       |                        0：AMP，1：NM                         |
  | 2-4（3Bits） | MOTION_SPEED_UNIT  | 0-4（5-7 保留） |           0：RADS，1：DEGS，2：REVS，3：RPM，4：HZ           |
  | 5-6（2Bits） |  MOTION_POS_UNIT   |  0-2（3 保留）  |                    0：RAD，1：DEG，2：REV                    |
  |  7（1Bit）   |       LATCH        |       0-1       | 0：不锁存指令，直接即时执行；1：锁存指令，等待 FID = 13 同步触发 |

  第二字节（速度/力矩控制帧命令字）

  |   位/Bits    |        符号        | 范围/类型 |          说明          |
  | :----------: | :----------------: | :-------: | :--------------------: |
  |  0（1Bit）   | VELOCITY_OR_TORQUE |    0-1    | 0：TORQUE，1：VELOCITY |
  | 1-7（7Bits） |        保留        |     /     |           /            |

  后续（真实包体）

  |     位/Bits     |     符号     |             范围/类型              |                            说明                            |
  | :-------------: | :----------: | :--------------------------------: | :--------------------------------------------------------: |
  | 0-31（4Bytes）  |   VELOCITY   |          float32（小端）           |                         速度给定值                         |
  | 32-47（2Bytes） |  TORQUE_FF   | float16（小端 IEEE 754 binary 16） |          扭矩前馈值，若给定为扭矩，则为扭矩给定值          |
  | 48-63（2Bytes） | TORQUE_LIMIT |          float16（小端）           | 扭矩限幅值，为 0 代表不限制扭矩，但受配置 max_current 影响 |

  从机回复：FID = 15，在此期间，从机也可能间歇性回复 FID = 14，需要同时做好处理。

- **FRAME_ID = 10（文件下载帧）**

  适用：主机向从机下发文件（典型用途为固件更新）。由于协议的"一发一收"和时间片机制，所有操作耗时均不能超过实时控制中一个槽位的时长。在主机准备好向某节点发送文件后，主机应当将内部对应这个节点的状态机状态切换为"发送中"，停止在每个实时控制循环中下发控制指令，同时使用原先的时间片长度，通过每个槽位的双向数据段总字节数 Nmax 计算出一个保守的数据段长度，并把原文件拆分成多段依次发送。而从机如果涉及擦写 FLASH，应当视内存大小自行开辟一个缓冲区，待缓冲区填充全满或半满时，一次性擦除 FLASH 然后写入。

  一次文件下发的完整生命周期为：**BEGIN（握手）→ DATA（分片，可重复多轮）→ FIN（结束并校验）→ 从机校验通过后重启进入新固件**；任意阶段可由主机发送 ABORT 主动中止。会话以主机递增生成的 SESSION_ID 标识，整个运行周期内不重复；从机以该 ID 区分"新会话"与"旧会话重传"，与文件内容解耦。整个文件的完整性以 **CRC64-ECMA-182** 校验（与统一回复中 SW_CRC64 同算法），由主机在 FIN 时下发预期值，从机收齐后回算比对，通过才提交并重启，避免半截固件被启动。

  公共包头（所有 COMMAND 共有，位于 DATA 段起始）：

  |     位/Bits     |    符号    |  格式  |  范围  |                  说明                  |
  | :------------: | :--------: | :----: | :----: | :------------------------------------: |
  |  0-7（1Byte）  |  COMMAND   | uint8  |  0-3   |            0=BEGIN, 1=DATA, 2=FIN, 3=ABORT            |
  | 8-23（2Bytes） | SESSION_ID | uint16 | 0-65535 |     主机递增会话标识，运行周期内不重复     |

  COMMAND = 0（BEGIN）的后续字段（DATA_LEN = 8）：

  |     位/Bits     |   符号   |  格式  |     范围     |                        说明                        |
  | :-------------: | :------: | :----: | :----------: | :------------------------------------------------: |
  | 24-31（1Byte）  |  TARGET  | uint8  |    0-255     | 写入区域：0=APP，1=BL，其余保留。从机据此选择擦写分区 |
  | 32-63（4Bytes） | FILE_SIZE | uint32 | 0-4294967295 |    文件总字节数，从机据此预分配缓冲并预检空间是否足够    |

  COMMAND = 1（DATA）的后续字段（DATA_LEN = 7 + len(FILE_DATA)）：

  |          位/Bits           |    符号     |  格式  |     范围     |                             说明                             |
  | :------------------------: | :---------: | :----: | :----------: | :----------------------------------------------------------: |
  |     24-55（4Bytes）        | FILE_OFFSET | uint32 | 0-4294967295 |      本段 FILE_DATA 在文件中的起始字节偏移，建议按升序发送      |
  | 56-(55+m*8)<br />（m Bytes） | FILE_DATA  |   /    |      /       | 文件数据分片；m = DATA_LEN − 7，受帧上限 255 与单槽位 Nmax 双重约束取小值 |

  COMMAND = 2（FIN）的后续字段（DATA_LEN = 11）：

  |     位/Bits     |     符号      |  格式  | 范围 |                        说明                        |
  | :------------: | :-----------: | :----: | :--: | :------------------------------------------------: |
  | 24-87（8Bytes） | EXPECTED_CRC64 | uint64 |  /   | 预期 CRC64-ECMA-182，从机收齐后回算比对，通过才提交并重启 |

  COMMAND = 3（ABORT）：无后续字段，仅公共包头（DATA_LEN = 3）。

  从机应答格式（5 Bytes，对所有 COMMAND 统一）：

  |    位/Bits     | 符号  |  格式  |     范围     |                  说明                  |
  | :------------: | :--: | :----: | :----------: | :------------------------------------: |
  | 0-7（1Byte）   | STATUS | uint8  |    0-255     |              见下文状态码              |
  | 8-39（4Bytes） | INFO | uint32 | 0-4294967295 |        含义随 STATUS 而定，见下文        |

  STATUS 状态码：

  - 0 = BUSY：写入未完成（正在擦写或缓冲未满尚不可提交），主机下一时间片重发相同包。擦写 FLASH 期间从机可能停止响应串口包，此无响应情形同样按 BUSY 处理。
  - 1 = ACK：分片已成功写入，INFO = 已写入文件偏移末尾（即下次续传起点 NEXT_OFFSET），主机从该 OFFSET 续传。对 BEGIN 的 ACK：INFO 固定为 0，表示会话已就绪、可从 OFFSET 0 开始；从机不保留跨会话已提交进度，BEGIN 总是将文件传输重置为从头开始。
  - 2 = ERROR：错误退出，INFO 复用作错误码（见下文）。
  - 3 = VERIFY_OK：FIN 已收到且 CRC64 校验通过，从机即将重启进入新固件。
  - 4 = VERIFY_FAIL：FIN 已收到但 CRC64 校验失败，INFO 复用作错误码，主机可决定重传相关分片或 ABORT。

  错误码（STATUS = 2 或 4 时 INFO 取值）：

  - 0x01 空间不足（FILE_SIZE 超出 TARGET 分区可用容量）
  - 0x02 擦除失败
  - 0x03 写入失败
  - 0x04 CRC64 校验失败
  - 0x05 SESSION_ID 不匹配（非当前活动会话）
  - 0x06 TARGET 不支持
  - 0x07 参数非法（OFFSET 越界、DATA_LEN 与 FILE_SIZE 矛盾等）

  重传与幂等规则：

  - 主机收到 STATUS=0 或无响应时，下一时间片重发上一包（COMMAND / SESSION_ID / OFFSET 完全相同），直至 STATUS=1。
  - 若从机其实已写入该 OFFSET 段（应答 ACK 丢失），收到重发包时识别 `FILE_OFFSET < 当前已提交末尾` → 不重复写入（幂等），直接回 STATUS=1 + 当前 NEXT_OFFSET。
  - BEGIN 重发（相同 SESSION_ID）视为幂等再握手，从机重置并回 STATUS=1、INFO=0（不依赖此前进度）。
  - 收到 SESSION_ID 与当前活动会话不符的 DATA / FIN 包，从机回 STATUS=2（INFO=0x05），不写入。
  - **主机须在收到 BEGIN 的 ACK 后才开始发送 DATA，在收到末段 DATA 的 ACK 后才发 FIN**，以保证从机始终处于活动会话状态。

  会话生命周期：

  - BEGIN：开始一次新会话。从机不保留跨会话已提交进度，BEGIN 总是将文件传输重置为从 OFFSET 0 开始（丢弃未提交缓冲，且不依赖此前已写入偏移）。若该节点已有活动会话（不论 SESSION_ID 是否相同），旧会话被隐式中止后再开始新会话——此设计容忍主机重启后以新 SESSION_ID 重新发起，但代价是此前未完成的传输须从头重传。
  - DATA：按 FILE_OFFSET 升序分片下发；从机按到达顺序累加计算 CRC64（与分片提交同步）。
  - FIN：主机宣告数据发送完毕并下发 EXPECTED_CRC64；从机 flush 剩余缓冲、回算 CRC64 比对，回 STATUS=3（通过）或 4（失败）。主机收到 STATUS=3 后应停止对该节点发包并等待其重启；从机重启后以新固件的统一回复（含新 SW_CRC64）重新上线。
  - ABORT：主机主动中止；从机丢弃缓冲、回 STATUS=1（INFO=当前已提交 OFFSET）确认中止，节点回到正常控制 / 轮询。

  时序与约束：

  - 单包 DATA_LEN 受帧上限 255 与单槽位时序预算 Nmax_data 双重约束，取小值。传输中途 Nmax_data 可能随在线节点数变化而变化，从机按 FILE_OFFSET 续传即可容忍变长包。
  - 文件传输期间该节点槽位全部用于文件操作、不发控制指令；若该节点处于 ARMED 状态且传输耗时超过其看门狗超时，将触发 DISARM。固件更新通常应在节点待机或 bootloader 模式下进行；bootloader 模式下看门狗不适用或由 bootloader 自行喂狗。
  - 无响应判据：连续无响应累积超过超时离线时间（默认 2s）则主机将该节点移出在线列表并终止其文件传输流程。

- **FRAME_ID = 11（文件上传帧）**

  适用：主机要求从机上传指定文件

- **FRAME_ID = 12（串口示波器帧）**

  由主机设置哪些节点需要启用示波器功能，以及按多少频率轮询示波器通道，支持自定义多节点、多通道。启用该功能后，主机需要在实时循环中该节点对应的槽位中按频率插入串口示波器询问帧，因此显而易见，开启了串口示波器功能的节点，其实时控制帧的发送频率会下降。因此这个功能设计主要用于调试。

  例如：实时控制循环频率为 2KHz，设置 ID 为 2 的节点以 1KHz 频率上报示波器数据，则主机需要在实时循环中隔一个循环就将控制帧代替为串口示波器询问帧。

  串口示波器支持主机对通道的参数进行增、删、改、查操作，但通道数据是只读的。<u>当前版本中，首先实现对通道总数的查询和查询连续 n 个通道的数据这两项功能</u>。所有通道数据以 float32 表示（单精度浮点数，4 字节），对于下位机，通道数据可以在中断或 RTOS 任务中更新，一般通过在 RS485Protocol 类中预置一段最大长度固定的缓冲区，然后将缓冲区引用暴露给上层，由上层代码自行选择何时更新。

  由于实时控制频率固定，因此双向理论最大字节数也已固定。为了避免 OVERRUN，主机需要自行判断一次询问能够获取的最大通道数量，并取一安全值。

  通道索引采用 **0 基索引**：第一个通道为 0，最后一个通道为 `CHANNEL_COUNT - 1`。主机不得向广播 ID 发送串口示波器帧；未分配 ID 的节点不得响应 FRAME_ID = 12。

  主机发送格式（当前版本）：

  |    位/Bits    |    符号    |  格式  | 范围 |              说明              |
  | :-----------: | :--------: | :----: | :--: | :----------------------------: |
  | 0-7（1Byte）  |  COMMAND   | uint8  | 0-1  | 0=QUERY_CHANNEL_COUNT，1=READ_CHANNELS |
  | 8-23（0 或 2 Bytes） | PAYLOAD |   /    |  /   | 视 COMMAND 而定，见下文        |

  COMMAND = 0（QUERY_CHANNEL_COUNT，查询通道总数）：

  主机发送：

  |    位/Bits   |   符号   |  格式 | 范围 |       说明       |
  | :----------: | :------: | :---: | :--: | :--------------: |
  | 0-7（1Byte） | COMMAND  | uint8 |  0   | 固定为 0         |

  主机发送 DATA_LEN = 1。

  从机回复：

  |     位/Bits    |      符号      |  格式 |   范围   |             说明             |
  | :------------: | :------------: | :---: | :------: | :--------------------------: |
  | 0-7（1Byte）   |    COMMAND     | uint8 |    0     | 固定回显 0                   |
  | 8-15（1Byte）  | CHANNEL_COUNT  | uint8 | 0-255    | 当前可用示波器通道总数       |

  从机回复 DATA_LEN = 2。

  COMMAND = 1（READ_CHANNELS，读取连续通道）：

  主机发送：

  |     位/Bits    |      符号      |  格式 |   范围   |          说明          |
  | :------------: | :------------: | :---: | :------: | :--------------------: |
  | 0-7（1Byte）   |    COMMAND     | uint8 |    1     | 固定为 1               |
  | 8-15（1Byte）  |  START_INDEX   | uint8 | 0-255    | 起始通道索引 i         |
  | 16-23（1Byte） | REQUEST_COUNT  | uint8 | 0-255    | 请求读取的通道数量 n   |

  主机发送 DATA_LEN = 3。

  从机回复：

  |          位/Bits           |     符号      |    格式   |   范围   |                        说明                        |
  | :------------------------: | :-----------: | :-------: | :------: | :------------------------------------------------: |
  | 0-7（1Byte）               |    COMMAND    |   uint8   |    1     | 固定回显 1                                         |
  | 8-15（1Byte）              | RETURN_COUNT  |   uint8   | 0-63     | 实际返回的通道数量 r                               |
  | 16-(15+32*r)（4*r Bytes）  | VALUE[0..r-1] | float32[] |    /     | 从 START_INDEX 起按通道索引递增排列，小端 IEEE-754 |

  从机回复 DATA_LEN = `2 + 4 * RETURN_COUNT`。

  回复规则：

  - 正常情况下，`RETURN_COUNT = REQUEST_COUNT`。
  - 若 `START_INDEX >= CHANNEL_COUNT`，则 `RETURN_COUNT = 0`。
  - 若请求范围超过通道末尾，则 `RETURN_COUNT = CHANNEL_COUNT - START_INDEX`。
  - 若 `REQUEST_COUNT = 0`，则 `RETURN_COUNT = 0`。
  - 从机回复 DATA_LEN 不得超过 255，因此单帧最多返回 63 个 float32 通道值。
  - 主机还必须受单槽位双向数据段预算 `Nmax_data` 约束。读取连续通道时，主机发送 DATA 为 3 Bytes，从机回复 DATA 为 `2 + 4 * RETURN_COUNT` Bytes，因此应满足 `3 + 2 + 4 * REQUEST_COUNT <= Nmax_data`；若不满足，主机应拆分为多次读取。

- ##### FRAME_ID = 13（同步运动触发帧，广播）

  主机以广播 ID（15）、DATA_LEN = 0 发送，该总线上的所有节点收到后检查是否有锁存的运动控制指令，若有则直接触发。**该帧禁止回复**。

  通常情况下，此帧的发送节点（若主机需要触发同步运动）应当是在一个周期的末尾，但为了方便编程，在 RS485 协议实际实现中将同步运动触发帧定义为在 RS485 协议的实时循环首位开始发送。即：若主机 API 接口发出同步运动触发指令，则 RS485 协议对其进行缓存，然后选择一个最近到来的实时控制循环，在头部插入该指令，然后立即继续后续的分槽位发送。

  从机触发条件：收到的节点 ID == 15 且自身 ID != 15。

- ##### FRAME_ID = 14（低频状态反馈帧，可以由主机询问，也可由节点响应控制帧时回传）*Misc Feedback*

  可以设定在回传 n 次实时状态反馈帧后，改为回传一次低频状态反馈帧。主要用于一些慢速变化的状态回传等。

  最大不超过 500ms 获取一次。默认获取频率为 50Hz。

  数据段（10Bytes）：

  |     位/Bits     |           符号           |  格式  |      范围      |                             说明                             |
  | :-------------: | :----------------------: | :----: | :------------: | :----------------------------------------------------------: |
  | 0-11（12Bits）  |      DC_BUS_VOLTAGE      | uint12 |   [0, +4095]   | VOLT_PER_LSB = 0.2，量程范围：[0, +4095] * 0.2 = [0, +819.0] V |
  | 12-23（12Bits） |      DC_BUS_CURRENT      | int12  | [-2048, +2047] |      AMPERE_PER_LSB = 0.2，量程范围：[-409.6, +409.4] A      |
  | 23-32（9Bits）  |    CORE_TEMP_CELSIUS     |  int9  |  [-255, +254]  |                    核心温度，单位摄氏度/℃                    |
  | 32-41（9Bits）  |   MOSFET_TEMP_CELSIUS    |  int9  |  [-255, +254]  |                  MOSFET 温度，单位摄氏度/℃                   |
  | 42-50（9Bits）  |    MOTOR_TEMP_CELSIUS    |  int9  |  [-255, +254]  |                电机线圈绕组温度，单位摄氏度/℃                |
  | 51-55（5Bits）  | PROTOCOL_RESPONSE_AVG_US | uint5  |    [0, +31]    | 从串口触发 IDLE 中断调用 RS485 协议处理函数开始，到协议处理完毕、DMA 发送启动的总时间平均值，单位为微秒 |
  | 56-61（6Bits）  | PROTOCOL_RESPONSE_MAX_US | uint6  |    [0, +63]    | 从串口触发 IDLE 中断调用 RS485 协议处理函数开始，到协议处理完毕、DMA 发送启动的总时间最大值，单位为微秒 |
  | 62-67（6Bits）  |   RT_TASK_TIME_AVG_US    | uint6  |    [0, +63]    |             实时任务执行的时间平均值，单位为微秒             |
  | 68-73（6Bits）  |   RT_TASK_TIME_MAX_US    | uint6  |    [0, +63]    |             实时任务执行的时间最大值，单位为微秒             |
  | 74-79（6Bits）  |   MID_TASK_TIME_MAX_US   | uint6  |    [0, +63]    |             中速任务执行的时间最大值，单位为微秒             |

- ##### FRAME_ID = 15（实时状态反馈帧，可以由主机询问，也可由节点响应控制帧时回传）*RT Feedback*

  数据段（15Bytes）：

  |     位/Bits      |          符号          |  格式  |           范围           |                             说明                             |
  | :--------------: | :--------------------: | :----: | :----------------------: | :----------------------------------------------------------: |
  |       0-3        |         STATE          | uint4  |      0-15（>8保留）      |     [IDLE, ..., OPEN_LOOP_VELOCITY_CONTROL]，共 9 个状态     |
  |       4-5        |      CONTROL_MODE      | uint2  |           0-3            |    [POSITION, VELOCITY, CURRENT, HYBRID]，共 3 种控制模式    |
  |        6         |       HAS_ERROR        |  bool  |           0/1            | 为 1 代表当前存在影响系统运行的错误（通常为不可自恢复态），但具体错误需要通过其他指令进行查询 |
  |        7         |        IS_ARMED        |  bool  |           0/1            |             为 1 代表当前电机已经使能整流桥输出              |
  |  8-23（2Bytes）  |  OUTPUT_SINGLE_ROUND   | uint16 |  0-65535（0.0055°/LSB）  | **输出轴**的单圈绝对值角度，以 0-65535 映射到 0-360°（或 0-2PI），最小有效位为 0.0055° 或 9.6e-5 rad。 |
  | 24-55（4Bytes）  | OUTPUT_MULTI_ROUND_RAD | float  | [-FLOAT_MAX, +FLOAT_MAX] |          **输出轴**的多圈累计角度，单位为弧度 rad。          |
  | 56-87（4Bytes）  | OUTPUT_VELOCITY_RAD_S  | float  | [-FLOAT_MAX, +FLOAT_MAX] |           **输出轴**的转速，单位为弧度每秒 rad/s。           |
  | 88-119（4Bytes） |    OUTPUT_TORQUE_NM    | float  | [-FLOAT_MAX, +FLOAT_MAX] |            **输出轴**的估算扭矩，单位为牛米 NM。             |
