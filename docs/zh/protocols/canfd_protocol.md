### iFOC - CANFD 协议

#### 前言 - CAN 总线仲裁机制

* **非破坏性**仲裁：当节点 B 发送 (Tx) 了隐性电平 1，回读/接收（Rx）的却是显性电平 0，故节点 B 就知道仲裁失败了，从而转为接收状态。

仲裁发生在报文的 ID 段，且是从大端（MSB）开始仲裁，**ID 越小，优先级越高**。0 优先于 1。

例子：节点 1 的 ID 为 0x15A，补全到 11 位标准帧 ID 对应二进制：00101011010b

节点 2 ID 为 0x1F6，对应二进制：00111110110b

节点 3 ID 为 0x3D2，对应二进制：01111010010b

在 CAN 网段上的节点 123 进行报文发送时，在仲裁段进行总线仲裁：

* 第一个位都是 0，没有抉择出优先级高的报文，三个节点都继续发送；
* 第二个位，节点 3 是 1，其他两个节点是 0，节点 3 退出发送，转为接收，节点 1、2 继续发送；
* 第三个位，节点 2 是 1，节点 1 是 0，节点 2 退出发送，转为接收，节点 1 继续发送。

最终，经过总线仲裁节点 1 获得总线占用，进行报文发送。而节点 2、3 仲裁失败进入“只听”模式，在检测到总线空闲的第一时间再次尝试发送，继续进行总线仲裁。

#### 基本约定

* 一主多从通信，控制循环频率由主机决定。例如循环频率为 1kHz，则主机每 1ms 生成一次实时控制批次。一个批次最多包含每个在线节点的一帧实时控制帧，并可在末尾追加一帧同步触发帧。
* CAN FD 的“周期/时间片”只表示主机生成实时控制批次的节拍，不表示从机必须在该周期内回复，也不用于将回复与某个周期强绑定。任意时刻收到的合法帧均应按 `CLASS + DIRECTION + NODE_ID + FRAME_ID` 解析。
* CAN 控制器和驱动可能具有软件队列、发送邮箱和硬件 FIFO。主机必须保证同一控制批次的帧按节点 ID 递增进入发送路径，并保证同步触发帧不会先于该批次中的任一控制帧发出。若控制器按 CAN ID 选择发送邮箱，则该顺序由本文 ID 布局自然保证；若控制器按 FIFO/邮箱号选择，则驱动必须显式保证顺序。
* 采用小端字节序：所有多字节整数和浮点数均低字节在前。CAN 在线路上仍按标准规定在每个字节内先发送高位，本文的“小端”不改变 CAN 物理层位序。
* 字段紧密排列，不插入 C/C++ 结构体对齐空洞。CAN FD 的 DLC 只能表示 `0..8、12、16、20、24、32、48、64` Bytes；逻辑数据长度不能直接映射到 DLC 时，发送方在末尾补 0 到下一个合法 DLC，接收方按对应帧格式或显式长度字段忽略补零。
* 除了一发一收的实时状态反馈帧之外，从机节点可主动上报低频状态和错误/异常状态，以及可能的低频消息。对于没有被分配 ID 的节点，需要以广播 ID 发送节点状态帧，以便主机进行发现。
* 总线存在可设定时长的看门狗（watchdog）机制，具体时长由从机在各自配置中设定。当从机处于激活状态（ARMED）时，若超过看门狗超时仍未收到目标为本机的合法实时控制帧、FID=15 轮询帧或适用于本机的同步触发帧，则认为主机离线，从机自动失活（DISARM）。后台服务帧不得单独为运动控制看门狗喂狗。

##### 调度、积压与负载约束

CAN 的仲裁能确定“多个已经开始竞争的帧谁先发送”，但不能保证尚未进入控制器发送队列的帧时序。

主机每个实时周期应按以下顺序调度：

1. 对每个在线节点生成至多一帧实时控制帧或 FID=15 轮询帧；
2. 按 `NODE_ID` 从小到大提交该批次；
3. 若本周期需要同步触发，在该批次所有锁存控制帧之后提交广播 FID=13；
4. 在实时批次之后，仅按配置的预算插入有限数量的服务/文件帧；
5. 接收线程持续清空 RX FIFO，不以“当前周期是否发送过对应请求”作为丢弃回复的条件。

主机至少应监测：发送队列深度、最老待发送帧年龄、FID=15 反馈年龄、控制批次跨周期积压次数和 CAN 控制器错误计数。CAN FD 没有 RS485 的单槽位 `OVERRUN`，但会出现等价且更隐蔽的 `TX_BACKLOG` / 反馈过期；若上一个控制批次在下一个周期开始时仍未发送完，主机不得继续无限累加旧控制帧，应丢弃可覆盖的旧绝对控制量、保留一次性命令，并向上层报告总线过载。

#### 帧头设计

基于以上仲裁机制和基本约定，对帧头设计有以下的考量：

* **消息类型：实时回复 > 低频状态 > 异步事件 > 后台大批量传输**：Bit 10..9 设计为帧类型 CLASS，00b 代表实时控制/回复，01b 代表心跳或低频状态，10b 代表异步事件和服务消息，11b 代表批量传输等后台消息。CLASS 位于最高两位，因此该优先级跨越主机和从机方向生效。
* **对于同种消息类型，主机发送优先于从机回复**：Bit 8 设计为方向 DIRECTION，0 代表主机发送，1 代表从机发送。DIRECTION 只在相同 CLASS 的帧之间参与仲裁。

据此，11 位标准帧 ID 按如下结构设计：

```
Bit 10..9       Bit 8       Bit 7..4       Bit 3..0
+--------------+-----------+--------------+--------------+
| CLASS        | DIRECTION | NODE_ID      | FRAME_ID     |
| 2 bits       | 1 bit     | 4 bits       | 4 bits       |
+--------------+-----------+--------------+--------------+
```

字段定义：

- `CLASS`
  - `00`：实时控制 / 回复帧（包括同步运动触发帧）
  - `01`：心跳或低频状态
  - `10`：异步事件、服务消息
  - `11`：批量传输等后台消息
- `DIRECTION`
  - `0`：主机发送
  - `1`：从机发送
- `NODE_ID`
  - `1..14`：节点 ID 有效范围
  - `0` 为主机 ID，从机不允许占用
  - `15` 为广播 ID
  - `DIRECTION = 0` 时表示目标节点；`DIRECTION = 1` 时表示源节点
- `FRAME_ID`
  - `0..15`：具体业务帧类型

保证：

1. `CLASS` 位于最高两位，因此任意 CLASS=00 实时帧都优先于 CLASS=01 心跳/低频帧，后者又优先于 CLASS=10 服务帧和 CLASS=11 后台传输帧。
2. 同一 `CLASS` 内，主机发送帧的 `DIRECTION=0` 优先于从机发送帧的 `DIRECTION=1`。
3. 同一 `CLASS` 和 `DIRECTION` 内，较小的 `NODE_ID` 永远优先。
4. 同一 `CLASS`、`DIRECTION` 和 `NODE_ID` 内，再由较小的 `FRAME_ID` 获得更高优先级。

标准帧 ID 的编码公式为：

```
CAN_ID = (CLASS << 9) | (DIRECTION << 8) | (NODE_ID << 4) | FRAME_ID
```

本协议仅使用 11 位标准 CAN FD 数据帧，不使用扩展帧和经典 CAN 帧。无请求数据时发送 DLC = 0 的 CAN FD 数据帧。

为避免同一 FID 被不同实现任意放入不同 CLASS，规定如下：

| FRAME_ID | 用途 | 固定 CLASS |
| :---: | :--- | :---: |
| 0 | 动态节点 ID 分配 | `10` |
| 1 | 节点心跳/状态上报 | `01` |
| 2 | 节点信息服务 | `10` |
| 3 | 操作码服务 | `00` |
| 4 | 参数服务 | `10` |
| 5..8 | 实时运动控制 | `00` |
| 9 | 保留 | / |
| 10..11 | 文件传输 | `11` |
| 12 | 调试示波器服务 | `10` |
| 13 | 同步运动触发 | `00` |
| 14 | 低频状态 | `01` |
| 15 | 实时状态轮询/反馈 | `00` |

接收方应校验 FID 与 CLASS 的组合；不符合上表的组合视为非法帧，不能仅按低 4 位 FID 解析。

#### 数据段设计

* DLC：DLC 0..8 分别表示 0..8 Bytes；DLC 9..15 分别表示 12、16、20、24、32、48、64 Bytes。本文将字段实际占用称为“逻辑长度”，将 DLC 对应长度称为“总线数据长度”。固定格式按定义解析；可变长格式必须携带 `CHUNK_LEN`、`NAME_LEN` 等显式长度，禁止通过 DLC 减去包头来推断有效数据长度。

* CRC：传统 CAN 中的循环冗余校验（CRC）为 15 位，而在 CAN FD 中为 17 位（最多 16 个数据字节）或 21 位（20 - 64 个数据字节）。 在传统 CAN 中，CRC 中可以包含 0 到 3 个填充位，而在CAN FD 中，总是有 6 个固定填充位（CRC-17）或 7 个固定填充位（CRC-21）以提高通信可靠性，并且 ISO-CANFD 在 CRC 域起始还包含 3 位填充位计数和 1 位填充位计数检验位。
* CAN 硬件 CRC 足以保护单帧在线传输，因此单帧业务数据不再附加 CRC。跨多帧重组的参数请求和完整文件仍分别使用 CRC16/CRC64，防止会话错组、缓存覆盖或存储过程损坏。
* 帧时间必须同时计算仲裁速率段和数据速率段，并计入最坏位填充、CRC、ACK、EOF、IFS 及可能的错误重发。简单地对每个数据字节增加 20% 只能作为粗略估算，不能作为硬实时准入的唯一依据；实现应使用经过 ISO CAN FD 帧格式校验的计算器，并以目标控制器实测结果校准。本文后续所有“每周期可发送”判断均以最坏帧时间而非裸数据字节数为准。

#### 帧类型

* **FRAME_ID = 0（新节点 ID 分配帧，又称 Dynamic Node Allocation - DNA）**

  若主机收到来自广播 ID 的 FRAME_ID = 1 状态帧，则应当为该节点决定一个独特且唯一的节点 ID，并通过 FRAME_ID = 0 帧下发给节点；该帧的目标节点 ID 为广播 ID 15。所有节点都需要接收该帧，并判断 UUID 是否与本机 UUID 一致，若一致则将本机节点 ID 更改为指定 ID（15 代表回到未分配状态），若目标 ID != 15，则立即以新 ID 发送一帧节点状态上报帧。

  主机下发格式（5 Bytes，`CLASS=10, DIRECTION=0, NODE_ID=15, FID=0`）：

  | 位/Bits | 符号 | 类型 | 说明 |
  | :---: | :---: | :---: | :--- |
  | 0-31 | UUID | uint32 | 目标节点 UUID |
  | 32-39 | ASSIGNED_NODE_ID | uint8 | `1..14` 为分配的新 ID；15 表示解除分配；其它值非法 |

  该帧禁止直接回复。目标节点分配成功后以新 ID 尽快发送一帧 FID=1；主机以该心跳作为分配确认。主机在确认前不得把同一 ID 分配给其它 UUID。DNA 帧可以重发；UUID 与已分配 ID 均相同时必须幂等处理。

  未分配节点都使用相同 CAN ID。若两个节点恰好同时发送不同心跳内容，冲突发生在数据段而不是仲裁段，会产生 CAN 错误帧，而不会自动选出胜者。因此随机时延是协议正确性的一部分：未分配节点启动后和每次发送失败后都应重新选择随机退避，建议窗口至少为 0~255ms，并在连续失败时指数扩大到不超过心跳周期。随机源应混入硬件 UUID 和自由运行计时器，不能让同型号设备使用相同伪随机种子。

  （可选功能）主机可以选择维护 `NODE_ID -> UUID` 映射，若同一 ID 的 FID=1/FID=2 在生命周期内出现不同 UUID，应立即将该 ID 标记为冲突，停止向其发送运动命令，并通过目标 UUID 的广播 FID=0 重新分配。

* **FRAME_ID = 1（节点状态上报帧）** *Heartbeat*

  由从机按一段固定时间间隔自主发布。该间隔最长为 3s，默认为 1s。对于未分配节点 ID 的从机节点，应当以广播 ID 上报自身状态。为避免 ID 碰撞造成总线错误，未分配 ID 的从机节点在发送状态时应当在固定间隔基础上加入一个随机的时延，随机源应当混入硬件 UUID。

  从机发布格式（12 Bytes）：

  |     位/Bits     |    符号    | 范围/类型 |                             说明                             |
  | :-------------: | :--------: | :-------: | :----------------------------------------------------------: |
  | 0-31（4Bytes）  | UPTIME_SEC | uint32_t  | 代表节点自启动以来经过的秒数，不允许溢出，如果回滚则证明该节点发生过重启。 |
  | 32-39（1Byte）  |   HEALTH   |  uint8_t  | 0 代表 OK，其余可自定。可使用 error_count 填充该字段，代表有多少个错误。 |
  | 40-47（1Byte）  |    MODE    |  uint8_t  | OPERATIONAL = 0，INITIALIZATION = 1，MAINTENANCE = 2，SOFTWARE_UPDATE = 3，其余保留 |
  | 48-55（1Byte）  |  SUB_MODE  |  uint8_t  | 对于电机节点，与 STATE 相同。[IDLE, ..., OPEN_LOOP_VELOCITY_CONTROL]，共 9 个状态 |
  | 56-63（1Byte）  |    VSSC    |  uint8_t  |  Vendor Specific Status Code (VSSC)，可用于指示固件更新进度  |
  | 64-95（4Bytes） |    UUID    | uint32_t  | 从机的 32 位唯一 ID，是从机的唯一身份标识，在整个生命周期内不允许重复、不允许变化。 |

  已分配节点使用 `CLASS=01, DIRECTION=1, NODE_ID=self.node_id, FID=1`。未分配节点使用 `NODE_ID=15` 并遵守上述随机退避。心跳是状态快照，新的待发心跳应覆盖尚未提交到 CAN 控制器的旧心跳，禁止在软件队列中无限累积。主机连续 3s 未收到某节点的 FID=1，也未收到该节点其它合法帧，则将其判为离线。

* **FRAME_ID = 2（节点信息获取帧）**

  主机在发现已分配 ID 的新节点、节点重启上线或本地缺少对应 UUID 信息时发送。该服务使用 `CLASS=10`，不得发送给广播 ID；未分配节点不得响应。

  主机请求：DATA 为空，DLC = 0。

  从机回复格式（逻辑长度 `18 + NODE_NAME_LEN` Bytes，最大 32 Bytes）：

  | 位/Bits | 符号 | 类型 | 说明 |
  | :---: | :---: | :---: | :--- |
  | 0-31 | UUID | uint32 | 芯片唯一信息派生的 32 位硬件 UUID |
  | 32-39 | SW_VER_MAJOR | uint8 | 软件大版本，按 iFOC 约定通常表示年份 |
  | 40-71 | SW_VER_MINOR | uint32 | 四个小端字节分别表示十六进制 `MM-DD-HH-mm` |
  | 72-135 | SW_CRC64 | uint64 | 固件映像 CRC64-ECMA-182 |
  | 136-143 | NODE_NAME_LEN | uint8 | 节点名有效字节数，范围 0..14 |
  | 144.. | NODE_NAME | ASCII bytes | 不要求 `\0` 结尾；DLC 补零不属于名称 |

  因 CAN FD DLC 离散取值，总线数据长度根据逻辑长度向上取 20、24 或 32 Bytes。`NODE_NAME_LEN` 是名称的唯一长度依据。节点信息在运行中不可变，从机应在初始化阶段预编码回复，避免在接收路径中执行字符串拼接。主机对同一节点同一时刻最多保持一个 FID=2 请求在途，建议 100ms 后重试。

* **FRAME_ID = 3（执行操作码帧）**

  FID=3 是重要的服务事务，固定使用 `CLASS=00`。为了在允许延迟收包的 CAN FD 队列中区分重试、旧回复和新操作，CAN FD 版本增加 `SESSION_ID` 和 `OPCODE_ECHO`；为灵活起见，参数 `ARGUMENT` 可以是变长的。主机下发的请求有 2 字节固定占用，剩余长度（按 CAN FD DLC 计算）的字节全为参数，最大 62 字节；从机上传的回复有 3 字节固定占用，剩余长度的字节全为返回参数，最大 61 字节。

  主机请求（可变长度 2 + n Bytes，遵循 CAN FD DLC，若有填充字节必须置为 0）：

  | 位/Bits | 符号 | 类型 | 说明 |
  | :---: | :---: | :---: | :--- |
  | 0-7 | SESSION_ID | uint8 | 主机按节点递增的非 0 会话 ID |
  | 8-15 | OPCODE | uint8 | 操作码 |
  | 16... | ARGUMENT | / | 操作码参数，长度由各操作码定义 |

  从机回复（可变长度 3 + n Bytes，遵循 CAN FD DLC，若有填充字节必须置为 0）：

  | 位/Bits | 符号 | 类型 | 说明 |
  | :---: | :---: | :---: | :--: |
  | 0-7 | SESSION_ID | uint8 | 回显请求会话 ID |
  | 8-15 | OPCODE_ECHO | uint8 | 回显操作码 |
  | 16-23 | STATUS | uint8 | 0=不支持，1=失败，2=成功，3=处理中 |
  | 24... | ARGUMENT | / | 操作码返回参数，长度由各操作码定义 |

  操作码定义（主从机都需要严格校验发包和回包的参数长度）：

  * **OPCODE = 0：保留**

    * 主机不应下发 OPCODE = 0 的请求。任何 OPCODE = 0 的请求会被忽略执行，从机可选返回 STATUS = 0 的回复。

  * **OPCODE = 1：保存参数 / SAVE_PARAM**

    * 下发（总长 3）参数长度：1，参数为 `uint8_t domain_id`，代表想要保存参数的域 ID；
    * 返回（总长 4）参数长度：1，返回参数为已保存的 `domain_id`。

  * **OPCODE = 2：擦除参数 / ERASE_PARAM**

    * 下发（总长 3）参数长度：1，参数为 `uint8_t domain_id`，代表想要擦除参数的域 ID；
    * 返回（总长 4）参数长度：1，返回参数为已擦除的 `domain_id`。

  * **OPCODE = 3：重启节点 / REBOOT**

    * 下发（总长 7）参数长度：5

      | 字节/Bytes |     符号     |  类型   |             说明              |
      | :--------: | :----------: | :-----: | :---------------------------: |
      |     0      | MAGIC_WORD_1 | uint8_t |  0xAA，不满足则拒绝执行命令   |
      |     1      | MAGIC_WORD_2 | uint8_t |  0xBB，不满足则拒绝执行命令   |
      |     2      | MAGIC_WORD_3 | uint8_t |  0xCC，不满足则拒绝执行命令   |
      |     3      | MAGIC_WORD_4 | uint8_t |  0xDD，不满足则拒绝执行命令   |
      |     4      |    target    | uint8_t | 0=正常重启，1=进入 Bootloader |

    * 返回（总长 3）参数长度 0，执行状态以 STATUS 表示。（主机也需要容忍没有收到回包的情况）

  * **OPCODE = 4：切换电机蜂鸣状态 / TOGGLE_BEEP_IDENTIFY**

    * 下发（总长 2）参数长度：0；
    * 返回（总长 4）参数长度：1，返回参数为当前蜂鸣状态。该操作码只能在电机处于 IDLE 状态时调用，否则 STATUS 置为 1。

  * **OPCODE = 5：获取当前错误值 / GET_ERROR**

    * 下发（总长 2）参数长度：0；
    * 返回（总长 12）参数长度：9，前 8 字节为 `(uint64_t)motor_error`，按小端排列，后 1 字节强制填充全 0。

  * **OPCODE = 6：按 mask 清除错误 / CLEAR_ERROR_MASK**

    * 下发（总长 12）参数长度：10，前 8 字节为 `(uint64_t)error_mask`，按小端排列，后 2 字节强制填充全 0；
    * 返回（总长 12）参数长度：9，前 8 字节为清除完错误后的最新 `(uint64_t)motor_error`，按小端排列，后 1 字节强制填充全 0。

  * **OPCODE = 7：设置 MotorState / SET_MOTOR_STATE**

    * 下发（总长 3）参数长度：1，参数为 `(uint8_t)request_state`；
    * 返回（总长 4）参数长度：1，参数为按 `request_state` 设置后，状态机返回的`(uint8_t)response_state`；

  * **OPCODE = 8：通过键名获取 NVM 文件信息 / GET_NVM_FILE_INFO_BY_KEY**

    * 下发（总长 12）参数长度：10，参数为 ASCII 形式的字符串 key，以 `\0` 结尾。若 key 长度为 10，也可以不需要 `\0` 结尾。

    * 返回（总长 3/20）参数长度：0/17（若未找到该文件，则返回参数长度为 0，STATUS = 1 失败）

      | 字节/Bytes | 符号 |   类型   |       说明       |
      | :--------: | :--: | :------: | :--------------: |
      |    0-9     | KEY  |    /     |     参数回显     |
      |   10-13    | SIZE | uint32_t | 查找到的文件大小 |
      |   14-16    | 保留 |    /     |        /         |

  * **OPCODE = 9：通过键名删除 NVM 文件 / DELETE_NVM_FILE_BY_KEY**

    * 下发（总长 12）参数长度：10，参数为 ASCII 形式的字符串 key，以 `\0` 结尾。若 key 长度为 10，也可以不需要 `\0` 结尾。
    * 返回（总长 3）参数长度：0，删除状态由 STATUS 传递。该操作码只能在电机处于 IDLE 状态时调用，否则 STATUS 置为 1。

  * **OPCODE = 10：NVM 用户区全部擦除 / ERASE_NVM_USER_AREA**

    * 下发（总长 6）参数长度：4，该操作码只能在电机处于 IDLE 状态时调用，否则 STATUS 置为 1。

      | 字节/Bytes |     符号     |  类型   |            说明            |
      | :--------: | :----------: | :-----: | :------------------------: |
      |     0      | MAGIC_WORD_1 | uint8_t | 0x11，不满足则拒绝执行命令 |
      |     1      | MAGIC_WORD_2 | uint8_t | 0x22，不满足则拒绝执行命令 |
      |     2      | MAGIC_WORD_3 | uint8_t | 0x33，不满足则拒绝执行命令 |
      |     3      | MAGIC_WORD_4 | uint8_t | 0x44，不满足则拒绝执行命令 |

    * 返回（总长 3）参数长度：0，擦除状态由 STATUS 传递。


  主机对同一节点同一时刻只允许一个 FID=3 操作在途；超时重试必须复用完全相同的 `SESSION_ID + OPCODE + ARGUMENT`。从机应缓存最近一次已完成事务及回复，收到完全相同的重试时直接重发缓存结果，不得再次执行保存、擦除、重启等操作。若同一 SESSION_ID 携带不同内容，回复失败且不得执行。重启操作允许节点在成功接收后直接重启而来不及回复；主机可将“旧节点离线且同 UUID 新心跳上线”作为重启成功的替代确认。

* **FRAME_ID = 4（Get/Set 参数服务帧，类似 DroneCAN GetSet / CANopen SDO）**

  适用：主机读取、修改、枚举和持久化指定节点的参数。FID=4 属于服务帧，实时要求较低。

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

  其中按 index 查询 / 修改是开销最小的路径；按 name 查询 / 修改主要用于参数导入、缓存失效或跨固件版本校验；初始轮询只应在节点上线、`SW_CRC64` 变化或本地缓存缺失时执行。FID=4 使用 `CLASS=10`，在 CAN 仲裁中低于 CLASS=00 实时帧和 CLASS=01 心跳/低频帧，但高于 CLASS=11 文件传输；主机软件仍应限制参数事务速率，建议默认以 10Hz~50Hz 推进，避免持续的主机服务帧压制从机回复。

  FID=4 每帧总线数据长度最大为 64 Bytes。请求对象或回复对象无法装入一帧时必须按下述会话协议分片，不允许依赖 CAN 控制器自动分包。

  ##### **参数模型**

  本协议以 **domain.index.name.value** 的形式定义参数：

  - `domain`：参数作用域，`uint8_t`，范围 0-255。例如板级参数可定义为 `domain_id = 0`，电机参数可定义为 `domain_id = 1`。
  - `index`：参数在该 domain 下的有序索引，`uint8_t`，范围 0-255。
  - `name`：参数名，ASCII string，最大 31 Bytes，用于上位机显示和跨版本校验。由于参数已由 `domain_id` 区分作用域，`NAME` 不应携带 `board.`、`motor.` 等作用域前缀。
  - `value`：参数值，采用本节定义的 `TypedValue` 编码。

  下位机应维护一个参数注册表。每个参数建议包含：

  | 字段          | 类型         | 说明                           |
  | :------------ | :----------- | :----------------------------- |
  | DOMAIN_ID     | uint8        | 参数作用域                     |
  | INDEX         | uint8        | 作用域内索引                   |
  | NAME          | string[<=31] | 参数名，建议同一 domain 内唯一 |
  | TYPE          | uint3        | 参数值类型                     |
  | ACCESS_FLAGS  | uint8        | 读写、只读、易失、持久化等标志 |
  | VALUE         | TypedValue   | 当前值                         |
  | DEFAULT_VALUE | TypedValue   | 默认值，可选                   |
  | MIN_VALUE     | TypedValue   | 最小值，仅数值类型适用，可选   |
  | MAX_VALUE     | TypedValue   | 最大值，仅数值类型适用，可选   |

  同一 `SW_CRC64` 下，`domain.index` 到参数的映射必须稳定。上位机可以缓存 `(UUID, SW_CRC64, DOMAIN_ID) -> name/index/type`，从而在后续读写中优先使用更短的 domain.index 访问。若固件修改了参数表的排列、名称、类型、访问属性或默认 / 限幅信息，则该固件的 `SW_CRC64` 必须变化；若 `SW_CRC64` 未变化但参数表结构变化，视为违反协议。若上位机使用 name 访问且拥有可靠缓存，建议置位 `FLAGS.INDEX_HINT_VALID` 并同时携带 index hint；下位机应优先校验 `domain + name`，若 name 与有效 index hint 不一致，应返回错误。

  ##### TypedValue 编码

  `TypedValue` 由 1 Byte 类型/长度头和后续值组成：

  | 位/Bits | 符号 | 范围 | 说明 |
  | :---: | :---: | :---: | :--- |
  | 0-2 | VALUE_TYPE | 0..7 | 值类型 |
  | 3-7 | VALUE_LEN | 0..31 | 值数据长度，单位 Byte |

  | VALUE_TYPE | 名称 | VALUE_LEN | 编码 |
  | :---: | :--- | :---: | :--- |
  | 0 | EMPTY | 0 | 空值 |
  | 1 | BOOLEAN | 1 | 0=false，非 0=true |
  | 2 | FLOAT32 | 4 | IEEE 754 float32，小端 |
  | 3 | UINT64 | 8 | uint64，小端 |
  | 4 | INT64 | 8 | int64，小端 |
  | 5 | STRING | 0..31 | ASCII，不要求 `\0` 结尾 |
  | 6 | BYTES | 0..31 | 原始字节 |
  | 7 | RESERVED | / | 保留 |

  接收方必须验证类型与长度的合法组合。SET 时还必须验证目标参数声明类型、写权限和范围；成功或限幅后必须在回复中返回实际写入值。

  ##### 参数请求与回复对象

  完整 `PARAM_REQUEST` 对象：

  | 偏移 | 符号 | 类型 | 说明 |
  | :---: | :---: | :---: | :--- |
  | 0 | OP | uint8 | 参数语义操作 |
  | 1 | DOMAIN_ID | uint8 | 参数作用域 |
  | 2 | INDEX | uint8 | 参数索引；按 name 访问时可作为 hint |
  | 3 | FIELD_MASK | uint8 | 要求回复包含的字段 |
  | 4 | VALUE_TYPE_LEN | uint8 | TypedValue 头；GET/EXEC 使用 EMPTY |
  | 5.. | VALUE | 0..31 Bytes | TypedValue 数据 |
  | 后续 | NAME_LEN | uint8，可选 | GET_BY_NAME/SET_BY_NAME 时存在 |
  | 后续 | NAME | 0..31 Bytes，可选 | 参数名 |

  OP 定义：

  | OP | 名称 | 说明 |
  | :---: | :--- | :--- |
  | 0x00 | GET_COUNT | 查询 domain 的参数数量 |
  | 0x01 | GET_BY_INDEX | 按 domain.index 读取 |
  | 0x02 | GET_BY_NAME | 按 domain.name 读取 |
  | 0x03 | SET_BY_INDEX | 按 domain.index 写入 RAM 当前值 |
  | 0x04 | SET_BY_NAME | 按 domain.name 写入 RAM 当前值 |
  | 0x05 | RESERVED          | 保留 |
  | 0x06 | RESERVED | 保留 |
  | 0x07 | EXEC_LOAD_DEFAULT | 仅在 RAM 加载默认值 |
  | 0x08..0xFF | RESERVED | 保留 |

  FIELD_MASK 定义：Bit0=`CURRENT_VALUE`，Bit1=`NAME`，Bit2=`DEFAULT_VALUE`，Bit3=`MIN_VALUE`，Bit4=`MAX_VALUE`，Bit5=`ACCESS_FLAGS`，Bit6=`PARAM_TYPE`，Bit7 保留。

  完整 `PARAM_RESPONSE` 对象：

  | 偏移 | 符号 | 类型 | 说明 |
  | :---: | :---: | :---: | :--- |
  | 0 | OP_ECHO | uint8 | 回显 OP |
  | 1 | RESULT_CODE | uint8 | 参数语义结果 |
  | 2 | DOMAIN_ID | uint8 | 回显 domain |
  | 3 | INDEX | uint8 | 实际参数索引 |
  | 4 | FIELD_MASK_PRESENT | uint8 | 实际包含字段 |
  | 后续 | ACCESS_FLAGS | uint8，可选 | bit5 置位时存在 |
  | 后续 | PARAM_TYPE | uint8，可选 | bit6 置位时存在，低 3 位为 VALUE_TYPE |
  | 后续 | CURRENT/DEFAULT/MIN/MAX | TypedValue，可选 | 按 bit0/2/3/4 顺序出现 |
  | 后续 | NAME_LEN + NAME | uint8 + bytes，可选 | bit1 置位时最后出现 |

  RESULT_CODE：0=`OK`，1=`NOT_FOUND`，2=`TYPE_MISMATCH`，3=`RANGE_CLAMPED`，4=`RANGE_REJECTED`，5=`READ_ONLY`，6=`NAME_INDEX_MISMATCH`，7=`NOT_ALLOWED`，8=`NVM_ERROR`。

  ACCESS_FLAGS：Bit0=`READABLE`，Bit1=`WRITABLE`，Bit2=`VOLATILE`，Bit3=`PERSISTENT`，Bit4=`WRITE_REQUIRES_DISARMED`，Bit5=`RESTART_REQUIRED`，Bit6..7 保留。

  ##### CAN FD 会话层

  所有主机请求都以 4 Bytes 公共头开始：

  | 偏移 | 符号 | 类型 | 说明 |
  | :---: | :---: | :---: | :--- |
  | 0 | COMMAND | uint8 | 传输命令 |
  | 1-2 | SESSION_ID | uint16 | 每节点独立递增的非 0 会话 ID |
  | 3 | FLAGS | uint8 | Bit0=`INDEX_HINT_VALID`，其余填 0 |

  COMMAND：0=`INLINE_REQ`，1=`BEGIN_REQ`，2=`WRITE_REQ_CHUNK`，3=`EXEC_REQ`，4=`READ_RSP_CHUNK`，5=`ABORT`。

  所有从机回复都以 5 Bytes 公共头开始：

  | 偏移 | 符号 | 类型 | 说明 |
  | :---: | :---: | :---: | :--- |
  | 0 | STATUS | uint8 | 传输状态 |
  | 1-2 | SESSION_ID | uint16 | 回显会话 ID |
  | 3-4 | INFO | uint16 | 随状态变化 |

  STATUS：0=`BUSY`，1=`ACK`，2=`INLINE_RSP`，3=`RSP_READY`，4=`RSP_CHUNK`，5=`DONE`，6=`ERROR`。`ERROR` 时 INFO 为：1 非法命令，2 非法会话，3 错误偏移，4 CRC 错误，5 对象过长，6 参数不存在，7 类型错误，8 范围错误，9 只读，10 NVM 忙，11 ARMED 状态不允许，12 domain 不存在，13 name/index 不匹配。

  单帧内联请求：

  ```
  INLINE_REQ = request_header(4) + PARAM_REQUEST
  INLINE_RSP = response_header(5) + PARAM_RESPONSE
  ```

  `INLINE_REQ` 的逻辑长度不得超过 64 Bytes，即 `4 + PARAM_REQUEST_LEN <= 64`。若 `5 + PARAM_RESPONSE_LEN <= 64`，从机回 `INLINE_RSP`，`INFO=PARAM_RESPONSE_LEN`；否则缓存完整回复并回 `RSP_READY`，`INFO=RSP_TOTAL_LEN`。由于 DLC 可能补零，接收方必须使用 INFO，而不能用 DLC 推导回复对象长度。

  长请求上传：

  | COMMAND | 公共头后的字段 | 约束 |
  | :--- | :--- | :--- |
  | BEGIN_REQ | `TOTAL_LEN:uint16 + PAYLOAD_CRC16:uint16` | 逻辑长度 8；CRC-16/MODBUS 覆盖完整 PARAM_REQUEST |
  | WRITE_REQ_CHUNK | `OFFSET:uint16 + CHUNK_LEN:uint8 + CHUNK_DATA` | `CHUNK_LEN<=57`，逻辑长度 `7+CHUNK_LEN` |
  | EXEC_REQ | 无 | 逻辑长度 4；从机校验长度和 CRC 后执行 |

  WRITE 的 ACK 中 `INFO=NEXT_OFFSET`。`OFFSET==NEXT_OFFSET` 时追加；`OFFSET<NEXT_OFFSET` 是重试，禁止重复写入并直接返回当前 NEXT_OFFSET；`OFFSET>NEXT_OFFSET` 返回 BAD_OFFSET。同一节点同一时刻只允许一个活动参数会话。

  长回复读取：

  ```
  READ_RSP_CHUNK = request_header(4) + OFFSET:uint16 + MAX_CHUNK_LEN:uint8
  RSP_CHUNK = response_header(5) + TOTAL_LEN:uint16 + OFFSET:uint16
              + CHUNK_LEN:uint8 + CHUNK_DATA
  ```

  `CHUNK_LEN<=54`，且不得大于请求的 `MAX_CHUNK_LEN`。主机以 `OFFSET + CHUNK_LEN` 推进，直到等于 TOTAL_LEN。丢包时重发相同 OFFSET；从机必须返回相同字节。尾帧 DLC 补零不计入 CHUNK_DATA。

  ABORT 只有请求公共头，从机释放会话并幂等回复 DONE。参数会话建议在 200ms~1000ms 无进展后超时释放；主机超时重试必须复用同一 SESSION_ID 和相同内容，收到旧 SESSION_ID 的延迟回复应丢弃但计入诊断。未分配节点不得响应 FID=4，主机不得向广播 ID 发送 FID=4。

  ##### 典型流程与缓存

  初次连接先通过 FID=2 获得 UUID 和 SW_CRC64。主机先对目标 domain 执行 GET_COUNT，再按 index 请求 `NAME + PARAM_TYPE + ACCESS_FLAGS + CURRENT_VALUE`；过长回复用 RSP_CHUNK 拉取。完成后缓存 `(UUID, SW_CRC64, DOMAIN_ID) -> index/name/type/access_flags`。缓存命中时按 index 读写；CRC 变化时废弃映射重新枚举。FID=4 只修改 RAM，持久化必须显式通过 FID=3 保存。

* **FRAME_ID = 7（位置控制帧）**

  主机下发逻辑格式共 16 Bytes：

  Byte 0 为运动控制公共头：

  | 位/Bits | 符号 | 范围 | 说明 |
  | :---: | :---: | :---: | :--- |
  | 0 | MOTION_REF | 0..1 | 0=BASE，1=OUTPUT |
  | 1 | MOTION_TORQUE_UNIT | 0..1 | 0=AMP，1=NM |
  | 2-4 | MOTION_SPEED_UNIT | 0..4 | 0=RAD/S，1=DEG/S，2=REV/S，3=RPM，4=HZ |
  | 5-6 | MOTION_POS_UNIT | 0..2 | 0=RAD，1=DEG，2=REV |
  | 7 | LATCH | 0..1 | 0=立即执行；1=锁存并等待 FID=13 |

  Byte 1 为位置命令字：

  | 位/Bits | 符号 | 范围 | 说明 |
  | :---: | :---: | :---: | :--- |
  | 0 | RELATIVE | 0..1 | 0=绝对位置；1=相对位置 |
  | 1 | RELATIVE_CURR_BASED | 0..1 | 仅 RELATIVE=1 有效；0=相对触发时真实位置，1=相对触发时目标位置 |
  | 2 | TRAJECTORY | 0..1 | 0=直接位置控制；1=轨迹规划 |
  | 3 | TRAJECTORY_S_CURVE | 0..1 | 仅 TRAJECTORY=1 有效；0=梯形，1=S 形 |
  | 4-7 | 保留 | / | 发送方必须填 0 |

  Byte 2..15：

  | 位/Bits | 符号 | 类型 | 说明 |
  | :---: | :---: | :---: | :--- |
  | 0-31 | POSITION | float32 | 位置给定值，单位由公共头决定 |
  | 32-47 | VELOCITY_FF | float16 | 速度前馈，IEEE 754 binary16，小端 |
  | 48-63 | VELOCITY_LIMIT | float16 | 速度限幅；0 表示不额外限幅 |
  | 64-79 | TORQUE_FF | float16 | 扭矩前馈 |
  | 80-95 | TORQUE_LIMIT | float16 | 扭矩限幅；0 表示不额外限幅，但仍受系统配置限制 |
  | 96-111 | COMMAND_SEQ | uint16 | 位置命令序号，见去重规则 |

  `float16` 必须按 IEEE 754 binary16 编解码，包括符号、5 位指数和 10 位尾数；不得使用 bfloat16，也不得直接截取 float32 高/低 16 位。接收方应拒绝 NaN 和 Infinity。超出 binary16 有限范围的值不得静默变为 Infinity，发送 API 应报告范围错误；不能精确表示的正常值按 round-to-nearest, ties-to-even 转换。

  位置命令去重与一次性规则：

  - 主机对每个节点独立递增 `COMMAND_SEQ`，从 1 开始，回绕按 uint16 处理；只有确实创建了新的用户位置命令时才递增，链路重试必须复用原序号。
  - 从机至少记录最近一次“已接收”和“已执行”的序号。相同 `COMMAND_SEQ` 且内容相同的重复帧不得重复应用；相同序号但内容不同视为协议错误。
  - `RELATIVE=1`、`TRAJECTORY=1` 的命令以及两者组合均是一次性命令。无论主机软件重试、发送队列重复还是反馈延迟，都只能执行一次；执行或成功锁存后，主机转为发送 FID=15 轮询，不得每周期重新下发该位置帧。
  - `LATCH=1` 时，首次收到命令只更新锁存区；重复帧不得重复入队。收到 FID=13 后，对尚未触发的该序号执行一次并标为已执行。下一条新序号可覆盖尚未触发的旧锁存命令，节点可通过异步事件报告该覆盖。
  - 绝对、直接位置命令允许由上位机选择周期刷新，但每次刷新同一目标时必须保持同一序号；如果将其视为新的控制目标，则分配新序号。

  *轨迹规划启用时，下位机应检查 `traj_output_accel_limit_rpm`、`traj_output_decel_limit_rpm` 和 `traj_output_speed_limit_rpm` 均大于 0 再执行。无效时不得退化为无限速的直接跳变，应拒绝该命令、保持原目标。*

  从机回复 FID=15；到达低频状态发布周期时可独立发送 FID=14。

* **FRAME_ID = 8（速度/力矩控制帧）**

  主机下发逻辑格式共 12 Bytes：

  第一字节（控制指令共有的信息头）

  | **位/Bits**  |      **符号**      |    范围/类型    |                           **说明**                           |
  | :----------: | :----------------: | :-------------: | :----------------------------------------------------------: |
  |  0（1Bit）   |     MOTION_REF     |       0-1       |          0：BASE，1：OUTPUT（不采用 ELEC 进行下发）          |
  |  1（1Bit）   | MOTION_TORQUE_UNIT |       0-1       |                        0：AMP，1：NM                         |
  | 2-4（3Bits） | MOTION_SPEED_UNIT  | 0-4（5-7 保留） |           0：RADS，1：DEGS，2：REVS，3：RPM，4：HZ           |
  | 5-6（2Bits） |  MOTION_POS_UNIT   |  0-2（3 保留）  |                    0：RAD，1：DEG，2：REV                    |
  |  7（1Bit）   |       LATCH        |       0-1       | 0：不锁存指令，直接即时执行；1：锁存指令，等待 FID = 13 同步触发 |

  第二字节（速度/力矩控制帧命令字）

  | **位/Bits**  |      **符号**      | 范围/类型 |        **说明**        |
  | :----------: | :----------------: | :-------: | :--------------------: |
  |  0（1Bit）   | VELOCITY_OR_TORQUE |    0-1    | 0：TORQUE，1：VELOCITY |
  | 1-7（7Bits） |        保留        |     /     |           /            |

  后续（真实包体）

  |   **位/Bits**   |   **符号**   |             范围/类型              |                          **说明**                          |
  | :-------------: | :----------: | :--------------------------------: | :--------------------------------------------------------: |
  | 0-31（4Bytes）  |   VELOCITY   |          float32（小端）           |                         速度给定值                         |
  | 32-63（4Bytes） |  TORQUE_FF   |          float32（小端）           |          扭矩前馈值，若给定为扭矩，则为扭矩给定值          |
  | 64-79（2Bytes） | TORQUE_LIMIT | float16（小端 IEEE 754 binary 16） | 扭矩限幅值，为 0 代表不限制扭矩，但受配置 max_current 影响 |

  `float16` 转换规则与 FID=7 相同。接收方应拒绝 NaN/Infinity 和非法枚举值。

  从机回复 FID=15；到达低频状态发布周期时可独立发送 FID=14。

* **FRAME_ID = 10（文件下载帧）**

  适用：主机向从机下发文件，典型用途为固件更新。FID=10 固定使用 `CLASS=11`，所以它低于所有 CLASS=00 实时帧、CLASS=01 心跳/低频帧和 CLASS=10 服务帧；在同为 CLASS=11 时，主机方向仍优先于从机方向。主机开始文件下载会话后，必须暂停该节点的实时控制帧，并为文件帧设置有限发送预算；不能把文件帧无限灌入发送队列，否则会延迟其它节点的控制帧和从机反馈。

  一次文件下发的生命周期为：**BEGIN → DATA（多片）→ FIN → 校验通过后提交/重启**；任意阶段可发送 ABORT。主机为每个目标节点生成非 0、运行周期内不重复的 `SESSION_ID`。完整文件使用 CRC64-ECMA-182 校验，主机在 FIN 中给出预期值，从机收齐并 flush 后校验，通过才提交并重启。

  FID=10采用**主机主动下发、逐片等待ACK的停等模式**。同一节点同一时刻最多允许一条文件请求处于等待状态：主机发送 BEGIN、DATA、FIN 或 ABORT 后，必须等待该命令对应的合法回复，才能发送下一条文件命令。主机不得在等待从机处理期间连续发送多个 DATA，也不得仅依据发送成功而推进文件偏移。ABORT是唯一例外：用户取消或当前请求重试耗尽时，主机可以用ABORT终止在途请求；一旦进入 ABORTING 状态，就不得再重发被其替代的旧请求。

  CAN FD DATA 字段最大为 64 Bytes。所有长度超过 8 字节的逻辑包都要按 DLC 规则补零；`CHUNK_LEN` 是有效数据长度，不能从 DLC 推断。

  公共包头（3 Bytes，所有 COMMAND 共有，位于 DATA 段起始）：

  | 偏移 | 符号 | 类型 | 说明 |
  | :---: | :---: | :---: | :--- |
  | 0 | COMMAND | uint8 | 0=BEGIN，1=DATA，2=FIN，3=ABORT |
  | 1-2 | SESSION_ID | uint16 | 非 0 会话 ID，小端 |

  BEGIN（逻辑长度 8，DLC=8）：`TARGET:uint8`（0=APP，1=BL）位于偏移 3，`FILE_SIZE:uint32` 位于偏移 4-7，0 不合法。

  DATA 布局（逻辑长度 `8 + CHUNK_LEN`，DLC 根据逻辑长度向上取合法值）：

  | 偏移 | 符号 | 类型 | 说明 |
  | :---: | :---: | :---: | :--- |
  | 3-6 | FILE_OFFSET | uint32 | 分片在文件中的起始偏移 |
  | 7 | CHUNK_LEN | uint8 | 有效 FILE_DATA 长度，范围 1..56 |
  | 8..`7+CHUNK_LEN` | FILE_DATA | bytes | 分片内容，最大 56 Bytes |
  

DATA 的固定开销为 8 Bytes，因此 `CHUNK_LEN <= 56` 才能装入 64 Bytes。除最后一片外，主机应使用 56 Bytes。`FILE_OFFSET` 按 `NEXT_OFFSET` 顺序推进；不允许通过乱序分片让从机产生稀疏文件。

FIN（逻辑长度 11，DLC=12）携带偏移 3-10 的 `EXPECTED_CRC64:uint64`。逻辑长度为11时，DLC补1个0字节。ABORT（逻辑长度 3，DLC=3）只有公共包头。

**从机回复格式（统一逻辑长度 8，DLC = 8）**：

| 偏移 | 符号 | 类型 | 说明 |
  | :---: | :--- | :--- | :--- |
  | 0 | COMMAND_ECHO | uint8 | 回显当前回复对应的 COMMAND |
  | 1-2 | SESSION_ID | uint16 | 回显请求会话ID，小端 |
  | 3 | STATUS | uint8 | 见下方状态定义 |
  | 4-7 | INFO | uint32 | 按 STATUS 和 COMMAND 解释，小端 |

STATUS定义：

| STATUS | 名称 | INFO含义 | 适用COMMAND | 主机处理 |
  | :---: | :--- | :--- | :--- | :--- |
  | 0 | BUSY | 当前已提交的 `NEXT_OFFSET`，仅用于诊断 | BEGIN/DATA/FIN/ABORT | 等待退避后重发完全相同的请求，不推进状态或偏移 |
  | 1 | ACK | DATA时为 `NEXT_OFFSET`；BEGIN时必须为0；ABORT时为从机当前已提交偏移 | BEGIN/DATA/ABORT | DATA仅在ACK后推进；BEGIN或ABORT收到ACK后进入下一状态 |
  | 2 | ERROR | 错误码 | 任意 COMMAND | 终止本次文件会话并报告错误；不得自动更换SESSION_ID继续写入 |
  | 3 | VERIFY_OK | 保留，必须为0 | FIN | 文件校验、写入和提交成功；主机停止向该节点发送FID=10，等待重启后的新心跳和FID=2信息 |
  | 4 | VERIFY_FAIL | 错误码 | FIN | 文件校验或提交失败；本次会话失败，不得继续发送DATA |

各命令成功回复的完成条件：

| COMMAND | 成功回复 | 从机发送成功回复前必须满足的条件 | 主机下一状态 |
  | :--- | :--- | :--- | :--- |
  | BEGIN | ACK，INFO=0 | 已校验TARGET和FILE_SIZE，目标区域已完成必要的准备/擦除，并已具备接收第一片DATA的能力 | 发送`FILE_OFFSET=0`的DATA |
  | DATA | ACK，INFO=`FILE_OFFSET+CHUNK_LEN` | 已完成分片校验与可靠提交，并已具备接收下一片DATA的能力 | INFO小于FILE_SIZE时发送下一片DATA；等于FILE_SIZE时发送FIN |
  | FIN | VERIFY_OK，INFO=0 | `NEXT_OFFSET==FILE_SIZE`，所有缓存已flush，完整文件CRC64校验及提交均成功 | 结束发送并等待节点重启 |
  | ABORT | ACK，INFO=当前已提交偏移 | 已停止或收敛当前在途操作、释放会话资源，后续相同ABORT仍可幂等回复 | 返回IDLE |

DATA 的 ACK 同时承担流控许可：从机只有在将数据可靠提交到目标存储或已为其保留不会被覆盖的缓存空间，并且具备接收下一片 DATA 的能力后，才可以返回 ACK。仅由 CAN 控制器收到帧、软件尚未处理完成时不得提前 ACK。主机收到匹配当前请求的 ACK 且`INFO==FILE_OFFSET+CHUNK_LEN`后，才把本地`NEXT_OFFSET`更新为 INFO 并允许发送下一片。若 DATA ACK 的 INFO 小于或等于当前`FILE_OFFSET`，说明它是旧分片的延迟回复，应忽略；INFO 大于`FILE_OFFSET+CHUNK_LEN`则属于非法越级确认。BUSY 中的 INFO 即使大于主机当前偏移，也不得作为推进依据。

错误码定义：

| 错误码 | 名称 | 说明 |
  | :---: | :--- | :--- |
  | `0x01` | SPACE_INSUFFICIENT | 目标存储空间不足 |
  | `0x02` | ERASE_FAILED | 目标区域擦除失败 |
  | `0x03` | WRITE_FAILED | 数据写入或提交失败 |
  | `0x04` | FILE_CRC_ERROR | 完整文件CRC64校验失败 |
  | `0x05` | SESSION_MISMATCH | SESSION_ID与当前活动会话不匹配 |
  | `0x06` | TARGET_UNSUPPORTED | 不支持请求的APP/BL目标区域 |
  | `0x07` | INVALID_ARGUMENT | 命令、长度或其它参数非法 |
  | `0x08` | NONCONTIGUOUS_OFFSET | FILE_OFFSET大于从机当前NEXT_OFFSET |

  主机必须校验`COMMAND_ECHO`、`SESSION_ID`、`STATUS`和`INFO`的组合是否合法。旧会话或其它节点的延迟回复必须丢弃，不能推进当前会话。收到 BUSY 或在单包超时内没有收到回复时，主机重发完全相同的 COMMAND、SESSION_ID、FILE_OFFSET、FILE_DATA 或 EXPECTED_CRC64；重试不得生成新的 SESSION_ID，也不得改变 DATA 内容。

  从机必须缓存当前正在处理的请求标识及足以判断请求一致性的内容。在该请求完成前收到完全相同的重试，只能回复 BUSY 或重发已经生成的最终回复，不得再次启动擦除、写入、校验、提交或重启操作；在当前请求尚未完成时，相同 SESSION_ID 和 COMMAND 若携带不同参数、FILE_OFFSET 或 DATA 内容，必须回复ERROR。上一条DATA完成并ACK后，同一会话携带下一个 NEXT_OFFSET 的新 DATA 属于合法的新请求。请求完成后，从机应缓存最近一次最终回复，至少保留到收到当前会话的下一条合法命令或会话超时，以便在回复丢失时重发。

  幂等规则：相同会话的 BEGIN 在准备/擦除尚未完成时必须回BUSY，完成后的重发必须回 ACK+0，不得重复擦除；新的 SESSION_ID 中止旧会话并从零开始。DATA 的 `FILE_OFFSET==NEXT_OFFSET` 时写入；`FILE_OFFSET<NEXT_OFFSET` 只有在内容与已提交数据一致时才作为丢 ACK 重试处理，并返回当前 `NEXT_OFFSET`，不得重复写入；`FILE_OFFSET > NEXT_OFFSET` 必须报错。FIN 只能在 `NEXT_OFFSET==FILE_SIZE` 后处理。ABORT 幂等并回 ACK。收到 VERIFY_OK 后主机停止向该节点发包，等待同 UUID 的新心跳和 FID=2 信息。

  文件传输期间节点不发送实时控制回复；主机应设置单包重试超时（默认 500ms）、最大重试次数（默认 3 次）；重试耗尽后应发送 ABORT（若链路仍可用）并将会话标记为失败。停等等待期间，主机可以继续发送其它节点的实时控制帧和高优先级服务帧，但不得为当前文件会话预先排队下一条 DATA。

* **FRAME_ID = 11（文件上传帧）**

* **FRAME_ID = 12（串口示波器帧）**

  由主机设置哪些节点需要启用示波器功能，以及按多少频率轮询示波器通道，支持自定义多节点、多通道。FID=12 固定使用 `CLASS=10`，属于服务帧；在 CAN FD 下它不替代实时控制帧，而是进入受限服务队列。主机必须先提交本周期的实时控制批次，再根据剩余总线负载发送示波器请求；当服务队列积压时应降低示波器频率或暂停示波器，不得挤占实时控制批次。

  例如：实时控制循环频率为 2kHz，设置 ID 为 2 的节点以 1kHz 读取示波器数据，则主机每两周期排入一个 FID=12 请求；该请求和回复可能延迟到后续周期，主机按帧 ID、命令和节点来源解析，不要求在同一周期完成。

  串口示波器支持主机对通道的参数进行增、删、改、查操作，但通道数据是只读的。当前版本中，首先实现对通道总数的查询和查询连续 n 个通道的数据这两项功能。所有通道数据以 float32 表示（单精度浮点数，4 字节），对于下位机，通道数据可以在中断或 RTOS 任务中更新，一般通过在 CANFDProtocol 类中预置一段最大长度固定的缓冲区，然后将缓冲区引用暴露给上层，由上层代码自行选择何时更新。

  通道索引采用 **0 基索引**：第一个通道为 0，最后一个通道为 `CHANNEL_COUNT - 1`。主机不得向广播 ID 发送串口示波器帧；未分配 ID 的节点不得响应 FRAME_ID = 12。

  主机发送格式（当前版本）：

  | 偏移 | 符号 | 类型 | 说明 |
  | :---: | :--- | :---: | :--- |
  | 0 | COMMAND | uint8 | 0=QUERY_CHANNEL_COUNT，1=READ_CHANNELS |
  | 1 | START_INDEX | uint8 | COMMAND=1 时有效，0 基索引 |
  | 2 | REQUEST_COUNT | uint8 | COMMAND=1 时有效 |

  `QUERY_CHANNEL_COUNT` 只发送 1 Byte，DLC=1。从机回复 2 Bytes：`COMMAND_ECHO:uint8`、`CHANNEL_COUNT:uint8`，DLC=2。

  `READ_CHANNELS` 发送 3 Bytes，DLC=3。从机回复格式为 `COMMAND_ECHO:uint8`、`RETURN_COUNT:uint8`、`VALUE[RETURN_COUNT]:float32[]`。`RETURN_COUNT` 最大为 15，因为最大逻辑长度为 `2 + 4 * RETURN_COUNT <= 64`。最后一帧若逻辑长度不是 CAN FD 合法 DLC，末尾补零；补零不属于通道值。

  回复规则：

  - 正常情况下，`RETURN_COUNT = REQUEST_COUNT`。
  - 若 `START_INDEX >= CHANNEL_COUNT`，则 `RETURN_COUNT = 0`。
  - 若请求范围超过通道末尾，则 `RETURN_COUNT = CHANNEL_COUNT - START_INDEX`。
  - 若 `REQUEST_COUNT = 0`，则 `RETURN_COUNT = 0`。
  - 从机回复逻辑长度不得超过 64，因此除去固定 2 字节开销，单帧最多返回 15 个 float32 通道值。

* **FRAME_ID = 13（同步运动触发帧，广播）**

  主机以广播 ID（15）、DATA_LEN = 0 发送，该总线上的所有节点收到后检查是否有锁存的运动控制指令，若有则直接触发。**该帧禁止回复**。

  此帧的发送时机：主机拼接节点控制帧（ID 从 0 到 n）+ **同步运动触发帧**，一并发出，然后主机接收来自节点的实时状态反馈帧。

  从机触发条件：收到的节点 ID == 15 且自身 ID != 15。

* **FRAME_ID = 14（低频状态反馈帧，由节点周期性低频回传）** *Misc Feedback*

  FID=14 固定使用 `CLASS=01`，由已分配节点自主周期发布，也可以由实现选择在处理实时控制/轮询后发送。默认发布周期为 20ms，最大不得超过 500ms；节点待发队列中最多保留一帧最新 FID=14，旧快照应覆盖，不能堆积。

  数据段逻辑长度为 12 Bytes：

  |     位/Bits     |        符号         |  格式  |      范围      |                             说明                             |
  | :-------------: | :-----------------: | :----: | :------------: | :----------------------------------------------------------: |
  | 0-12（13Bits） |   DC_BUS_VOLTAGE    | uint13 |   [0, +8191]   | VOLT_PER_LSB = 0.1，量程范围：[0, +8191] * 0.1 = [0, +819.1] V |
  | 13-25（13Bits） |   DC_BUS_CURRENT    | int13 | [-4096, +4095] |      AMPERE_PER_LSB = 0.1，量程范围：[-409.6, +409.5] A      |
  | 26-34（9Bits） | CORE_TEMP_CELSIUS | int9 | [-256, +255] | 核心温度，单位 ℃ |
  | 35-43（9Bits） | MOSFET_TEMP_CELSIUS | int9 | [-256, +255] | MOSFET 温度，单位 ℃ |
  | 44-52（9Bits） | MOTOR_TEMP_CELSIUS | int9 | [-256, +255] | 电机绕组温度，单位 ℃ |
  | 53-58（6Bits） | CAN_RESPONSE_AVG_US | uint6 | [0,63] | 从接收目标帧到回复进入发送队列的平均处理时间，单位 us；超出饱和 |
  | 59-64（6Bits） | CAN_RESPONSE_MAX_US | uint6 | [0,63] | 上述时间最大值，单位 us；超出饱和 |
  | 65-70（6Bits） | RT_TASK_TIME_AVG_US | uint6 | [0,63] | 实时任务执行平均时间，单位 us；超出饱和 |
  | 71-76（6Bits） | RT_TASK_TIME_MAX_US | uint6 | [0,63] | 实时任务执行最大时间，单位 us；超出饱和 |
  | 77-82（6Bits） | MID_TASK_TIME_AVG_US | uint6 | [0,63] | 中速任务执行平均时间，单位 us；超出饱和 |
  | 83-88（6Bits） | MID_TASK_TIME_MAX_US | uint6 | [0,63] | 中速任务执行最大时间，单位 us；超出饱和 |
  | 89-95（7Bits） | RTOS_CPU_UTIL_PCT | uint7 | [0,100] | RTOS 任务总 CPU 使用率，单位 %；可由空闲任务运行时间得出 |

  对于原始统计不带平均值功能的，可以在高频处理中断中对任务执行时间进行平均。

* **FRAME_ID = 15（实时状态反馈帧，作为实时控制帧的响应回传）** *RT Feedback*

  FID=15 固定使用 `CLASS=00`。主机可发送 DATA_LEN=0、DLC=0 的 FID=15 轮询帧；从机也可在收到 FID=7 / FID=8 后自主回复。主机必须接受回复在发送后续控制批次甚至多个周期后到达，只按来源节点和 `FEEDBACK_SEQ` 判断新旧，不得因没有“当前周期请求”而丢弃合法帧。

  数据段逻辑长度为 16 Bytes：

  | 位/Bits | 符号 | 类型 | 说明 |
  | :---: | :--- | :---: | :--- |
  | 0-3 | STATE | uint4 | 0..8，有效状态为 IDLE..OPEN_LOOP_VELOCITY_CONTROL，其余保留 |
  | 4-5 | CONTROL_MODE | uint2 | 0=POSITION，1=VELOCITY，2=CURRENT，3=HYBRID |
  | 6 | HAS_ERROR | bool | 1 表示存在影响运行的错误；具体错误由 FID=3 opcode=5 查询 |
  | 7 | IS_ARMED | bool | 1 表示整流桥已使能 |
  | 8-23 | OUTPUT_SINGLE_ROUND | uint16 | 输出轴单圈绝对角度，65536 映射 0..360° |
  | 24-55 | OUTPUT_MULTI_ROUND_RAD | float32 | 输出轴累计角度，单位 rad |
  | 56-87 | OUTPUT_VELOCITY_RAD_S | float32 | 输出轴速度，单位 rad/s |
  | 88-119 | OUTPUT_TORQUE_NM | float32 | 输出轴估算扭矩，单位 Nm |
  | 120-127 | FEEDBACK_SEQ | uint8 | 节点单调递增采样序号 |

  `STATE`、`CONTROL_MODE` 的保留值和所有浮点字段的 NaN/Infinity 均应视为非法反馈。主机对同一节点按 `FEEDBACK_SEQ` 做新旧判定，最大反馈延迟要求必须小于 128 个快照，回绕比较可以选择使用半范围规则。节点必须在每次产生新的状态快照时递增该序号，而不是每次硬件传输层发送重传时递增。对于没有丢包检测需求的实现，也不能删除该字段，因为它用于区分 CAN FD 允许的延迟回包。
