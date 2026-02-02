## iFOC - IAP & Bootloader 设计

### 前言

为了兼容更多生态、避免重复造轮子，iFOC 系统目前规划了 CAN、串口、EtherCAT（FoE）这三种远程固件更新，也就是在应用编程（In-Application Programming，IAP）方式（按计划支持时间先后排序）。

不管是通过哪种通信方式进行的 IAP 固件更新，我们均要求：

1. 整个更新流程绝对可控、绝对安全，需要考虑分包校验错误，组包校验错误，app 不完整或错误导致的程序死锁或跑飞等，保证在单备份区情况下，出现任何 app 无法自主恢复的错误时能够退回到 Bootloader 方便重新刷写固件，必要时需要采取双备份区策略。
1. 对于 MTU 较小（如经典 CAN，8 Bytes）的总线，不能收到一个分包就写入 FLASH，必须利用好可用的内存空间做好缓存，例如开辟一块 16，32 KBytes 的缓冲区（视内存大小而定），收到后一次性擦除-写入。
1. 对于 Bootloader 可操作的 FLASH 区域做出限制，防止选择错误大小的固件导致 FLASH 末尾的用户参数区域被覆盖。
1. Bootloader 应尽量少的占用启动时间，不应该留 “等待超时时间”，而是设计为只能由硬中断（App 不完整或跑飞时）或 App 触发进入 Bootloader。
1. Bootloader 应绝对精简，不使用任何操作系统，同时也尽量不使用 C++，在编译时采取最节省空间的选项，以最大化为 app 留足空间。

### CAN IAP

对于基于 CAN 总线的固件更新，为了尽可能兼容现有协议以及上位机工具，我们采用 DroneCAN (UAVCAN v0) 协议规定的固件更新方式。

#### 优点

- DroneCAN 协议作为一款较为成熟的机载通信协议，兼容广泛，有现成上位机（DroneCAN GUI Tool、各种飞控地面站）可选，方便直接复用上位机快速更新固件。
- DroneCAN 协议的固件更新模式为从节点主动向主机请求，充分考虑到从节点自身的处理时间，避免主机在节点处理完成前重复下发导致总线占用。
- 基于 CAN 总线的固件更新优势之一在于免拆卸节点，即可对一条总线上的所有节点依次更新固件，非常适用于驱动器已经安装到机械结构（如机械臂、人形机器人关节）中而不方便拆装的环境。

#### 缺点

- DroneCAN 协议对于节点固件的版本校验过度依赖上位机且无法天然区分高低版本（上位机从节点回报 `uavcan.protocol.GetNodeInfo` 中提取出节点自身计算出的固件 CRC64，并将该 CRC64 与上位机指定路径下的固件进行比较，若不同则代表需要更新固件，对于版本的约束也仅仅只有节点自行返回的值），对于节点固件的完整性校验更是几乎不存在（通过 `uavcan.protocol.file.BeginFirmwareUpdate` 请求节点进入到固件更新模式，该请求包体中仅包含 `source_node_id` 和 `image_file_remote_path` ，毫无固件的完整性保证，如完整固件的 CRC64 值等；对于分包传输过程，由节点主动发送 `uavcan.protocol.file.Read` 请求，同样仅包含 `offset` 和 `path`，对分包的完整性保障仅靠 CAN 总线物理层）。
- 考虑到 CAN 协议中存在一个物理节点对应多个软件节点，如双驱 iFOC 等情况，并且为了简化 Bootloader 设计，不考虑引入 Protobuf 配置读写模块，因此同一节点在正常模式下和在 Bootloader 模式下，Bootloader 无法读取到设置的节点 ID，Node ID 将有很大可能性不同（必须在 Bootloader 中设计 Dynamic Node Allocation，即 DroneCAN DNA 支持）
- 目前 iFOC CAN Bootloader 的设计暂未考虑双备份 app 模式，因此一旦正式开始从文件服务器下载固件，就代表原有的 app 区域正在被擦除重写，因此如果在此过程中断电，必然会导致程序无法启动。我们拟依赖硬件看门狗和备份寄存器，实现当 app 不完整或启动错误跑飞后，能够及时回到 Bootloader 方便重新刷写固件。

#### DroneCAN 协议固件更新流程

参考 1：https://github.com/dronecan/libcanard/blob/master/examples/ESCNode/esc_node.c

参考 2：https://dronecan.github.io/Implementations/Libuavcan/Tutorials/11._Firmware_update/

1. 主机向节点发送 `uavcan.protocol.file.BeginFirmwareUpdate` 请求，节点解包出 `source_node_id` 和 `image_file_remote_path`，对重复请求进行过滤。

2. 节点回复 `uavcan.protocol.file.BeginFirmwareUpdate` ，根据实际情况填充 `response.error` 部分。

3. **（注意）**节点应在确认 2. 中回复已经发出后，再进入到 Bootloader 模式。对于 iFOC::Protocol::UAVCANProtocol 中采用的异步发送方式，可以在 RTOS 任务中判断更新标志位和缓冲队列长度实现。进入 Bootloader 时，至少需要携带参数 `image_file_remote_path`，此处根据硬件抽象层（HAL）平台的不同，可以选择通过备份寄存器传参（然后不断电 reset），或是通过写入非易失配置区域来实现。对于带有备份寄存器的平台，为了避免引入复杂的 FLASH 读写库，我们倾向于采用备份寄存器传参。

4. 在 Bootloader 中，节点向主机发送 `uavcan.protocol.file.Read` 请求，包含在 1. 中解包出的 `path` 和分包偏移量 `offset`，`offset` 初始为 0。

5. Bootloader 同时需监听主机发来的 `uavcan.protocol.file.Read` 回复（Response），解包出该分包包含的内容和长度，确认内容已写入缓冲区（或直接写入 FLASH）后递增分包偏移量 `offset`，然后重复 4. 中所述步骤。

6. 包尾终止标志：在 `uavcan.protocol.file.Read` 的回复定义中，data[] 数组的长度为 256，且中间分包每次都会以最大长度进行传送，所以当收到一个分包的数据长度小于 256，即可认为是包尾，在完整接收完该数据包后，固件更新结束。

7. （附录 1）对 DroneCAN GUI Tool 的文件服务器实现分析：在原始的 `uavcan.protocol.file.Path` 定义中，path 的最大长度为 200 字节，这种基于文件绝对路径的传输协议定义无法保证传输请求的精简性，同时若 path 是传递给 Bootloader 的必要参数之一，如此长的长度上限显然也无法接受。因此，在 DroneCAN GUI Tool 的 `widgets/file_server.py` 下，实现了如下函数：`def FileServer_PathKey(path)` 。该函数的功能为：将用户传入的绝对路径经过 CRC32 和 Base64 编码归一化为固定 7 字节的路径哈希值，用来在整个双向传输链路中替代原始的 `path`，使得 path 能够定长且精简。若后续我们需要自己设计基于 DroneCAN 协议的固件更新模块，也可以参考这种实现，来尽可能简化协议负载。下面附上一段该函数的注释。

   > return key used in file read request for a path. This is kept to 7 bytes to keep the READ request in 2 frames.

8. （附录 2）关于判断固件更新进度：注意到 `uavcan.protocol.file.GetInfo` 中可以通过 `path` 获取 `size` 变量，因此 Bootloader 可以在开始向文件服务器请求分包之前，先发送 `GetInfo` 查询整体大小，并可以根据自身 App 区大小决定是否接受该固件。在固件传输过程中，可以计算出下载进度（百分比），并通过 `uavcan.protocol.NodeStatus` 中的 `vendor_specific_status_code` 显示。

#### 基于硬件看门狗 + 备份寄存器的 STM32 / AT32 Bootloader 实现

1. Bootloader 与 App 之间使用 `bkp_struct_t` 结构体通过备份寄存器互相传参，`bkp_struct_t` 主要包含以下内容：

   1. uint8_t reg (union)

      1. [0] bootloader_presented: 由 Bootloader 在初始化时置位，标记着当前系统中存在 Bootloader。

         用途：在 App 请求跳转到 Bootloader 时，会先检查是否有该标记，若无则终止跳转以免出现非预期的结果。

      2. [1] update_requested: 由 App 在初始化时复位，在请求跳转时置位，标记着是否有固件更新请求。

         用途：在 Bootloader 启动时，若先读取到备份寄存器内容合法且该标记置位，尝试将 `app_node_id` 应用到自身 canard，且切换节点模式到 `SOFTWARE_UPDATE`，并根据 `dronecan_image_path` 开始固件更新流程。

      3. [2] controllable_hardfault: 由 App 在初始化时复位，在 HardFault 中断中置位，标记着是否产生了**可控的** HardFault 中断。

         “可控的”解释：若 App 区域不完整，则 Bootloader 改变中断向量表后跳转到 App，可能出现程序跑飞的情况，此时进入的 HardFault 中断称为不可控的。若是除此之外的情况（例如由完整的 App 自身设计而导致进入 HardFault，此时称为可控的）

         用途：App 在可控 HardFault 中断中递增 controllable_hardfault_count，当超过三次时，停留在 Bootloader 区。

      4. [3] app_init_success: 由 App 在初始化完成后置位，可以辅助判断 App 区域是否完整。

      5. [4:7] 保留

   2. struct version

      1. uint8_t major: 定义对应 `uavcan.protocol.SoftwareVersion::major`
      2. uint8_t minor: 定义对应 `uavcan.protocol.SoftwareVersion::minor`
      3. uint32_t vcs_commit: 定义对应 `uavcan.protocol.SoftwareVersion::vcs_commit`

      （附录）iFOC 固件版本定义规则：major 对应固件编译年份，minor 对应固件编译月份，vcs_commit 为 32 位无符号整数，可以拆分为 4 个字节，这四个字节按 16 位字面量表示固件编译的 月份 日期 小时 分钟（MM-DD-HH-mm），例如：2 月 1 日 16 点 32 分编译的固件，其 vcs_commit 的十六进制为 0x02011632，对应十进制整数为 33625650。

   3. uint8_t controllable_hardfault_count

      由 App 在初始化时复位，可控 HardFault 中断计数，当超过三次时，Bootloader 不执行跳转。

   4. uint8_t app_node_id

      当 `flags.update_requested` 置位时，Bootloader 读取该变量，若合法则直接 “继承” 原 App 的节点 ID，开始固件更新。否则，则需要通过 Dynamic Node-ID Allocation 从最低优先级的节点 ID 开始枚举。

   5. uint8_t file_server_node_id

      当 `flags.update_requested` 置位时，Bootloader 读取该变量，若文件服务器节点 ID 合法则向该节点请求分包传输，开始固件更新。否则，该固件更新请求视为不合法，需要重新初始化相关变量并在 Bootloader 等待。

   6. uint8_t dronecan_image_path[7]

      当 `flags.update_requested` 置位时，Bootloader 读取该路径并向文件服务器请求对应文件大小，若符合要求则将节点模式更新到 SOFTWARE_UPDATE，开始固件更新。

   7. uint8_t crc8

      `bkp_struct_t` 前面所有成员（不包含 `crc8` 本身）的 CRC8 校验值，用于判断备份寄存器中内容是否合法。

2. 硬件看门狗设置超时时间数量级为几百 ms，App 可在中断中定时喂狗，Bootloader 可在主循环中喂狗。

3. AT32 中，通过 CRM_CTRLSTS 寄存器判断复位原因（STM32 相似）

   - LPRSTF：低功耗复位标志
   - WWDTRSTF / WDTRSTF：窗口看门狗 / 看门狗复位标志
   - SWRSTF：软件复位标志
   - PORRSTF：上电 / 低电压复位标志
   - NRSTF：NRST 管脚复位标志

4. Bootloader 启动流程：

   1. 看门狗复位 / 软件复位：读取备份寄存器中的 `bkp_struct_t` 结构体
      - 若校验通过，数据有效：
        - 若 `app_init_success`：
          - 看门狗复位：
            - 若 `controllable_hardfault`，且 `controllable_hardfault_count <= 3`：不再初始化，直接跳转 App
            - 不满足条件：在 Bootloader 中等待
          - 软件复位：
            - 若 `update_requested`：开始固件更新过程
            - 不满足条件：初始化结构体，跳转 App
        - 不满足条件：初始化结构体，在 Bootloader 中等待
      - 若校验未通过：初始化并跳转
   2. 其他复位：直接初始化 `bkp_struct_t`，置位 `bootloader_presented`，跳转 App

5. （由 Bootloader 触发的）固件更新流程：收到 `uavcan.protocol.file.BeginFirmwareUpdate`，校验完整性后填充 `fw_update_struct_t`，检测到 `file_server_node_id` 合法且 `file_full_size` 为 0 时，发送 `uavcan.protocol.file.GetInfo` 查询完整大小（此处同样适用重传机制，复用 `last_requested_tick`），校验完大小合法后填充 `file_full_size`，正式开始分包传输。

6. 在分包下载中的超时退出与重传机制：为尽可能利用 CAN 总线带宽以及收发两方的处理速度，Bootloader 轮询分包在主循环中以 10KHz 的频率运行，更新 `last_requested_offset` 和 `last_requested_tick`。分包的接收在处理 `uavcan.protocol.file.Read` 的函数中，收包完成的标志为：`offset` 更新（`offset` 固定代表已收到包的总长度），同时更新 `last_response_tick`。

   - 轮询函数中，当 `last_requested_offset == offset` 时，说明未收到分包，持续等待。若当前 tick - `last_requested_tick` 大于指定刻（初步定为 100ms），则重发相同分包，更新 `last_requested_tick` 后继续等待。若当前 tick - `last_response_tick` 大于指定刻（初步定为 5000ms），则认为主节点已停止响应这次固件更新流程，需要对 `offset` 等内容进行重置，退回到 Bootloader 继续等待。

### 串口 IAP / Serial IAP



### EtherCAT FoE IAP / File over EtherCAT

