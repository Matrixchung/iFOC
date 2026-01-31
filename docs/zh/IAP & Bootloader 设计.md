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
- 考虑到 CAN 协议中存在一个物理节点对应多个软件节点，如双驱 iFOC 等情况，并且为了简化 Bootloader 设计，不考虑引入 Protobuf 配置读写模块，因此同一节点在正常模式下和在 Bootloader 模式下，Bootloader无法读取到设置的节点 ID，Node ID 将有很大可能性不同（必须在 Bootloader 中设计 Dynamic Node Allocation，即 DroneCAN DNA 支持）
- 目前 iFOC CAN Bootloader 的设计暂未考虑双备份 app 模式，因此一旦正式开始从文件服务器下载固件，就代表原有的 app 区域正在被擦除重写，因此如果在此过程中断电，必然会导致程序无法启动。我们拟依赖硬件看门狗和备份寄存器，实现当 app 不完整或启动错误跑飞后，能够及时回到 Bootloader 方便重新刷写固件。

#### DroneCAN 协议固件更新流程

参考 1：https://github.com/dronecan/libcanard/blob/master/examples/ESCNode/esc_node.c

参考 2：https://dronecan.github.io/Implementations/Libuavcan/Tutorials/11._Firmware_update/

1. 主机向节点发送 `uavcan.protocol.file.BeginFirmwareUpdate` 请求，节点解包出 `source_node_id` 和 `image_file_remote_path`，对重复请求进行过滤。
2. 节点回复 `uavcan.protocol.file.BeginFirmwareUpdate` ，根据实际情况填充 `response.error` 部分。
3. **（注意）**节点应在确认 2. 中回复已经发出后，再进入到 Bootloader 模式。对于 iFOC::Protocol::UAVCANProtocol 中采用的异步发送方式，可以在 RTOS 任务中判断更新标志位和缓冲队列长度实现。
4. 在 Bootloader 中，节点向主机发送 `uavcan.protocol.file.Read` 请求，包含在 1. 中解包出的 `path` 和分包偏移量 `offset`，`offset` 初始为 0。
5. Bootloader 同时需监听主机发来的 `uavcan.protocol.file.Read` 回复（Response），解包出该分包包含的内容和长度，确认内容已写入缓冲区（或直接写入 FLASH）后递增分包偏移量 `offset`，然后重复 4. 中所述步骤。
6. 包尾终止标志：在 `uavcan.protocol.file.Read` 的回复定义中，data[] 数组的长度为 256，且中间分包每次都会以最大长度进行传送，所以当收到一个分包的数据长度小于 256，即可认为是包尾，在完整接收完该数据包后，固件更新结束。

### 串口 IAP / Serial IAP



### EtherCAT FoE IAP / File over EtherCAT

