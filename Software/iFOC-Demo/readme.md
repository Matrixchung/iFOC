## 踩的坑
1. FreeRTOS 默认 TOTAL_HEAP_SIZE (3072 Bytes) 过小，导致创建多于三个任务时堆栈溢出
2. FreeRTOS 的 vPortEnter/ExitCritical() 不是中断安全操作，在中断上下文内需要调用 portENABLE/DISABLE_INTERRUPTS()
3. STM32 DMA ADC 采样周期需要设置最大，以免采样周期过短，DMA 频繁中断导致任务卡在 vPortFree() 或者等待信号量阶段
4. 注意 MAX_SYSCALL_INTERRUPT_PRIORITY 的设置，低于此优先级的中断不会被 vPortEnterCritical() 打断，同样不能在中断内调用 FreeRTOS API
5. 不用的 DMA 中断（如 ADC DMA 是直接读取数组、串口发送 DMA）等可以关掉，其他中断同理
6. 编译后可以根据输出计算内存占用量，内存占用 = BSS + DATA，STM32F103CB 系列最大内存为 20 KBytes，RCC 外设可以默认使用 LL 库减少内存占用
7. HAL 库的硬件 I2C 会被 BUSY 寄存器置位打断，需使用 LL 库重写（更新：LL 库在高速发送状态下依然会卡死，仍然需要用软件模拟 I2C，RTOS 中需要配合 DWT_Delay 使用）
8. 软件模拟 I2C 序列内 delay_us() 仍会明显拖慢程序运行速度，目前解决方法有几个方向，即：1) 降低屏幕刷新率，或采用响应式刷新 2) 以空间换时间，引入 WYSIWYG 按 OLED_PAGE 为最小单位只刷新更新的像素 ...
9. 解决 RTOS 串口 DMA 发送掉帧：while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) osThreadYield(); （不完美，同一时刻两个任务调用串口，仍会掉帧）
10. portMAX_DELAY / osWaitForever 式任务，可以不用 osDelay()
11. ADC 提高精度 (打开 ADC1 的内置 Vrefint Channel)：Vchannel = Vrefint_cal * (ADchx / ADrefint)，其中 Vrefint_cal 可查阅芯片手册，部分型号为单独校准后写入内部寄存器 VREFINT_CAL，其余型号为参考值（如 F103CBT6 = 1.2 V）
12. 在 FreeRTOS 内使用 printf() 需要打开 NEWLIB 微库，并调高默认的 128 Words 任务堆栈大小
13. CubeMX 默认配置的 defaultTask 无法删除，且如果任务只执行一次（无循环），最后需要使用 osThreadTerminate(NULL) 释放任务堆栈。
14. 在串口 DMA 接收初始化时清空 huart->Instance->SR (Status Register) 和 huart->Instance->DR (Data Register) 可以防止 DMA 启动时接收到大量数据死机的情况（待测试）
15. 串口 RX 中途断开会触发错误回调，回调函数中检测到串口处于 DMA 模式就会关闭 DMA 接收，导致重连后串口无法收到数据。目前解决方法暂时未知，有可能可以通过定时检测 DMA 状态然后重启 DMA 接收实现。
16. 链接器 linker 参数加入 -u_printf_float 可以开启 printf 的浮点支持，但代价是编译后的二进制文件会增加接近 10 KB，目前的解决方法是用自定义的 float2char 函数。
17. 一直卡在中断里出不来的时候（有可能是 LL 库没有清除标志位，或优先级高于 MAX_SYSCALL_PRIORITY 的中断触发太快等等），debug 会卡在 vPortFree 的 configASSERT() 内，注意排查中断相关区域。
18. 串口接收、发送问题，记得检查 USB 转串口芯片
19. 当两个待比较的字符串相等时，strcmp 返回 0