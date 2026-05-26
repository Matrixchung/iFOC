### iFOC-GIM6010-v4 工程

#### 适用硬件规格

AT32F403ACGT7 + DRV8301 + MT6835 + CAN

- iFOC-GIM6010-v4 驱动板（可加离轴编码器）
- iFOC-GIM6010-v4.1 驱动板（可加离轴编码器）

#### 开发环境

为减少无关代码，将 AT32 库文件进行了打包，在开发前请先将根目录的 libraries.zip 解压到工程根目录下。

- CLion 2025.3.2
- gcc version 15.2.1 20251203 (Arm GNU Toolchain 15.2.Rel1 (Build arm-15.86))
- AT32 Workbench
- Open On-Chip Debugger 0.11.0+dev-snapshot (2024-08-05-13:55) (雅特力专版)

#### 主要逻辑部分

User/Apps/app.cpp