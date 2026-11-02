# 第三方来源与许可

本仓库的原创底盘应用、主机工具、测试与说明采用根目录MIT许可。第三方文件的原有版权与许可优先，不因根目录LICENSE改变。

- `vendor/Drivers/CMSIS/`：Arm CMSIS Core；保留上游Apache-2.0许可及逐文件版权。
- `vendor/Drivers/CMSIS/Device/ST/STM32F4xx/`：ST设备头文件；保留该目录上游许可及逐文件说明。
- `vendor/Drivers/STM32F4xx_HAL_Driver/`：ST HAL；保留上游许可及逐文件说明（含BSD-3-Clause条款）。
- `src/system_stm32f4xx.c`、`src/nav_startup.s`：ST系统/启动文件；`src/LICENSE.txt`保留上游许可说明，不将其重新授予MIT。
- `include/stm32f4xx_hal_conf.h`：ST模板派生配置；`include/LICENSE.txt`保留上游许可说明。

HAL/CMSIS由ST官方STM32CubeF4 V1.28.3安装包取得，HAL文件版本V1.8.5。芯片包固定`Keil::STM32F4xx_DFP@3.1.1`，工具安装器、芯片包本体不随仓库发布。

电机帧依据Emm V5协议实现；不依赖或发布原车封闭的`DiPanlib.lib`。后续只读参数解析依据[厂家Emm V5 Rev1.3说明](https://devpress.csdn.net/xian/68945b69080e555a88d61cd3.html)。本仓库不复制整本说明书。

机械臂应用现已归档在arm/；其来源为原机械臂本地提交8f4403b（原始溯源资料保留本地）。arm/project/Drivers包含原ST HAL/CMSIS，各层LICENSE.txt及文件头版权保留，根目录MIT不重新授权这些第三方文件。公开内容不包含原主控完整Flash备份或第三方付费工具。归档BIN/HEX包含本项目使用的上游运行代码，其相关许可同样保留在对应标签源码中。

`src/nav/generated`与`src/nav/qp`保留MATLAB生成源码的原始文件头、用途/授权提示，根目录MIT不覆盖或扩大这些文件的上游授权。这里只发布可部署算法C，未发布MATLAB模型或测试数据。
