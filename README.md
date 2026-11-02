# STM32 F407 底盘与机械臂

当前底盘 **nav1i / v2.5.0-nav1i-20261102**，机械臂 **R9D**。两套独立F407VET6固件，外部晶振25MHz，系统168MHz，尚未合并。

先读[详细交接](docs/HANDOFF_NAV1I_20261102.md)、[底盘命令](docs/COMMANDS_NAV1I.md)、[机械臂命令](arm/docs/serial_protocol.md)。

本次最新底盘加入中央通道连续路线、作业站停稳、150/200RPM预算、1rad/s同步转向、位移守恒里程计和控制用短时响应预测。只完成离线检查/编译，**尚未实车验证**。生产C异步链路检查整圈约65～71秒；不能当作实车60秒或定位精度保证。

## 构建

```powershell
python scripts/build_nav.py
powershell -ExecutionPolicy Bypass -File arm/build.ps1
```

底盘用GNU Arm GCC硬浮点，必要时设置`CHASSIS_GCC`；产物`build/nav1i/`。机械臂用Keil ARMCC5，构建路径可在脚本中调整。当前可下载镜像分别在`firmware/current/`、`arm/firmware/current/`，均下载0x08000000。下载前核对目标并备份，保留sector7标定；底盘大于64KiB，旧只擦sector0～3脚本不适用。构建不会烧录。

本次发布快照仅含最新STM32源码、HAL/CMSIS、工程/构建入口、当前固件、使用说明与许可证；不含仿真工程、仿真/测试/实车数据、测试夹具和本地中间提交。生成的`src/nav/generated`和`src/nav/qp`是可部署STM32算法C源码，不是MATLAB仿真工程。

公开快照隐私处理：仅发布BIN/HEX，不发布带本机调试路径的ELF/AXF；需要调试符号请从源码本机构建。工具安装位置通过PATH或`CHASSIS_GCC`、`KEIL_UV4`、`KEIL_FROMELF`环境变量配置，不保留贡献者的本机绝对路径。
