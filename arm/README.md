# F407机械臂R9D（独立固件）

F407VET6，HSE25MHz/168MHz。当前APP/BSP/平台及HAL/CMSIS完整保留，本次仅重新编译0错误0警告，未实物重跑。此前用户已确认grab/rough/stack取放可用。

构建：`powershell -ExecutionPolicy Bypass -File arm/build.ps1`，Keil ARMCC5.06，工程`project/MDK-ARM/GC1_Arm_F407_R9D.uvprojx`目标ArmBenchF407。脚本从platform同步BSP，输出根目录ArmBenchF407.bin与MDK的F407Output。发布镜像在firmware/current/。

接线/命令/物理零点恢复见[协议](docs/serial_protocol.md)，合并冲突及下一轮测试见[总交接](../docs/HANDOFF_NAV1I_20261102.md)。调试PA9/PA10，Emm5为PC10/PC11，舵机PC6/PC7/PC8。底盘固件的PA0/PA1不能套来接此独立固件。

上电PWM关闭，不自动动作。必须保留sector7配置；Flash中的高度记录不等于驱动器跨断电物理绝对定位。新板没有原板有效标定时先标零/标高度；不要照抄历史数字。
