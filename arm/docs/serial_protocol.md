# 机械臂协议入口（集成版）

最新接线和命令已经统一到[整车命令表](../../docs/WIRING_AND_COMMANDS.md)，请以该表及`project/APP/arm_task.c`为准。

当前使用`arm `前缀和主控PA9/PA10控制台；五个Emm共享PA0/PA1。旧独立工程的PC10/PC11电机接线不适用于集成整车。最新固件在根目录firmware/current，机械臂引擎已经包含其中。
