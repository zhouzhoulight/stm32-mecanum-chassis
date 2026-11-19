# 当前机械臂共享代码

`project/APP/arm_task.c`是当前R9E状态机，`platform/profile_store.c`负责配置；主控`src/nav/nav_arm.c`以`ARM_NAV_BUILD`方式复用它们。PC6/PC7/PC8/PC9是四路舵机；升降ID5与四轮共享主控PA0/PA1总线。

机械臂已编入整车主控。当前镜像由根目录 `scripts/build_nav.py` 生成到 `build/nav1k/`。`firmware/current/` 中的 BIN/HEX 属于旧版发布包。

本目录保留独立 Keil 工程、平台源码和 `build.ps1` 供参考。独立工程电机口和整车总线使用不同引脚。当前验证范围见[新版本交接](../docs/HANDOFF_V2743_20261119.md)。

四路舵机及两个机械臂电机的接线见[整车接线表](../docs/WIRING_20261119.md)。驱动器断电后重新确认实体零点。`seed` 直接启用 PWM。相机 view/qr 高度先标定后使用。
