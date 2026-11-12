# 当前机械臂共享代码

`project/APP/arm_task.c`是当前R9E状态机，`platform/profile_store.c`负责配置；主控`src/nav/nav_arm.c`以`ARM_NAV_BUILD`方式复用它们。PC6/PC7/PC8/PC9是四路舵机；升降ID5与四轮共享主控PA0/PA1总线。

**整车的最新镜像是根目录firmware/current中的主控BIN/HEX，机械臂已经编入，不需要第三块板。** 旧独立R9D镜像已移除，不能用独立机械臂固件覆盖整车主控。

本目录仍保留旧独立Keil工程/平台源码和build.ps1以便参考，它不代表本次已重新验证的发布入口；其独立电机口与共享总线不同。当前可复现构建使用根目录`python scripts/build_nav.py`，本次验证范围见[总交接](../docs/HANDOFF_F407_LIDAR_20261112.md)。

操作、四路舵机接线与物理参考恢复见[当前命令表](../docs/WIRING_AND_COMMANDS.md)。Flash配置不保证驱动器断电后的物理零点；seed直接启用PWM，不是读取真实角度。相机view/qr高度先标定后使用。
