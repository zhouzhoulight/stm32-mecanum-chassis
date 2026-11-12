# STM32 麦轮底盘、机械臂与 LD19 雷达固件

更新日期：2026-11-12。主控与独立雷达板均为 STM32F407VET6，25MHz 外部晶振、168MHz 系统时钟。

| 内容 | 当前版本 | 验证状态 |
| --- | --- | --- |
| 主控：底盘＋机械臂＋板上避障规划 | `v2.7.31-clockretry-20261111` | 重新编译与离线候选一致；**未烧录、未实车复测** |
| 雷达处理板 | `LD19_BENCH_F407VE_0.3.3`，协议2 | 重新编译与此前烧录镜像一致；本次未操作硬件 |
| 机械臂 | 集成 R9E 状态机 | 已编入主控；`arm/` 保存共享依赖及独立工程源码 |

先阅读[当前交接说明](docs/HANDOFF_F407_LIDAR_20261112.md)、[接线与命令](docs/WIRING_AND_COMMANDS.md)、[双板协议](docs/LIDAR_LINK.md)。

此次仅发布最新 STM32 源码、HAL/CMSIS、构建入口、当前 BIN/HEX 及详细交接文档；不包含仿真、测试数据、实测日志、调试符号、上位机运行数据和本地中间提交。`src/nav/generated/`、`src/nav/qp/` 是实际部署的算法 C，须保留。

## 编译

Python 3.9+、GNU Arm Embedded GCC（本次为 STM32 GNU GCC 13.3.1）；雷达板还需要 CMake 3.20+ 和 Ninja。先将工具加入 PATH，从仓库根目录执行：

```powershell
python scripts/build_nav.py
python lidar/tools/build.py
```

主控也可设置 `CHASSIS_GCC`；雷达可设置 `ARM_GCC`、`CMAKE`、`NINJA`，分别指向工具可执行文件。脚本只编译，不连接串口或下载器，不发运动指令。

- 主控输出 `build/nav1k/chassis-nav1k.bin`、`.hex`、`build.json`。
- 雷达输出 `lidar/build/Release/LD19Bench.bin`、`.hex`、`build.json`。
- 当前发布镜像：`firmware/current/`、`lidar/firmware/current/`。
- `chassis.csolution.yml` 是可选 CMSIS-Toolbox 入口，已补齐雷达源文件。本次只验证上述 Python/GCC 构建，未声称验证所有 IDE。

BIN 下载基址均为 `0x08000000`；HEX 自带地址。主控保留 sector7（`0x08060000..0x0807FFFF`）的机械臂标定，整片擦除会丢失；Flash 高度记录不等于驱动器跨断电物理绝对定位。

本版仍为诊断候选。同步重试改善已在离线验证中出现，自身边缘回波、原料盘边缘误检及严重打滑仍有反例，不能承诺全图毫米级定位或已完成无障碍整圈实测。

第三方许可见 [THIRD_PARTY.md](THIRD_PARTY.md) 及各目录原始许可证。
