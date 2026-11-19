# STM32 麦轮底盘、机械臂与 LD19 双板固件

2026-11-19 更新。主控和雷达板均用 STM32F407VET6、25 MHz 外部晶振与 168 MHz 系统时钟。

| 工程 | 当前源码 | 验证状态 |
| --- | --- | --- |
| 底盘、任务与机械臂主控 | `v2.7.43-arm2-preview-20261118` | STM32 编译、主控离线回归 |
| 雷达处理板 | `LD19_BENCH_F407VE_0.3.5_FRONT150` | STM32 编译 |

主控包含正式初赛状态机、固定顺序调试入口，以及新伸缩轴的只读诊断和机械臂规划预览。当前实车取放任务使用已标定的原机构。本次版本未烧录到实车。

[整车接线表](docs/WIRING_20261119.md) · [技术交接与操作入口](docs/HANDOFF_V2743_20261119.md) · [双板通信协议](docs/LIDAR_LINK.md) · [许可证](THIRD_PARTY.md)

## 构建

使用 Python 3.9+、GNU Arm GCC。雷达板另需 CMake 3.20+ 和 Ninja。工具可放入 PATH，也可用 `CHASSIS_GCC`、`ARM_GCC`、`CMAKE`、`NINJA` 指定。从仓库根目录运行：

```powershell
python scripts/build_nav.py
python lidar/tools/build.py
```

构建输出分别在 `build/nav1k/` 和 `lidar/build/Release/`。历史发布包位于 `firmware/current/` 和 `lidar/firmware/current/`，具体版本以包内 `build.json` 为准。下载主控时保留 sector 7（`0x08060000–0x0807FFFF`）的机械臂标定。
