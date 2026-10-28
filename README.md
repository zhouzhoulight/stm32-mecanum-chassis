# STM32 麦克纳姆轮底盘

STM32F407VET6 + Emm V5 四轮底盘，使用 VS Code / Keil Studio 构建，WT901CTTL 提供航向记录。

**这是 v2.0 基线快照，保存于 `v2.0-baseline` 标签。基础控制可用，整车航向保持尚未实现；不是完整的自动导航或赛题程序。**

已归档的 F407 实测：一次前进并停止，以及一次无纠偏左横移（119.5 cm、后偏3.5 cm，IMU起终点变化+5.455°）。停止/失能回复与现场观察分别记录。F103上的一米标定不能当作F407精度证据。

## 从哪里看

- [接线与硬件](docs/wiring.md)：主控、无线下载器、四轮、陀螺仪。
- [编译、下载与调试](docs/build-and-flash.md)：已验证的macOS工具版本、检查命令和F5配置。
- [命令说明](docs/commands.md)：先通信，再单次低速试车。
- [实测与验证边界](docs/validation.md)：已有结果与尚未验证的项目。
- `src/`、`include/`：主控固件；`tests/`：离线测试；`records/`：选取的脱敏实测证据。
- `firmware/v2.0-baseline/`：现场使用过的BIN、HEX及SHA-256；`vendor/`：带上游许可的HAL/CMSIS。

## 快速开始

打开 `F407底盘测试.code-workspace`，安装推荐的Keil Studio扩展和[所需工具](docs/build-and-flash.md)。

```sh
python3 scripts/check.py --firmware
python3 scripts/project.py probes
python3 scripts/project.py ports
```

第一条只做离线测试和编译；后两条枚举设备，不发运动。下载另行执行 `python3 scripts/project.py flash`，会要求输入 `FLASH`，覆盖板内程序且不自动备份。上电只发送停止/失能，不自动行走。

确认接线与空地后，控制台中先 `status`、`probe 1`至`probe 4`、`imu`，再用 `arm all` 配合一次方向指令。基线默认5 RPM、160脉冲，不能把脉冲数当成厘米。

原始调车目录保留在本地；本仓库未包含机械臂固件、个人设备标识或完整Flash备份。许可与第三方来源见 [LICENSE](LICENSE) 和 [THIRD_PARTY.md](THIRD_PARTY.md)。
