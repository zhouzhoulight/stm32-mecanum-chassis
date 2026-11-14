# STM32 麦轮底盘、机械臂与 LD19 双板固件

更新：2026-11-14。主控和雷达板均为STM32F407VET6，HSE25MHz、SYSCLK168MHz。

**接手入口：[详细技术栈、代码结构、接线、状态机与未完成事项](docs/HANDOFF_V2738_20261114.md)。** 此文优先于旧版说明；二维码板实体回波要求及待验收矩阵见第9节。

| 部分 | 当前代码 | 状态 |
|---|---|---|
| 主控：底盘＋机械臂＋九路口规划 | v2.7.38-scene-20261114 | 已编译、离线回归；尚未烧录/实车验证 |
| LD19处理板 | LD19_BENCH_F407VE_0.3.5_FRONT150，协议2 | 此前已有实板使用；本次仅重编译，未下载 |
| 机械臂 | 主控集成R9系列状态机，依赖在arm/ | 本次APP及标定存储代码不变 |

最近已知实车主控为v2.7.37，不能把候选版仿真结果称为实车结果。本版修复转盘弱回波、障碍确认/冻结、通行路口与工位坐标混用、拐角误制动和接近段制动。**二维码板专门处理尚未新增，严重单轮打滑仍有明确失败反例。**

## 构建

Python3.9+、GNU Arm GCC（验证为13.3.1）；雷达再需CMake3.20+和Ninja。工具加入PATH，或设置CHASSIS_GCC、ARM_GCC、CMAKE、NINJA为对应可执行文件。仓库根目录执行：

```powershell
python scripts/build_nav.py
python lidar/tools/build.py
```

两个入口只编译，不访问串口/下载器、不发运动指令。输出为build/nav1k/、lidar/build/Release/。发布BIN/HEX及build.json位于firmware/current/、lidar/firmware/current/。

BIN基址0x08000000；保留主控sector7（0x08060000..0x0807FFFF）的机械臂标定，不能随意全片擦除。Flash记录不提供驱动器跨断电物理绝对参考。

## 范围与验证

仅最新STM32源码、HAL/CMSIS、生成算法C、构建依赖、当前BIN/HEX和交接文档。无仿真文件/数据、测试数据、实车日志、上位机或本地中间开发提交。src/nav/generated/、src/nav/qp/是实机依赖，必须保留。

31组本机C回归通过；107例软件在环含6例旧版对照，候选96例完整导航、4例预期停车、1例强打滑失败。100RPM纯导航约117～156秒，不含24次取放，未证明整场150秒或全图3mm精度。详见交接的模型假设与边界。

接线命令：[WIRING_AND_COMMANDS](docs/WIRING_AND_COMMANDS.md)。双板协议：[LIDAR_LINK](docs/LIDAR_LINK.md)。雷达：[lidar说明](lidar/README.md)。许可：[THIRD_PARTY](THIRD_PARTY.md)。
