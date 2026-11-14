# LD19 独立处理板（STM32F407VET6）

当前LD19_BENCH_F407VE_0.3.5_FRONT150，协议2，25MHz HSE/168MHz SYSCLK。源码在firmware/，发布BIN/HEX和manifest在firmware/current/。本次只重建，没有下载。

| 功能 | 雷达板引脚 | 对端 |
|---|---|---|
| LD19输入，230400 | PA3 USART2 RX | LD19 TX，沿用已核对线束和5V供电，不按颜色猜线 |
| 板间链路，230400 | PB10 TX / PB11 RX | 主控PA3 RX / PA2 TX，另接GND |
| 电脑调试，460800 | PA9 TX / PA10 RX | 适配器RX / TX，另接GND |
| W25Q128 | PA5 SCK/PA6 MISO/PA7 MOSI，PC13 CS | PC13不能当LED |
| 24C02 | PB6 SCL / PB7 SDA | 3.3V上拉，7位地址0x50 |
| SWD | PA13 / PA14 / GND | SWDIO / SWCLK / 共地 |

普通引脚不能接电机电源，两路独立电源输出不并联。

- 原始47字节/12点/CRC8，分圈、滤波、线/点特征、跨圈跟踪、运动去畸变。
- 原始270°朝前、180°朝右，中心X+120mm、Y0。只用原始195°..345°即车前150°；后方窄窗已停用。
- 0x76提供线段几何；时钟/配置通过完整帧优先队列发送。主控拥有地图、历史投影、记忆与九路口规划；雷达不替主控生成路线。
- 主控运动提示用于去畸变，不是独立定位真值；shadow/pose_match不等于已接入的全局定位。
- 主控48个紧凑障碍在RAM；W25Q128/24C02诊断不等于永久地图或完整SLAM。
- 二维码板专用关联尚待实现/验收，不能整块忽略，见[总交接第9节](../docs/HANDOFF_V2738_20261114.md)。

仓库根目录python lidar/tools/build.py只编译，工具从PATH或ARM_GCC/CMAKE/NINJA获取。重建BIN44868B，哈希见build.json；本次未烧录。
