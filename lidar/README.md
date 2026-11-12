# LD19独立处理板（STM32F407VET6）

当前`LD19_BENCH_F407VE_0.3.3`，协议版本2，25MHz HSE/168MHz SYS。整车运行时由主控接收特征、跟踪、视野与时钟数据，主控自己做规划。电脑雷达可视化软件不随此次STM32源码发布。

## 接线

| 功能 | 雷达板引脚 | 对端/要求 |
| --- | --- | --- |
| LD19数据，230400 8N1 | PA3 USART2 RX | LD19 TX→PA3；LD19供电按其原线束/手册，GND共地 |
| 板间USART3，230400 8N1 | PB10 TX、PB11 RX | PB10→主控PA3，主控PA2→PB11，另接GND |
| 电脑调试USART1，460800 8N1 | PA9 TX、PA10 RX | 适配器RX→PA9、TX→PA10 |
| W25Q128 SPI1 | PA5/PA6/PA7、PC13 CS | SCK/MISO/MOSI；PC13仅作CS，不能作为LED |
| 24C02 I²C1 | PB6 SCL、PB7 SDA | 已上拉至3.3V；7位地址0x50，HAL移位为0xA0 |
| SWD | PA13/PA14、GND、可选NRST | DAPLink；正常BOOT0低 |

不要凭颜色猜LD19未标注的四线定义，保留已实测有效的线束；本固件使用雷达TX数据，未提供转速控制PWM。普通数据/逻辑引脚不得接电机电源。主控与雷达板电源输出不并接。

## 算法与界限

- `APP/Src/ld19_parser.c`：47字节、每包12点、CRC验证与重同步。
- `scan_builder.c`：分圈与时间整理；`features.c`：滤波、线/点簇及轨迹证据。
- `motion_history.c`：利用主控运动提示去畸变；`pose_match.c`、`shadow_localizer.c`：候选相对定位。由运动提示派生的shadow不能再冒充独立绝对位置。
- `app.c`：电脑与整车链路隔离，FOV、分批轨迹与时钟回包。
- TIM2为1MHz微秒时基；DMA缓冲在普通SRAM，不能放CCM。
- 当前安装：雷达在车中心前120mm、Y=0，原始270°朝前、180°朝右；可见180..360°及后方88..92°窄窗，不是完整360°无遮挡。
- 主控的最多48个圆形障碍记忆在RAM；W25Q128和24C02目前保留存储检测，并未实现永久建图/完整SLAM。

构建：仓库根目录`python lidar/tools/build.py`，工具从PATH或ARM_GCC/CMAKE/NINJA环境变量寻找。只编译、不连接硬件。产物在`lidar/build/Release/`，发布BIN/HEX在`firmware/current/`（相对此目录）。本次43596字节BIN与此前烧录0.3.3镜像相同，哈希见总交接；新构建本次未下载。

接手先读[总交接](../docs/HANDOFF_F407_LIDAR_20261112.md)及[协议](../docs/LIDAR_LINK.md)。当前自身边缘回波和原料盘边缘仍存在误停反例，不代表已完成整圈避障实测。
