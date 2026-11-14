# 当前整车接线与调试命令

> 2026-11-14更新：当前候选主控v2.7.38、雷达0.3.5_FRONT150；本次未烧录/发车。接线仍沿用本文，版本与状态以最新交接为准。见[最新详细交接](HANDOFF_V2738_20261114.md)。

对应集成主控v2.7.31＋雷达板0.3.3，两板均为F407VET6/HSE25MHz。通信3.3V TTL、8N1，设备共地，不随意并联电源输出。

## 主控引脚

| 功能 | 主控引脚 | 对端 |
| --- | --- | --- |
| 电脑USART1，115200 | PA9 TX、PA10 RX | 适配器RX→PA9、TX→PA10 |
| 五Emm共享UART4，115200 | PA0 TX、PA1 RX | 驱动器RX←PA0、TX→PA1；四轮ID4/1/3/2、升降ID5 |
| 雷达板USART2，230400 | PA2 TX、PA3 RX | 主控PA2→雷达PB11；雷达PB10→主控PA3 |
| WT901CTTL USART3 | PD8 TX、PD9 RX | PD8→WT901 RX、WT901 TX→PD9，按当前IMU配置 |
| 视觉UART5 | PC12 TX、PD2 RX | PC12→上位机RX、TX→PD2，协议见nav_vision.c |
| 夹爪/云台/物料盘/摄像头 | PC6/PC7/PC8/PC9 | 舵机1/2/3/4，TIM3、50Hz |
| SWD | PA13、PA14、GND、可选NRST | SWDIO、SWCLK、共地；正常运行BOOT0低 |

舵机电源按型号供电，与主控共地；不得将负载接到下载器3.3V输出。共享电机总线用原已配置连接，不能把电脑TX再并到驱动器TX上。雷达板存储与LD19接线见[lidar说明](../lidar/README.md)。

## 坐标与当前实测值

车体+X朝前、+Y朝左，IMU−Y朝右。启停区1车头朝图左时，导航起点系+X朝图左、+Y朝图下；不可混用地图左下原点坐标与导航起点坐标。

雷达中心在车中心前120mm、Y=0；原始270°朝车前、180°朝车右。需镜像与yaw同时处理。

当前PWM：夹爪闭1560/开1850，云台内1000/外2130，料盘1/2/3为550/1450/2350µs。历史安全升降高度LOCAL−2306脉冲仅在正确实体参考下有效；取放高度以有效Flash标定为准。驱动器断电后计数不能直接当原物理绝对位置。

## 主控常用命令

各命令以换行结尾。下表不是自动发车脚本，修改配置通常要求整车与机械臂空闲。

| 命令 | 用途 |
| --- | --- |
| `status` / `odom` / `perf` / `imu` | 状态、里程计、运算开支、IMU |
| `stop` 或 `!` | 整车停止并中止任务 |
| `clear` | 停止完成后清状态，不补回丢失原点 |
| `origin` | 当前已确认实体起点声明，禁止跑图途中反复调用 |
| `algorithm cascade` / `algorithm adrc` / `algorithm lqr` / `algorithm mpc` | 选择控制律；MPC暂禁整圈 |
| `mode base` / `mode fused` | 两种里程计，均不自动提供绝对XY |
| `control xy` | XY路径跟踪 |
| `limit 100` | 轮速上限，不直接运动 |
| `run forward 100 5` | 前进100mm、5RPM；同理back/left/right |
| `pose X_MM Y_MM YAW_MDEG RPM` | 平移转向到导航目标 |
| `turn YAW_MDEG RPM` | 导航绝对航向，不是相对角增量 |
| `route sensorplan 50` | 用雷达和地图在MCU上规划50%档，不运动 |
| `route preflight` / `route start` | 前检/启动有效路线；start会运动 |
| `route pause` / `route continue` | 任务边界暂停/继续，不重建原点 |
| `mission status` / `mission stop` | 完整任务状态/停止 |

完整取放还要求有效规划、segmented及QR、等高平面确认、机械臂参考和避让姿态；入口见nav_mission.c。`mission flat rough`声明两个平面等高；`mission start`会启动两批完整任务，不是只读检查。

## 雷达配置与快照

当前安装的静态配置如下，等待新扫描、配置应答和同步有效后才启用避障：

```text
lidar mirror on
lidar mount 120 0 90000
lidar self 145 145 145 145
lidar memory on
lidar status
lidar obstacle on
route avoid on
```

外参改变会清除旧坐标会话；配置需重启后核对，不假定已持久保存。memory是RAM障碍记忆，不是W25Q128永久地图。停止后优先保留以下输出：

```text
status
odom
lidar status
lidar tracks
lidar memory list
lidar hazard
lidar hazard live
lidar hazard near
lidar hazard memory
lidar plan
lidar plan live
lidar plan memory
```

LIDAR_CLOCK细分请求、接受、RTT拒绝、处理耗时拒绝、旧包、入队失败及跳变；HAZARD是实际触发瞬间，PLAN是冻结规划输入，不能只拿停车后的最新扫描判断。

## 机械臂命令

集成固件使用`arm `前缀避免同名。以下ID为1夹爪、2云台、3料盘、4摄像头。

| 命令 | 用途 |
| --- | --- |
| `arm status` / `arm points` | 姿态、限位、参考、标定点 |
| `arm probe` | 只读配置/位置/目标/误差等 |
| `arm r` / `arm flags` / `arm tpos` / `arm perr` / `arm current` | 单项只读 |
| `arm seed 1 1560` | 声明已知当前姿态并立即输出，不是角度测量 |
| `arm servo 1 1850` / `arm j 2 20` | 缓变到PWM/相对增加20µs |
| `arm rate 2 700` | 设置变化率µs/s |
| `arm turnlimits 1000 2130` | 云台范围，不是夹爪范围 |
| `arm jawlimits 1380 2260` | 夹爪允许范围，接触值仍须实测 |
| `arm recover zero` | 仅当前确在标定实体零点时恢复参考，不运动 |
| `arm recover clearance` | 仅当前确在标定避让高度时恢复参考，不运动 |
| `arm jog -50` / `arm jog 50` | 相对升降；本机构通常负向上、正向下 |
| `arm limits LOW HIGH` / `arm speed RPM` / `arm accel VALUE` | 软限位、速度、加速度 |
| `arm arrival app 20` | 应用容差20脉冲，不是实际精度保证 |
| `arm motion fast` | 机械臂快速模式 |
| `arm grab 2` | 车外抓入2号盘 |
| `arm rough 2` | 2号盘取出放粗加工平面 |
| `arm unrough 2` | 车外粗加工取回2号盘 |
| `arm stack 2` | 2号盘取出按码垛高度放置 |
| `arm point tray` / `arm point rough` / `arm point stack` / `arm point swing` | 当前位置记录工位高度，不换参考 |
| `arm point grabpick` / `arm point grabdrop` | 分别标定grab抓取/放盘 |
| `arm point view` / `arm point qr` | 标定俯视/全局及二维码升降高度 |
| `arm height view` / `arm height qr` | 到已标记相机高度，未标记会拒绝，不影响普通路线 |
| `arm save` | 写Flash，不等于驱动器跨断电定位 |

`arm measure on`会释放升降力矩，需先支撑机构；之后可`arm measure mark pick`/`clearance`/`drop`、`arm measure list`、`arm measure hold`。hold重新取得力矩并检查位置，若发生位移先处理原因，不反复清零掩盖。完整语法以`arm ?`及arm_task.c为准。
