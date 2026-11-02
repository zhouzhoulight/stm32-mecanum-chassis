# nav1i 串口速查

USART1 PA9 TX/PA10 RX，115200 8N1，ASCII命令后回车。`!`单字符直接急停。全部配置RAM保存，复位后重新设置；`origin`只在已知静止物理标记处发。

## 连续中央路线（新功能）

起点为启停1原中心、车头图左，裸车。先阅读[完整交接](HANDOFF_NAV1I_20261102.md)，本版尚未实车验证。

```text
status
clear
mode base
algorithm cascade
control xy
odom delay 55 0
odom lever 195
origin
```

origin后等至少1秒，status确认valid=1/fault=0、反馈正常。初次低档核对几何/轮向通过后，目标档：

```text
limit 200
route plan 100
```

`route plan`不会运动，确认`[ROUTE_PLAN] valid=1`。场地和实物准备好才发：

```text
route start
```

整圈一次执行，不用主机逐条发goto。依次8次STATION，最后DONE；圈内禁止搬车、复位和重新origin。初次验收用`limit 100`+`route plan 50`，中档`limit 150`+`route plan 75`。50/75/100是百分比，不是RPM。停止后要从相同物理起点重新plan再跑，不支持故障断点续跑。

全程机械臂不动作；到站不自动等5秒。最终蓝区目标向场内各内缩20mm，量误差应对相应目标，见完整交接。

## 诊断、配置及旧单段

| 命令 | 作用 |
|---|---|
| `status` / `odom` / `perf` | 版本、状态、双里程计、反馈、指令限幅、DWT和路线状态；perf另有硬件循环/日志信息 |
| `imu` | IMU及校准服务状态 |
| `clear` | 停止完成、原因已查明后退出故障状态，不运动 |
| `mode base` / `mode fused` | 选择控制用里程计；两套一直记录 |
| `algorithm cascade` / `adrc` / `lqr` / `mpc` | 后三项也要完整algorithm前缀；route只支持cascade |
| `control xy` / `control hold` | 世界XY纠偏/航向保持；route要求xy |
| `limit 10/20/30/50/75/100/150/200` | 斜杠表示选一个数字；设置允许上限，不运动 |
| `odom scale 12924 13389` | 前后/横移输入脉冲每米，无真实距离数据不任意改 |
| `odom delay 55 0` | IMU/编码器有效延迟ms |
| `odom lever 195` | 有效旋转杆臂mm，改变后重新plan |
| `run left 100 5` | 初始世界左移100mm/5RPM；其他forward/back/right同理 |
| `goto 500 0 10` | 旧单段世界目标，mm/RPM，仍减速停稳 |
| `turn 90000 5` | 世界绝对航向90°，mdeg/RPM，5或10；方向必须现场核对 |
| `pose 1200 0 90000 50` | 旧慢速同步平移转向，非新高速连续route |
| `adrc b0 1000` | 单段ADRC比例，默认1000，不影响新route算法 |
| `stop` / `!` | 所有轮FE+失能；检查STOP_ACK，缺ACK不能说已物理停稳 |

旧run/goto/pose保留最多100RPM及距离门槛；设置limit200不让这些命令变成高速route。输入被拒时先看完整E/FAULT和status，不连续清零。

## IMU静止校准

车放平、静止、不接触：`imu tx on` → `imu cal acc` → 等约5秒及CALSW=0确认 → `imu cal check`（必要时）→ `imu drift`10秒 → `imu tx off`。`imu cal stop`用于停止校准；`imu save`显式保存到传感器，本轮默认不发。校准或漂移检查中不允许发运动命令。
