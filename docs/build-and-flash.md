# 编译、烧录与 VS Code

本地已验证环境为macOS：Python3、VS Code Keil Studio Pack、CMSIS-Toolbox 2.15.0、Arm GNU GCC13.3.1、Keil STM32F4xx_DFP3.1.1、pyOCD0.45.1。依赖版本是本次工程的验证组合，不代表工具当前最新版。Windows串口控制台尚未移植，Python控制台依赖POSIX termios。

## 安装与编译

1. 安装VS Code推荐的`arm.keil-studio-pack`扩展，安装CMSIS工具及指定GCC和芯片包。
2. 用本仓库的`F407底盘测试.code-workspace`打开工程。
3. 运行 `python3 scripts/check.py --firmware`。它执行ASan/UBSan主机测试、Python测试、CMSIS构建与实际ELF/向量检查，不接触主板。

`scripts/project.py`支持工具完整路径环境变量：`CHASSIS_GCC`、`CHASSIS_CBUILD`、`CHASSIS_PYOCD`。如果不设置，会查找本机VS Code扩展、macOS STM32CubeIDE安装目录和PATH；不包含原作者用户名路径。CMSIS工程固定GCC13.3.1，脚本会据检测到的版本设置`GCC_TOOLCHAIN_*`。

```sh
export CHASSIS_GCC=/path/to/arm-none-eabi-gcc
export CHASSIS_CBUILD=/path/to/cbuild
export CHASSIS_PYOCD=/path/to/pyocd
python3 scripts/project.py build
```

构建输出在`build/`与`out/`，不纳入Git。生成的`.cmsis/tools-environment.yml`等本机配置也不上传。首次编译需要芯片包已安装或可由Keil工具下载，工具链和安装器不打包进仓库。

## 下载

先核对芯片F407VET6、BOOT0=0及SWD接线，独立处理电机供电和现场停止条件。烧录命令：

```sh
python3 scripts/project.py probes
python3 scripts/project.py flash --probe auto
```

只有一个调试器时使用auto；多个设备必须指定完整UID。输入`FLASH`才下载。脚本先编译，失败不烧旧文件；使用512KiB目标配置、扇区擦写、禁止自动解除读保护。脚本不自动备份原程序。下载后保持暂停，按RESET或调试器继续才运行。

`firmware/`下归档的BIN/HEX来自现场使用版本，附SHA-256。BIN加载地址0x08000000。ELF包含本机调试路径，因此未作为公开历史固件发布；可自行编译获得。修改路径/工具版本后的新构建不保证逐字节等于归档BIN，应分别记录构建与读回结果。

## 控制台与F5

```sh
python3 scripts/project.py ports
python3 scripts/project.py console --port auto
```

先输入`status`或`help`，收到主控回复再查驱动器。Ctrl+C尝试发送stop后关闭端口；不要让两个程序抢同一串口。

F5配置需要先在启动VS Code的环境里设置`CHASSIS_GDB`和`CHASSIS_PYOCD`为完整可执行文件路径；`CHASSIS_GDB`指向`arm-none-eabi-gdb`。这组可移植F5路径配置已做静态检查，未在另一台电脑重新验证。未设置时先用编译/下载任务，不使用F5。

F5的预任务会编译、覆盖板内程序并停在main。电机动作中不能打断点：CPU暂停会暂停软件停止期限，外部电机驱动器仍可能执行已收到的命令。
