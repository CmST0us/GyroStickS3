# StickS3 Gyroflow 陀螺仪记录器

基于 ESP-IDF 的 **M5Stack StickS3** 固件：挂在相机热靴上，用板载 BMI270 记录陀螺仪（+加速度计）数据，
导出为 [Gyroflow](https://gyroflow.xyz) 的 `.gcsv` 文件，用于视频防抖。

- 单击主按键 **开始 / 停止**录制，长按 **休眠 / 唤醒**
- 为长时间记录做了功耗和存储优化：录制时 CPU 轻睡眠、屏幕只亮几秒、日志压缩存储
- 无需 SD 卡：数据存放在板载 8 MB Flash 的专用分区（约 7.3 MB 可用）
- 通过 USB 用 `tools/gyrostick.py` 一条命令取回并转换为 `.gcsv`

> **状态说明**：固件已在 ESP-IDF v5.5.5 下编译通过，并在 QEMU 里用模拟 IMU 端到端验证了
> “录制 → 压缩写入 Flash → USB 协议读回 → 解码为 .gcsv”整条数据链路（见下方[测试](#测试)）。
> **LCD、按键、PMIC、BMI270 真机行为、功耗数据没有在真机上验证过**（开发环境里没有硬件），
> 相关假设和核对来源写在 [docs/HARDWARE.md](docs/HARDWARE.md)。第一次上机请先按“[首次上机检查](#首次上机检查)”走一遍。

## 按键

| 按键 | 操作 | 功能 |
|---|---|---|
| 主按键（默认前面板 KEY1 / GPIO11） | 单击 | 开始 / 停止录制 |
| | 长按（1.2 s） | 录制中先保存再休眠；休眠中唤醒 |
| 辅助按键（默认侧面 KEY2 / GPIO12） | 单击 | 状态页：电量、剩余空间/时长、实时加速度（用于检查轴向） |
| | 长按 | 擦除菜单：主按键单击 = 确认，辅助按键单击 = 取消 |

> “蓝色按键”如果是侧面那一个，在 `menuconfig` 里把主/辅按键的 GPIO 对调（11 ↔ 12）即可。
> 休眠唤醒用的是主按键，所以它必须是 RTC GPIO（0–21），11 和 12 都满足。

屏幕只在按键操作后亮几秒（可配置），录制期间屏幕是熄灭的以省电；想看进度单击辅助按键。
休眠期间误碰不会开机：唤醒后必须把长按做完，否则立刻回到休眠。

## 安装方向与轴向（orientation）

固件把 BMI270 的**原始**三轴数据写进文件，并在文件头写入 Gyroflow 的 `orientation` 字符串，
由 Gyroflow 负责换算到相机坐标。`menuconfig → StickS3 Gyroflow logger → How the StickS3 sits on the camera` 提供预设
（视角：站在相机后面看）：

| 预设 | orientation |
|---|---|
| **平放在热靴上，屏幕朝上，USB-C 朝右**（默认） | `Zxy` |
| 竖立在热靴上，屏幕朝向摄影师，USB-C 朝右 | `YxZ` |
| 平放，屏幕朝上，USB-C 朝左 | `ZXY` |
| 竖立，屏幕朝向摄影师，USB-C 朝左 | `yXZ` |
| 自定义 | 自己填 |

字符串是根据 BMI270 在 StickS3 上的轴向（X 指向 USB 口，Y 指向竖屏时屏幕右侧，Z 垂直屏幕向外）和 Gyroflow 的
IMU 坐标约定推出来的，推导过程见 [docs/HARDWARE.md](docs/HARDWARE.md)。这两个前提都来自公开资料、没有实物验证，
所以 **导入 Gyroflow 后请确认一次**：如果陀螺仪曲线和光流曲线对不上，用 Gyroflow 的
“Guess IMU orientation”或手动改 IMU orientation 即可，不需要重录。

屏幕 UI 方向跟随预设：USB 口在右边时屏幕内容旋转为横屏，文字从左往右读。

## 编译与烧录

需要 ESP-IDF **v5.5.x**。

```bash
. $IDF_PATH/export.sh
idf.py set-target esp32s3
idf.py menuconfig          # 可选：StickS3 Gyroflow logger 菜单（采样率、量程、安装方向、按键、休眠时间…）
idf.py build flash monitor
```

默认配置：200 Hz，陀螺仪 ±1000 dps，加速度计 ±8 g，记录 6 轴。
Flash 布局见 `partitions.csv`：应用 512 KB，其余约 7.4 MB 全部给追加式日志分区 `gyrolog`。

如果设备在休眠、电脑看不到串口，先长按主按键唤醒（USB 供电时不会自动休眠）。

## 取回数据

```bash
pip install pyserial
python tools/gyrostick.py settime            # 拍摄前同步一次设备时钟（文件名和 timestamp 用）
python tools/gyrostick.py list               # 看设备上有哪些录制
python tools/gyrostick.py pull -o ./logs     # 下载并转换为 .gcsv（每次录制一个文件）
python tools/gyrostick.py pull -o ./logs --erase   # 导出成功后清空设备
```

- 输出文件名形如 `GYRO_0003_20261004_141530.gcsv`。把它改成和视频同名（`C0001.MP4` → `C0001.gcsv`）放在同一目录，
  Gyroflow 会自动识别；也可以手动加载。
- `convert` 子命令可以转换 `dump` 或 `esptool read_flash 0x90000 0x770000 part.bin` 得到的分区镜像。
- `--orientation XYZ` 可覆盖固件写入的 orientation；`--sessions 2,3` 只导出部分录制。

### 时间戳与同步

时间戳取自 BMI270 自己的传感器时钟（39.0625 µs 计数器），采样间隔绝对均匀；录制期间丢失的数据（FIFO 溢出）
会被检测出来并还原成时间轴上的空洞。视频和陀螺仪之间的偏移请照常在 Gyroflow 里同步；
BMI270 时钟相对相机时钟可能有零点几个百分点的漂移，长片段建议在开头和结尾各做一个同步点。
`pull --mcu-clock` 可以用 MCU 晶振修正时间标尺，但只有关闭“录制时轻睡眠”（见下）录制的数据才准确，默认不建议用。

## 存储容量

日志用 Delta + 自适应 Rice 编码压缩（[docs/FORMAT.md](docs/FORMAT.md)），可用约 7.3 MiB。
下表用**合成的手持运动数据**测得（真实数据取决于抖动强度和传感器噪声，仅供参考）：

| 配置 | 约占用 | 约可录制 |
|---|---|---|
| 200 Hz 6 轴（默认） | 0.7–1.0 KB/s | 2–3 小时 |
| 200 Hz 仅陀螺仪 | 0.43 KB/s | 约 5 小时 |
| 100 Hz 6 轴 | 0.5 KB/s | 约 4 小时 |
| 400 Hz 6 轴 | 1.5 KB/s | 约 1.4 小时 |

设备在就绪页和录制页显示剩余空间和估算剩余时长。存满会自动停止并保存，不会覆盖旧数据。

## 功耗设计

下面是设计思路和估算，**没有实测**：

- **休眠**：长按后 ESP32-S3 进入 deep sleep，LCD 电源（PM1 的 L3B 轨）和背光关闭，引脚保持低电平避免漏电，
  BMI270 挂起。靠主按键低电平唤醒。ESP32-S3 本身 deep sleep 约 10 µA 量级，整机待机电流取决于板上 PMIC 等器件。
- **录制**：BMI270 把数据攒在 2 KB FIFO，每 160 ms 唤醒读一次（FIFO 能存约 0.78 s，留足 Flash 写页的停顿裕量）；
  两次读取之间 CPU 进入 light sleep。屏幕熄灭、不用 PSRAM、不开 Wi-Fi/蓝牙，CPU 80 MHz。
  预计平均几毫安（主要是 BMI270 的陀螺仪约 0.7 mA 和 I2C 读取），录制时长受 Flash 容量限制（见上）而不是电池。
- **闲置**：就绪后 60 s 无操作自动 deep sleep（USB 供电 / 正在和电脑通信时不休眠）。
- **低电量**：低于 3.35 V 自动停止、保存并休眠，避免丢数据。

`menuconfig` 里可以关闭 “Use CPU light sleep between FIFO reads”：这样 MCU 晶振全程准确，
可以配合 `pull --mcu-clock` 校准 BMI270 时钟，代价是录制电流高一个数量级。

## 首次上机检查

1. 烧录后按一下辅助按键：状态页应显示 `IMU:OK`，电量合理，最后一行是加速度（g，芯片坐标）。
   屏幕朝上平放时第三个数应约为 `+1.0`；USB 口朝下立起来时第一个数应约为 `-1.0`。不符合说明
   BMI270 轴向假设有误，见 [docs/HARDWARE.md](docs/HARDWARE.md) 里的自定义 orientation 方法。
2. 单击主按键录一小段，再 `gyrostick.py pull`，打开 `.gcsv` 看数据是否随晃动变化。
3. 装到相机上拍一段，在 Gyroflow 里确认 orientation 和同步。

## 目录结构

```
main/                应用：状态机、BMI270 驱动、录制、LCD/UI、按键、USB 协议、休眠
components/gyl/      日志页格式 + 压缩编解码（纯 C，主机可测）
components/imu_fifo/ BMI270 FIFO 帧解析（纯 C，主机可测）
components/bmi270_api/ Bosch BMI270 官方驱动（BSD-3-Clause，仅用于初始化和配置固件上传）
tools/gyrostick.py   主机工具：取数、解码、导出 .gcsv
test/                主机单元测试、QEMU 端到端测试、UI 预览
docs/                数据格式、硬件笔记
```

## 测试

```bash
make -C test/host check        # 编解码往返、FIFO 解析、按键状态机（C，带 ASan/UBSan）
python3 test/test_tool.py      # C 编码器 ↔ Python 解码器交叉验证、时间轴重建、.gcsv 导出
make -C test/host previews     # 用真实的 ui.c 把每个界面渲染成 PPM 图片
# 需要 ESP-IDF 环境 + qemu-system-xtensa：
python3 test/qemu/run_selftest.py
```

QEMU 测试用 `CONFIG_GYROLOG_SIM_IMU`（带一次丢帧和 24 位时间计数器回绕的模拟 FIFO）和
`CONFIG_GYROLOG_SELFTEST`（脚本化录制两次）构建固件，在 QEMU 里启动后通过模拟串口用 `gyrostick.py`
和**真实的固件主机协议代码**对话：读回页面、转换成 `.gcsv`、检查时间间隔 / 丢帧还原 / 数据波形、擦除。

## 第三方代码

- Bosch Sensortec BMI270 SensorAPI（BSD-3-Clause），`components/bmi270_api/`
- `font8x8_basic.h`（Daniel Hepper，公有领域），`main/font8x8_basic.h`
