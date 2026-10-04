# StickS3 硬件笔记与 orientation 推导

这份笔记记录固件用到的硬件事实、它们的来源，以及哪些是假设。**以下均未在真机上验证**——
开发环境没有硬件，只能交叉核对公开的开源实现。

## 板上资源（固件用到的部分）

| 资源 | 值 | 来源 |
|---|---|---|
| SoC | ESP32-S3-PICO-1-N8R8（8 MB Flash，8 MB PSRAM，固件不启用 PSRAM） | [thc1006/zephyr-m5stack-sticks3](https://github.com/thc1006/zephyr-m5stack-sticks3) |
| 按键 | KEY1 = GPIO11，KEY2 = GPIO12，低电平有效 | 同上；[M5Unified](https://github.com/m5stack/M5Unified) `M5Unified.inl` 的 StickS3 分支 |
| 内部 I2C | SDA = GPIO47，SCL = GPIO48 | M5Unified `_pin_table_i2c_ex_in`；[M5GFX](https://github.com/m5stack/M5GFX) 的 StickS3 自动识别 |
| IMU | BMI270，I2C 0x68，无磁力计 | Zephyr 移植 README；[whitefirer/sticks3-toolkit](https://github.com/whitefirer/sticks3-toolkit) |
| 电源管理 | M5PM1，I2C 0x6E | [m5stack/M5PM1](https://github.com/m5stack/M5PM1) |
| LCD | ST7789P3 135×240，SPI：MOSI 39、SCLK 40、DC 45、CS 41、RST 21、背光 38（PWM）；偏移 x=52 y=40，反色开 | M5GFX `M5GFX.cpp` |
| LCD 供电 | M5PM1 GPIO2（"L3B" 轨）输出高 | M5GFX 同一段代码 |

没有 SD 卡槽，所以日志存放在板载 Flash 的自定义分区里（`partitions.csv`）。

### M5PM1 寄存器（来自 M5PM1 官方库头文件）

| 寄存器 | 用途 |
|---|---|
| 0x00 | 设备 ID（用来探测芯片是否应答） |
| 0x06 bit0 | `CHG_EN` 充电使能，固件启动时置 1（官方库注明复位事件会清零该位） |
| 0x09 | `I2C_CFG`，写 0 关闭 PM1 的 I2C 空闲睡眠（M5GFX / M5Unified 同样处理） |
| 0x0A | `WDT_CNT`，写 0 关闭 PM1 看门狗 |
| 0x10/0x11/0x13/0x16 | GPIO 模式 / 输出 / 驱动 / 功能，GPIO2 = LCD 轨，GPIO3 = 喇叭功放使能（保持低） |
| 0x22–0x23 | 电池电压，mV（16 位，小端；真机实测满电读到 `0x1068` = 4200 mV） |
| 0x24–0x25 | USB（VIN）电压，mV（16 位，小端；真机插着 USB 读到 `0x1496` = 5270 mV）；固件用 ≥ 4 V 判断是否插着 USB |

## BMI270 轴向与 Gyroflow orientation

### 1. BMI270 芯片轴相对机身

[M5Unified 的 IMU 代码](https://github.com/m5stack/M5Unified) 对 StickS3 写了 “BMI270: X=+Y, Y=-X, Z=+Z”，
即 M5 的标准机体坐标 = (芯片 Y, 芯片 −X, 芯片 Z)。M5 的标准机体坐标按（竖屏、USB 口在下）定义为
X 向右、Y 向上、Z 垂直屏幕向外（这一条是本项目的假设，没有找到官方文字说明）。因此芯片轴为：

- 芯片 **X**：指向 USB 口一端（M5 的 −Y）
- 芯片 **Y**：竖屏时屏幕右侧（M5 的 X）
- 芯片 **Z**：垂直屏幕向外

固件直接写入芯片原始值，不做软件换算，轴向全靠文件头的 `orientation` 字段表达。

### 2. Gyroflow 期望的 IMU 坐标

`.gcsv` 的 `orientation` 三个字母表示：输出的第 i 个轴取输入的哪个轴，小写 = 取反
（[gyroflow `imu_transforms.rs`](https://github.com/gyroflow/gyroflow) 的 `orient()`：`out[i] = ±in[letter_i]`）。
输出轴本身（"XYZ"）对应的相机方向，从两个独立的线索推出，结论一致：

- **线索 A（代码）**：积分器先做 `(gx,gy,gz) → (-gy, gx, gz)`，最后在 `frame_transform.rs` 里对旋转矩阵做
  `diag(-1,1,1)` 共轭后直接作用在图像射线（x 右、y 下、z 前）上。化简后 Gyroflow 的 "XYZ" 坐标是
  **X = 向上，Y = 向左，Z = 向后（朝摄影师）**，右手系。
- **线索 B（文档）**：[gcsv 文档](https://docs.gyroflow.xyz/app/technical-details/gcsv-format) 说示意图（X 右、Y 上、Z 向后）对应 `YxZ`。
  把图里的轴当作 IMU 原始轴代入 `YxZ`：X' = IMU Y = 上，Y' = −IMU X = 左，Z' = IMU Z = 后——和线索 A 完全一致。

### 3. 预设

相机方向记为 右 R、上 U、后 B（朝摄影师）、左 L、前 F。

| 安装 | 芯片 (X, Y, Z) 对应相机方向 | 求 (上, 左, 后) | orientation |
|---|---|---|---|
| 平放，屏幕朝上，USB 朝右 | (R, F, U) | (Z, −X, −Y) | `Zxy` |
| 竖立，屏幕朝后，USB 朝右 | (R, U, B) | (Y, −X, Z) | `YxZ` |
| 平放，屏幕朝上，USB 朝左 | (L, B, U) | (Z, X, Y) | `ZXY` |
| 竖立，屏幕朝后，USB 朝左 | (L, −U, B) | (−Y, X, Z) | `yXZ` |

### 4. 如果真机和假设不符：自己推导

1. 把设备按实际安装姿态（相机水平）静置，在辅助按键的状态页读 (ax, ay, az)：读数约 +1 的那个轴指向**上**
   （读数 −1 则该轴指向下）。
2. 把相机前端抬起（镜头朝天）：此时读数约 +1 的轴指向**前**（镜头方向）；它的反方向就是**后**（摄影师一侧）。
3. 第三个轴由右手定则确定：**左 = 后 × 上**（Gyroflow 的坐标是 上、左、后，右手系：上 × 左 = 后）。
4. 写 orientation：第一个字母 = “上”对应的芯片轴，第二个 = “左”，第三个 = “后”，对应芯片轴反向就小写。
   在 `menuconfig` 选 “Custom” 填入，或者 `gyrostick.py pull --orientation ...` 覆盖，或者直接在 Gyroflow 里改。

示例（平放、屏幕朝上、USB 朝右）：静置时 az ≈ +1 → 芯片 Z = 上；镜头朝天时 ay ≈ +1 → 芯片 Y = 前，所以后 = −Y；
左 = 后 × 上 = (−Y) × Z = −X；字母依次为 上=`Z`、左=`x`（−X）、后=`y`（−Y）→ `Zxy`。

## 低功耗相关的硬件要点

- ESP32-S3 的 GPIO 在 light sleep 时默认被“隔离”（输入输出和上下拉全部关闭）。固件对唤醒用的按键引脚用
  `gpio_wakeup_enable`（会让该引脚保持正常配置），对 LCD 引脚调用 `gpio_sleep_sel_dis` 让它们在 light sleep 里继续输出低电平。
- Deep sleep 前 LCD 相关引脚设为低并 `gpio_hold_en`，LCD 轨（PM1 GPIO2）关闭，防止向断电的屏幕倒灌电流。
- 主按键用 ext1（低电平）从 deep sleep 唤醒，需要 RTC GPIO；唤醒后固件要求长按做完才开机。
- 屏幕背光 PWM 和 LCD 的 SPI 外设只在屏幕亮着时初始化，熄屏即释放。
