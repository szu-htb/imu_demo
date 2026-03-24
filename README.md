# imu_demo

基于 BMI088 的 ROS 2 IMU 功能包，提供数据采集、陀螺仪零偏校准、姿态估计的完整 pipeline。采用 Component Composition 架构，采集与预处理节点运行在同一进程中以降低通信延迟。

当前版本已完成参数化，硬件与传感器配置统一从 [`config/bmi088.yaml`](./config/bmi088.yaml) 加载，`launch` 文件会通过包的 `share/imu_demo/config` 自动定位该配置文件。

## 架构

```
┌─ component_container（同进程）──────────────────────────────┐
│  ImuNode (默认 200Hz，可配置)                               │
│    SPI 读取 → 中值滤波(窗口3) → 陀螺仪零偏校准              │
│    ├─ /imu/data_raw          原始数据（诊断用）              │
│    └─ /imu/data_calibrated   滤波+校准后数据                │
└──────────────────────────────┬──────────────────────────────┘
                               │ /imu/data_calibrated
                               ▼
                    complementary_filter（独立进程）
                    ├─ /imu/data           姿态四元数
                    └─ /imu/rpy/filtered   欧拉角（debug）
```

## Topic 接口

| Topic | 类型 | QoS | 说明 |
|-------|------|-----|------|
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | BEST_EFFORT | 原始 6 轴数据（默认 200Hz，可配置） |
| `/imu/data_calibrated` | `sensor_msgs/msg/Imu` | RELIABLE | 陀螺仪零偏校准后的数据 |
| `/imu/data` | `sensor_msgs/msg/Imu` | — | 含姿态四元数（complementary filter 输出） |
| `/imu/rpy/filtered` | `geometry_msgs/msg/Vector3Stamped` | — | Roll/Pitch/Yaw 欧拉角 |

## 默认配置

默认参数定义在 [`config/bmi088.yaml`](./config/bmi088.yaml)：

- SPI 设备：`/dev/spidev1.0`（SPI_NO_CS 模式）
- GPIO 片选：加速度计 GPIO 394（Physical 24），陀螺仪 GPIO 396（Physical 26）
- 发布频率：`200 Hz`
- 加速度计配置：`±6g`，`200 Hz ODR`
- 陀螺仪配置：`±500 dps`，`200 Hz ODR`
- 校准时长：`3.0 s`

## 参数说明

| 参数名 | 含义 | 默认值 |
|-------|------|--------|
| `spi_device` | SPI 设备节点 | `/dev/spidev1.0` |
| `acc_cs_gpio` | ACC 片选 GPIO 编号 | `394` |
| `gyro_cs_gpio` | GYRO 片选 GPIO 编号 | `396` |
| `publish_rate_hz` | 节点发布频率 | `200` |
| `frame_id` | IMU 消息的 `frame_id` | `imu_link` |
| `calibration_duration_sec` | 启动时陀螺仪零偏校准时长 | `3.0` |
| `acc_range_g` | 加速度计量程，语义值 | `6` |
| `gyro_range_dps` | 陀螺仪量程，语义值 | `500` |
| `acc_odr_hz` | 加速度计 ODR，语义值 | `200` |
| `gyro_odr_hz` | 陀螺仪 ODR，语义值 | `200` |
| `spi_speed_hz` | SPI 时钟频率 | `5000000` |

说明：

- `acc_range_g`、`gyro_range_dps`、`acc_odr_hz`、`gyro_odr_hz` 都是语义值，上层不直接写寄存器值。
- 这些语义值会在驱动层映射到 BMI088 对应寄存器配置。
- 节点层只负责参数加载、发布频率和校准流程，不处理驱动层寄存器逻辑。

## 硬件依赖

本包与 **RDK X5 + BMI088** 硬件强耦合，默认接线如下：

可以使用`gpio readall` 来获取详细的引脚编号，具体可以见:
```
root@ubuntu:~# gpio readall
 +-----+-----+-----------+--RDK X5--+-----------+-----+-----+
 | BCM | xPi |    Name   | Physical |   Name    | xPi | BCM |
 +-----+-----+-----------+----++----+-----------+-----+-----+
 |     |     |      3.3v |  1 || 2  | 5v        |     |     |
 |   2 | 390 |     SDA.5 |  3 || 4  | 5v        |     |     |
 |   3 | 389 |     SCL.5 |  5 || 6  | 0v        |     |     |
 |   4 | 420 | I2S1_MCLK |  7 || 8  | TxD.1     | 383 | 14  |
 |     |     |        0v |  9 || 10 | RxD.1     | 384 | 15  |
 |  17 | 380 |  GPIO. 17 | 11 || 12 | I2S1_BCLK | 421 | 18  |
 |  27 | 379 |  GPIO. 27 | 13 || 14 | 0v        |     |     |
 |  22 | 388 |  GPIO. 22 | 15 || 16 | GPIO. 23  | 382 | 23  |
 |     |     |      3.3v | 17 || 18 | GPIO. 24  | 402 | 24  |
 |  10 | 398 | SPI1_MOSI | 19 || 20 | 0v        |     |     |
 |   9 | 397 | SPI1_MISO | 21 || 22 | GPIO. 25  | 387 | 25  |
 |  11 | 395 | SPI1_SCLK | 23 || 24 | SPI1_CSN1 | 394 | 8   |
 |     |     |        0v | 25 || 26 | SPI1_CSN0 | 396 | 7   |
 |   0 | 355 |     SDA.0 | 27 || 28 | SCL.0     | 354 | 1   |
 |   5 | 399 |   GPIO. 5 | 29 || 30 | 0v        |     |     |
 |   6 | 400 |   GPIO. 6 | 31 || 32 | PWM6      | 356 | 12  |
 |  13 | 357 |      PWM7 | 33 || 34 | 0v        |     |     |
 |  19 | 422 | I2S1_LRCK | 35 || 36 | GPIO. 16  | 381 | 16  |
 |  26 | 401 |   GPIO.26 | 37 || 38 | I2S1_DIN  | 423 | 20  |
 |     |     |        0v | 39 || 40 | I2S1_DOUT | 424 | 21  |
 +-----+-----+-----------+----++----+-----------+-----+-----+
 | BCM | xPi |    Name   | Physical |   Name    | xPi | BCM |
 +-----+-----+-----------+--RDK X5--+-----------+-----+-----+

```
## 依赖安装

```bash
sudo apt install ros-humble-imu-complementary-filter ros-humble-rclcpp-components
```
[imu_tools 仓库](https://github.com/CCNYRoboticsLab/imu_tools)

## 编译与运行

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select imu_demo
source install/setup.bash
ros2 launch imu_demo imu.launch.py
```

`imu.launch.py` 会自动从包安装目录加载参数文件：

```text
share/imu_demo/config/bmi088.yaml
```

如果需要修改默认配置，优先编辑 [`config/bmi088.yaml`](./config/bmi088.yaml)。

## 启动日志

节点启动后会打印三类关键信息：

- 已加载的 driver 配置：SPI 设备、片选 GPIO、量程、ODR、SPI 频率
- 已加载的 node 配置：`frame_id`、发布频率、实际 timer 周期、校准时长
- 驱动初始化结果：BMI088 是否成功完成初始化

如果驱动初始化失败，配置日志仍会先打印出来，便于排查 YAML 或硬件接线问题。

## 数据分析

`scripts/analyze_static.py` 用于离线分析录制的静态 bag 数据，输出统计量和时域图表。

**依赖（本地机器）：**
```bash
source /opt/ros/humble/setup.bash  # 提供 rosbag2_py、rclpy
pip install matplotlib numpy       # 若未安装
```

**用法：**
```bash
# bag 放到 data/imu/raw/ 下后，在 ros2_ws 根目录执行
python3 src/imu_demo/scripts/analyze_static.py data/imu/raw/<bag_name>

# 自定义实验名称
python3 src/imu_demo/scripts/analyze_static.py data/imu/raw/<bag_name> --name my_experiment
```

**输出目录** `data/imu/analysis/<timestamp>_<bag_name>/`：

```
├── plots/
│   ├── gyro_raw.png          陀螺仪原始时域图
│   ├── gyro_calibrated.png   陀螺仪校准后时域图
│   ├── acc_raw.png           加速度计原始时域图
│   ├── acc_calibrated.png    加速度计校准后时域图
│   └── rpy_filtered.png      欧拉角时域图（Yaw 含线性拟合）
├── summary.json              结构化统计量（零偏、噪声、漂移速率）
├── summary.txt               终端输出副本
└── command.txt               复现用的调用命令
```

完整采集和分析流程见 `data/imu/SOP.md`。

## 已知限制

- **Yaw 漂移**：BMI088 无磁力计，Yaw 仅靠陀螺仪积分，漂移不可避免。需要绝对航向时应外接磁力计
- **启动校准**：启动后会执行 `calibration_duration_sec` 指定时长的陀螺仪零偏校准。校准期间仅发布原始数据，设备须保持静止
