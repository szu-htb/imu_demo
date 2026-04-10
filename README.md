# imu_demo

基于 BMI088 内核 IIO 驱动的 ROS 2 IMU 功能包。

当前版本不再在用户态直接驱动 SPI/I2C，也不再负责 BMI088 初始化配置。节点只做三件事：

- 配置 IIO buffer 的 sysfs 节点
- 从 `/dev/iio:deviceX` 连续读取 scan frame
- 转换为 `sensor_msgs/msg/Imu` 并发布

这相当于把 MCU 工程里“应用任务自己读寄存器”的模式，切换为“底层驱动已经把数据搬进环形缓冲，上层任务只消费缓冲”。

## 架构

```text
BMI088 kernel driver
  -> IIO device + IIO buffer
  -> /sys/bus/iio/devices/iio:deviceX
  -> /dev/iio:deviceX

imu_demo::Bmi088IioReader
  -> 配置 scan_elements / buffer
  -> 阻塞读取 24-byte scan frame

imu_demo::ImuNode
  -> 原始值换算为 SI 单位
  -> 发布 /imu/data_calibrated
```

## 帧格式

当前内核驱动输出的单帧 scan 数据大小为 `24 bytes`：

- `accel_x/y/z`: `int16_t`
- `gyro_x/y/z`: `int16_t`
- `padding`: `4 bytes`
- `timestamp`: `int64_t`

其中时间戳直接来自 IIO buffer，而不是 ROS 节点收到数据时再现取 `now()`。

## 参数

默认参数见 [`config/bmi088.yaml`](./config/bmi088.yaml)：

| 参数名 | 含义 | 默认值 |
| --- | --- | --- |
| `frame_id` | 发布消息的 `frame_id` | `imu_link` |
| `output_topic` | IMU 输出 topic | `imu/data_calibrated` |
| `iio_device_path` | IIO 字符设备路径 | `/dev/iio:device1` |
| `iio_sysfs_dir` | IIO sysfs 目录 | `/sys/bus/iio/devices/iio:device1` |
| `buffer_length` | IIO kfifo 深度 | `128` |
| `poll_timeout_ms` | 用户态 poll 超时 | `200` |
| `acc_range_g` | 当前驱动配置的加速度计量程，用于 raw -> m/s² 换算 | `6` |
| `gyro_range_dps` | 当前驱动配置的陀螺仪量程，用于 raw -> rad/s 换算 | `500` |

注意：

- `acc_range_g` 和 `gyro_range_dps` 必须与你内核驱动当前实际配置一致。
- 如果内核侧后续补上标准 `scale` 属性，ROS 侧可以再进一步去掉这两个参数。

## 编译

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select imu_demo
source install/setup.bash
```

## 运行

```bash
ros2 launch imu_demo imu.launch.py
```

运行前需要确认：

- 内核 BMI088 IIO 驱动已加载
- `/sys/bus/iio/devices/iio:deviceX/name` 为 `bmi088`
- `/dev/iio:deviceX` 存在且可读

## 当前限制

- 当前节点只消费固定全通道 scan layout
- 量程换算仍依赖 YAML 参数，而不是从 IIO `scale` 自动读取
- buffer 的采样频率、硬件时间戳语义仍以当前内核实现为准
