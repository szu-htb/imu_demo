# DRDY 硬件中断驱动采样

**状态：** 已确认，待实现
**创建日期：** 2026-03-23

## 目标

用 BMI088 GYRO 的 DRDY 硬件中断替代 `sleep_until` 软件定时，将采样时间戳误差从"随机 0~6ms"降为"固定 ~30μs"，同时消除采样间隔不均匀导致的姿态积分误差。

## 背景与约束

- 当前：`sleep_until` 1ms 循环，抖动 std ~80μs，最差 6ms
- libgpiod 不可用，只能用 sysfs edge detection + `poll()`
- sysfs GPIO 在 Linux 中已标记为 Deprecated，但 RDK X5 上可用（已验证）
- GPIO 编号为软件概念，可能随系统更新变化，需用 `hb_gpioinfo` 验证

## 设计决策（已确认）

| 决策 | 原因 | 被否决的备选方案 |
|------|------|----------------|
| 只等 GYRO DRDY（1000Hz） | GYRO 是姿态积分主力；ACC 800Hz，每 5 帧重复 1 帧可接受 | 双中断、等 ACC DRDY |
| 单线程阻塞 poll() | poll() 直接在 kernel 态睡眠；DRDY 触发后内核直接唤醒，无中间线程切换 | 独立中断线程 + condvar |
| Bridge 模式抽象总线 | 后续可能换 I2C；传感器协议和总线传输独立变化 | 不抽象，直接改 |
| sysfs edge detection | libgpiod 不可用；sysfs 已验证可工作 | libgpiod（不可用） |

## 硬件信息

| 信号 | BMI088 引脚 | RDK X5 BCM | Linux GPIO | Physical Pin |
|------|------------|-----------|-----------|-------------|
| GYRO DRDY | INT3 | BCM 20 | 423 | Pin 38 |
| ACC DRDY | INT1 | BCM 16 | 381 | Pin 36 |

> ⚠️ **实现前必须** 在板子上运行 `hb_gpioinfo` 确认 GPIO 423 仍对应 BCM 20。

## BMI088 寄存器配置（GYRO 侧）

在 `initialize()` 中写入：

| 寄存器 | 地址 | 写入值 | 含义 |
|--------|------|--------|------|
| GYRO_INT_CTRL | 0x15 | 0x80 | 使能 data ready 中断输出 |
| INT3_INT4_IO_CONF | 0x16 | 0x02 | INT3 推挽输出，上升沿有效 |
| INT3_INT4_IO_MAP | 0x18 | 0x01 | 将 data ready 映射到 INT3 引脚 |

## 实现步骤

### Step 1：抽象总线接口（IBusInterface）
- [ ] 新增 `include/imu_demo/bus_interface.hpp`，定义 `IBusInterface` 纯虚接口
- [ ] 接口方法：`write_reg()`、`read_reg()`、`read_burst()`
- [ ] 新增 `SpiBusInterface`，将现有 `Bmi088Driver` 中的 SPI 细节迁移进去
- [ ] `Bmi088Driver` 改为持有 `IBusInterface`，不再直接操作 SPI/GPIO

### Step 2：抽象中断接口（IDataReadyInterface）
- [ ] 新增 `include/imu_demo/drdy_interface.hpp`，定义 `IDataReadyInterface`
- [ ] 接口方法：`wait_drdy(int timeout_ms) -> bool`
- [ ] 实现 `SysfsGpioDrdy`：export GPIO → 配置 edge=rising → poll()

### Step 3：配置 BMI088 INT 寄存器
- [ ] 在 `Bmi088Driver::initialize()` 中写入上表三个寄存器
- [ ] `Bmi088Config` 增加 `gyro_int_gpio` 字段（默认 423）
- [ ] `config/bmi088.yaml` 增加对应配置项

### Step 4：替换 sampling_loop() 定时方式
- [ ] `ImuNode` 持有 `IDataReadyInterface`
- [ ] `sampling_loop()` 将 `sleep_until` 替换为 `drdy_->wait_drdy(10)`

### Step 5：验证
- [ ] 录制 30s 数据，跑 `analyze_static.py` 对比时序直方图
- [ ] 对比指标：std（目标 <20μs）、最大间隔（目标 <200μs）、潜在丢帧数

## 待确认问题

- [x] 在板子上运行 `hb_gpioinfo`，确认 GPIO 423 对应 BCM 20（GYRO INT3）
- [x] 确认 BMI088 INT3 引脚已通过硬件连接到 RDK X5 Physical Pin 38

> hb_gpioinfo：GPIO 423 = DSP_I2S1_DI（I2S 未使用，可用），GPIO 381 = LSIO_UART7_CTS（UART7 流控未使用，可用）

## 上下文 / 备注

- sysfs edge detection 在 RDK X5 已验证（Step 1 的 edge 文件写入无报错）
- 参考文档：https://developer.d-robotics.cc/rdk_doc/Advanced_development/linux_development/driver_development_x5/driver_gpio_dev
- 中断机制和总线类型正交，无论 SPI 还是 I2C，INT 引脚连接方式不变
- 时间戳误差分析：当前随机 0~6ms → 中断后固定 ~30μs（中断延迟 + SPI 传输），固定偏移可标定补偿

## 后续计划
P0: 增加上自检流程
P1: 硬件滤波流程