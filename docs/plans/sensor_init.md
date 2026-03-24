# 传感器初始化优化 + 自检流程

**状态：** 已完成
**创建日期：** 2026-03-23

## 目标

在现有 `initialize()` 基础上增加两阶段结构：先自检验证硬件健康，再做正常配置。

## 新初始化流程

```txt
initialize()
  ├─ 阶段0：基础唤醒（一次）
  │    dummy read → 唤醒 ACC → GYRO Normal 模式 → 验证 Chip ID
  │
  ├─ 阶段1：自检（enable_self_test=true 时执行）
  │    ACC：临时改为 24G / 1600Hz → 正负激励各等 50ms → 读数对比阈值
  │    GYRO：写触发位 → 轮询 BIST_RDY → 检查 BIST_FAIL
  │    → Soft Reset 两颗芯片 → 等待复位稳定
  │    → 重新执行阶段0（Reset 后 ACC 回到 suspend + I2C 模式）
  │
  └─ 阶段2：正常配置（用户 config）
       ACC range/ODR → GYRO range/bandwidth → 完成
```

加速度计的self_test的标准流程:

1. set ±24g range by writing 0x03 to register ACC_RANGE(0x41)
2. set ODR = 1.6kHz, continous sampling mode, “normal mode”(norm_avg4) by writing 0xAC to register ACC_CONF(0x40)
    - Continuous filter function: set bit7 in ACC_CONF → 0x80
    - “normal avg4 mode”: ACC_CONF |= 0x02<<4 → 0x20
    - ODR=1.6kHz: ACC_CONF |= 0x0C
    - 合计: 0x80 | 0x20 | 0x0C = 0xAC
3. Wait for > 2 ms
4. Enable the positive self-test polarity (i.e. write 0x0D to register ACC_SELF_TEST (0x6D))
5. Wait for > 50ms
6. Read the accelerometer offset values for each axis (positive self-test response)
7. Enable the negative self-test polarity (i.e. write 0x09 to register ACC_SELF_TEST (0x6D))
8. Wait for > 50ms
9. Read the accelerometer offset values for each axis (negative self-test response)
10. Disable the self-test (i.e. write 0x00 to register ACC_SELF_TEST (0x6D))
11. Calculate difference of positive and negative self-test response and compare with the expected
values (see table below)
12. Wait for > 50ms to let the sensor settle to normal mode steady state operation
13. Reset

Table 9: Accelerometer self-test: resulting minimum difference signal between positive and negative
self-test signal

## 关键参数（来自原厂 API）

**ACC 自检寄存器：**

- `ACC_SELF_TEST` = 0x6D，正激励 0x0D，负激励 0x09，关闭 0x00
- 写激励后等待 **50ms**；发送激励前至少等待 **3ms**
- 自检期间临时量程：**24G / 1600Hz**（范围外数据会被滤波误判）
- 通过阈值（`|pos - neg|` 的原始 ADC 值）：
  - X / Y：≥ 1365 LSB（= 1000mg @ 24G range）
  - Z：≥ 683 LSB（= 500mg @ 24G range）

**GYRO 自检寄存器（0x3C）：**

- Bit0 `bist_en`：写 1 触发
- Bit1 `bist_rdy`：轮询直到为 1（通常 <10ms）
- Bit2 `bist_fail`：0 = 通过，1 = 失败

**Soft Reset：**

- ACC：`0x7E` 写 `0xB6`
- GYRO：`0x14` 写 `0xB6`
- Reset 后 ACC 复回 I2C 模式 + Suspend，必须重做 dummy read + 唤醒

## 设计决策

| 决策 | 内容 |
| --- | --- |
| 自检失败处理 | `throw std::runtime_error`，附带轴名称和实际/期望值，ImuNode 捕获后 RCLCPP_ERROR 打印并停止初始化 |
| 自检可跳过 | `Bmi088Config::enable_self_test`（默认 true），YAML 写 `enable_self_test: false` 可跳过 |
| 阈值单位 | 原始 ADC int16_t，不经 scale 转换，避免依赖临时量程 |
| 代码结构 | `initialize()` 拆成私有子方法：`boot_sequence()` / `perform_self_tests()` / `soft_reset_sensors()` / `configure_sensors()` |

## 启动时间影响

| 场景 | 额外耗时 |
| --- | --- |
| 自检开启 | +~200ms（两次 50ms 激励 + Reset 延迟） |
| 自检关闭 | 和现在一致 |

## 涉及文件

| 文件 | 改动 |
| --- | --- |
| `bmi088_driver.hpp` | 新增 `enable_self_test` 字段 + 软复位寄存器常量 + 4 个私有方法声明 |
| `bmi088_driver.cpp` | 重构 `initialize()`，新增 4 个私有方法 |
| `imu_node.cpp` | `load_driver_config()` 新增 `enable_self_test` 参数声明 |
| `config/bmi088.yaml` | 新增 `enable_self_test: true` |

## 实现步骤

- [x] Step 1：新增寄存器常量和 `Bmi088Config::enable_self_test`
- [x] Step 2：实现 `boot_sequence()`（从现有 initialize() 提取）
- [x] Step 3：实现 `perform_acc_self_test()`
- [x] Step 4：实现 `perform_gyro_self_test()`
- [x] Step 5：实现 `soft_reset_sensors()`
- [x] Step 6：实现 `configure_sensors()`（从现有 initialize() 提取）
- [x] Step 7：重组 `initialize()` 调用以上方法
- [x] Step 8：更新 `load_driver_config()` 和 YAML
- [x] Step 9：更新 CLAUDE.md 当前状态（移入已完成、更新进行中/待办）
