# 陀螺仪零偏校准下沉到驱动层

**状态：** 已完成
**创建日期：** 2026-03-24

## 目标

将陀螺仪静态零偏校准从 `ImuNode`（节点层）下沉到 `Bmi088Driver`（驱动层），使驱动层输出"开箱即用"的校准数据。

未校准的 ~0.002 rad/s 偏差，10 分钟积分漂移 ~70°，静态校准是初始化的必要操作。

## 设计决策

| 决策 | 原因 |
|------|------|
| 校准参数用 `calibration_samples`（样本数）而非时间 | 驱动层不感知发布频率，样本数是纯硬件语义；节点层从 `duration × rate` 计算后传入 |
| `read_imu_data()` 内部减零偏 | 调用方拿到的就是可用数据，不需要知道校准细节 |
| 中值滤波不下沉 | 600 样本取均值，少量 SPI 尖刺影响可忽略；中值滤波仍是节点层热路径处理 |
| `calibration_samples = 0` 跳过校准 | 与 `enable_self_test = false` 同理，提供跳过选项 |

## 涉及文件

| 文件 | 改动 |
|------|------|
| `bmi088_driver.hpp` | `Bmi088Config` 加 `calibration_samples`；`Bmi088Driver` 加 `calibrate_gyro()` + bias 成员 |
| `bmi088_driver.cpp` | 实现 `calibrate_gyro()`；`initialize()` 末尾调用；`read_imu_data()` 减零偏 |
| `imu_node.hpp` | 删除 `calibrate_gyro()`、`gyro_bias_x/y/z_` |
| `imu_node.cpp` | 删除校准相关代码；`timer_callback()` 去掉减零偏；`load_driver_config()` 计算并设置 `calibration_samples` |

## 实现步骤

- [x] Step 1：`Bmi088Config` 加 `calibration_samples` + `Bmi088Driver` 加 bias 成员和 `calibrate_gyro()` 声明
- [x] Step 2：实现 `calibrate_gyro()` + `initialize()` 末尾调用 + `read_imu_data()` 减零偏
- [x] Step 3：清理 `ImuNode`（删校准代码、删 bias 成员、简化 timer_callback）
- [x] Step 4：固定 500 样本，删除 `calibration_duration_sec`
- [x] 收尾：更新 CLAUDE.md 当前状态

## 验证

1. 编译通过
2. 启动节点，日志显示 `Gyro calibration done: bias=[...]`（来自驱动层 log_ 回调）
3. `ros2 topic echo /imu/data_calibrated` 确认陀螺仪静止时接近零
