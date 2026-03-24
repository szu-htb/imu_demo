#include "imu_demo/bmi088_driver.hpp"

#include "imu_demo/bmi088_registers.hpp"
#include "imu_demo/spi_bus.hpp"

#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>

namespace acc = bmi088_regs::acc;
namespace gyro = bmi088_regs::gyro;

static constexpr double GRAVITY_MPS2 = 9.80665;

// ---------------------------------------------------------
// 构造：根据 bus_type 创建对应的总线实现
// ---------------------------------------------------------
Bmi088Driver::Bmi088Driver(const Bmi088Config& config, LogCallback log) : config_(config), log_(std::move(log))
{
    if (config_.bus_type == "spi") {
        acc_bus_ = std::make_unique<SpiBusInterface>(
            config_.spi_device, config_.acc_cs_gpio, config_.spi_speed_hz, /*has_dummy_byte=*/true);
        gyro_bus_ = std::make_unique<SpiBusInterface>(
            config_.spi_device, config_.gyro_cs_gpio, config_.spi_speed_hz, /*has_dummy_byte=*/false);
    }
    else {
        throw std::invalid_argument("Unsupported bus_type: " + config_.bus_type);
        // 未来：} else if (config_.bus_type == "i2c") { ... }
    }

    switch (config_.acc_range_g) {
        case 3:
            acc_scale_ = (3.0 * GRAVITY_MPS2) / 32768.0;
            break;
        case 6:
            acc_scale_ = (6.0 * GRAVITY_MPS2) / 32768.0;
            break;
        case 12:
            acc_scale_ = (12.0 * GRAVITY_MPS2) / 32768.0;
            break;
        case 24:
            acc_scale_ = (24.0 * GRAVITY_MPS2) / 32768.0;
            break;
        default:
            throw std::invalid_argument("Unsupported acc_range_g: " + std::to_string(config_.acc_range_g));
    }

    switch (config_.gyro_range_dps) {
        case 125:
            gyro_scale_ = (125.0 * M_PI / 180.0) / 32768.0;
            break;
        case 250:
            gyro_scale_ = (250.0 * M_PI / 180.0) / 32768.0;
            break;
        case 500:
            gyro_scale_ = (500.0 * M_PI / 180.0) / 32768.0;
            break;
        case 1000:
            gyro_scale_ = (1000.0 * M_PI / 180.0) / 32768.0;
            break;
        case 2000:
            gyro_scale_ = (2000.0 * M_PI / 180.0) / 32768.0;
            break;
        default:
            throw std::invalid_argument("Unsupported gyro_range_dps: " + std::to_string(config_.gyro_range_dps));
    }
}

// ---------------------------------------------------------
// boot_sequence：唤醒两颗芯片并验证 Chip ID
// soft_reset 后 ACC 复回 I2C + Suspend，必须重新调用
// ---------------------------------------------------------
void Bmi088Driver::boot_sequence()
{
    // dummy read → CSB1 上升沿触发 ACC 从 I2C 切换到 SPI 模式（规格书§6.1.2）
    uint8_t dummy;
    acc_bus_->read_reg(acc::CHIP_ID, &dummy);
    usleep(1000);

    // ACC 唤醒：Suspend → Active → Enable（规格书§4.4）
    acc_bus_->write_reg(acc::PWR_CONF, 0x00);
    usleep(500);  // 规格书要求 ≥450μs
    acc_bus_->write_reg(acc::PWR_CTRL, 0x04);
    usleep(50000);  // 规格书要求 ≥1ms，留 50ms 余量确保稳定

    // GYRO 确认处于 Normal 模式
    gyro_bus_->write_reg(gyro::LPM1, 0x00);
    usleep(10000);

    // 验证 Chip ID
    uint8_t acc_id, gyro_id;
    acc_bus_->read_reg(acc::CHIP_ID, &acc_id);
    gyro_bus_->read_reg(gyro::CHIP_ID, &gyro_id);
    if (acc_id != acc::CHIP_ID_VALUE || gyro_id != gyro::CHIP_ID_VALUE) {
        char buf[64];
        snprintf(buf, sizeof(buf), "BMI088 Chip ID mismatch: ACC=0x%02X GYRO=0x%02X", acc_id, gyro_id);
        throw std::runtime_error(buf);
    }
    char buf[64];
    snprintf(buf, sizeof(buf), "Boot sequence done: ACC ID=0x%02X GYRO ID=0x%02X", acc_id, gyro_id);
    log_(buf);
}

// ---------------------------------------------------------
// read_acc_raw：读取 ACC 三轴原始 ADC（自检用）
// ---------------------------------------------------------
void Bmi088Driver::read_acc_raw(int16_t& ax, int16_t& ay, int16_t& az)
{
    uint8_t raw[6] = {};
    acc_bus_->read_burst(acc::DATA_START, raw, 6);
    ax = static_cast<int16_t>((raw[1] << 8) | raw[0]);
    ay = static_cast<int16_t>((raw[3] << 8) | raw[2]);
    az = static_cast<int16_t>((raw[5] << 8) | raw[4]);
}

// ---------------------------------------------------------
// perform_acc_self_test：加速度计自检（规格书§4.6）
// 临时切 24G/1600Hz，正负激励对比阈值，失败则 throw
// ---------------------------------------------------------
void Bmi088Driver::perform_acc_self_test()
{
    // 临时量程：24G range + 1600Hz normal mode
    acc_bus_->write_reg(acc::RANGE, 0x03);
    acc_bus_->write_reg(acc::CONF, 0xAC);
    usleep(3000);  // 规格书要求 >2ms

    // 正激励
    acc_bus_->write_reg(acc::SELF_TEST, 0x0D);
    usleep(55000);  // 规格书要求 >50ms
    int16_t pos_x, pos_y, pos_z;
    read_acc_raw(pos_x, pos_y, pos_z);

    // 负激励
    acc_bus_->write_reg(acc::SELF_TEST, 0x09);
    usleep(55000);
    int16_t neg_x, neg_y, neg_z;
    read_acc_raw(neg_x, neg_y, neg_z);

    // 关闭自检
    acc_bus_->write_reg(acc::SELF_TEST, 0x00);
    usleep(55000);  // 等传感器回稳

    // 阈值检查：|pos - neg| 的原始 ADC 值
    const int diff_x = std::abs(pos_x - neg_x);
    const int diff_y = std::abs(pos_y - neg_y);
    const int diff_z = std::abs(pos_z - neg_z);

    // X/Y ≥ 1365 LSB（1000mg @ 24G），Z ≥ 683 LSB（500mg @ 24G）
    constexpr int THRESHOLD_XY = 1365;
    constexpr int THRESHOLD_Z = 683;

    if (diff_x < THRESHOLD_XY) {
        throw std::runtime_error("ACC self-test FAIL: X diff=" + std::to_string(diff_x) +
                                 " (min=" + std::to_string(THRESHOLD_XY) + ")");
    }
    if (diff_y < THRESHOLD_XY) {
        throw std::runtime_error("ACC self-test FAIL: Y diff=" + std::to_string(diff_y) +
                                 " (min=" + std::to_string(THRESHOLD_XY) + ")");
    }
    if (diff_z < THRESHOLD_Z) {
        throw std::runtime_error("ACC self-test FAIL: Z diff=" + std::to_string(diff_z) +
                                 " (min=" + std::to_string(THRESHOLD_Z) + ")");
    }
    log_("ACC self-test passed: diff X=" + std::to_string(diff_x) + " Y=" + std::to_string(diff_y) +
         " Z=" + std::to_string(diff_z));
}

// ---------------------------------------------------------
// perform_gyro_self_test：陀螺仪内置自检（规格书§4.6）
// 写触发位 → 轮询 BIST_RDY → 检查 BIST_FAIL
// ---------------------------------------------------------
void Bmi088Driver::perform_gyro_self_test()
{
    // bit0 = bist_en，写 1 触发
    gyro_bus_->write_reg(gyro::SELF_TEST, 0x01);

    // 轮询 bit1(bist_rdy)，通常 <10ms
    uint8_t status = 0;
    for (int i = 0; i < 50; ++i) {
        usleep(1000);
        gyro_bus_->read_reg(gyro::SELF_TEST, &status);
        if (status & 0x02) {
            break;
        }
    }

    if (!(status & 0x02)) {
        throw std::runtime_error("GYRO self-test timeout: BIST_RDY never set");
    }

    // bit2 = bist_fail：0 = 通过，1 = 失败
    if (status & 0x04) {
        throw std::runtime_error("GYRO self-test FAIL: BIST_FAIL bit set");
    }
    log_("GYRO self-test passed");
}

// ---------------------------------------------------------
// soft_reset_sensors：软复位两颗芯片
// Reset 后 ACC 复回 I2C 模式 + Suspend，调用方必须重新 boot_sequence()
// ---------------------------------------------------------
void Bmi088Driver::soft_reset_sensors()
{
    acc_bus_->write_reg(acc::SOFT_RESET, bmi088_regs::SOFT_RESET_CMD);
    gyro_bus_->write_reg(gyro::SOFT_RESET, bmi088_regs::SOFT_RESET_CMD);
    usleep(50000);  // 等待两颗芯片复位完成
}

// ---------------------------------------------------------
// configure_sensors：按用户 config 写入量程和 ODR
// 寄存器值在此处以局部变量计算，不作为成员持有
// ---------------------------------------------------------
void Bmi088Driver::configure_sensors()
{
    // 将量程/ODR 映射到寄存器值（仅初始化时需要）
    uint8_t acc_range_reg;
    switch (config_.acc_range_g) {
        case 3:
            acc_range_reg = 0x00;
            break;
        case 6:
            acc_range_reg = 0x01;
            break;
        case 12:
            acc_range_reg = 0x02;
            break;
        default:
            acc_range_reg = 0x03;
            break;  // 24g
    }

    // ACC_CONF 使用固定的 normal mode(OSR=normal)，对外只暴露 ODR 语义
    uint8_t acc_conf_reg;
    switch (config_.acc_odr_hz) {
        case 100:
            acc_conf_reg = 0xA8;
            break;
        case 200:
            acc_conf_reg = 0xA9;
            break;
        case 400:
            acc_conf_reg = 0xAA;
            break;
        case 800:
            acc_conf_reg = 0xAB;
            break;
        case 1600:
            acc_conf_reg = 0xAC;
            break;
        default:
            throw std::invalid_argument("Unsupported acc_odr_hz: " + std::to_string(config_.acc_odr_hz));
    }

    uint8_t gyro_range_reg;
    switch (config_.gyro_range_dps) {
        case 2000:
            gyro_range_reg = 0x00;
            break;
        case 1000:
            gyro_range_reg = 0x01;
            break;
        case 500:
            gyro_range_reg = 0x02;
            break;
        case 250:
            gyro_range_reg = 0x03;
            break;
        default:
            gyro_range_reg = 0x04;
            break;  // 125 dps
    }

    // GYRO_BANDWIDTH 同时编码 ODR 和滤波带宽，为每个 ODR 选用一组固定的高响应 profile
    uint8_t gyro_bandwidth_reg;
    switch (config_.gyro_odr_hz) {
        case 100:
            gyro_bandwidth_reg = 0x07;
            break;
        case 200:
            gyro_bandwidth_reg = 0x06;
            break;
        case 400:
            gyro_bandwidth_reg = 0x03;
            break;
        case 1000:
            gyro_bandwidth_reg = 0x02;
            break;
        case 2000:
            gyro_bandwidth_reg = 0x00;
            break;
        default:
            throw std::invalid_argument("Unsupported gyro_odr_hz: " + std::to_string(config_.gyro_odr_hz));
    }

    acc_bus_->write_reg(acc::RANGE, acc_range_reg);
    acc_bus_->write_reg(acc::CONF, acc_conf_reg);
    usleep(1000);

    gyro_bus_->write_reg(gyro::RANGE, gyro_range_reg);
    gyro_bus_->write_reg(gyro::BANDWIDTH, gyro_bandwidth_reg);
    usleep(1000);
}

// ---------------------------------------------------------
// calibrate_gyro：阻塞式陀螺仪零偏校准
// 读取 N 个样本取均值，结果存入 gyro_bias_x/y/z_
// ---------------------------------------------------------
void Bmi088Driver::calibrate_gyro()
{
    if (config_.calibration_samples == 0) {
        log_("Gyro calibration skipped");
        return;
    }

    log_("Gyro calibration: collecting " + std::to_string(config_.calibration_samples) + " samples...");

    double sum_gx = 0.0, sum_gy = 0.0, sum_gz = 0.0;
    size_t count = 0;

    // 直接读 GYRO SPI，跳过 ACC 读取（校准不需要加速度计数据）
    while (count < config_.calibration_samples) {
        uint8_t raw[6] = {};
        if (!gyro_bus_->read_burst(gyro::DATA_START, raw, 6)) {
            usleep(1000);
            continue;
        }
        const int16_t rx = static_cast<int16_t>((raw[1] << 8) | raw[0]);
        const int16_t ry = static_cast<int16_t>((raw[3] << 8) | raw[2]);
        const int16_t rz = static_cast<int16_t>((raw[5] << 8) | raw[4]);
        sum_gx += rx * gyro_scale_;
        sum_gy += ry * gyro_scale_;
        sum_gz += rz * gyro_scale_;
        ++count;
    }

    gyro_bias_x_ = sum_gx / static_cast<double>(count);
    gyro_bias_y_ = sum_gy / static_cast<double>(count);
    gyro_bias_z_ = sum_gz / static_cast<double>(count);

    char buf[128];
    snprintf(buf,
             sizeof(buf),
             "Gyro calibration done: bias=[%.6f, %.6f, %.6f] rad/s",
             gyro_bias_x_,
             gyro_bias_y_,
             gyro_bias_z_);
    log_(buf);
}

// ---------------------------------------------------------
// initialize：四阶段初始化
//   阶段0：boot_sequence（唤醒 + Chip ID）
//   阶段1：自检（可选）→ soft reset → 重新 boot
//   阶段2：configure_sensors（用户量程/ODR）
//   阶段3：calibrate_gyro（零偏校准）
// ---------------------------------------------------------
bool Bmi088Driver::initialize()
{
    boot_sequence();

    if (config_.enable_self_test) {
        perform_acc_self_test();
        perform_gyro_self_test();
        soft_reset_sensors();
        boot_sequence();
    }

    configure_sensors();
    calibrate_gyro();
    return true;
}

// ---------------------------------------------------------
// 数据读取
// ---------------------------------------------------------
bool Bmi088Driver::read_imu_data(ImuRawData& data)
{
    uint8_t acc_raw[6] = {};
    uint8_t gyro_raw[6] = {};

    if (!acc_bus_->read_burst(acc::DATA_START, acc_raw, 6) || !gyro_bus_->read_burst(gyro::DATA_START, gyro_raw, 6)) {
        return false;
    }

    const int16_t raw_ax = static_cast<int16_t>((acc_raw[1] << 8) | acc_raw[0]);
    const int16_t raw_ay = static_cast<int16_t>((acc_raw[3] << 8) | acc_raw[2]);
    const int16_t raw_az = static_cast<int16_t>((acc_raw[5] << 8) | acc_raw[4]);
    const int16_t raw_gx = static_cast<int16_t>((gyro_raw[1] << 8) | gyro_raw[0]);
    const int16_t raw_gy = static_cast<int16_t>((gyro_raw[3] << 8) | gyro_raw[2]);
    const int16_t raw_gz = static_cast<int16_t>((gyro_raw[5] << 8) | gyro_raw[4]);

    data.ax = raw_ax * acc_scale_;
    data.ay = raw_ay * acc_scale_;
    data.az = raw_az * acc_scale_;
    data.gx = raw_gx * gyro_scale_ - gyro_bias_x_;
    data.gy = raw_gy * gyro_scale_ - gyro_bias_y_;
    data.gz = raw_gz * gyro_scale_ - gyro_bias_z_;

    return true;
}
