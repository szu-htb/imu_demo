#include "imu_demo/bmi088_driver.hpp"

#include "imu_demo/spi_bus.hpp"

#include <unistd.h>

#include <cmath>
#include <stdexcept>
#include <string>

// ---------------------------------------------------------
// 寄存器地址
// ---------------------------------------------------------
static constexpr uint8_t ACC_CHIP_ID_REG = 0x00;
static constexpr uint8_t ACC_PWR_CTRL = 0x7D;
static constexpr uint8_t ACC_CONF = 0x40;
static constexpr uint8_t ACC_RANGE = 0x41;

static constexpr uint8_t GYRO_CHIP_ID_REG = 0x00;
static constexpr uint8_t GYRO_LPM1 = 0x11;
static constexpr uint8_t GYRO_RANGE = 0x0F;
static constexpr uint8_t GYRO_BANDWIDTH = 0x10;

static constexpr uint8_t ACC_DATA_START = 0x12;
static constexpr uint8_t GYRO_DATA_START = 0x02;

static constexpr double GRAVITY_MPS2 = 9.80665;

// ---------------------------------------------------------
// 构造：根据 bus_type 创建对应的总线实现
// ---------------------------------------------------------
Bmi088Driver::Bmi088Driver(const Bmi088Config& config) : config_(config)
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
// 初始化：严格按规格书§3 状态机执行
// 寄存器值在此处以局部变量计算，不作为成员持有
// ---------------------------------------------------------
bool Bmi088Driver::initialize()
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

    // Step 1：dummy read -> CSB1 上升沿触发 ACC 从 I2C 模式切换到 SPI 模式（规格书§6.1.2）
    uint8_t dummy;
    acc_bus_->read_reg(ACC_CHIP_ID_REG, &dummy);
    usleep(1000);

    // Step 2：唤醒 ACC（上电默认 Suspend 模式）
    acc_bus_->write_reg(ACC_PWR_CTRL, 0x04);
    usleep(50000);  // 规格书要求 ≥1ms，留 50ms 余量确保稳定

    // Step 3：确认 GYRO 处于 Normal 模式
    gyro_bus_->write_reg(GYRO_LPM1, 0x00);
    usleep(10000);

    // Step 4：验证 Chip ID
    uint8_t acc_id, gyro_id;
    acc_bus_->read_reg(ACC_CHIP_ID_REG, &acc_id);
    gyro_bus_->read_reg(GYRO_CHIP_ID_REG, &gyro_id);
    if (acc_id != 0x1E || gyro_id != 0x0F) {
        return false;
    }

    // Step 5：配置 ACC
    acc_bus_->write_reg(ACC_RANGE, acc_range_reg);
    acc_bus_->write_reg(ACC_CONF, acc_conf_reg);
    usleep(1000);

    // Step 6：配置 GYRO
    gyro_bus_->write_reg(GYRO_RANGE, gyro_range_reg);
    gyro_bus_->write_reg(GYRO_BANDWIDTH, gyro_bandwidth_reg);
    usleep(1000);

    return true;
}

// ---------------------------------------------------------
// 数据读取
// ---------------------------------------------------------
bool Bmi088Driver::read_imu_data(ImuRawData& data)
{
    uint8_t acc_raw[6] = {};
    uint8_t gyro_raw[6] = {};

    if (!acc_bus_->read_burst(ACC_DATA_START, acc_raw, 6) || !gyro_bus_->read_burst(GYRO_DATA_START, gyro_raw, 6)) {
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
    data.gx = raw_gx * gyro_scale_;
    data.gy = raw_gy * gyro_scale_;
    data.gz = raw_gz * gyro_scale_;

    return true;
}
