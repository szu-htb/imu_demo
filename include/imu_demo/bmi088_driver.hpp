#ifndef BMI088_DRIVER_HPP_
#define BMI088_DRIVER_HPP_

#include "imu_demo/bus_interface.hpp"

#include <cstdint>
#include <memory>
#include <string>

struct Bmi088Config {
    // 通信层选择：通过 bus_type 在运行时切换，上层代码零改动
    std::string bus_type = "spi";  // "spi" | "i2c"

    // SPI 参数（bus_type: spi 时生效）
    std::string spi_device = "/dev/spidev1.0";
    int acc_cs_gpio = 394;
    int gyro_cs_gpio = 396;
    uint32_t spi_speed_hz = 5000000;

    // 传感器量程与 ODR
    int acc_range_g = 6;
    int gyro_range_dps = 500;
    int acc_odr_hz = 800;
    int gyro_odr_hz = 1000;
};

struct ImuRawData {
    double ax, ay, az;  // m/s²
    double gx, gy, gz;  // rad/s
};

class Bmi088Driver {
public:
    explicit Bmi088Driver(const Bmi088Config& config);

    bool initialize();
    bool read_imu_data(ImuRawData& data);

private:
    Bmi088Config config_;

    std::unique_ptr<IBusInterface> acc_bus_;
    std::unique_ptr<IBusInterface> gyro_bus_;

    // 构造时从量程配置计算，read_imu_data() 热路径使用
    double acc_scale_ = 0.0;
    double gyro_scale_ = 0.0;
};

#endif  // BMI088_DRIVER_HPP_
