#ifndef BMI088_DRIVER_HPP_
#define BMI088_DRIVER_HPP_

#include "imu_demo/bus_interface.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

class SpiDevice;  // 前向声明，完整定义在 spi_bus.hpp（仅 .cpp 引入）

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

    // 上电自检：默认开启，YAML 写 enable_self_test: false 可跳过
    bool enable_self_test = true;

    // 陀螺仪零偏校准样本数：0 = 跳过校准
    size_t calibration_samples = 500;
};

struct ImuRawData {
    double ax, ay, az;  // m/s²
    double gx, gy, gz;  // rad/s
};

class Bmi088Driver {
public:
    using LogCallback = std::function<void(const std::string&)>;

    explicit Bmi088Driver(
        const Bmi088Config& config, LogCallback log = [](const std::string&) {});
    ~Bmi088Driver();  // 定义在 .cpp，SpiDevice 完整类型在 .cpp 可见

    bool initialize();
    bool read_imu_data(ImuRawData& data);

private:
    void boot_sequence();
    void perform_acc_self_test();
    void perform_gyro_self_test();
    void soft_reset_sensors();
    void configure_sensors();
    void calibrate_gyro();

    // 读取 ACC 原始 ADC（自检用，不经 scale 转换）
    void read_acc_raw(int16_t& ax, int16_t& ay, int16_t& az);

    Bmi088Config config_;
    LogCallback log_;

    // spi_device_ 必须在 acc_bus_/gyro_bus_ 之前声明：
    // 成员析构逆序，确保两个总线实例先析构，再销毁 SpiDevice（关闭 fd）
    std::unique_ptr<SpiDevice> spi_device_;
    std::unique_ptr<IBusInterface> acc_bus_;
    std::unique_ptr<IBusInterface> gyro_bus_;

    // 构造时从量程配置计算，read_imu_data() 热路径使用
    double acc_scale_ = 0.0;
    double gyro_scale_ = 0.0;

    // 陀螺仪零偏（initialize 时校准，read_imu_data 内部自动减去）
    double gyro_bias_x_ = 0.0;
    double gyro_bias_y_ = 0.0;
    double gyro_bias_z_ = 0.0;
};

#endif  // BMI088_DRIVER_HPP_
