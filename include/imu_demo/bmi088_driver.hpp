#ifndef BMI088_DRIVER_HPP_
#define BMI088_DRIVER_HPP_

#include <string>
#include <cstdint>

struct Bmi088Config {
    // Hardware
    std::string spi_device = "/dev/spidev1.0";
    int acc_cs_gpio = 394;
    int gyro_cs_gpio = 396;

    // Sensor settings
    int acc_range_g = 6;
    int gyro_range_dps = 500;
    int acc_odr_hz = 200;
    int gyro_odr_hz = 200;

    uint32_t spi_speed_hz = 5000000;
};

struct ImuRawData {
    double ax, ay, az;  // m/s²
    double gx, gy, gz;  // rad/s
};

class Bmi088Driver {
public:
    /**
     * @param spi_dev   SPI 设备路径（SPI_NO_CS 模式，避免同一控制器多 fd 冲突）
     * @param acc_gpio  ACC 片选的 Linux GPIO 编号（Physical 24 = 394）
     * @param gyro_gpio GYRO 片选的 Linux GPIO 编号（Physical 26 = 396）
     */
    explicit Bmi088Driver(const Bmi088Config& config);
    ~Bmi088Driver();

    bool initialize();
    bool read_imu_data(ImuRawData& data);

private:
    Bmi088Config config_;
    int spi_fd_;
    int acc_cs_fd_;
    int gyro_cs_fd_;
    uint8_t acc_range_reg_;
    uint8_t acc_conf_reg_;
    uint8_t gyro_range_reg_;
    uint8_t gyro_bandwidth_reg_;
    double acc_scale_;
    double gyro_scale_;

    void cs_low(int cs_fd);
    void cs_high(int cs_fd);
    bool spi_transfer(const uint8_t* tx, uint8_t* rx, size_t len, int cs_fd);

    void    write_register(uint8_t reg, uint8_t value, int cs_fd);
    uint8_t read_acc_register(uint8_t reg);
    uint8_t read_gyro_register(uint8_t reg);
    bool    read_acc_burst(uint8_t start_reg, uint8_t* data);
    bool    read_gyro_burst(uint8_t start_reg, uint8_t* data);
};

#endif  // BMI088_DRIVER_HPP_
