#ifndef BMI088_DRIVER_HPP_
#define BMI088_DRIVER_HPP_

#include <string>
#include <cstdint>

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
    Bmi088Driver(const std::string& spi_dev, int acc_gpio, int gyro_gpio);
    ~Bmi088Driver();

    bool initialize();
    bool read_imu_data(ImuRawData& data);

private:
    int spi_fd_;
    int acc_cs_fd_;
    int gyro_cs_fd_;

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