#ifndef BMI088_NODE_HPP_
#define BMI088_NODE_HPP_

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

// ==========================================
// HAL 层：BMI088 硬件驱动封装
// ==========================================
class Bmi088Driver {
public:
    /**
     * @param spi_dev   SPI 设备路径，使用 SPI_NO_CS 模式，CS 由 sysfs GPIO 手动控制
     * @param acc_gpio  加速度计片选的 Linux GPIO 编号（xPi 列，Physical 24 = 394）
     * @param gyro_gpio 陀螺仪片选的 Linux GPIO 编号（xPi 列，Physical 26 = 396）
     */
    Bmi088Driver(const std::string& spi_dev, int acc_gpio, int gyro_gpio);
    ~Bmi088Driver();

    bool initialize();
    bool read_imu_data(double& ax, double& ay, double& az,
                       double& gx, double& gy, double& gz);

private:
    int spi_fd_;
    int acc_cs_fd_;   // /sys/class/gpio/gpioXXX/value 的 fd，保持常开
    int gyro_cs_fd_;

    void cs_low(int cs_fd);
    void cs_high(int cs_fd);

    void    write_register(uint8_t reg, uint8_t value, int cs_fd);
    uint8_t read_acc_register(uint8_t reg);
    uint8_t read_gyro_register(uint8_t reg);
    void    read_acc_burst(uint8_t start_reg, uint8_t* data);
    void    read_gyro_burst(uint8_t start_reg, uint8_t* data);
};

// ==========================================
// ROS 2 节点层
// ==========================================
class Bmi088Node : public rclcpp::Node {
public:
    Bmi088Node();

private:
    void timer_callback();

    std::unique_ptr<Bmi088Driver> driver_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // BMI088_NODE_HPP_
