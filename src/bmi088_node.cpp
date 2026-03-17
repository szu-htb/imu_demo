#include "imu_demo/bmi088_node.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <stdexcept>
#include <string>

using namespace std::chrono_literals;

// ---------------------------------------------------------
// 寄存器地址
// ---------------------------------------------------------
static constexpr uint8_t ACC_CHIP_ID_REG  = 0x00;
static constexpr uint8_t ACC_PWR_CTRL     = 0x7D;
static constexpr uint8_t ACC_CONF         = 0x40;
static constexpr uint8_t ACC_RANGE        = 0x41;

static constexpr uint8_t GYRO_CHIP_ID_REG = 0x00;
static constexpr uint8_t GYRO_LPM1        = 0x11;
static constexpr uint8_t GYRO_RANGE       = 0x0F;
static constexpr uint8_t GYRO_BANDWIDTH   = 0x10;

static constexpr uint8_t ACC_DATA_START   = 0x12;
static constexpr uint8_t GYRO_DATA_START  = 0x02;

// ---------------------------------------------------------
// sysfs GPIO 工具函数
// ---------------------------------------------------------

// 导出 GPIO（若已导出则跳过）
static void gpio_export(int gpio_num)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d", gpio_num);
    if (access(path, F_OK) == 0) return;  // 已导出，跳过

    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) throw std::runtime_error("Cannot open /sys/class/gpio/export");
    char buf[16];
    int len = snprintf(buf, sizeof(buf), "%d", gpio_num);
    write(fd, buf, len);
    close(fd);
    usleep(100000);  // 等待 sysfs 创建目录
}

// 设置 GPIO 方向
static void gpio_set_direction(int gpio_num, const char* dir)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio_num);
    int fd = open(path, O_WRONLY);
    if (fd < 0) throw std::runtime_error(std::string("Cannot open gpio direction: ") + path);
    write(fd, dir, strlen(dir));
    close(fd);
}

// 打开 value 文件并保持 fd 常开（避免每次 open/close 的开销）
static int gpio_open_value(int gpio_num)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio_num);
    int fd = open(path, O_RDWR);
    if (fd < 0) throw std::runtime_error(std::string("Cannot open gpio value: ") + path);
    return fd;
}

// ---------------------------------------------------------
// 构造 / 析构
// ---------------------------------------------------------

Bmi088Driver::Bmi088Driver(const std::string& spi_dev, int acc_gpio, int gyro_gpio)
    : spi_fd_(-1), acc_cs_fd_(-1), gyro_cs_fd_(-1)
{
    // 1. 初始化 sysfs GPIO（CS 引脚）
    gpio_export(acc_gpio);
    gpio_export(gyro_gpio);
    gpio_set_direction(acc_gpio,  "out");
    gpio_set_direction(gyro_gpio, "out");
    acc_cs_fd_  = gpio_open_value(acc_gpio);
    gyro_cs_fd_ = gpio_open_value(gyro_gpio);

    // 初始化时 CS 保持高电平（不选中）
    cs_high(acc_cs_fd_);
    cs_high(gyro_cs_fd_);

    // 2. 打开 SPI 设备，使用 SPI_NO_CS 模式（CS 由上层 GPIO 手动控制）
    spi_fd_ = open(spi_dev.c_str(), O_RDWR);
    if (spi_fd_ < 0) throw std::runtime_error("Failed to open SPI device: " + spi_dev);

    uint32_t mode  = SPI_MODE_0 | SPI_NO_CS;
    uint32_t speed = 5000000;
    ioctl(spi_fd_, SPI_IOC_WR_MODE32,      &mode);
    ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
}

Bmi088Driver::~Bmi088Driver()
{
    if (acc_cs_fd_  >= 0) { cs_high(acc_cs_fd_);  close(acc_cs_fd_); }
    if (gyro_cs_fd_ >= 0) { cs_high(gyro_cs_fd_); close(gyro_cs_fd_); }
    if (spi_fd_     >= 0) close(spi_fd_);
}

// ---------------------------------------------------------
// CS 控制（sysfs GPIO value 文件写入）
// ---------------------------------------------------------
void Bmi088Driver::cs_low(int cs_fd)
{
    lseek(cs_fd, 0, SEEK_SET);
    write(cs_fd, "0", 1);
}

void Bmi088Driver::cs_high(int cs_fd)
{
    lseek(cs_fd, 0, SEEK_SET);
    write(cs_fd, "1", 1);
}

// ---------------------------------------------------------
// SPI 底层读写（CS 由调用方或内部管理）
// ---------------------------------------------------------

// 写寄存器（ACC / GYRO 通用）
void Bmi088Driver::write_register(uint8_t reg, uint8_t value, int cs_fd)
{
    uint8_t tx[2] = { static_cast<uint8_t>(reg & 0x7F), value };
    struct spi_ioc_transfer tr = {};
    tr.tx_buf   = (unsigned long)tx;
    tr.len      = 2;
    tr.speed_hz = 5000000;

    cs_low(cs_fd);
    ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
    cs_high(cs_fd);
}

// 读 ACC 单字节：3 字节事务，rx[1]=Dummy，rx[2]=数据
uint8_t Bmi088Driver::read_acc_register(uint8_t reg)
{
    uint8_t tx[3] = { static_cast<uint8_t>(reg | 0x80), 0x00, 0x00 };
    uint8_t rx[3] = {};
    struct spi_ioc_transfer tr = {};
    tr.tx_buf   = (unsigned long)tx;
    tr.rx_buf   = (unsigned long)rx;
    tr.len      = 3;
    tr.speed_hz = 5000000;

    cs_low(acc_cs_fd_);
    ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
    cs_high(acc_cs_fd_);
    return rx[2];
}

// 读 GYRO 单字节：标准 2 字节事务
uint8_t Bmi088Driver::read_gyro_register(uint8_t reg)
{
    uint8_t tx[2] = { static_cast<uint8_t>(reg | 0x80), 0x00 };
    uint8_t rx[2] = {};
    struct spi_ioc_transfer tr = {};
    tr.tx_buf   = (unsigned long)tx;
    tr.rx_buf   = (unsigned long)rx;
    tr.len      = 2;
    tr.speed_hz = 5000000;

    cs_low(gyro_cs_fd_);
    ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
    cs_high(gyro_cs_fd_);
    return rx[1];
}

// ACC Burst Read：8 字节，跳过 rx[0]（无关）和 rx[1]（Dummy）
void Bmi088Driver::read_acc_burst(uint8_t start_reg, uint8_t* data)
{
    uint8_t tx[8] = { static_cast<uint8_t>(start_reg | 0x80) };
    uint8_t rx[8] = {};
    struct spi_ioc_transfer tr = {};
    tr.tx_buf   = (unsigned long)tx;
    tr.rx_buf   = (unsigned long)rx;
    tr.len      = 8;
    tr.speed_hz = 5000000;

    cs_low(acc_cs_fd_);
    ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
    cs_high(acc_cs_fd_);
    std::memcpy(data, rx + 2, 6);
}

// GYRO Burst Read：7 字节，跳过 rx[0]（无关）
void Bmi088Driver::read_gyro_burst(uint8_t start_reg, uint8_t* data)
{
    uint8_t tx[7] = { static_cast<uint8_t>(start_reg | 0x80) };
    uint8_t rx[7] = {};
    struct spi_ioc_transfer tr = {};
    tr.tx_buf   = (unsigned long)tx;
    tr.rx_buf   = (unsigned long)rx;
    tr.len      = 7;
    tr.speed_hz = 5000000;

    cs_low(gyro_cs_fd_);
    ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
    cs_high(gyro_cs_fd_);
    std::memcpy(data, rx + 1, 6);
}

// ---------------------------------------------------------
// 初始化：严格按规格书状态机
// ---------------------------------------------------------
bool Bmi088Driver::initialize()
{
    // Step 1：假读触发 ACC I2C → SPI 模式切换（CS 上升沿）
    read_acc_register(ACC_CHIP_ID_REG);
    usleep(1000);

    // Step 2：唤醒 ACC（上电默认 Suspend）
    write_register(ACC_PWR_CTRL, 0x04, acc_cs_fd_);
    usleep(50000);

    // Step 3：确认 GYRO 处于 Normal 模式
    write_register(GYRO_LPM1, 0x00, gyro_cs_fd_);
    usleep(10000);

    // Step 4：验证 Chip ID
    uint8_t acc_id  = read_acc_register(ACC_CHIP_ID_REG);
    uint8_t gyro_id = read_gyro_register(GYRO_CHIP_ID_REG);
    fprintf(stderr, "[BMI088] acc_id=0x%02X (expect 0x1E), gyro_id=0x%02X (expect 0x0F)\n",
            acc_id, gyro_id);
    if (acc_id != 0x1E || gyro_id != 0x0F) {
        return false;
    }

    // Step 5：配置 ACC 量程与输出速率
    // ACC_RANGE 0x01 = ±6g
    // ACC_CONF  0xA9 = 性能模式 | 正常过采样 | 200Hz ODR
    write_register(ACC_RANGE, 0x01, acc_cs_fd_);
    write_register(ACC_CONF,  0xA9, acc_cs_fd_);
    usleep(1000);

    // Step 6：配置 GYRO 量程与带宽
    // GYRO_RANGE     0x00 = ±2000°/s
    // GYRO_BANDWIDTH 0x02 = 1000Hz ODR，116Hz 滤波带宽
    write_register(GYRO_RANGE,     0x00, gyro_cs_fd_);
    write_register(GYRO_BANDWIDTH, 0x02, gyro_cs_fd_);
    usleep(1000);

    return true;
}

// ---------------------------------------------------------
// 数据读取
// ---------------------------------------------------------
bool Bmi088Driver::read_imu_data(double& ax, double& ay, double& az,
                                 double& gx, double& gy, double& gz)
{
    uint8_t acc_raw[6]  = {};
    uint8_t gyro_raw[6] = {};

    read_acc_burst(ACC_DATA_START,  acc_raw);
    read_gyro_burst(GYRO_DATA_START, gyro_raw);

    int16_t raw_ax = static_cast<int16_t>((acc_raw[1]  << 8) | acc_raw[0]);
    int16_t raw_ay = static_cast<int16_t>((acc_raw[3]  << 8) | acc_raw[2]);
    int16_t raw_az = static_cast<int16_t>((acc_raw[5]  << 8) | acc_raw[4]);

    int16_t raw_gx = static_cast<int16_t>((gyro_raw[1] << 8) | gyro_raw[0]);
    int16_t raw_gy = static_cast<int16_t>((gyro_raw[3] << 8) | gyro_raw[2]);
    int16_t raw_gz = static_cast<int16_t>((gyro_raw[5] << 8) | gyro_raw[4]);

    constexpr double ACC_SCALE  = (6.0 * 9.80665)         / 32768.0;
    constexpr double GYRO_SCALE = (2000.0 * M_PI / 180.0) / 32768.0;

    ax = raw_ax * ACC_SCALE;
    ay = raw_ay * ACC_SCALE;
    az = raw_az * ACC_SCALE;
    gx = raw_gx * GYRO_SCALE;
    gy = raw_gy * GYRO_SCALE;
    gz = raw_gz * GYRO_SCALE;

    return true;
}

// ---------------------------------------------------------
// ROS 2 节点
// ---------------------------------------------------------
Bmi088Node::Bmi088Node() : Node("bmi088_node")
{
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

    try {
        // spidev1.0（SPI_NO_CS 模式）
        // ACC CS → Linux GPIO 394（Physical 24，SPI1_CSN1）
        // GYRO CS → Linux GPIO 396（Physical 26，SPI1_CSN0）
        driver_ = std::make_unique<Bmi088Driver>("/dev/spidev1.0", 394, 396);
        if (!driver_->initialize()) {
            RCLCPP_ERROR(this->get_logger(), "BMI088 Chip ID mismatch! Check wiring.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "BMI088 initialized successfully.");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Driver init exception: %s", e.what());
        return;
    }

    timer_ = this->create_wall_timer(
        5ms, std::bind(&Bmi088Node::timer_callback, this));
}

void Bmi088Node::timer_callback()
{
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp    = this->get_clock()->now();
    msg.header.frame_id = "imu_link";

    double ax, ay, az, gx, gy, gz;
    if (driver_ && driver_->read_imu_data(ax, ay, az, gx, gy, gz)) {
        msg.linear_acceleration.x = ax;
        msg.linear_acceleration.y = ay;
        msg.linear_acceleration.z = az;
        msg.angular_velocity.x    = gx;
        msg.angular_velocity.y    = gy;
        msg.angular_velocity.z    = gz;
        msg.orientation_covariance[0] = -1.0;
        imu_pub_->publish(msg);
    }
}

// ---------------------------------------------------------
// main
// ---------------------------------------------------------
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Bmi088Node>());
    rclcpp::shutdown();
    return 0;
}
