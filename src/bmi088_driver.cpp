#include "imu_demo/bmi088_driver.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

// ---------------------------------------------------------
// 寄存器地址
// ---------------------------------------------------------
static constexpr uint8_t  ACC_CHIP_ID_REG  = 0x00;
static constexpr uint8_t  ACC_PWR_CTRL     = 0x7D;
static constexpr uint8_t  ACC_CONF         = 0x40;
static constexpr uint8_t  ACC_RANGE        = 0x41;

static constexpr uint8_t  GYRO_CHIP_ID_REG = 0x00;
static constexpr uint8_t  GYRO_LPM1        = 0x11;
static constexpr uint8_t  GYRO_RANGE       = 0x0F;
static constexpr uint8_t  GYRO_BANDWIDTH   = 0x10;

static constexpr uint8_t  ACC_DATA_START   = 0x12;
static constexpr uint8_t  GYRO_DATA_START  = 0x02;

static constexpr uint32_t SPI_SPEED_HZ     = 5000000;  // BMI088 最高安全频率（规格书§6.1）
static constexpr double   GRAVITY_MPS2     = 9.80665;

// ---------------------------------------------------------
// sysfs GPIO 工具函数
// ---------------------------------------------------------

static void gpio_export(int gpio_num)
{
    char dir_path[64];
    snprintf(dir_path, sizeof(dir_path), "/sys/class/gpio/gpio%d/direction", gpio_num);

    if (access(dir_path, F_OK) != 0) {
        int fd = open("/sys/class/gpio/export", O_WRONLY);
        if (fd < 0) throw std::runtime_error("Cannot open /sys/class/gpio/export");
        char buf[16];
        int len = snprintf(buf, sizeof(buf), "%d", gpio_num);
        write(fd, buf, len);
        close(fd);

        // 轮询等待 sysfs 完成目录创建，而非固定 sleep（最多等 100ms）
        for (int i = 0; i < 100; ++i) {
            if (access(dir_path, F_OK) == 0) return;
            usleep(1000);
        }
        throw std::runtime_error("GPIO export timeout for GPIO " + std::to_string(gpio_num));
    }
}

static void gpio_set_direction(int gpio_num, const char* dir)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio_num);
    int fd = open(path, O_WRONLY);
    if (fd < 0) throw std::runtime_error(std::string("Cannot open gpio direction: ") + path);
    write(fd, dir, strlen(dir));
    close(fd);
}

// value 文件保持常开，避免 200Hz 下反复 open/close 的系统调用开销
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

Bmi088Driver::Bmi088Driver(const Bmi088Config& config)
    : config_(config),
      spi_fd_(-1),
      acc_cs_fd_(-1),
      gyro_cs_fd_(-1),
      acc_range_reg_(0),
      acc_conf_reg_(0),
      gyro_range_reg_(0),
      gyro_bandwidth_reg_(0),
      acc_scale_(0.0),
      gyro_scale_(0.0)
{
    if (config_.spi_speed_hz == 0 || config_.spi_speed_hz > SPI_SPEED_HZ) {
        throw std::invalid_argument("spi_speed_hz must be in range 1.." + std::to_string(SPI_SPEED_HZ));
    }

    switch (config_.acc_range_g) {
    case 3:
        acc_range_reg_ = 0x00;
        acc_scale_ = (3.0 * GRAVITY_MPS2) / 32768.0;
        break;
    case 6:
        acc_range_reg_ = 0x01;
        acc_scale_ = (6.0 * GRAVITY_MPS2) / 32768.0;
        break;
    case 12:
        acc_range_reg_ = 0x02;
        acc_scale_ = (12.0 * GRAVITY_MPS2) / 32768.0;
        break;
    case 24:
        acc_range_reg_ = 0x03;
        acc_scale_ = (24.0 * GRAVITY_MPS2) / 32768.0;
        break;
    default:
        throw std::invalid_argument(
            "Unsupported BMI088 acc_range_g: " + std::to_string(config_.acc_range_g));
    }

    // ACC_CONF 使用固定的 normal mode(OSR=normal/avg4)，对外只暴露 ODR 语义。
    switch (config_.acc_odr_hz) {
    case 100:
        acc_conf_reg_ = 0xA8;
        break;
    case 200:
        acc_conf_reg_ = 0xA9;
        break;
    case 400:
        acc_conf_reg_ = 0xAA;
        break;
    case 800:
        acc_conf_reg_ = 0xAB;
        break;
    case 1600:
        acc_conf_reg_ = 0xAC;
        break;
    default:
        throw std::invalid_argument(
            "Unsupported BMI088 acc_odr_hz: " + std::to_string(config_.acc_odr_hz));
    }

    switch (config_.gyro_range_dps) {
    case 2000:
        gyro_range_reg_ = 0x00;
        gyro_scale_ = (2000.0 * M_PI / 180.0) / 32768.0;
        break;
    case 1000:
        gyro_range_reg_ = 0x01;
        gyro_scale_ = (1000.0 * M_PI / 180.0) / 32768.0;
        break;
    case 500:
        gyro_range_reg_ = 0x02;
        gyro_scale_ = (500.0 * M_PI / 180.0) / 32768.0;
        break;
    case 250:
        gyro_range_reg_ = 0x03;
        gyro_scale_ = (250.0 * M_PI / 180.0) / 32768.0;
        break;
    case 125:
        gyro_range_reg_ = 0x04;
        gyro_scale_ = (125.0 * M_PI / 180.0) / 32768.0;
        break;
    default:
        throw std::invalid_argument(
            "Unsupported BMI088 gyro_range_dps: " + std::to_string(config_.gyro_range_dps));
    }

    // GYRO_BANDWIDTH 寄存器同时编码 ODR 和滤波带宽。这里对外只暴露 ODR，
    // 并为每个 ODR 选用一组固定的高响应 profile。
    switch (config_.gyro_odr_hz) {
    case 100:
        gyro_bandwidth_reg_ = 0x07;
        break;
    case 200:
        gyro_bandwidth_reg_ = 0x06;
        break;
    case 400:
        gyro_bandwidth_reg_ = 0x03;
        break;
    case 1000:
        gyro_bandwidth_reg_ = 0x02;
        break;
    case 2000:
        gyro_bandwidth_reg_ = 0x00;
        break;
    default:
        throw std::invalid_argument(
            "Unsupported BMI088 gyro_odr_hz: " + std::to_string(config_.gyro_odr_hz));
    }

    int acc_gpio  = config_.acc_cs_gpio;
    int gyro_gpio = config_.gyro_cs_gpio;
    const std::string& spi_dev = config_.spi_device;

    gpio_export(acc_gpio);
    gpio_export(gyro_gpio);
    gpio_set_direction(acc_gpio,  "out");
    gpio_set_direction(gyro_gpio, "out");
    acc_cs_fd_  = gpio_open_value(acc_gpio);
    gyro_cs_fd_ = gpio_open_value(gyro_gpio);
    cs_high(acc_cs_fd_);
    cs_high(gyro_cs_fd_);

    spi_fd_ = open(spi_dev.c_str(), O_RDWR);
    if (spi_fd_ < 0) throw std::runtime_error("Failed to open SPI device: " + spi_dev);

    // SPI_NO_CS：禁用内核硬件 CS，改由 sysfs GPIO 手动控制
    // 原因：同一控制器同时持有多个 spidev fd 时，硬件 CS 会导致 ioctl 数据全零
    uint32_t mode  = SPI_MODE_0 | SPI_NO_CS;
    uint32_t speed = config_.spi_speed_hz;
    ioctl(spi_fd_, SPI_IOC_WR_MODE32,       &mode);
    ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
}

Bmi088Driver::~Bmi088Driver()
{
    if (acc_cs_fd_  >= 0) { cs_high(acc_cs_fd_);  close(acc_cs_fd_); }
    if (gyro_cs_fd_ >= 0) { cs_high(gyro_cs_fd_); close(gyro_cs_fd_); }
    if (spi_fd_     >= 0) close(spi_fd_);
}

// ---------------------------------------------------------
// CS / SPI 底层
// ---------------------------------------------------------

void Bmi088Driver::cs_low(int cs_fd)
{
    write(cs_fd, "0", 1);  // sysfs value 写入不依赖文件指针，lseek 不必要
}

void Bmi088Driver::cs_high(int cs_fd)
{
    write(cs_fd, "1", 1);
}

// 所有读写函数的公共 SPI 事务，消除重复的 spi_ioc_transfer 初始化
bool Bmi088Driver::spi_transfer(const uint8_t* tx, uint8_t* rx, size_t len, int cs_fd)
{
    struct spi_ioc_transfer tr = {};
    tr.tx_buf   = (unsigned long)tx;
    tr.rx_buf   = (unsigned long)rx;
    tr.len      = len;
    tr.speed_hz = config_.spi_speed_hz;
    cs_low(cs_fd);
    int ret = ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
    cs_high(cs_fd);
    return ret >= 0;
}

void Bmi088Driver::write_register(uint8_t reg, uint8_t value, int cs_fd)
{
    uint8_t tx[2] = { static_cast<uint8_t>(reg & 0x7F), value };
    spi_transfer(tx, nullptr, 2, cs_fd);
}

uint8_t Bmi088Driver::read_acc_register(uint8_t reg)
{
    uint8_t tx[3] = { static_cast<uint8_t>(reg | 0x80), 0x00, 0x00 };
    uint8_t rx[3] = {};
    spi_transfer(tx, rx, 3, acc_cs_fd_);
    return rx[2];  // rx[1] 是 ACC 的 Dummy Byte（规格书§6.1.2），真实数据在 rx[2]
}

uint8_t Bmi088Driver::read_gyro_register(uint8_t reg)
{
    uint8_t tx[2] = { static_cast<uint8_t>(reg | 0x80), 0x00 };
    uint8_t rx[2] = {};
    spi_transfer(tx, rx, 2, gyro_cs_fd_);
    return rx[1];
}

bool Bmi088Driver::read_acc_burst(uint8_t start_reg, uint8_t* data)
{
    uint8_t tx[8] = { static_cast<uint8_t>(start_reg | 0x80) };
    uint8_t rx[8] = {};
    if (!spi_transfer(tx, rx, 8, acc_cs_fd_)) return false;
    std::memcpy(data, rx + 2, 6);  // 跳过 rx[0]（无关）和 rx[1]（ACC Dummy Byte）
    return true;
}

bool Bmi088Driver::read_gyro_burst(uint8_t start_reg, uint8_t* data)
{
    uint8_t tx[7] = { static_cast<uint8_t>(start_reg | 0x80) };
    uint8_t rx[7] = {};
    if (!spi_transfer(tx, rx, 7, gyro_cs_fd_)) return false;
    std::memcpy(data, rx + 1, 6);  // 跳过 rx[0]（无关）
    return true;
}

// ---------------------------------------------------------
// 初始化：严格按规格书§3 状态机执行
// ---------------------------------------------------------
bool Bmi088Driver::initialize()
{
    // Step 1：dummy read -> CSB1 上升沿触发 ACC 从 I2C 模式切换到 SPI 模式（规格书§6.1.2）
    read_acc_register(ACC_CHIP_ID_REG);
    usleep(1000);

    // Step 2：唤醒 ACC（上电默认 Suspend 模式）
    write_register(ACC_PWR_CTRL, 0x04, acc_cs_fd_);
    usleep(50000);  // 规格书要求 ≥1ms，留 50ms 余量确保稳定

    // Step 3：确认 GYRO 处于 Normal 模式
    write_register(GYRO_LPM1, 0x00, gyro_cs_fd_);
    usleep(10000);

    // Step 4：验证 Chip ID
    uint8_t acc_id  = read_acc_register(ACC_CHIP_ID_REG);
    uint8_t gyro_id = read_gyro_register(GYRO_CHIP_ID_REG);
    if (acc_id != 0x1E || gyro_id != 0x0F) {
        return false;
    }

    // Step 5：配置 ACC
    write_register(ACC_RANGE, acc_range_reg_, acc_cs_fd_);
    write_register(ACC_CONF,  acc_conf_reg_,  acc_cs_fd_);
    usleep(1000);

    // Step 6：配置 GYRO
    write_register(GYRO_RANGE,     gyro_range_reg_,     gyro_cs_fd_);
    write_register(GYRO_BANDWIDTH, gyro_bandwidth_reg_, gyro_cs_fd_);
    usleep(1000);

    return true;
}

// ---------------------------------------------------------
// 数据读取
// ---------------------------------------------------------
bool Bmi088Driver::read_imu_data(ImuRawData& data)
{
    uint8_t acc_raw[6]  = {};
    uint8_t gyro_raw[6] = {};

    if (!read_acc_burst(ACC_DATA_START, acc_raw) ||
        !read_gyro_burst(GYRO_DATA_START, gyro_raw)) {
        return false;
    }

    int16_t raw_ax = static_cast<int16_t>((acc_raw[1]  << 8) | acc_raw[0]);
    int16_t raw_ay = static_cast<int16_t>((acc_raw[3]  << 8) | acc_raw[2]);
    int16_t raw_az = static_cast<int16_t>((acc_raw[5]  << 8) | acc_raw[4]);

    int16_t raw_gx = static_cast<int16_t>((gyro_raw[1] << 8) | gyro_raw[0]);
    int16_t raw_gy = static_cast<int16_t>((gyro_raw[3] << 8) | gyro_raw[2]);
    int16_t raw_gz = static_cast<int16_t>((gyro_raw[5] << 8) | gyro_raw[4]);

    data.ax = raw_ax * acc_scale_;
    data.ay = raw_ay * acc_scale_;
    data.az = raw_az * acc_scale_;
    data.gx = raw_gx * gyro_scale_;
    data.gy = raw_gy * gyro_scale_;
    data.gz = raw_gz * gyro_scale_;

    return true;
}
