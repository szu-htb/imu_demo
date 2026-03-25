#include "imu_demo/spi_bus.hpp"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstring>
#include <stdexcept>
#include <string>

// ---------------------------------------------------------
// sysfs GPIO 工具函数（仅 spi_bus.cpp 内部使用）
// ---------------------------------------------------------

static void gpio_export(int gpio_num)
{
    char dir_path[64];
    snprintf(dir_path, sizeof(dir_path), "/sys/class/gpio/gpio%d/direction", gpio_num);

    if (access(dir_path, F_OK) != 0) {
        int fd = open("/sys/class/gpio/export", O_WRONLY);
        if (fd < 0)
            throw std::runtime_error("Cannot open /sys/class/gpio/export");
        char buf[16];
        int len = snprintf(buf, sizeof(buf), "%d", gpio_num);
        write(fd, buf, len);
        close(fd);

        // 轮询等待 sysfs 完成目录创建，而非固定 sleep（最多等 100ms）
        for (int i = 0; i < 100; ++i) {
            if (access(dir_path, F_OK) == 0)
                return;
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
    if (fd < 0)
        throw std::runtime_error(std::string("Cannot open gpio direction: ") + path);
    write(fd, dir, strlen(dir));
    close(fd);
}

// value 文件保持常开，避免 200Hz+ 下反复 open/close 的系统调用开销
static int gpio_open_value(int gpio_num)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio_num);
    int fd = open(path, O_RDWR);
    if (fd < 0)
        throw std::runtime_error(std::string("Cannot open gpio value: ") + path);
    return fd;
}

// ---------------------------------------------------------
// SpiDevice
// ---------------------------------------------------------

SpiDevice::SpiDevice(const std::string& device, uint32_t speed_hz)
{
    fd_ = open(device.c_str(), O_RDWR);
    if (fd_ < 0)
        throw std::runtime_error("Failed to open SPI device: " + device);

    // SPI_NO_CS：禁用内核硬件 CS，改由 sysfs GPIO 手动控制
    // 原因：同一控制器同时持有多个 spidev fd 时，硬件 CS 会导致 ioctl 数据全零
    uint32_t mode = SPI_MODE_0 | SPI_NO_CS;
    ioctl(fd_, SPI_IOC_WR_MODE32, &mode);
    ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz);
}

SpiDevice::~SpiDevice()
{
    if (fd_ >= 0)
        close(fd_);
}

// ---------------------------------------------------------
// SpiBusInterface
// ---------------------------------------------------------

SpiBusInterface::SpiBusInterface(const SpiDevice& spi, int cs_gpio, uint32_t speed_hz, bool has_dummy_byte)
    : spi_fd_(spi.fd()), cs_fd_(-1), speed_hz_(speed_hz), has_dummy_byte_(has_dummy_byte)
{
    gpio_export(cs_gpio);
    gpio_set_direction(cs_gpio, "out");
    cs_fd_ = gpio_open_value(cs_gpio);
    cs_high();
}

SpiBusInterface::~SpiBusInterface()
{
    if (cs_fd_ >= 0) {
        cs_high();
        close(cs_fd_);
    }
}

void SpiBusInterface::cs_low()
{
    write(cs_fd_, "0", 1);
}

void SpiBusInterface::cs_high()
{
    write(cs_fd_, "1", 1);
}

bool SpiBusInterface::spi_transfer(const uint8_t* tx, uint8_t* rx, size_t len)
{
    struct spi_ioc_transfer tr = {};
    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = len;
    tr.speed_hz = speed_hz_;
    cs_low();
    int ret = ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
    cs_high();
    return ret >= 0;
}

bool SpiBusInterface::write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {static_cast<uint8_t>(reg & 0x7F), value};
    return spi_transfer(tx, nullptr, 2);
}

bool SpiBusInterface::read_reg(uint8_t reg, uint8_t* out)
{
    if (has_dummy_byte_) {
        // ACC：tx[0]=地址, tx[1]=dummy, tx[2]=数据（规格书 §6.1.2）
        uint8_t tx[3] = {static_cast<uint8_t>(reg | 0x80), 0x00, 0x00};
        uint8_t rx[3] = {};
        if (!spi_transfer(tx, rx, 3))
            return false;
        *out = rx[2];
    }
    else {
        // GYRO：tx[0]=地址, tx[1]=数据
        uint8_t tx[2] = {static_cast<uint8_t>(reg | 0x80), 0x00};
        uint8_t rx[2] = {};
        if (!spi_transfer(tx, rx, 2))
            return false;
        *out = rx[1];
    }
    return true;
}

bool SpiBusInterface::read_burst(uint8_t start_reg, uint8_t* data, size_t len)
{
    // BMI088 burst 最多读 6 字节数据，加头最多 8 字节，用固定大小 buffer
    uint8_t tx[32] = {};
    uint8_t rx[32] = {};

    if (has_dummy_byte_) {
        // ACC：1（地址）+ 1（dummy）+ len（数据）
        tx[0] = start_reg | 0x80;
        if (!spi_transfer(tx, rx, len + 2))
            return false;
        memcpy(data, rx + 2, len);
    }
    else {
        // GYRO：1（地址）+ len（数据）
        tx[0] = start_reg | 0x80;
        if (!spi_transfer(tx, rx, len + 1))
            return false;
        memcpy(data, rx + 1, len);
    }
    return true;
}
