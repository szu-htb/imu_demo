#ifndef SPI_BUS_HPP_
#define SPI_BUS_HPP_

#include "imu_demo/bus_interface.hpp"

#include <cstdint>
#include <string>

// SPI 设备 RAII 封装：管理 /dev/spidevX.X 的 open/配置/close
// 可被多个 SpiBusInterface 共享引用，fd 生命周期由此对象控制
class SpiDevice {
public:
    SpiDevice(const std::string& device, uint32_t speed_hz);
    ~SpiDevice();
    int fd() const { return fd_; }

private:
    int fd_ = -1;
};

// BMI088 SPI 总线实现
// ACC 和 GYRO 共用同一 SpiDevice，通过各自的 sysfs GPIO 做片选
// ACC 读寄存器时有 dummy byte（规格书 §6.1.2），GYRO 没有，
// 由 has_dummy_byte 参数在内部处理，上层不感知
class SpiBusInterface : public IBusInterface {
public:
    SpiBusInterface(const SpiDevice& spi, int cs_gpio, uint32_t speed_hz, bool has_dummy_byte);
    ~SpiBusInterface() override;

    bool write_reg(uint8_t reg, uint8_t value) override;
    bool read_reg(uint8_t reg, uint8_t* out) override;
    bool read_burst(uint8_t start_reg, uint8_t* data, size_t len) override;

private:
    bool spi_transfer(const uint8_t* tx, uint8_t* rx, size_t len);
    void cs_low();
    void cs_high();

    int spi_fd_;  // 构造时从 SpiDevice 取出，SpiDevice 负责关闭
    int cs_fd_;
    uint32_t speed_hz_;
    bool has_dummy_byte_;
};

#endif  // SPI_BUS_HPP_
