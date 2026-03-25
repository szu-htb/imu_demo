#ifndef IIC_BUS_HPP_
#define IIC_BUS_HPP_

#include "imu_demo/bus_interface.hpp"

#include <cstdint>
#include <string>

// BMI088 I2c 总线实现
// ACC 和 GYRO 共用同一 I2c bus， 通过寄存器地址区分设备（ACC 0x19，GYRO 0x69），
// ACC 读寄存器时有 dummy byte（规格书 §6.1.2），GYRO 没有，
// 由 has_dummy_byte 参数在内部处理，上层不感知
class I2cBusInterface : public IBusInterface {
public:
    I2cBusInterface(const std::string& I2c_device, uint8_t addr, uint32_t speed_hz);
    ~I2cBusInterface() override;

    bool write_reg(uint8_t reg, uint8_t value) override;
    bool read_reg(uint8_t reg, uint8_t* out) override;
    bool read_burst(uint8_t start_reg, uint8_t* data, size_t len) override;

private:
    bool i2c_read(uint8_t* rx, size_t len);
    bool i2c_write(const uint8_t* tx, size_t len);
    void i2c_start();
    void i2c_stop();

    uint8_t dev_addr_;
    uint32_t speed_hz_;
    bool has_dummy_byte_;
};

#endif  // IIC_BUS_HPP_