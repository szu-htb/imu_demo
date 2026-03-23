#ifndef BUS_INTERFACE_HPP_
#define BUS_INTERFACE_HPP_

#include <cstddef>
#include <cstdint>

// 总线抽象接口：屏蔽 SPI/I2C 的传输细节
// Bmi088Driver 只依赖此接口，不感知具体总线协议
class IBusInterface {
public:
    virtual ~IBusInterface() = default;

    virtual bool write_reg(uint8_t reg, uint8_t value) = 0;
    virtual bool read_reg(uint8_t reg, uint8_t* out) = 0;

    // 连续读取 len 字节到 data，起始寄存器为 start_reg
    virtual bool read_burst(uint8_t start_reg, uint8_t* data, size_t len) = 0;
};

#endif  // BUS_INTERFACE_HPP_
