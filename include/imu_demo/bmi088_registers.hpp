#ifndef BMI088_REGISTERS_HPP_
#define BMI088_REGISTERS_HPP_

#include <cstdint>

// BMI088 寄存器地址定义（规格书 BST-BMI088-DS001）
// 用 namespace 隔离 ACC / GYRO 同名寄存器（如 CHIP_ID、SOFT_RESET）
namespace bmi088_regs {

namespace acc {
constexpr uint8_t CHIP_ID = 0x00;
constexpr uint8_t DATA_START = 0x12;
constexpr uint8_t CONF = 0x40;
constexpr uint8_t RANGE = 0x41;
constexpr uint8_t SELF_TEST = 0x6D;
constexpr uint8_t PWR_CONF = 0x7C;
constexpr uint8_t PWR_CTRL = 0x7D;
constexpr uint8_t SOFT_RESET = 0x7E;

constexpr uint8_t CHIP_ID_VALUE = 0x1E;
}  // namespace acc

namespace gyro {
constexpr uint8_t CHIP_ID = 0x00;
constexpr uint8_t DATA_START = 0x02;
constexpr uint8_t RANGE = 0x0F;
constexpr uint8_t BANDWIDTH = 0x10;
constexpr uint8_t LPM1 = 0x11;
constexpr uint8_t SELF_TEST = 0x3C;
constexpr uint8_t SOFT_RESET = 0x14;

constexpr uint8_t CHIP_ID_VALUE = 0x0F;
}  // namespace gyro

constexpr uint8_t SOFT_RESET_CMD = 0xB6;

}  // namespace bmi088_regs

#endif  // BMI088_REGISTERS_HPP_
