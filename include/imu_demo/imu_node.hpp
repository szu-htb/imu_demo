#ifndef IMU_NODE_HPP_
#define IMU_NODE_HPP_

#include "imu_demo/bmi088_driver.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include <array>
#include <cstdint>
#include <memory>

struct ImuNodeConfig {
    int publish_rate_hz = 200;
    int fifo_poll_rate_hz = 200;
    std::string frame_id = "imu_link";
};

class ImuNode : public rclcpp::Node {
public:
    explicit ImuNode(const rclcpp::NodeOptions& options);

private:
    Bmi088Config load_driver_config();
    ImuNodeConfig load_node_config();
    void validate_configs(const Bmi088Config& driver_config, const ImuNodeConfig& node_config);

    void timer_callback();
    void publish_sample(const ImuRawData& raw, const rclcpp::Time& stamp);
    void publish_fifo_samples(const ImuRawData* raw_samples, size_t sample_count, const rclcpp::Time& newest_stamp);

    static double median3(double a, double b, double c);
    ImuRawData apply_median_filter(const ImuRawData& data);

    std::unique_ptr<Bmi088Driver> driver_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr poll_timer_;
    rclcpp::Clock::SharedPtr clock_;

    ImuNodeConfig node_config_;

    // 预分配消息，避免热路径每帧零初始化协方差数组
    sensor_msgs::msg::Imu msg_;
    std::array<ImuRawData, Bmi088Driver::kGyroFifoMaxFrames> fifo_buf_{};

    // 中值滤波：窗口 3 的环形 buffer
    static constexpr size_t kMedianWindow = 3;
    std::array<ImuRawData, kMedianWindow> median_buf_{};
    size_t median_count_ = 0;
    int64_t gyro_sample_period_ns_ = 0;
    bool use_gyro_fifo_ = false;
    bool fifo_overrun_warned_ = false;
};

#endif  // IMU_NODE_HPP_