#ifndef IMU_NODE_HPP_
#define IMU_NODE_HPP_

#include "imu_demo/bmi088_driver.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include <array>
#include <memory>

struct ImuNodeConfig {
    int publish_rate_hz = 200;
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
    void publish_sample(const ImuRawData& raw);

    static double median3(double a, double b, double c);
    ImuRawData apply_median_filter(const ImuRawData& data);

    std::unique_ptr<Bmi088Driver> driver_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr poll_timer_;
    rclcpp::Clock::SharedPtr clock_;

    ImuNodeConfig node_config_;

    // 预分配消息，避免热路径每帧零初始化协方差数组
    sensor_msgs::msg::Imu msg_;

    // 中值滤波：窗口 3 的环形 buffer
    static constexpr size_t kMedianWindow = 3;
    std::array<ImuRawData, kMedianWindow> median_buf_{};
    size_t median_count_ = 0;
};

#endif  // IMU_NODE_HPP_
