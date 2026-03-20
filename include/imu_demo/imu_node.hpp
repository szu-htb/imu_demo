#ifndef IMU_NODE_HPP_
#define IMU_NODE_HPP_

#include <array>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "imu_demo/bmi088_driver.hpp"

struct ImuNodeConfig {
    double publish_rate_hz = 200.0;
    std::string frame_id = "imu_link";
    double calibration_duration_sec = 3.0;
};


class ImuNode : public rclcpp::Node {
public:
    explicit ImuNode(const rclcpp::NodeOptions& options);

private:
    Bmi088Config load_driver_config();
    ImuNodeConfig load_node_config();
    void validate_configs(const Bmi088Config& driver_config, const ImuNodeConfig& node_config);

    void timer_callback();
    
    static double median3(double a, double b, double c);
    ImuRawData apply_median_filter(const ImuRawData& data);

    std::unique_ptr<Bmi088Driver> driver_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr cal_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    ImuNodeConfig node_config_;
    size_t calibration_target_samples_ = 1;

    // 中值滤波：窗口 3 的环形 buffer
    static constexpr size_t kMedianWindow = 3;
    std::array<ImuRawData, kMedianWindow> median_buf_{};
    size_t median_count_ = 0;

    // 陀螺仪零偏校准
    size_t cal_count_ = 0;
    bool calibrated_ = false;
    double gyro_bias_x_ = 0.0;
    double gyro_bias_y_ = 0.0;
    double gyro_bias_z_ = 0.0;
};



#endif  // IMU_NODE_HPP_
