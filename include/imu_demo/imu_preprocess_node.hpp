#ifndef IMU_PREPROCESS_NODE_HPP_
#define IMU_PREPROCESS_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuPreprocessNode : public rclcpp::Node {
public:
    explicit ImuPreprocessNode(const rclcpp::NodeOptions& options);

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;

    static constexpr size_t kCalibrationSamples = 200;
    size_t sample_count_ = 0;
    bool calibrated_ = false;
    double gyro_bias_x_ = 0.0;
    double gyro_bias_y_ = 0.0;
    double gyro_bias_z_ = 0.0;
};

#endif  // IMU_PREPROCESS_NODE_HPP_
