#ifndef IMU_NODE_HPP_
#define IMU_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "imu_demo/bmi088_driver.hpp"

class ImuNode : public rclcpp::Node {
public:
    explicit ImuNode(const rclcpp::NodeOptions& options);

private:
    void timer_callback();

    std::unique_ptr<Bmi088Driver> driver_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // IMU_NODE_HPP_
