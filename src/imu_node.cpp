#include "imu_demo/imu_node.hpp"

using namespace std::chrono_literals;

ImuNode::ImuNode() : Node("bmi088_node")
{
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

    try {
        driver_ = std::make_unique<Bmi088Driver>("/dev/spidev1.0", 394, 396);
        if (!driver_->initialize()) {
            RCLCPP_ERROR(this->get_logger(), "BMI088 Chip ID mismatch! Check wiring.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "BMI088 initialized successfully.");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Driver init exception: %s", e.what());
        return;
    }

    // timer 仅在初始化成功后创建，故 timer_callback 中 driver_ 必然有效
    timer_ = this->create_wall_timer(
        5ms, std::bind(&ImuNode::timer_callback, this));
}

void ImuNode::timer_callback()
{
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp    = this->get_clock()->now();
    msg.header.frame_id = "imu_link";

    ImuRawData data;
    if (driver_->read_imu_data(data)) {
        msg.linear_acceleration.x = data.ax;
        msg.linear_acceleration.y = data.ay;
        msg.linear_acceleration.z = data.az;
        msg.angular_velocity.x    = data.gx;
        msg.angular_velocity.y    = data.gy;
        msg.angular_velocity.z    = data.gz;
        msg.orientation_covariance[0] = -1.0;  // -1 告知下游（EKF 等）本节点不提供姿态估计
        imu_pub_->publish(msg);
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuNode>());
    rclcpp::shutdown();
    return 0;
}
