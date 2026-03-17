#include "imu_demo/imu_preprocess_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

ImuPreprocessNode::ImuPreprocessNode(const rclcpp::NodeOptions& options)
    : Node("imu_preprocess_node", options)
{
    pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "imu/data_calibrated", rclcpp::SensorDataQoS().reliable());

    sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data_raw", rclcpp::SensorDataQoS(),
        std::bind(&ImuPreprocessNode::imu_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
        "Collecting %zu samples for gyro bias calibration...", kCalibrationSamples);
}

void ImuPreprocessNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    if (!calibrated_) {
        // 累加陀螺仪读数用于求均值
        gyro_bias_x_ += msg->angular_velocity.x;
        gyro_bias_y_ += msg->angular_velocity.y;
        gyro_bias_z_ += msg->angular_velocity.z;
        ++sample_count_;

        if (sample_count_ >= kCalibrationSamples) {
            gyro_bias_x_ /= static_cast<double>(kCalibrationSamples);
            gyro_bias_y_ /= static_cast<double>(kCalibrationSamples);
            gyro_bias_z_ /= static_cast<double>(kCalibrationSamples);
            calibrated_ = true;
            RCLCPP_INFO(this->get_logger(),
                "Gyro calibration done. Bias: [%.6f, %.6f, %.6f] rad/s",
                gyro_bias_x_, gyro_bias_y_, gyro_bias_z_);
        }
        // 校准期间不发布，避免下游融合未校准数据
        return;
    }

    auto out = *msg;
    out.angular_velocity.x -= gyro_bias_x_;
    out.angular_velocity.y -= gyro_bias_y_;
    out.angular_velocity.z -= gyro_bias_z_;
    pub_->publish(out);
}

RCLCPP_COMPONENTS_REGISTER_NODE(ImuPreprocessNode)
