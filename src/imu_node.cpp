#include "imu_demo/imu_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>

using namespace std::chrono_literals;

ImuNode::ImuNode(const rclcpp::NodeOptions& options)
    : Node("bmi088_node", options)
{
    raw_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "imu/data_raw", rclcpp::SensorDataQoS());
    cal_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "imu/data_calibrated", rclcpp::SensorDataQoS().reliable());

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

    RCLCPP_INFO(this->get_logger(),
        "Collecting %zu samples for gyro bias calibration...", kCalibrationSamples);

    // timer 仅在初始化成功后创建，故 timer_callback 中 driver_ 必然有效
    timer_ = this->create_wall_timer(
        5ms, std::bind(&ImuNode::timer_callback, this));
}

double ImuNode::median3(double a, double b, double c)
{
    if (a > b) std::swap(a, b);
    if (b > c) std::swap(b, c);
    if (a > b) std::swap(a, b);
    return b;
}

ImuRawData ImuNode::apply_median_filter(const ImuRawData& data)
{
    median_buf_[median_count_ % kMedianWindow] = data;
    ++median_count_;

    if (median_count_ < kMedianWindow) {
        return data;
    }

    const auto& b = median_buf_;
    ImuRawData out;
    out.ax = median3(b[0].ax, b[1].ax, b[2].ax);
    out.ay = median3(b[0].ay, b[1].ay, b[2].ay);
    out.az = median3(b[0].az, b[1].az, b[2].az);
    out.gx = median3(b[0].gx, b[1].gx, b[2].gx);
    out.gy = median3(b[0].gy, b[1].gy, b[2].gy);
    out.gz = median3(b[0].gz, b[1].gz, b[2].gz);
    return out;
}

void ImuNode::timer_callback()
{
    ImuRawData raw;
    if (!driver_->read_imu_data(raw)) {
        return;
    }

    // 1. 发布原始数据（未经任何处理）
    auto raw_msg = sensor_msgs::msg::Imu();
    raw_msg.header.stamp    = this->get_clock()->now();
    raw_msg.header.frame_id = "imu_link";
    raw_msg.linear_acceleration.x = raw.ax;
    raw_msg.linear_acceleration.y = raw.ay;
    raw_msg.linear_acceleration.z = raw.az;
    raw_msg.angular_velocity.x    = raw.gx;
    raw_msg.angular_velocity.y    = raw.gy;
    raw_msg.angular_velocity.z    = raw.gz;
    raw_msg.orientation_covariance[0] = -1.0;
    raw_pub_->publish(raw_msg);

    // 2. 中值滤波去尖刺
    ImuRawData filtered = apply_median_filter(raw);

    // 3. 零偏校准
    if (!calibrated_) {
        gyro_bias_x_ += filtered.gx;
        gyro_bias_y_ += filtered.gy;
        gyro_bias_z_ += filtered.gz;
        ++cal_count_;

        if (cal_count_ >= kCalibrationSamples) {
            gyro_bias_x_ /= static_cast<double>(kCalibrationSamples);
            gyro_bias_y_ /= static_cast<double>(kCalibrationSamples);
            gyro_bias_z_ /= static_cast<double>(kCalibrationSamples);
            calibrated_ = true;
            RCLCPP_INFO(this->get_logger(),
                "Gyro calibration done. Bias: [%.6f, %.6f, %.6f] rad/s",
                gyro_bias_x_, gyro_bias_y_, gyro_bias_z_);
        }
        return;
    }

    // 4. 减去零偏，发布校准数据
    auto cal_msg = raw_msg;
    cal_msg.linear_acceleration.x = filtered.ax;
    cal_msg.linear_acceleration.y = filtered.ay;
    cal_msg.linear_acceleration.z = filtered.az;
    cal_msg.angular_velocity.x = filtered.gx - gyro_bias_x_;
    cal_msg.angular_velocity.y = filtered.gy - gyro_bias_y_;
    cal_msg.angular_velocity.z = filtered.gz - gyro_bias_z_;
    cal_pub_->publish(cal_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(ImuNode)
