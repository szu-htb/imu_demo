#include "imu_demo/imu_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>

ImuNode::ImuNode(const rclcpp::NodeOptions& options)
    : Node("bmi088_node", options)
{
    /* 创建发布/订阅者 */
    raw_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "imu/data_raw", rclcpp::SensorDataQoS());
    cal_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "imu/data_calibrated", rclcpp::SensorDataQoS().reliable());

    /* 加载参数 */
    const Bmi088Config driver_config = load_driver_config();
    node_config_ = load_node_config();
    validate_configs(driver_config, node_config_);

    calibration_target_samples_ = std::max<size_t>(
        1, static_cast<size_t>(node_config_.calibration_duration_sec * node_config_.publish_rate_hz));

    const auto period_ns = std::chrono::nanoseconds(
        static_cast<int64_t>(1e9 / node_config_.publish_rate_hz));

    /* 打印参数 */
    RCLCPP_INFO(
        this->get_logger(),
        "BMI088 config: spi=%s, acc_cs=%d, gyro_cs=%d, spi_speed=%u Hz",
        driver_config.spi_device.c_str(),
        driver_config.acc_cs_gpio,
        driver_config.gyro_cs_gpio,
        driver_config.spi_speed_hz);

    RCLCPP_INFO(
        this->get_logger(),
        "Sensor config: acc_range=%d g, gyro_range=%d dps, acc_odr=%d Hz, gyro_odr=%d Hz",
        driver_config.acc_range_g,
        driver_config.gyro_range_dps,
        driver_config.acc_odr_hz,
        driver_config.gyro_odr_hz);

    RCLCPP_INFO(
        this->get_logger(),
        "Node config: frame_id=%s, publish_rate=%.2f Hz, period=%.3f ms, calibration=%.2f s (%zu samples)",
        node_config_.frame_id.c_str(),
        node_config_.publish_rate_hz,
        static_cast<double>(period_ns.count()) / 1e6,
        node_config_.calibration_duration_sec,
        calibration_target_samples_);


    /* 创建BmiDriver对象 */
    try {
        driver_ = std::make_unique<Bmi088Driver>(driver_config);
        if (!driver_->initialize()) {
            RCLCPP_ERROR(this->get_logger(), "BMI088 Chip ID mismatch! Check wiring.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "BMI088 driver initialized successfully.");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Driver init exception: %s", e.what());
        return;
    }

    /* 创建定时器定时调度 */
    timer_ = this->create_wall_timer(
        period_ns,
        std::bind(&ImuNode::timer_callback, this));
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
    raw_msg.header.frame_id = node_config_.frame_id;
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

        if (cal_count_ >= calibration_target_samples_) {
            gyro_bias_x_ /= static_cast<double>(cal_count_);
            gyro_bias_y_ /= static_cast<double>(cal_count_);
            gyro_bias_z_ /= static_cast<double>(cal_count_);
            calibrated_ = true;
            RCLCPP_INFO(this->get_logger(),
                "Gyro calibration done with %zu samples. Bias: [%.6f, %.6f, %.6f] rad/s",
                cal_count_, gyro_bias_x_, gyro_bias_y_, gyro_bias_z_);
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


Bmi088Config ImuNode::load_driver_config()
{
    Bmi088Config config;
    config.spi_device = this->declare_parameter<std::string>("spi_device", config.spi_device);
    config.acc_cs_gpio = this->declare_parameter<int>("acc_cs_gpio", config.acc_cs_gpio);
    config.gyro_cs_gpio = this->declare_parameter<int>("gyro_cs_gpio", config.gyro_cs_gpio);
    config.acc_range_g = this->declare_parameter<int>("acc_range_g", config.acc_range_g);
    config.gyro_range_dps = this->declare_parameter<int>("gyro_range_dps", config.gyro_range_dps);
    config.acc_odr_hz = this->declare_parameter<int>("acc_odr_hz", config.acc_odr_hz);
    config.gyro_odr_hz = this->declare_parameter<int>("gyro_odr_hz", config.gyro_odr_hz);
    config.spi_speed_hz = this->declare_parameter<int>("spi_speed_hz", static_cast<int>(config.spi_speed_hz));
    return config;
}

ImuNodeConfig ImuNode::load_node_config()
{
    ImuNodeConfig config;
    config.publish_rate_hz =
        this->declare_parameter<double>("publish_rate_hz", config.publish_rate_hz);
    config.frame_id =
        this->declare_parameter<std::string>("frame_id", config.frame_id);
    config.calibration_duration_sec =
        this->declare_parameter<double>("calibration_duration_sec", config.calibration_duration_sec);
    return config;
}

void ImuNode::validate_configs(const Bmi088Config& driver_config, const ImuNodeConfig& node_config)
{
    if (driver_config.acc_cs_gpio < 0 || driver_config.gyro_cs_gpio < 0) {
        throw std::runtime_error("GPIO number must be non-negative");
    }
    if (driver_config.spi_speed_hz == 0) {
        throw std::runtime_error("spi_speed_hz must be > 0");
    }
    if (node_config.publish_rate_hz <= 0.0) {
        throw std::runtime_error("publish_rate_hz must be > 0");
    }
    if (node_config.calibration_duration_sec <= 0.0) {
        throw std::runtime_error("calibration_duration_sec must be > 0");
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ImuNode)
