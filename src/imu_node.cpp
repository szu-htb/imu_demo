#include "imu_demo/imu_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>
#include <functional>

ImuNode::ImuNode(const rclcpp::NodeOptions& options) : Node("bmi088_node", options)
{
    const Bmi088Config driver_config = load_driver_config();
    node_config_ = load_node_config();
    use_gyro_fifo_ = driver_config.use_gyro_fifo;
    validate_configs(driver_config, node_config_);

    try {
        driver_ = std::make_unique<Bmi088Driver>(
            driver_config, [this](const std::string& msg) { RCLCPP_INFO(this->get_logger(), "%s", msg.c_str()); });
        if (!driver_->initialize()) {
            RCLCPP_ERROR(this->get_logger(), "BMI088 initialization failed.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "BMI088 driver initialized successfully.");
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Driver init exception: %s", e.what());
        return;
    }

    // FIFO 模式下逐帧回推时间戳要用到 gyro ODR 对应的固定采样周期。
    gyro_sample_period_ns_ = 1000000000LL / static_cast<int64_t>(driver_config.gyro_odr_hz);

    // 缓存 clock 指针，避免热路径每帧原子引用计数
    clock_ = get_clock();

    // 不变字段初始化一次，热路径只更新 stamp 和数据
    msg_.header.frame_id = node_config_.frame_id;
    msg_.orientation_covariance[0] = -1.0;

    pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_calibrated", rclcpp::SensorDataQoS().reliable());

    const int timer_rate_hz = use_gyro_fifo_ ? node_config_.fifo_poll_rate_hz : node_config_.publish_rate_hz;
    if (use_gyro_fifo_) {
        RCLCPP_INFO(this->get_logger(),
                    "Timer mode: FIFO poll=%d Hz, gyro_odr=%d Hz",
                    node_config_.fifo_poll_rate_hz,
                    driver_config.gyro_odr_hz);
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Timer mode: direct read=%d Hz", node_config_.publish_rate_hz);
    }
    poll_timer_ = create_wall_timer(std::chrono::microseconds(1000000 / timer_rate_hz),
                                    std::bind(&ImuNode::timer_callback, this));
}

void ImuNode::publish_sample(const ImuRawData& raw, const rclcpp::Time& stamp)
{
    msg_.header.stamp = stamp;
    msg_.linear_acceleration.x = raw.ax;
    msg_.linear_acceleration.y = raw.ay;
    msg_.linear_acceleration.z = raw.az;
    msg_.angular_velocity.x = raw.gx;
    msg_.angular_velocity.y = raw.gy;
    msg_.angular_velocity.z = raw.gz;
    pub_->publish(msg_);
}

void ImuNode::publish_fifo_samples(const ImuRawData* raw_samples, size_t sample_count, const rclcpp::Time& newest_stamp)
{
    for (size_t i = 0; i < sample_count; ++i) {
        const int64_t offset_samples = static_cast<int64_t>(sample_count - 1 - i);
        const auto offset = std::chrono::nanoseconds(offset_samples * gyro_sample_period_ns_);
        publish_sample(raw_samples[i], newest_stamp - rclcpp::Duration(offset));
    }
}

void ImuNode::timer_callback()
{
    if (use_gyro_fifo_) {
        Bmi088Driver::FifoReadResult result;
        if (!driver_->read_imu_fifo_data(fifo_buf_.data(), fifo_buf_.size(), result)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read IMU FIFO data");
            return;
        }

        if (result.overrun && !fifo_overrun_warned_) {
            fifo_overrun_warned_ = true;
            RCLCPP_WARN(this->get_logger(), "Gyro FIFO overrun detected. FIFO samples may have been dropped.");
        }

        if (result.sample_count == 0) {
            return;
        }

        publish_fifo_samples(fifo_buf_.data(), result.sample_count, clock_->now());
        return;
    }

    ImuRawData raw;
    if (!driver_->read_imu_data(raw)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read IMU data");
        return;
    }
    publish_sample(raw, clock_->now());
}

Bmi088Config ImuNode::load_driver_config()
{
    Bmi088Config config;
    config.bus_type = declare_parameter<std::string>("bus_type", config.bus_type);
    if (config.bus_type == "spi") {
        config.spi_device = declare_parameter<std::string>("spi_device", config.spi_device);
        config.acc_cs_gpio = declare_parameter<int>("acc_cs_gpio", config.acc_cs_gpio);
        config.gyro_cs_gpio = declare_parameter<int>("gyro_cs_gpio", config.gyro_cs_gpio);
        config.spi_speed_hz = declare_parameter<int>("spi_speed_hz", static_cast<int>(config.spi_speed_hz));

        RCLCPP_INFO(this->get_logger(),
                    "BMI088 config: spi=%s, acc_cs=%d, gyro_cs=%d, spi_speed=%u Hz",
                    config.spi_device.c_str(),
                    config.acc_cs_gpio,
                    config.gyro_cs_gpio,
                    config.spi_speed_hz);
    }

    config.acc_range_g = declare_parameter<int>("acc_range_g", config.acc_range_g);
    config.gyro_range_dps = declare_parameter<int>("gyro_range_dps", config.gyro_range_dps);
    config.acc_odr_hz = declare_parameter<int>("acc_odr_hz", config.acc_odr_hz);
    config.gyro_odr_hz = declare_parameter<int>("gyro_odr_hz", config.gyro_odr_hz);
    config.use_gyro_fifo = declare_parameter<bool>("use_gyro_fifo", config.use_gyro_fifo);

    RCLCPP_INFO(this->get_logger(),
                "Sensor config: acc_range=%d g, gyro_range=%d dps, acc_odr=%d Hz, gyro_odr=%d Hz, gyro_fifo=%s",
                config.acc_range_g,
                config.gyro_range_dps,
                config.acc_odr_hz,
                config.gyro_odr_hz,
                config.use_gyro_fifo ? "enabled" : "disabled");

    config.enable_self_test = declare_parameter<bool>("enable_self_test", config.enable_self_test);
    RCLCPP_INFO(this->get_logger(), "Self-test: %s", config.enable_self_test ? "enabled" : "disabled");

    return config;
}

ImuNodeConfig ImuNode::load_node_config()
{
    ImuNodeConfig config;
    config.publish_rate_hz = declare_parameter<int>("publish_rate_hz", config.publish_rate_hz);
    config.fifo_poll_rate_hz = declare_parameter<int>("fifo_poll_rate_hz", config.fifo_poll_rate_hz);
    config.frame_id = declare_parameter<std::string>("frame_id", config.frame_id);

    RCLCPP_INFO(this->get_logger(),
                "Node config: frame_id=%s, publish_rate=%d Hz, fifo_poll_rate=%d Hz",
                config.frame_id.c_str(),
                config.publish_rate_hz,
                config.fifo_poll_rate_hz);

    return config;
}

void ImuNode::validate_configs(const Bmi088Config& driver_config, const ImuNodeConfig& node_config)
{
    if (driver_config.acc_cs_gpio < 0 || driver_config.gyro_cs_gpio < 0) {
        throw std::runtime_error("GPIO number must be non-negative");
    }
    if (node_config.publish_rate_hz <= 0) {
        throw std::runtime_error("publish_rate_hz must be > 0");
    }
    if (node_config.fifo_poll_rate_hz <= 0) {
        throw std::runtime_error("fifo_poll_rate_hz must be > 0");
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ImuNode)
