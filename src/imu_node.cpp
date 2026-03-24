#include "imu_demo/imu_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <chrono>

ImuNode::ImuNode(const rclcpp::NodeOptions& options) : Node("bmi088_node", options)
{
    const Bmi088Config driver_config = load_driver_config();
    node_config_ = load_node_config();
    validate_configs(driver_config, node_config_);

    // todo: 根据yaml的配置动态选择通信接口（SPI/I2C），目前只实现了SPI -->
    // 考虑抽象为一个初始化通信接口的函数，采用工厂模式封装接口，Node层零改动
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

    // 缓存 clock 指针，避免热路径每帧原子引用计数
    this->clock_ = this->get_clock();

    // 不变字段初始化一次，热路径只更新 stamp 和数据
    msg_.header.frame_id = node_config_.frame_id;
    msg_.orientation_covariance[0] = -1.0;

    this->pub_ =
        this->create_publisher<sensor_msgs::msg::Imu>("imu/data_calibrated", rclcpp::SensorDataQoS().reliable());
    RCLCPP_INFO(this->get_logger(), "IMU Rate: %dHz", node_config_.publish_rate_hz);

    this->poll_timer_ = this->create_wall_timer(std::chrono::milliseconds((1000 / node_config_.publish_rate_hz)),
                                                std::bind(&ImuNode::timer_callback, this));
}

double ImuNode::median3(double a, double b, double c)
{
    if (a > b)
        std::swap(a, b);
    if (b > c)
        std::swap(b, c);
    if (a > b)
        std::swap(a, b);
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

    ImuRawData filtered = apply_median_filter(raw);

    msg_.header.stamp = clock_->now();
    msg_.linear_acceleration.x = filtered.ax;
    msg_.linear_acceleration.y = filtered.ay;
    msg_.linear_acceleration.z = filtered.az;
    msg_.angular_velocity.x = filtered.gx;
    msg_.angular_velocity.y = filtered.gy;
    msg_.angular_velocity.z = filtered.gz;
    pub_->publish(msg_);
}

Bmi088Config ImuNode::load_driver_config()
{
    Bmi088Config config;
    config.bus_type = this->declare_parameter<std::string>("bus_type", config.bus_type);
    if (config.bus_type == "spi") {
        config.spi_device = this->declare_parameter<std::string>("spi_device", config.spi_device);
        config.acc_cs_gpio = this->declare_parameter<int>("acc_cs_gpio", config.acc_cs_gpio);
        config.gyro_cs_gpio = this->declare_parameter<int>("gyro_cs_gpio", config.gyro_cs_gpio);
        config.spi_speed_hz = this->declare_parameter<int>("spi_speed_hz", static_cast<int>(config.spi_speed_hz));

        RCLCPP_INFO(this->get_logger(),
                    "BMI088 config: spi=%s, acc_cs=%d, gyro_cs=%d, spi_speed=%u Hz",
                    config.spi_device.c_str(),
                    config.acc_cs_gpio,
                    config.gyro_cs_gpio,
                    config.spi_speed_hz);
    }

    config.acc_range_g = this->declare_parameter<int>("acc_range_g", config.acc_range_g);
    config.gyro_range_dps = this->declare_parameter<int>("gyro_range_dps", config.gyro_range_dps);
    config.acc_odr_hz = this->declare_parameter<int>("acc_odr_hz", config.acc_odr_hz);
    config.gyro_odr_hz = this->declare_parameter<int>("gyro_odr_hz", config.gyro_odr_hz);

    RCLCPP_INFO(this->get_logger(),
                "Sensor config: acc_range=%d g, gyro_range=%d dps, acc_odr=%d Hz, gyro_odr=%d Hz",
                config.acc_range_g,
                config.gyro_range_dps,
                config.acc_odr_hz,
                config.gyro_odr_hz);

    config.enable_self_test = this->declare_parameter<bool>("enable_self_test", config.enable_self_test);
    RCLCPP_INFO(this->get_logger(), "Self-test: %s", config.enable_self_test ? "enabled" : "disabled");

    return config;
}

ImuNodeConfig ImuNode::load_node_config()
{
    ImuNodeConfig config;
    config.publish_rate_hz = this->declare_parameter<int>("publish_rate_hz", config.publish_rate_hz);
    config.frame_id = this->declare_parameter<std::string>("frame_id", config.frame_id);

    RCLCPP_INFO(this->get_logger(),
                "Node config: frame_id=%s, publish_rate=%d Hz",
                config.frame_id.c_str(),
                config.publish_rate_hz);

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
}

RCLCPP_COMPONENTS_REGISTER_NODE(ImuNode)
