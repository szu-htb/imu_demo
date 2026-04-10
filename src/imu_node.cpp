#include "imu_demo/imu_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <cmath>
#include <stdexcept>

namespace {

constexpr double kGravityMps2 = 9.80665;
constexpr double kPi = 3.14159265358979323846;

double accel_scale_from_range(int range_g)
{
    switch (range_g) {
        case 3:
        case 6:
        case 12:
        case 24:
            return (static_cast<double>(range_g) * kGravityMps2) / 32768.0;
        default:
            throw std::runtime_error("Unsupported acc_range_g: " + std::to_string(range_g));
    }
}

double gyro_scale_from_range(int range_dps)
{
    switch (range_dps) {
        case 125:
        case 250:
        case 500:
        case 1000:
        case 2000:
            return (static_cast<double>(range_dps) * kPi / 180.0) / 32768.0;
        default:
            throw std::runtime_error("Unsupported gyro_range_dps: " + std::to_string(range_dps));
    }
}

}  // namespace

ImuNode::ImuNode(const rclcpp::NodeOptions& options) : Node("bmi088_node", options)
{
    config_ = load_node_config();
    validate_config(config_);

    clock_ = get_clock();

    msg_.header.frame_id = config_.frame_id;
    msg_.orientation_covariance[0] = -1.0;

    pub_ = create_publisher<sensor_msgs::msg::Imu>(config_.output_topic, rclcpp::SensorDataQoS().reliable());

    std::string error;
    if (!reader_.open(config_.iio, error)) {
        throw std::runtime_error("Failed to open BMI088 IIO reader: " + error);
    }

    if (!reader_.apply_sensor_config(config_.sensor, error)) {
        throw std::runtime_error("Failed to apply BMI088 sensor config: " + error);
    }

    Bmi088SensorConfig active_sensor_config;
    if (!reader_.read_sensor_config(active_sensor_config, error)) {
        throw std::runtime_error("Failed to read BMI088 sensor config: " + error);
    }

    if (!reader_.enable_buffer_stream(error)) {
        throw std::runtime_error("Failed to enable BMI088 buffer stream: " + error);
    }

    config_.sensor = active_sensor_config;
    acc_scale_ = accel_scale_from_range(config_.sensor.acc_range_g);
    gyro_scale_ = gyro_scale_from_range(config_.sensor.gyro_range_dps);

    RCLCPP_INFO(this->get_logger(),
                "BMI088 IIO reader ready: dev=%s sysfs=%s buffer_length=%d frame_id=%s topic=%s "
                "acc_range=%dg gyro_range=%ddps accel_freq=%dHz gyro_freq=%dHz accel_scale=%.9f gyro_scale=%.9f",
                config_.iio.device_path.c_str(),
                config_.iio.sysfs_dir.c_str(),
                config_.iio.buffer_length,
                config_.frame_id.c_str(),
                config_.output_topic.c_str(),
                config_.sensor.acc_range_g,
                config_.sensor.gyro_range_dps,
                config_.sensor.accel_sampling_hz,
                config_.sensor.gyro_sampling_hz,
                acc_scale_,
                gyro_scale_);

    running_.store(true);
    reader_thread_ = std::thread(&ImuNode::reader_loop, this);
}

ImuNode::~ImuNode()
{
    running_.store(false);
    reader_.close();

    if (reader_thread_.joinable()) {
        reader_thread_.join();
    }
}

void ImuNode::publish_sample(const Bmi088IioSample& raw)
{
    msg_.header.stamp = rclcpp::Time(raw.timestamp_ns);

    msg_.linear_acceleration.x = static_cast<double>(raw.accel_x) * acc_scale_;
    msg_.linear_acceleration.y = static_cast<double>(raw.accel_y) * acc_scale_;
    msg_.linear_acceleration.z = static_cast<double>(raw.accel_z) * acc_scale_;

    msg_.angular_velocity.x = static_cast<double>(raw.gyro_x) * gyro_scale_;
    msg_.angular_velocity.y = static_cast<double>(raw.gyro_y) * gyro_scale_;
    msg_.angular_velocity.z = static_cast<double>(raw.gyro_z) * gyro_scale_;

    pub_->publish(msg_);
}

void ImuNode::reader_loop()
{
    while (rclcpp::ok() && running_.load()) {
        Bmi088IioSample sample;
        std::string error;

        if (!reader_.read_sample(sample, error)) {
            if (!running_.load()) {
                break;
            }

            if (!error.empty()) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(),
                                      *clock_,
                                      2000,
                                      "Failed to read BMI088 IIO sample: %s",
                                      error.c_str());
            }
            continue;
        }

        publish_sample(sample);
    }
}

ImuNodeConfig ImuNode::load_node_config()
{
    ImuNodeConfig config;

    config.frame_id = declare_parameter<std::string>("frame_id", config.frame_id);
    config.output_topic = declare_parameter<std::string>("output_topic", config.output_topic);
    config.sensor.acc_range_g = declare_parameter<int>("acc_range_g", config.sensor.acc_range_g);
    config.sensor.gyro_range_dps = declare_parameter<int>("gyro_range_dps", config.sensor.gyro_range_dps);
    config.sensor.accel_sampling_hz =
        declare_parameter<int>("accel_sampling_hz", config.sensor.accel_sampling_hz);
    config.sensor.gyro_sampling_hz =
        declare_parameter<int>("gyro_sampling_hz", config.sensor.gyro_sampling_hz);
    config.iio.device_path = declare_parameter<std::string>("iio_device_path", config.iio.device_path);
    config.iio.sysfs_dir = declare_parameter<std::string>("iio_sysfs_dir", config.iio.sysfs_dir);
    config.iio.buffer_length = declare_parameter<int>("buffer_length", config.iio.buffer_length);
    config.iio.poll_timeout_ms = declare_parameter<int>("poll_timeout_ms", config.iio.poll_timeout_ms);

    RCLCPP_INFO(this->get_logger(),
                "Node config: frame_id=%s topic=%s acc_range=%d g gyro_range=%d dps accel_freq=%d Hz "
                "gyro_freq=%d Hz dev=%s sysfs=%s buffer_length=%d poll_timeout_ms=%d",
                config.frame_id.c_str(),
                config.output_topic.c_str(),
                config.sensor.acc_range_g,
                config.sensor.gyro_range_dps,
                config.sensor.accel_sampling_hz,
                config.sensor.gyro_sampling_hz,
                config.iio.device_path.c_str(),
                config.iio.sysfs_dir.c_str(),
                config.iio.buffer_length,
                config.iio.poll_timeout_ms);

    return config;
}

void ImuNode::validate_config(const ImuNodeConfig& config)
{
    if (config.frame_id.empty()) {
        throw std::runtime_error("frame_id must not be empty");
    }

    if (config.output_topic.empty()) {
        throw std::runtime_error("output_topic must not be empty");
    }

    if (config.iio.device_path.empty()) {
        throw std::runtime_error("iio_device_path must not be empty");
    }

    if (config.iio.sysfs_dir.empty()) {
        throw std::runtime_error("iio_sysfs_dir must not be empty");
    }

    if (config.iio.buffer_length <= 0) {
        throw std::runtime_error("buffer_length must be > 0");
    }

    if (config.iio.poll_timeout_ms <= 0) {
        throw std::runtime_error("poll_timeout_ms must be > 0");
    }

    if (config.sensor.accel_sampling_hz <= 0) {
        throw std::runtime_error("accel_sampling_hz must be > 0");
    }

    if (config.sensor.gyro_sampling_hz <= 0) {
        throw std::runtime_error("gyro_sampling_hz must be > 0");
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ImuNode)
