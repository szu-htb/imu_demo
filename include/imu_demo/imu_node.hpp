#ifndef IMU_NODE_HPP_
#define IMU_NODE_HPP_

#include "imu_demo/bmi088_iio_reader.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <atomic>
#include <thread>

struct ImuNodeConfig {
    std::string frame_id = "imu_link";
    std::string output_topic = "imu/data_calibrated";
    Bmi088SensorConfig sensor;
    Bmi088IioConfig iio;
};

class ImuNode : public rclcpp::Node {
public:
    explicit ImuNode(const rclcpp::NodeOptions& options);
    ~ImuNode() override;

private:
    ImuNodeConfig load_node_config();
    void validate_config(const ImuNodeConfig& config);
    void reader_loop();
    void publish_sample(const Bmi088IioSample& raw);

    Bmi088IioReader reader_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
    rclcpp::Clock::SharedPtr clock_;

    ImuNodeConfig config_;
    sensor_msgs::msg::Imu msg_;
    double acc_scale_ = 0.0;
    double gyro_scale_ = 0.0;

    std::atomic<bool> running_{false};
    std::thread reader_thread_;
};

#endif  // IMU_NODE_HPP_
