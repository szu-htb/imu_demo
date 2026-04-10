#ifndef BMI088_IIO_READER_HPP_
#define BMI088_IIO_READER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

struct Bmi088IioConfig {
    std::string device_path = "/dev/iio:device1";
    std::string sysfs_dir = "/sys/bus/iio/devices/iio:device1";
    int buffer_length = 128;
    int poll_timeout_ms = 200;
};

struct Bmi088SensorConfig {
    int acc_range_g = 6;
    int gyro_range_dps = 500;
    int accel_sampling_hz = 400;
    int gyro_sampling_hz = 400;
};

struct Bmi088IioSample {
    int16_t accel_x = 0;
    int16_t accel_y = 0;
    int16_t accel_z = 0;
    int16_t gyro_x = 0;
    int16_t gyro_y = 0;
    int16_t gyro_z = 0;
    int64_t timestamp_ns = 0;
};

class Bmi088IioReader {
public:
    static constexpr size_t kScanFrameSize = 24;

    Bmi088IioReader() = default;
    ~Bmi088IioReader();

    bool open(const Bmi088IioConfig& config, std::string& error);
    bool apply_sensor_config(const Bmi088SensorConfig& config, std::string& error);
    bool read_sensor_config(Bmi088SensorConfig& config, std::string& error);
    bool enable_buffer_stream(std::string& error);
    bool read_sample(Bmi088IioSample& sample, std::string& error);
    void close();

private:
    bool configure_scan_elements(std::string& error);
    bool configure_buffer(std::string& error);
    bool read_sysfs_attr(const std::string& relative_path, std::string& value, std::string& error) const;
    bool write_sysfs_attr(const std::string& relative_path, const std::string& value, std::string& error);
    bool read_full(uint8_t* buf, size_t len, std::string& error);
    void decode_frame(const uint8_t* frame, Bmi088IioSample& sample) const;

    Bmi088IioConfig config_{};
    int fd_ = -1;
    bool buffer_enabled_ = false;
};

#endif  // BMI088_IIO_READER_HPP_
