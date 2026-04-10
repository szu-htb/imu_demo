#include "imu_demo/bmi088_iio_reader.hpp"

#include <cerrno>
#include <charconv>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sstream>
#include <string>
#include <unistd.h>

namespace {

int16_t decode_le16(const uint8_t* buf)
{
    return static_cast<int16_t>(static_cast<uint16_t>(buf[0]) |
                                (static_cast<uint16_t>(buf[1]) << 8));
}

int64_t decode_le64(const uint8_t* buf)
{
    uint64_t value = 0;

    for (int i = 0; i < 8; ++i) {
        value |= static_cast<uint64_t>(buf[i]) << (i * 8);
    }

    return static_cast<int64_t>(value);
}

std::string errno_message(const std::string& context)
{
    std::ostringstream oss;
    oss << context << ": " << std::strerror(errno);
    return oss.str();
}

bool read_int_from_string(const std::string& value, int& out)
{
    const char* begin = value.data();
    const char* end = begin + value.size();
    auto result = std::from_chars(begin, end, out);
    return result.ec == std::errc() && result.ptr == end;
}

void trim_trailing_newline(std::string& value)
{
    while (!value.empty() && (value.back() == '\n' || value.back() == '\r')) {
        value.pop_back();
    }
}

}  // namespace

Bmi088IioReader::~Bmi088IioReader()
{
    close();
}

bool Bmi088IioReader::open(const Bmi088IioConfig& config, std::string& error)
{
    close();
    config_ = config;

    if (!configure_scan_elements(error)) {
        return false;
    }

    if (!configure_buffer(error)) {
        return false;
    }

    fd_ = ::open(config_.device_path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd_ < 0) {
        error = errno_message("open IIO device failed");
        close();
        return false;
    }

    return true;
}

bool Bmi088IioReader::apply_sensor_config(const Bmi088SensorConfig& config, std::string& error)
{
    if (config.accel_sampling_hz <= 0) {
        error = "accel_sampling_hz must be > 0";
        return false;
    }

    if (config.gyro_sampling_hz <= 0) {
        error = "gyro_sampling_hz must be > 0";
        return false;
    }

    if (!write_sysfs_attr("accel_range", std::to_string(config.acc_range_g), error)) {
        return false;
    }

    if (!write_sysfs_attr("gyro_range", std::to_string(config.gyro_range_dps), error)) {
        return false;
    }

    if (!write_sysfs_attr("in_accel_sampling_frequency", std::to_string(config.accel_sampling_hz), error)) {
        return false;
    }

    if (!write_sysfs_attr("in_anglvel_sampling_frequency", std::to_string(config.gyro_sampling_hz), error)) {
        return false;
    }

    return true;
}

bool Bmi088IioReader::read_sensor_config(Bmi088SensorConfig& config, std::string& error)
{
    std::string value;
    int parsed_value = 0;

    if (!read_sysfs_attr("accel_range", value, error)) {
        return false;
    }
    if (!read_int_from_string(value, parsed_value)) {
        error = "failed to parse accel_range: " + value;
        return false;
    }
    config.acc_range_g = parsed_value;

    if (!read_sysfs_attr("gyro_range", value, error)) {
        return false;
    }
    if (!read_int_from_string(value, parsed_value)) {
        error = "failed to parse gyro_range: " + value;
        return false;
    }
    config.gyro_range_dps = parsed_value;

    if (!read_sysfs_attr("in_accel_sampling_frequency", value, error)) {
        return false;
    }
    if (!read_int_from_string(value, parsed_value)) {
        error = "failed to parse in_accel_sampling_frequency: " + value;
        return false;
    }
    config.accel_sampling_hz = parsed_value;

    if (!read_sysfs_attr("in_anglvel_sampling_frequency", value, error)) {
        return false;
    }
    if (!read_int_from_string(value, parsed_value)) {
        error = "failed to parse in_anglvel_sampling_frequency: " + value;
        return false;
    }
    config.gyro_sampling_hz = parsed_value;

    return true;
}

bool Bmi088IioReader::enable_buffer_stream(std::string& error)
{
    if (buffer_enabled_) {
        return true;
    }

    if (!write_sysfs_attr("buffer/enable", "1", error)) {
        return false;
    }

    buffer_enabled_ = true;
    return true;
}

bool Bmi088IioReader::read_sample(Bmi088IioSample& sample, std::string& error)
{
    struct pollfd pfd = {};
    pfd.fd = fd_;
    pfd.events = POLLIN;

    const int ret = ::poll(&pfd, 1, config_.poll_timeout_ms);
    if (ret < 0) {
        if (errno == EINTR) {
            error.clear();
            return false;
        }

        error = errno_message("poll IIO device failed");
        return false;
    }

    if (ret == 0) {
        error.clear();
        return false;
    }

    if ((pfd.revents & POLLIN) == 0) {
        error = "poll IIO device returned without POLLIN";
        return false;
    }

    uint8_t frame[kScanFrameSize];
    if (!read_full(frame, sizeof(frame), error)) {
        return false;
    }

    decode_frame(frame, sample);
    return true;
}

void Bmi088IioReader::close()
{
    if (buffer_enabled_) {
        std::string error;
        if (!write_sysfs_attr("buffer/enable", "0", error)) {
        }
        buffer_enabled_ = false;
    }

    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool Bmi088IioReader::configure_scan_elements(std::string& error)
{
    static const char* const kAttrs[] = {
        "scan_elements/in_accel_x_en",
        "scan_elements/in_accel_y_en",
        "scan_elements/in_accel_z_en",
        "scan_elements/in_anglvel_x_en",
        "scan_elements/in_anglvel_y_en",
        "scan_elements/in_anglvel_z_en",
        "scan_elements/in_timestamp_en",
    };

    for (const char* attr : kAttrs) {
        if (!write_sysfs_attr(attr, "1", error)) {
            return false;
        }
    }

    return true;
}

bool Bmi088IioReader::configure_buffer(std::string& error)
{
    if (!write_sysfs_attr("buffer/enable", "0", error)) {
        return false;
    }

    buffer_enabled_ = false;

    if (!write_sysfs_attr("buffer/length", std::to_string(config_.buffer_length), error)) {
        return false;
    }

    return true;
}

bool Bmi088IioReader::read_sysfs_attr(const std::string& relative_path,
                                      std::string& value,
                                      std::string& error) const
{
    const std::string path = config_.sysfs_dir + "/" + relative_path;
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
        error = errno_message("open sysfs attr failed: " + path);
        return false;
    }

    char buf[128];
    const ssize_t ret = ::read(fd, buf, sizeof(buf) - 1);
    ::close(fd);
    if (ret < 0) {
        error = errno_message("read sysfs attr failed: " + path);
        return false;
    }

    buf[ret] = '\0';
    value.assign(buf, static_cast<size_t>(ret));
    trim_trailing_newline(value);
    return true;
}

bool Bmi088IioReader::write_sysfs_attr(const std::string& relative_path,
                                       const std::string& value,
                                       std::string& error)
{
    const std::string path = config_.sysfs_dir + "/" + relative_path;
    const int fd = ::open(path.c_str(), O_WRONLY | O_CLOEXEC);
    if (fd < 0) {
        error = errno_message("open sysfs attr failed: " + path);
        return false;
    }

    const ssize_t ret = ::write(fd, value.c_str(), value.size());
    if (ret != static_cast<ssize_t>(value.size())) {
        error = errno_message("write sysfs attr failed: " + path);
        ::close(fd);
        return false;
    }

    ::close(fd);
    return true;
}

bool Bmi088IioReader::read_full(uint8_t* buf, size_t len, std::string& error)
{
    size_t done = 0;

    while (done < len) {
        const ssize_t ret = ::read(fd_, buf + done, len - done);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }

            error = errno_message("read IIO device failed");
            return false;
        }

        if (ret == 0) {
            error = "read IIO device returned EOF";
            return false;
        }

        done += static_cast<size_t>(ret);
    }

    return true;
}

void Bmi088IioReader::decode_frame(const uint8_t* frame, Bmi088IioSample& sample) const
{
    sample.accel_x = decode_le16(frame + 0);
    sample.accel_y = decode_le16(frame + 2);
    sample.accel_z = decode_le16(frame + 4);
    sample.gyro_x = decode_le16(frame + 6);
    sample.gyro_y = decode_le16(frame + 8);
    sample.gyro_z = decode_le16(frame + 10);
    sample.timestamp_ns = decode_le64(frame + 16);
}
