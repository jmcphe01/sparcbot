// trailer_sensor
//
// Thin ROS 2 wrapper around sparcbot_core's As5600 (trailer articulation angle sensor).
// No logic lives here beyond reading the sensor and publishing.
//
//   AS5600 (I2C)  ->  /trailer_angle   at rate_hz
//
// Parameters (from sparcbot.yaml)
//   rate_hz          publish rate [Hz]
//   zero_offset_deg  sensor reading, in degrees, with the trailer straight behind the robot
//                    (measure it with: ros2 run sparcbot_core as5600_test)
//   direction_sign   +1 or -1: chooses the direction in which the angle increases
//   i2c_device       optional, default /dev/i2c-1
//
// Topics
//   pub  trailer_angle   sparcbot_interfaces/TrailerAngle
//
// A failed I2C transfer skips that sample (a warning is printed, at most every 2 s) rather
// than stopping the node. If the magnet is missing or at the wrong distance the message is
// still published, with magnet_ok = false, so a consumer can decide what to do.

#include <cstdint>
#include <chrono>
#include <exception>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "sparcbot/hardware/as5600.hpp"
#include "sparcbot_interfaces/msg/trailer_angle.hpp"

using sparcbot_interfaces::msg::TrailerAngle;

namespace {

const char* magnetProblem(const As5600Status& status)
{
    if (!status.magnet_detected) return "not detected";
    if (status.magnet_too_weak) return "too weak (too far from the sensor)";
    return "too strong (too close to the sensor)";
}

class TrailerSensorNode : public rclcpp::Node {
public:
    TrailerSensorNode()
        : Node("trailer_sensor")
    {
        const double rate_hz = declare_parameter<double>("rate_hz");
        if (rate_hz <= 0.0) {
            throw std::invalid_argument("rate_hz must be > 0");
        }

        const double zero_offset_deg = declare_parameter<double>("zero_offset_deg");

        As5600Config config;
        config.zero_offset_rad = zero_offset_deg * std::numbers::pi / 180.0;
        config.direction_sign = static_cast<int>(declare_parameter<int64_t>("direction_sign"));
        config.i2c_device = declare_parameter<std::string>("i2c_device", config.i2c_device);

        // Throws if the bus cannot be opened or the sensor does not answer.
        sensor_ = std::make_unique<As5600>(config);

        publisher_ = create_publisher<TrailerAngle>("trailer_angle", rclcpp::QoS(rclcpp::KeepLast(1)));

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate_hz),
            [this]() { onTimer(); });

        RCLCPP_INFO(get_logger(),
                    "trailer_sensor running on %s at %.1f Hz (zero offset %.2f deg, direction %+d).",
                    config.i2c_device.c_str(), rate_hz, zero_offset_deg, config.direction_sign);
    }

private:
    void onTimer()
    {
        const auto stamp = get_clock()->now();

        uint16_t raw = 0;
        As5600Status status;
        try {
            raw = sensor_->readRawAngle();
            status = sensor_->readStatus();
        }
        catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "%s (skipping this sample)", e.what());
            return;
        }

        if (!status.magnetOk()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "AS5600 magnet %s: the angle is not reliable.",
                                 magnetProblem(status));
        }

        TrailerAngle msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "base_link";
        msg.raw = raw;
        // Convert the same sample that was just read, so angle_rad and raw always agree.
        msg.angle_rad = As5600::rawToAngleRad(raw, sensor_->config());
        msg.magnet_ok = status.magnetOk();
        msg.magnet_detected = status.magnet_detected;
        msg.magnet_too_weak = status.magnet_too_weak;
        msg.magnet_too_strong = status.magnet_too_strong;
        publisher_->publish(msg);
    }

    std::unique_ptr<As5600> sensor_;
    rclcpp::Publisher<TrailerAngle>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    int exit_code = 0;
    try {
        rclcpp::spin(std::make_shared<TrailerSensorNode>());
    }
    catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("trailer_sensor"), "%s", e.what());
        exit_code = 1;
    }

    rclcpp::shutdown();
    return exit_code;
}
