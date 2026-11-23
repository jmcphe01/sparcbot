#pragma once

#include <cstdint>
#include <string>

// AS5600 12-bit magnetic rotary position sensor, read over I2C.
// Used for the trailer articulation angle (hitch angle between robot and trailer).
//
// The sensor itself is never reconfigured or burned: it is read in its power-on
// state (RAW ANGLE register, 4096 counts per revolution) and the zero point and
// direction are applied here in software, so they can be changed from sparcbot.yaml
// without touching the chip.

struct As5600Config {
    // I2C bus. On the Raspberry Pi 4 the header pins SDA/SCL (GPIO 2/3) are bus 1.
    std::string i2c_device = "/dev/i2c-1";

    // 7-bit I2C address. The AS5600 is fixed at 0x36.
    int i2c_address = 0x36;

    // Sensor reading, in radians (raw counts * 2*pi / 4096), at which the angle should
    // be 0 -- i.e. the trailer straight behind the robot. Measure it by reading the raw
    // angle with the trailer straight (see tests/as5600_test.cpp).
    double zero_offset_rad = 0.0;

    // +1 or -1. Flips the direction in which the angle increases, so that a positive
    // angle means the trailer has swung counter-clockwise (seen from above) relative to
    // the robot, matching ROS REP-103. Applied after the zero offset.
    int direction_sign = +1;
};

// Contents of the STATUS register: is the magnet at a usable distance?
struct As5600Status {
    bool magnet_detected = false;    // MD: a magnet is present
    bool magnet_too_weak = false;    // ML: magnet too far away / too weak
    bool magnet_too_strong = false;  // MH: magnet too close / too strong

    // True when the magnet is detected and neither too weak nor too strong.
    bool magnetOk() const
    {
        return magnet_detected && !magnet_too_weak && !magnet_too_strong;
    }
};

class As5600 {
public:
    static constexpr int COUNTS_PER_REV = 4096;

    // Opens the I2C bus and checks that the sensor answers.
    // Throws std::invalid_argument for a bad config and std::runtime_error if the bus
    // cannot be opened or the sensor does not respond.
    explicit As5600(const As5600Config& config = As5600Config{});
    ~As5600();
    As5600(const As5600&) = delete;
    As5600& operator=(const As5600&) = delete;

    // Raw 12-bit angle, 0..4095. Throws std::runtime_error if the I2C transfer fails.
    uint16_t readRawAngle() const;

    // Calibrated angle in radians, wrapped to [-pi, pi), 0 at the configured zero.
    // Throws std::runtime_error if the I2C transfer fails.
    double readAngleRad() const;

    // Magnet status. Throws std::runtime_error if the I2C transfer fails.
    As5600Status readStatus() const;

    // Pure conversions (no I2C access).
    static double rawToAngleRad(uint16_t raw, const As5600Config& config);
    static double wrapToPi(double angle_rad);

    const As5600Config& config() const { return cfg_; }

private:
    // Reads `len` consecutive registers starting at `reg` in one I2C transaction
    // (register address write, repeated start, read).
    void readRegisters(uint8_t reg, uint8_t* buffer, uint16_t len) const;

    As5600Config cfg_;
    int fd_ = -1;
};