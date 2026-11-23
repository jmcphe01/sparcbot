#include "sparcbot/hardware/as5600.hpp"

#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <numbers>
#include <stdexcept>

namespace {

// AS5600 registers.
constexpr uint8_t REG_STATUS = 0x0B;
constexpr uint8_t REG_RAW_ANGLE_HIGH = 0x0C;   // RAW ANGLE = 0x0C (high) + 0x0D (low)

// STATUS register bits.
constexpr uint8_t STATUS_MH = 1u << 3;   // magnet too strong
constexpr uint8_t STATUS_ML = 1u << 4;   // magnet too weak
constexpr uint8_t STATUS_MD = 1u << 5;   // magnet detected

// RAW ANGLE is 12 bits: the high byte only carries bits 11..8 in its low nibble.
constexpr uint8_t RAW_ANGLE_HIGH_MASK = 0x0F;

constexpr double TWO_PI = 2.0 * std::numbers::pi;

} // namespace

As5600::As5600(const As5600Config& config)
    : cfg_(config)
{
    if (cfg_.direction_sign != 1 && cfg_.direction_sign != -1) {
        throw std::invalid_argument("AS5600 direction_sign must be +1 or -1");
    }
    if (cfg_.i2c_address < 0x03 || cfg_.i2c_address > 0x77) {
        throw std::invalid_argument("AS5600 i2c_address must be a 7-bit address (0x03..0x77)");
    }

    fd_ = ::open(cfg_.i2c_device.c_str(), O_RDWR | O_CLOEXEC);
    if (fd_ < 0) {
        throw std::runtime_error(
            "Could not open " + cfg_.i2c_device + ": " + std::strerror(errno) +
            " (is I2C enabled, and is this user in the 'i2c' group?)");
    }

    // Opening the bus succeeds even with nothing connected, so probe the sensor now to
    // fail with a clear message instead of on the first reading. A throwing constructor
    // does not run the destructor, so close the descriptor here.
    try {
        (void)readStatus();
    }
    catch (const std::exception& e) {
        ::close(fd_);
        fd_ = -1;
        throw std::runtime_error(
            std::string("AS5600 not responding: ") + e.what() +
            " (check wiring and power; `i2cdetect -y 1` should list 0x36)");
    }
}

As5600::~As5600()
{
    if (fd_ >= 0) {
        ::close(fd_);
    }
}

void As5600::readRegisters(uint8_t reg, uint8_t* buffer, uint16_t len) const
{
    // One combined transaction: write the register address, repeated start, read.
    i2c_msg messages[2] = {};
    messages[0].addr = static_cast<__u16>(cfg_.i2c_address);
    messages[0].flags = 0;
    messages[0].len = 1;
    messages[0].buf = &reg;
    messages[1].addr = static_cast<__u16>(cfg_.i2c_address);
    messages[1].flags = I2C_M_RD;
    messages[1].len = len;
    messages[1].buf = buffer;

    i2c_rdwr_ioctl_data transfer = {};
    transfer.msgs = messages;
    transfer.nmsgs = 2;

    const int done = ::ioctl(fd_, I2C_RDWR, &transfer);
    if (done < 0) {
        throw std::runtime_error(
            std::string("I2C read from AS5600 failed: ") + std::strerror(errno));
    }
    if (done != 2) {
        throw std::runtime_error("I2C read from AS5600 failed: incomplete transfer");
    }
}

uint16_t As5600::readRawAngle() const
{
    // Both bytes in a single burst, so they come from the same sample.
    uint8_t bytes[2] = {0, 0};
    readRegisters(REG_RAW_ANGLE_HIGH, bytes, 2);

    return static_cast<uint16_t>(
        (static_cast<uint16_t>(bytes[0] & RAW_ANGLE_HIGH_MASK) << 8) | bytes[1]);
}

double As5600::readAngleRad() const
{
    return rawToAngleRad(readRawAngle(), cfg_);
}

As5600Status As5600::readStatus() const
{
    uint8_t status = 0;
    readRegisters(REG_STATUS, &status, 1);

    As5600Status result;
    result.magnet_detected = (status & STATUS_MD) != 0;
    result.magnet_too_weak = (status & STATUS_ML) != 0;
    result.magnet_too_strong = (status & STATUS_MH) != 0;
    return result;
}

double As5600::rawToAngleRad(uint16_t raw, const As5600Config& config)
{
    const double sensor_rad =
        static_cast<double>(raw & 0x0FFF) * (TWO_PI / static_cast<double>(COUNTS_PER_REV));

    return wrapToPi(config.direction_sign * (sensor_rad - config.zero_offset_rad));
}

double As5600::wrapToPi(double angle_rad)
{
    return angle_rad - TWO_PI * std::floor((angle_rad + std::numbers::pi) / TWO_PI);
}