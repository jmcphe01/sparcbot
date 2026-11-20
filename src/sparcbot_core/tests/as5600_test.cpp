// AS5600 bench test: prints the raw sensor angle, the calibrated angle and the magnet status.


#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <numbers>
#include <string>
#include <thread>

#include "sparcbot/hardware/as5600.hpp"

namespace {

volatile std::sig_atomic_t g_stop = 0;

void onSignal(int)
{
    g_stop = 1;
}

const char* magnetText(const As5600Status& status)
{
    if (!status.magnet_detected) return "NOT DETECTED";
    if (status.magnet_too_weak) return "TOO WEAK (move the magnet closer)";
    if (status.magnet_too_strong) return "TOO STRONG (move the magnet away)";
    return "OK";
}

double toDegrees(double radians)
{
    return radians * 180.0 / std::numbers::pi;
}

} // namespace

int main(int argc, char** argv)
{
    As5600Config config;

    try {
        if (argc > 1) config.zero_offset_rad = std::stod(argv[1]) * std::numbers::pi / 180.0;
        if (argc > 2) config.direction_sign = std::stoi(argv[2]);
        if (argc > 3) config.i2c_device = argv[3];
    }
    catch (const std::exception&) {
        std::fprintf(stderr, "usage: %s [zero_offset_deg [direction_sign [i2c_device]]]\n", argv[0]);
        return 2;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    try {
        As5600 sensor(config);

        std::printf("AS5600 on %s, address 0x%02X, zero offset %.2f deg, direction %+d\n",
                    config.i2c_device.c_str(), config.i2c_address,
                    toDegrees(config.zero_offset_rad), config.direction_sign);
        std::printf("Ctrl-C to quit.\n");

        while (!g_stop) {
            try {
                const uint16_t raw = sensor.readRawAngle();
                const double sensor_deg = raw * 360.0 / As5600::COUNTS_PER_REV;
                const double angle_deg = toDegrees(As5600::rawToAngleRad(raw, config));
                const As5600Status status = sensor.readStatus();

                std::printf("raw=%4u  sensor_deg=%7.2f  angle_deg=%+7.2f  magnet: %s\n",
                            raw, sensor_deg, angle_deg, magnetText(status));
            }
            catch (const std::exception& e) {
                // A single failed transfer is not fatal on the bench; keep going.
                std::fprintf(stderr, "%s\n", e.what());
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    catch (const std::exception& e) {
        std::fprintf(stderr, "%s\n", e.what());
        return 1;
    }

    return 0;
}
