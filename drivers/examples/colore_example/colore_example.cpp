/*
 * colore_example.cpp
 *
 * Standalone Colore color-sensor demo (no ROS). Configures the sensor and
 * prints CIE-XYZ color readings. This is the same data path the ROS2 component
 * uses; see colore_component.cpp for how XYZ becomes sRGB and color matches.
 *
 * Build (on VMX / Pi, after drivers are installed):
 *   cd ~/studica_ws/drivers/examples/colore_example
 *   make
 *
 * Run — CAN (default, needs sudo for VMX GPIO/CAN):
 *   sudo ./colore_example                    # default CAN ID 0
 *   sudo ./colore_example can 5              # CAN ID 5
 *
 * Run — USB (Pi USB port; dialout group or sudo if permission denied):
 *   ./colore_example usb                     # default /dev/ttyACM0
 *   ./colore_example usb /dev/ttyACM1        # other port (ls /dev/ttyACM*)
 *
 * Environment:
 *   COLORE_CAN_CHANNEL=1       Use VMX CAN1 instead of CAN0
 *   COLORE_CAN_BITRATE=500     Match the device bitrate (1000 default | 500 | 250)
 */

#include "colore.hpp"
#include "colore_usb.hpp"

#include "VMXPi.h"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

namespace {

enum class Mode { Usb, Can };

struct Options {
    Mode mode = Mode::Can;
    std::string serial_port = "/dev/ttyACM0";
    uint8_t can_id = 0;
    int rate_hz = 5;
    int iterations = 20;
};

void print_usage(const char* prog)
{
    printf("Usage:\n");
    printf("  %s                         CAN on ID 0\n", prog);
    printf("  %s can [can_id]            VMX CAN bus (default CAN ID 0)\n", prog);
    printf("  %s usb [port]              USB serial (default port /dev/ttyACM0)\n", prog);
    printf("\n");
    printf("Environment:\n");
    printf("  COLORE_CAN_CHANNEL=1       Use VMX CAN1 instead of CAN0\n");
    printf("  COLORE_CAN_BITRATE=500     Match the device bitrate (1000 | 500 | 250)\n");
}

Options parse_args(int argc, char* argv[])
{
    Options opts;
    if (argc <= 1) {
        return opts;
    }

    const std::string mode = argv[1];
    if (mode == "usb" || mode == "serial") {
        opts.mode = Mode::Usb;
        if (argc >= 3) {
            opts.serial_port = argv[2];
        }
        return opts;
    }

    if (mode == "can") {
        opts.mode = Mode::Can;
        if (argc >= 3) {
            opts.can_id = static_cast<uint8_t>(std::atoi(argv[2]));
        }
        return opts;
    }

    print_usage(argv[0]);
    std::exit(1);
}

// Decode the 8-byte CAN telemetry frame: [seq_u16_le, x_i16, y_i16, z_i16], XYZ scaled x10000.
bool decode_xyz(const uint8_t* d, int n, uint16_t& seq, float& x, float& y, float& z)
{
    if (n < 8) return false;
    auto u16 = [](uint8_t lo, uint8_t hi) { return static_cast<uint16_t>(lo | (hi << 8)); };
    seq = u16(d[0], d[1]);
    x = static_cast<int16_t>(u16(d[2], d[3])) / 10000.0f;
    y = static_cast<int16_t>(u16(d[4], d[5])) / 10000.0f;
    z = static_cast<int16_t>(u16(d[6], d[7])) / 10000.0f;
    return true;
}

void run_usb(const Options& opts)
{
    studica_driver::ColoreUsb colore(opts.serial_port);
    if (!colore.IsOpen()) {
        printf("Failed to open %s (try: ls /dev/ttyACM*)\n", opts.serial_port.c_str());
        return;
    }

    // ROS uses "xyz"; the driver can also request "srgb"/"hex"/"rgb"/"raw".
    colore.ConfigureStreaming("xyz", 1000 / opts.rate_hz);

    std::string config;
    if (colore.RequestConfig(&config)) {
        printf("Colore USB config: %s\n", config.c_str());
    }

    for (int i = 0; i < opts.iterations; ++i) {
        studica_driver::ColoreUsb::Sample s;
        if (colore.ReadLatest(&s) && s.has_xyz) {
            printf("%3d | seq=%u  X=%.4f  Y=%.4f  Z=%.4f\n", i, s.seq, s.x, s.y, s.z);
        } else {
            printf("%3d | waiting for frame...\n", i);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / opts.rate_hz));
    }
}

void run_can(const Options& opts)
{
    std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50);
    if (!vmx || !vmx->IsOpen()) {
        printf("VMXPi failed to open (CAN unavailable). Run with sudo?\n");
        return;
    }

    studica_driver::Colore colore(opts.can_id, vmx);

    // Configure the sensor: XYZ output, sample interval from the rate, LED at 50%.
    colore.SetColorFormat(studica_driver::Colore::ColorFormat::XYZ);
    colore.SetSampleTimeMs(static_cast<uint16_t>(1000 / opts.rate_hz));
    colore.SetBrightness(50);

    for (int i = 0; i < opts.iterations; ++i) {
        uint8_t buf[8] = {0};
        const int n = colore.Read(COLORE_CAN_TELEM_XYZ, buf, sizeof(buf));

        uint16_t seq; float x, y, z;
        if (decode_xyz(buf, n, seq, x, y, z)) {
            printf("%3d | seq=%u  X=%.4f  Y=%.4f  Z=%.4f\n", i, seq, x, y, z);
        } else {
            printf("%3d | waiting for frame...\n", i);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / opts.rate_hz));
    }
}

}  // namespace

int main(int argc, char* argv[])
{
    const Options opts = parse_args(argc, argv);
    if (opts.mode == Mode::Usb) {
        printf("Colore USB example on %s at %d Hz\n", opts.serial_port.c_str(), opts.rate_hz);
        run_usb(opts);
    } else {
        printf("Colore CAN example on ID %u at %d Hz\n", opts.can_id, opts.rate_hz);
        run_can(opts);
    }
    return 0;
}
