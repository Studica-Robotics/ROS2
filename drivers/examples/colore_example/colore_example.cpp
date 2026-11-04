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

void run_usb(const Options& opts)
{
    studica_driver::ColoreUsb colore(opts.serial_port);
    if (!colore.IsOpen()) {
        printf("Failed to open %s (try: ls /dev/ttyACM*)\n", opts.serial_port.c_str());
        return;
    }

    // ROS uses "xyz"; the driver can also request "srgb"/"hex"/"rgb"/"raw".
    const auto result = colore.ConfigureStreaming("xyz", 1000 / opts.rate_hz);
    if (result != studica_driver::ColoreUsb::Result::Ok) {
        printf("Warning: streaming config: %s\n", studica_driver::ColoreUsb::ResultString(result));
    }

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
    if (!colore.IsCanReceiveStreamOpen()) {
        printf("Failed to init Colore on CAN ID %u\n", opts.can_id);
        return;
    }

    // Configure the sensor: XYZ output, sample interval from the rate, LED at 50%.
    if (!colore.SetColorFormat(studica_driver::Colore::ColorFormat::XYZ)) {
        printf("Warning: no ACK from CAN ID %u (check the ID and firmware)\n", opts.can_id);
    }
    colore.SetSampleTimeMs(static_cast<uint16_t>(1000 / opts.rate_hz));
    colore.SetBrightness(50);

    uint32_t fw = 0;
    if (colore.GetConfig(COLORE_CONFIG_FIRMWARE_VERSION, fw)) {
        printf("Colore firmware %u.%u.%u\n", (fw >> 16) & 0xFF, (fw >> 8) & 0xFF, fw & 0xFF);
    }

    for (int i = 0; i < opts.iterations; ++i) {
        uint16_t seq; float x, y, z;
        if (colore.ReadXyz(seq, x, y, z, std::chrono::milliseconds(3000 / opts.rate_hz))) {
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
