/*
 * parsec_example.cpp
 *
 * Standalone Parsec ToF demo (no ROS). Reads 100 frames and prints a zone grid.
 *
 * Build (on VMX / Pi, after drivers are installed):
 *   cd ~/studica_ws/drivers/examples/parsec_example
 *   make
 *
 * Run — CAN (default, needs sudo for VMX GPIO/CAN):
 *   sudo ./parsec_example                    # default CAN ID 0 
 *   sudo ./parsec_example can 5              # CAN ID 5
 *
 * Run — USB (Pi USB port; dialout group or sudo if permission denied):
 *   ./parsec_example usb                     # default /dev/ttyACM0
 *   ./parsec_example usb /dev/ttyACM1        # other port (ls /dev/ttyACM*)
 *
 */

#include "parsec.hpp"
#include "parsec_usb.hpp"

#include "VMXPi.h"

#include <chrono>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

enum class Mode { Usb, Can };

struct Options {
    Mode mode = Mode::Can;
    std::string serial_port = "/dev/ttyACM0";
    uint8_t can_id = 0;
    int odr_hz = 15;
    int iterations = 100;
};

bool is_valid_mm(int16_t d) { return d >= 1 && d <= 4000; }

int16_t min_valid_mm(const int16_t* fdist, int count)
{
    int16_t min_d = INT16_MAX;
    bool found = false;
    for (int i = 0; i < count; ++i) {
        if (is_valid_mm(fdist[i]) && fdist[i] < min_d) {
            min_d = fdist[i];
            found = true;
        }
    }
    return found ? min_d : -1;
}

const char* zone_label(int16_t d)
{
    if (d == -1) {
        return "off";
    }
    if (d == -2) {
        return "invalid";
    }
    return nullptr;
}

void print_usage(const char* prog)
{
    printf("Usage:\n");
    printf("  %s                         CAN on ID 0\n", prog);
    printf("  %s can [can_id]            VMX CAN bus (default CAN ID 0)\n", prog);
    printf("  %s usb [port]              USB serial (default port /dev/ttyACM0)\n", prog);
    printf("\n");
    printf("Environment:\n");
    printf("  PARSEC_CAN_CHANNEL=1       Use VMX CAN1 instead of CAN0\n");
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

void print_grid(const int16_t* fdist, int zones)
{
    const int side = (zones == 64) ? 8 : 4;
    for (int row = 0; row < side; ++row) {
        printf("    ");
        for (int col = 0; col < side; ++col) {
            const int idx = row * side + col;
            const int16_t d = fdist[idx];
            const char* label = zone_label(d);
            if (label) {
                printf("%7s ", label);
            } else {
                printf("%4d mm ", d);
            }
        }
        printf("\n");
    }
}

void run_usb(const Options& opts)
{
    studica_driver::ParsecUsb parsec(opts.serial_port);
    if (!parsec.IsOpen()) {
        printf("Failed to open %s\n", opts.serial_port.c_str());
        return;
    }

    if (!parsec.ConfigureStreaming(opts.odr_hz)) {
        printf("Warning: streaming config failed on %s\n", opts.serial_port.c_str());
    }

    std::string config;
    if (parsec.RequestConfig(&config)) {
        printf("Parsec USB config: %s\n", config.c_str());
    } else {
        printf("Warning: GETCONFIG not received yet on %s\n", opts.serial_port.c_str());
    }

    std::vector<int16_t> fdist(studica_driver::ParsecUsb::kMaxZones, 0);
    for (int i = 0; i < opts.iterations; ++i) {
        uint8_t seq = 0;
        uint8_t zones = 0;
        const int n = parsec.ReadLatestFdist(&seq, &zones, fdist.data(),
                                             static_cast<int>(fdist.size()));
        if (n <= 0) {
            printf("%3d | waiting for frame...\n", i);
        } else {
            const int16_t min_mm = min_valid_mm(fdist.data(), n);
            const int center = n / 2;
            printf("%3d | seq=%u zones=%u min=%d mm center[%d]=%d mm\n",
                   i, seq, zones, min_mm, center, fdist[static_cast<size_t>(center)]);
            print_grid(fdist.data(), n);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / opts.odr_hz));
    }
}

void run_can(const Options& opts)
{
    std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50);
    studica_driver::Parsec parsec(opts.can_id, vmx);
    if (parsec.GetCanID() != opts.can_id) {
        printf("Failed to init Parsec on CAN ID %u\n", opts.can_id);
        return;
    }

    uint8_t config[64] = {0};
    if (parsec.RequestConfig()) {
        vmx->time.DelayMilliseconds(50);
        if (parsec.GetConfigResponse(config, sizeof(config)) && config[0] != 0) {
            printf("Parsec CAN config: version=%u fdist_en=%u can_id=%u\n",
                   config[0], config[1], config[4]);
        }
    }

    std::vector<int16_t> fdist(studica_driver::ParsecUsb::kMaxZones, 0);
    for (int i = 0; i < opts.iterations; ++i) {
        uint8_t seq = 0;
        uint8_t zones = 0;
        const int n = parsec.ReadDataStreamCAN2(&seq, &zones, fdist.data(),
                                                static_cast<int>(fdist.size()));
        if (n <= 0) {
            printf("%3d | waiting for frame...\n", i);
        } else {
            const int16_t min_mm = min_valid_mm(fdist.data(), n);
            const int center = n / 2;
            printf("%3d | seq=%u zones=%u min=%d mm center[%d]=%d mm\n",
                   i, seq, zones, min_mm, center, fdist[static_cast<size_t>(center)]);
            print_grid(fdist.data(), n);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / opts.odr_hz));
    }
}

}  // namespace

int main(int argc, char* argv[])
{
    const Options opts = parse_args(argc, argv);
    if (opts.mode == Mode::Usb) {
        printf("Parsec USB example on %s at %d Hz\n", opts.serial_port.c_str(), opts.odr_hz);
        run_usb(opts);
    } else {
        printf("Parsec CAN example on ID %u at %d Hz\n", opts.can_id, opts.odr_hz);
        run_can(opts);
    }
    return 0;
}
