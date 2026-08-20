/*
 * titan_example.cpp
 *
 * Standalone Titan motor demo (no ROS). Spins all four motors briefly and prints
 * encoder/RPM telemetry, over either the VMX CAN bus or USB CDC serial. 
 *
 * Build (on VMX / Pi, after drivers are installed):
 *   cd ~/studica_ws/drivers/examples/titan_example
 *   make
 *
 * Run — CAN (default, needs sudo for VMX GPIO/CAN):
 *   sudo ./titan_example                 # default CAN ID 45
 *   sudo ./titan_example can 42          # CAN ID 42
 *
 * Run — USB (Pi USB port; dialout group or sudo if permission denied):
 *   ./titan_example usb                  # default /dev/ttyACM0
 *   ./titan_example usb /dev/ttyACM1     # other port (ls /dev/ttyACM*)
 */

#include "titan.hpp"
#include "titan_usb.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

namespace {

enum class Mode { Usb, Can };

struct Options {
    Mode mode = Mode::Can;
    std::string serial_port = "/dev/ttyACM0";
    uint8_t can_id = 45;
    uint16_t motor_freq = 15600;
    double dist_per_tick = 0.0006830601;  // 1464 counts ~= 1 rotation for this gearbox/wheel
    int iterations = 100;                 // ~5 s at 50 ms per iteration
};

void print_usage(const char* prog)
{
    printf("Usage:\n");
    printf("  %s                         CAN on ID 45\n", prog);
    printf("  %s can [can_id]            VMX CAN bus (default CAN ID 45)\n", prog);
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

// Shared demo body. Transport-agnostic, drives whichever controller it is given.
void drive(studica_driver::ITitanController& titan, const Options& opts)
{
    // Let the device settle after construction/config, then read identity.
    std::this_thread::sleep_for(std::chrono::seconds(1));
    printf("Serial Number: %s\n", titan.GetSerialNumber().c_str());
    printf("%s\n", titan.GetFirmwareVersion().c_str());
    printf("%s\n", titan.GetHardwareVersion().c_str());

    for (uint8_t m = 0; m < 4; ++m) {
        titan.ConfigureEncoder(m, opts.dist_per_tick);
        titan.ResetEncoder(m);
    }

    // Inversion flags — uncomment per motor as needed:
    // titan.InvertMotorDirection(0);
    // titan.InvertEncoderDirection(0);
    // titan.InvertMotorRPM(0);

    titan.Enable(true);

    const double speeds[4] = {-0.2, 0.2, -0.2, 0.2};

    for (int i = 0; i < opts.iterations; ++i) {
        for (uint8_t m = 0; m < 4; ++m) {
            titan.SetSpeed(m, speeds[m]);
        }
        if (i % 20 == 0) {  // ~1 Hz telemetry
            for (uint8_t m = 0; m < 4; ++m) {
                printf("M%u Distance: %f, RPM: %.2f, Raw: %d\n",
                       m, titan.GetEncoderDistance(m),
                       static_cast<double>(titan.GetRPM(m)), titan.GetEncoderCount(m));
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    titan.Enable(false);  // stop the motors (and, over USB, disarm the command timeout)
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

void run_usb(const Options& opts)
{
    studica_driver::TitanUsb titan(opts.serial_port, opts.motor_freq);
    if (!titan.IsOpen()) {
        printf("Failed to open %s\n", opts.serial_port.c_str());
        return;
    }
    drive(titan, opts);
}

void run_can(const Options& opts)
{
    studica_driver::Titan titan(opts.can_id, opts.motor_freq, static_cast<float>(opts.dist_per_tick));
    drive(titan, opts);
}

}  // namespace

int main(int argc, char* argv[])
{
    const Options opts = parse_args(argc, argv);
    if (opts.mode == Mode::Usb) {
        printf("Titan USB example on %s\n", opts.serial_port.c_str());
        run_usb(opts);
    } else {
        printf("Titan CAN example on ID %u\n", opts.can_id);
        run_can(opts);
    }
    return 0;
}
