#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "navx3.hpp"

namespace studica_driver {

/* Navx3Usb command results besides NAVX3_OK and NAVX3_TIMEOUT. */
#define NAVX3_USB_REJECTED          1   // ERR,<command>,<reason>
#define NAVX3_USB_UNKNOWN_COMMAND   2   // firmware too old for this command
#define NAVX3_USB_NOT_OPEN          -2

/**
 * Host driver for the NavX3 over USB CDC serial (e.g. /dev/ttyACM0).
 *
 * The NavX3 is silent after power-up until it gets a command, then streams
 * ~100 Hz "DATA:" lines; the driver sends HELP on open and whenever it goes
 * quiet. Commands have no terminator: the firmware takes each USB packet as
 * one command. Every command is answered "OK,<NAME>" or "ERR,<NAME>,<reason>".
 */
class Navx3Usb {
public:
    /** Opens the port and starts the reader thread. Check IsOpen(). */
    explicit Navx3Usb(const std::string& port, int baud = 115200);
    ~Navx3Usb();

    Navx3Usb(const Navx3Usb&) = delete;
    Navx3Usb& operator=(const Navx3Usb&) = delete;

    bool IsOpen() const { return fd_ >= 0; }

    /**
     * Copies the latest sample. A group counts as valid if it arrived within `maxAge`.
     * @return false if no group is valid
     */
    bool Read(Navx3Data& out, std::chrono::milliseconds maxAge);

    /*
     * Commands. Each waits for the reply and returns NAVX3_OK, NAVX3_TIMEOUT,
     * NAVX3_USB_REJECTED, NAVX3_USB_UNKNOWN_COMMAND or NAVX3_USB_NOT_OPEN.
     * Settings only change RAM.
     */
    int ResetYaw();
    /** Output data rate, 1-200 Hz. */
    int SetOutputRate(uint16_t hz);
    /** 6 (gyro and accelerometer) or 9 (with magnetometer) axes for yaw. */
    int SetYawAxes(int axes);
    int GetFirmwareVersion(int& major, int& minor, int& patch);
    int GetSensorUuid(uint32_t uuid[3]);
    /** Runs the IMU self test (~3.7 s, no data meanwhile). results: 8 codes. */
    int SelfTest(int32_t results[8]);

private:
    enum class Reply { None, Ok, Error, Unknown };

    bool sendCommand(const std::string& cmd);
    int command(const std::string& cmd, std::chrono::milliseconds timeout);
    /** Sends `cmd` and waits until `have` is true for the info its reply carries. */
    template <typename Have>
    int requestInfo(const std::string& cmd, std::chrono::milliseconds timeout, Have have);
    void readerLoop();
    void onLine(const std::string& line);

    int fd_{-1};
    std::string port_;
    std::thread reader_;
    std::atomic<bool> running_{false};
    std::mutex write_mutex_;

    std::mutex data_mutex_;
    Navx3Data sample_;
    std::chrono::steady_clock::time_point stamps_[10] = {};
    std::chrono::steady_clock::time_point last_data_{};
    std::atomic<int64_t> quiet_until_ns_{0};  // self test: no wake-ups until then

    // Replies and information lines, read by the reader thread.
    std::mutex reply_mutex_;
    std::condition_variable reply_cv_;
    std::mutex command_mutex_;  // one command awaiting a reply at a time
    std::string awaiting_;      // command awaiting a reply; empty for none
    Reply reply_{Reply::None};
    uint64_t info_request_{0};
    uint64_t firmware_request_{0}, uuid_request_{0}, self_test_request_{0};
    int firmware_[3] = {0, 0, 0};
    uint32_t uuid_[3] = {0, 0, 0};
    int32_t self_test_[8] = {0};
};

}  // namespace studica_driver
