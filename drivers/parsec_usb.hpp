#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace studica_driver {

/**
 * Host driver for Parsec over USB CDC serial (Pi /dev/ttyACM0).
 * Parses firmware binary distance frames (magic A5 5A, type=1, fdist[]); the
 * text between frames carries command replies and the GETCONFIG line.
 *
 * Commands are sent with no terminator: the firmware takes each USB packet as
 * one command. The sensor is silent after power-up until it gets a command.
 */
class ParsecUsb {
public:
    static constexpr int kMaxZones = 64;

    enum class Result {
        Ok,
        /** The sensor rejected the command or its value. */
        Rejected,
        /** The sensor doesn't know the command (older firmware). */
        Unsupported,
        /** No reply in time. */
        Timeout,
        /** Port not open, or the write failed. */
        NotOpen,
    };

    /** @param port e.g. /dev/ttyACM0 on Linux */
    explicit ParsecUsb(const std::string& port, int baud = 115200);
    ~ParsecUsb();

    ParsecUsb(const ParsecUsb&) = delete;
    ParsecUsb& operator=(const ParsecUsb&) = delete;

    bool IsOpen() const { return fd_ >= 0; }

    /** Send one text command (GETCONFIG, VERBOSE,0, …) as its own USB packet, no terminator. */
    bool SendCommand(const std::string& cmd);

    /** Wake the device, select compact binary output and start reading. Call once after open. */
    bool ConfigureStreaming();

    /** Send GETCONFIG and wait for the config line. */
    bool RequestConfig(std::string* response_out);

    /*
     * Settings. Each waits for the sensor's reply. They only change RAM, so a
     * power cycle restores the sensor's saved values.
     */
    /** Frames per second sent, 1-60 (4x4 reaches 60, 8x8 15). Also sets the ranging frequency. */
    Result SetOutputRate(int hz);
    /** 16 (4x4) or 64 (8x8). Restarts ranging at 15 Hz, so set rates afterwards. */
    Result SetResolution(int zones);
    /** Ranging frequency, 1-60 Hz. */
    Result SetRangingFrequency(int hz);
    /** Enabled zones, one bit each; masked zones read -1. */
    Result SetZoneMask(uint64_t mask);

    /**
     * Copy the latest parsed frame.
     * @return zone count (16 or 64), or 0 if no frame yet.
     */
    int ReadLatestFdist(uint8_t* seq_out, uint8_t* zones_out, int16_t* fdist, int max_zones);

    static const char* ResultString(Result result);

private:
    enum class Reply { None, Ok, Error, Unknown };

    int fd_{-1};
    std::string port_;
    std::thread reader_;
    std::atomic<bool> running_{false};
    std::mutex write_mutex_;

    std::mutex frame_mutex_;
    std::array<int16_t, kMaxZones> latest_fdist_{};
    uint8_t latest_seq_{0};
    uint8_t latest_zones_{0};
    bool has_frame_{false};

    std::vector<uint8_t> rx_buf_;
    std::string text_line_;

    // Replies and the GETCONFIG line, read by the reader thread.
    std::mutex reply_mutex_;
    std::condition_variable reply_cv_;
    std::mutex command_mutex_;  // one command awaiting a reply at a time
    std::string awaiting_;      // command awaiting a reply; empty for none
    Reply reply_{Reply::None};
    std::string config_line_;
    uint64_t config_count_{0};

    void readerLoop();
    void startReader();
    void stopReader();
    void consumeRxBuffer();
    void onTextByte(char c);
    void onLine(const std::string& line);
    Result command(const std::string& cmd, std::chrono::milliseconds timeout);
    static Reply classifyReply(const std::string& line);
    static bool isReplyTo(const std::string& cmd, const std::string& line);
};

}  // namespace studica_driver
