#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace studica_driver {

/**
 * Host driver for Colore over USB CDC serial (Pi /dev/ttyACM0).
 * Parses firmware text lines: "SRGB: r,g,b", "XYZ: x,y,z", "HEX: #RRGGBB",
 * "RGB: fz,fy,fxl".
 * Firmware stays silent until the first command, then streams per SAMPLETIME.
 *
 * Commands are sent with no terminator, one USB packet each. The firmware
 * answers "<command> OK", "ERR usage: ..." or "Unknown command: ...". Settings
 * only change RAM; the driver never sends EESAVE or CANID.
 */
class ColoreUsb {
public:
    struct Sample {
        uint32_t seq{0};
        bool has_srgb{false};
        uint8_t r{0}, g{0}, b{0};
        bool has_hex{false};
        std::string hex;
        bool has_xyz{false};
        float x{0.f}, y{0.f}, z{0.f};
        bool has_tristim{false};
        uint16_t fz{0}, fy{0}, fxl{0};
    };

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

    // Constructor opens the port and starts the reader thread
    explicit ColoreUsb(const std::string& port, int baud = 115200);
    ~ColoreUsb();

    // Delete copy constructor and assignment operator, enforcing single ownership of the USB handle
    ColoreUsb(const ColoreUsb&) = delete;
    ColoreUsb& operator=(const ColoreUsb&) = delete;

    bool IsOpen() const { return fd_ >= 0; }

    /** Send one text command (e.g. "COLORFORMAT,XYZ") as its own USB packet, no terminator. */
    bool SendCommand(const std::string& cmd);

    /** Set the firmware output format (COLORFORMAT) + sample interval and wake
     *  streaming. color_format selects what the firmware emits ("xyz", "srgb",
     *  "hex", "rgb", "raw"); the ROS component always uses "xyz" and derives the
     *  rest host-side. Call once after open. */
    Result ConfigureStreaming(const std::string& color_format, int sample_ms);

    /*
     * Settings. Each waits for the sensor's reply.
     */
    Result SetSampleTime(int ms);
    /** White LED, 0-100 %. */
    Result SetBrightness(int percent);
    Result SetStatusLed(bool enable, int percent);
    Result SetMeasureModeOff();
    /** Fixed mode: the target is z_mm millimetres away. */
    Result SetMeasureModeFixed(float z_mm);

    /** Send GETCONFIG and collect the multi-line response (joined). */
    bool RequestConfig(std::string* response_out);

    /** Copy the latest parsed sample. Returns true if any frame seen yet. */
    bool ReadLatest(Sample* out);

    static const char* ResultString(Result result);

private:
    enum class Reply { None, Ok, Error, Unknown };

    int fd_{-1};
    std::string port_;
    std::thread reader_;
    std::atomic<bool> running_{false};
    std::mutex write_mutex_;

    std::mutex sample_mutex_;
    Sample latest_{};
    bool has_sample_{false};
    uint32_t seq_counter_{0};

    std::string line_buf_;

    // Replies and GETCONFIG lines, read by the reader thread.
    std::mutex reply_mutex_;
    std::condition_variable reply_cv_;
    std::mutex command_mutex_;  // one command awaiting a reply at a time
    std::string awaiting_;      // command awaiting a reply; empty for none
    Reply reply_{Reply::None};
    std::string config_acc_;    // GETCONFIG lines so far
    std::string config_done_;   // last complete GETCONFIG reply
    uint64_t config_count_{0};

    void readerLoop();
    void startReader();
    void stopReader();
    void parseLine(const std::string& line);
    bool onTextLine(const std::string& line);
    Result command(const std::string& cmd,
                   std::chrono::milliseconds timeout = std::chrono::milliseconds(300));
};

}  // namespace studica_driver
