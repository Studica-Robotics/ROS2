#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace studica_driver {

/**
 * Host driver for Colore over USB CDC serial (Pi /dev/ttyACM0).
 * Parses firmware text lines: "SRGB: r,g,b", "XYZ: x,y,z", "HEX: #RRGGBB",
 * "RGB: fz,fy,fxl", optionally prefixed "DIST: <mm>  ".
 * Firmware stays silent until the first command, then streams per SAMPLETIME.
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
        bool has_dist{false};
        float dist_mm{-1.f};
    };

    // Constructor opens the port and starts the reader thread
    explicit ColoreUsb(const std::string& port, int baud = 115200);
    ~ColoreUsb();

    // Delete copy constructor and assignment operator, enforcing single ownership of the USB handle
    ColoreUsb(const ColoreUsb&) = delete;
    ColoreUsb& operator=(const ColoreUsb&) = delete;

    bool IsOpen() const { return fd_ >= 0; }

    /** Send a text command (e.g. "COLORFORMAT,XYZ"). Appends "\r\n". */
    bool SendCommand(const std::string& cmd);

    /** Set the firmware output format (COLORFORMAT) + sample interval and wake
     *  streaming. color_format selects what the firmware emits ("xyz", "srgb",
     *  "hex", "rgb", "raw"); the ROS component always uses "xyz" and derives the
     *  rest host-side. Call once after open. */
    bool ConfigureStreaming(const std::string& color_format, int sample_ms);

    /** Send GETCONFIG and collect the multi-line response (joined).
     *  Pauses the reader thread so the reply is not lost to the XYZ stream. */
    bool RequestConfig(std::string* response_out);

    /** Copy the latest parsed sample. Returns true if any frame seen yet. */
    bool ReadLatest(Sample* out);

private:
    int fd_{-1};
    std::string port_;
    std::thread reader_;
    std::atomic<bool> running_{false};

    std::mutex sample_mutex_;
    Sample latest_{};
    bool has_sample_{false};
    uint32_t seq_counter_{0};

    std::string line_buf_;

    void readerLoop();
    void startReader();
    void stopReader();
    void parseLine(const std::string& line);
    bool readLineBlocking(std::string* line_out, int timeout_ms);
};

}  // namespace studica_driver
