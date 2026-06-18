#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace studica_driver {

/**
 * Host driver for Parsec over USB CDC serial (Pi /dev/ttyACM0).
 * Parses firmware binary distance frames (magic 0x5AA5, type=1, fdist[]).
 */
class ParsecUsb {
public:
    static constexpr int kMaxZones = 64;

    /** @param port e.g. /dev/ttyACM0 on Linux */
    explicit ParsecUsb(const std::string& port, int baud = 115200);
    ~ParsecUsb();

    ParsecUsb(const ParsecUsb&) = delete;
    ParsecUsb& operator=(const ParsecUsb&) = delete;

    bool IsOpen() const { return fd_ >= 0; }

    /** Send a text command (GETCONFIG, VERBOSE,0, …). Appends \\r\\n. */
    bool SendCommand(const std::string& cmd);

    /** Wake device, set binary fdist-only output. Call once after open. */
    bool ConfigureStreaming(int odr_hz);

    /** Send GETCONFIG and collect the text response line. */
    bool RequestConfig(std::string* response_out);

    /**
     * Copy the latest parsed frame.
     * @return zone count (16 or 64), or 0 if no frame yet.
     */
    int ReadLatestFdist(uint8_t* seq_out, uint8_t* zones_out, int16_t* fdist, int max_zones);

private:
    static constexpr uint16_t kFrameMagic = 0x5AA5;

#pragma pack(push, 1)
    struct FrameHdr {
        uint16_t magic;
        uint16_t length;
        uint8_t type;
        uint8_t streamcount;
        uint8_t zones;
        uint8_t flags;
    };
#pragma pack(pop)

    int fd_{-1};
    std::string port_;
    std::thread reader_;
    std::atomic<bool> running_{false};

    std::mutex frame_mutex_;
    std::array<int16_t, kMaxZones> latest_fdist_{};
    uint8_t latest_seq_{0};
    uint8_t latest_zones_{0};
    bool has_frame_{false};

    std::vector<uint8_t> rx_buf_;

    void readerLoop();
    void startReader();
    void stopReader();
    void consumeRxBuffer();
    bool tryParseFrame(size_t offset, size_t* frame_len_out);
    bool readLine(std::string* line_out, int timeout_ms);
};

}  // namespace studica_driver
