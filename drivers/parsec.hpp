#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

struct VMXPi;

/**
 * Host-side driver for Parsec ToF sensor over CAN.
 * CAN address = BASE + (OFFSET * API index) + canID.
 * Use GetAddress(INDEX) to get the CAN ID for a given API index.
 */
namespace studica_driver {

#define PARSEC_BASE         0x060C0000u
#define PARSEC_OFFSET       0x40u

// API indices (GetAddress(index)); GETCONFIG=16 => BASE+0x400+canID e.g. 0x060C0400 for canID 0
#define PARSEC_META                 0
#define PARSEC_DATA0                1
#define PARSEC_DATA1                2
#define PARSEC_DATA2                3
#define PARSEC_DATA3                4
#define PARSEC_ACK                  5   // answers every SET_* command
#define PARSEC_GETCONFIG            16
#define PARSEC_CMD_RESP             17  // GETCONFIG reply
#define PARSEC_SET_OUTPUT_RATE      18  // [hz]
#define PARSEC_SET_RESOLUTION       19  // [16 | 64]
#define PARSEC_SET_FREQUENCY        20  // [hz]
#define PARSEC_SET_ZONE_MASK        21  // [mask u64 LE]

// ACK payload: [0]=command API index, [1]=status, [2..5]=value u32 LE
#define PARSEC_ACK_OK               0
#define PARSEC_ACK_BAD_LENGTH       1
#define PARSEC_ACK_BAD_ARGUMENT     2
#define PARSEC_ACK_EEPROM           3
#define PARSEC_ACK_DEVICE           4
/** SendCommand result when no ACK arrived. */
#define PARSEC_ACK_TIMEOUT          -1

// GETCONFIG reply (classic CAN, 8 bytes): [0]=version, [1]=fdist_en, [2]=can_fd_enabled,
// [3]=can_brs_status, [4]=CAN_ID, [5..7]=firmware major, minor, patch.

// DATA frames (classic CAN): a burst of 8-byte frames on DATA0, each
// [0]=seq, [1]=block, [2..7]=3 x int16 LE zones. Block bits 0-6 are the block
// index; bit 7 is set when the frame has 64 zones (else 16).
#define PARSEC_BLOCK_64_ZONES       0x80u

class Parsec {
public:
    struct Config {
        uint8_t version = 0;
        bool distance_enabled = false;
        bool can_fd = false;
        bool can_brs = false;
        uint8_t can_id = 0;
        uint8_t firmware[3] = {0, 0, 0};
    };

    /**
     * @param canID Device CAN ID (0..63), must match device's CANID setting.
     * @param vmx    VMXPi instance (e.g. std::make_shared<VMXPi>(true, 50)).
     */
    Parsec(uint8_t canID, std::shared_ptr<VMXPi> vmx);
    ~Parsec();

    /** CAN address for this device: PARSEC_BASE + PARSEC_OFFSET * index + canID */
    uint32_t GetAddress(uint32_t index) const;

    /** CAN address for any canID (for scanning). */
    static uint32_t GetAddressForCanID(uint8_t canID, uint32_t index);

    /** False if the CAN receive stream couldn't be opened. */
    bool IsOpen() const { return open_; }

    uint8_t GetCanID() const { return canID_; }

    /** Sends GETCONFIG and waits for the reply. Returns false if none arrived. */
    bool GetConfig(Config& out);

    /*
     * Settings. Each waits for the device's ACK and returns its status
     * (PARSEC_ACK_OK on success) or PARSEC_ACK_TIMEOUT. They only change RAM,
     * so a power cycle restores the sensor's saved values.
     */
    /** Frames per second sent, 1-60 (4x4 reaches 60, 8x8 15). */
    int SetOutputRate(uint8_t hz);
    /** 16 (4x4) or 64 (8x8). Restarts ranging at 15 Hz, so set rates afterwards. */
    int SetResolution(uint8_t zones);
    /** Ranging frequency, 1-60 Hz. */
    int SetRangingFrequency(uint8_t hz);
    /** Enabled zones, one bit each; masked zones read -1. */
    int SetZoneMask(uint64_t mask);

    /**
     * Reassembles distance frames from the DATA0 burst. Drains whatever has
     * arrived without waiting; a frame split across calls is kept until its
     * last block arrives (or dropped if it is over 100 ms old).
     * fdist[] is filled with int16 mm; -1 = masked or missing, -2 = no reading.
     * @param seqOut    optional; set to frame sequence number
     * @param zonesOut  optional; set to 16 or 64
     * @param fdist     output array, at least maxZones elements
     * @param maxZones  typically 64
     * @return zones filled if a new frame completed since the last call, 0 if
     *         none did, or -1 on error
     */
    int ReadDataStreamCAN2(uint8_t* seqOut, uint8_t* zonesOut, int16_t* fdist, int maxZones = 64);

    /** Raw Write: send one CAN frame of up to 8 bytes to the given address. */
    bool Write(uint32_t address, const uint8_t* data, size_t len, int32_t periodMS = 0);
    /** Read latest frame for address from blackboard. Returns bytes copied, 0 on failure. */
    int Read(uint32_t address, uint8_t* data, size_t maxLen);

    /** Diagnostic: read up to maxMessages from receive stream and print ID + data. Use when no device found. */
    void DumpReceiveStream(unsigned maxMessages = 32);

private:
    /** Like Read(), but only if the frame hasn't been read before. */
    int ReadNew(uint32_t address, uint8_t* data, size_t maxLen);
    /** Sends a SET_* command and waits for its ACK. */
    int SendCommand(uint32_t index, const uint8_t* payload, size_t len, std::chrono::milliseconds timeout);
    /** Adds one DATA0 block to the frame being assembled. Returns true when it completes. */
    bool FeedBlock(const uint8_t* d, std::chrono::steady_clock::time_point now);

    std::shared_ptr<VMXPi> vmx_;
    uint8_t canID_;
    unsigned int canrxhandle_{0};  // VMXCANReceiveStreamHandle
    bool open_{false};

    // Frame being assembled from DATA0 blocks.
    bool partial_active_{false};
    uint8_t partial_seq_{0};
    uint8_t partial_zones_{0};
    uint32_t partial_blocks_{0};  // received blocks, one bit each
    std::chrono::steady_clock::time_point partial_start_{};
    int16_t partial_fdist_[64] = {0};
};

} // namespace studica_driver
