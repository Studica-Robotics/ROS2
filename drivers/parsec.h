#ifndef PARSEC_H
#define PARSEC_H

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

struct VMXPi;

/**
 * Host-side driver for Parsec ToF sensor over CAN.
 * CAN ID layout matches firmware: address = (DEVICE_TYPE + MANID + canID) + OFFSET * index.
 * Use GetAddress(INDEX) to get the CAN ID for a given API index.
 */
namespace studica_driver {

// Match Parsec firmware: fdcan_cmd.h (OFFSET*index: 0x40*16=0x400, so GETCONFIG id0=0x06090400)
#define PARSEC_DEVICE_TYPE  0x06000000u
#define PARSEC_MANID        0x00090000u
#define PARSEC_OFFSET       0x40u
#define PARSEC_BASE         (PARSEC_DEVICE_TYPE + PARSEC_MANID)  // 0x06090000

// API indices (GetAddress(index)); GETCONFIG=16 => BASE+0x400+canID e.g. 0x06090400 for canID 0
#define PARSEC_META         0
#define PARSEC_DATA0        1
#define PARSEC_DATA1        2
#define PARSEC_DATA2        3
#define PARSEC_DATA3        4
#define PARSEC_GETCONFIG    16
#define PARSEC_CMD_RESP     17

// GETCONFIG response layout:
// - CAN FD: CMD_RESP can be up to 44 bytes (full config with API IDs and zone mask).
// - CAN2 (Classic): CMD_RESP is 8 bytes only: [0]=version, [1]=fdist_en, [2]=can_fd_enabled,
//   [3]=can_brs_status, [4]=CAN_ID, [5..7]=reserved. Use fixed PARSEC_* indices for addresses.
// Full 44-byte layout (when len>=44): [8..11] META, [12..15] DATA0, ... [32..35] CMD_RESP, [36..43] zone mask.

// DATA frames:
// - CAN FD: DATA0..DATA3, each [0]=seq, [1]=zones, [2..]=int16 LE (16 zones per chunk).
// - CAN2: only DATA0 is used; multiple 8-byte frames: [0]=seq, [1]=block_idx, [2..7]=3×int16 LE.
//   Reassemble with ReadDataStreamCAN2().

class Parsec {
public:
    /**
     * @param canID Device CAN ID (0..63), must match device's CANID setting.
     * @param vmx    VMXPi instance (e.g. std::make_shared<VMXPi>(true, 50)).
     */
    Parsec(uint8_t canID, std::shared_ptr<VMXPi> vmx);
    ~Parsec();

    /** CAN address for this device: (PARSEC_BASE + canID_) + PARSEC_OFFSET * index */
    uint32_t GetAddress(uint32_t index) const;

    /** CAN address for any canID (for scanning). address = PARSEC_BASE + canID + PARSEC_OFFSET * index */
    static uint32_t GetAddressForCanID(uint8_t canID, uint32_t index);

    /** Send GETCONFIG request; response arrives on CMD_RESP. Returns true if send ok. */
    bool RequestConfig();
    /**
     * Read GETCONFIG response from blackboard (CMD_RESP).
     * configOut at least 8 bytes (classic CAN); 44 bytes for full config (CAN FD). Returns true if entry found.
     */
    bool GetConfigResponse(uint8_t* configOut, unsigned maxLen = 64);

    /** Read one DATA frame (DATA0..DATA3) from blackboard. dataOut at least 64 bytes. Returns payload length or 0. */
    int ReadDataChunk(uint32_t dataIndex, uint8_t* dataOut, unsigned maxLen = 64);

    /**
     * CAN2 only: reassemble distance data from DATA0 stream (multiple 8-byte frames with block_idx).
     * Call PullCANData() is done inside. fdist[] filled with int16 LE; -1 = invalid/missing.
     * @param seqOut    optional; set to frame sequence number
     * @param zonesOut  optional; set to number of zones (from received blocks)
     * @param fdist     output array, at least maxZones elements
     * @param maxZones  typically 16 or 64
     * @return number of zones filled (0 if no DATA0 frames), or -1 on error
     */
    int ReadDataStreamCAN2(uint8_t* seqOut, uint8_t* zonesOut, int16_t* fdist, int maxZones = 64);

    /** Parse CAN ID from config response bytes [8..35] (7 x uint32 LE). */
    static void ParseConfigApiIds(const uint8_t* config, uint32_t* meta, uint32_t* data0, uint32_t* data1,
                                  uint32_t* data2, uint32_t* data3, uint32_t* getconfig, uint32_t* cmdResp);
    /** Parse zone mask from config bytes [36..43] (uint64 LE). */
    static uint64_t ParseConfigZoneMask(const uint8_t* config);

    uint8_t GetCanID() const { return canID_; }

    /** Raw Write: send one CAN frame to given address (8 bytes for classic). */
    bool Write(uint32_t address, const uint8_t* data, size_t len, int32_t periodMS = 0);
    /** Pull received CAN frames from HW into receive stream/blackboard (call before batch ReadCached). */
    void PullCANData();
    /** Raw Read: pull CAN data then get latest frame for address from blackboard. Returns bytes copied, 0 on failure. */
    int Read(uint32_t address, uint8_t* data, size_t maxLen);
    /** Read from blackboard only (no PullCANData). Use after PullCANData() for batch scan to avoid VMX warnings. */
    int ReadCached(uint32_t address, uint8_t* data, size_t maxLen);

    /** Diagnostic: read up to maxMessages from receive stream and print ID + data. Use when no device found. */
    void DumpReceiveStream(unsigned maxMessages = 32);

    /** VMX CAN channel (0 or 1). Set env PARSEC_CAN_CHANNEL=1 to use CAN1. */
    static unsigned GetCANChannel();

private:
    std::shared_ptr<VMXPi> vmx_;
    uint8_t canID_;
    unsigned int canrxhandle_{0};  // VMXCANReceiveStreamHandle
};

} // namespace studica_driver

#endif // PARSEC_H
