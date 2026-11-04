#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>

struct VMXPi;

/**
 * Host-side driver for Colore (color sensor board) over CAN.
 *
 * Wire format: CAN 2.0B (29-bit extended ID), classic data frames only.
 * CAN address = BASE + (OFFSET * API index) + canID.
 *
 * - Telemetry: API 1, [0..1]=seq_u16_le, [2..7]=X, Y, Z as int16 LE x 10000 (CIE XYZ).
 * - Commands: API 0x12 .. 0x1A, each answered on CONFIG_ACK (API 5):
 *   [0]=command API index, [1]=status, [2..5]=value_u32_le.
 *
 * Settings only change RAM; the driver never sends the CAN ID command (the
 * device saves that to EEPROM).
 *
 * Note: COLORE_* below are preprocessor macros (not C++ namespace members).
 */
namespace studica_driver {

#define COLORE_BASE         0x0D0C0000u
#define COLORE_OFFSET       0x40u

/* API indices for GetAddress(). */
#define COLORE_API_COLOR            0x01u
#define COLORE_API_CONFIG_ACK       0x05u
#define COLORE_CMD_SET_MEASMODE     0x12u
#define COLORE_CMD_SET_COLOR_FORMAT 0x13u
#define COLORE_CMD_SET_SAMPLE_TIME  0x14u
#define COLORE_CMD_SET_BRIGHTNESS   0x15u
#define COLORE_CMD_SET_USB_OVERRIDE 0x16u
#define COLORE_CMD_SET_STATUS_LED   0x17u
#define COLORE_CMD_SET_CAN_ID       0x18u  // never sent: the device saves it to EEPROM
#define COLORE_CMD_SET_Z_MM         0x19u
#define COLORE_CMD_GET_CONFIG       0x1Au

#define COLORE_ACK_OK      0x00u
#define COLORE_ACK_ERR_LEN 0x01u
#define COLORE_ACK_ERR_ARG 0x02u
#define COLORE_ACK_ERR_EE  0x03u

/* GetConfig() items. */
#define COLORE_CONFIG_COLOR_FORMAT     1
#define COLORE_CONFIG_SAMPLE_TIME      2
#define COLORE_CONFIG_MEASURE_MODE     3  // active | auto_z << 8
#define COLORE_CONFIG_BRIGHTNESS       4
#define COLORE_CONFIG_USB_OVERRIDE     5
#define COLORE_CONFIG_STATUS_LED       6  // enable | percent << 16
#define COLORE_CONFIG_CAN_ID           7
#define COLORE_CONFIG_FIRMWARE_VERSION 8  // major << 16 | minor << 8 | patch

class Colore {
public:
    // Values match the firmware COLORFORMAT codes
    enum class ColorFormat : uint8_t {
        RGB  = 0,
        RAW  = 1,
        SRGB = 2,
        HEX  = 3,
        XYZ  = 4
    };

    /**
     * @param canID Device CAN ID (0..63), must match the device's CANID setting.
     * @param vmx   VMXPi instance (e.g. std::make_shared<VMXPi>(true, 50)); IsOpen() must be true for CAN.
     */
    Colore(uint8_t canID, std::shared_ptr<VMXPi> vmx);
    ~Colore();

    /** CAN extended ID: COLORE_BASE + COLORE_OFFSET * index + canID. */
    uint32_t GetAddress(uint32_t index) const;
    static uint32_t GetAddressForCanID(uint8_t canID, uint32_t index);

    /** False if the CAN receive stream couldn't be opened (no data or ACKs possible). */
    bool IsCanReceiveStreamOpen() const { return can_rx_stream_open_; }

    uint8_t GetCanID() const { return canID_; }

    /**
     * Latest XYZ reading, if one arrived within `maxAge`.
     * @return false if no reading is that recent
     */
    bool ReadXyz(uint16_t& seq, float& x, float& y, float& z, std::chrono::milliseconds maxAge);

    /*
     * Each command waits for the device's ACK and returns true only on
     * COLORE_ACK_OK. ackValue, if given, receives the ACK's value.
     */
    bool SetMeasureModeOff(uint32_t* ackValue = nullptr);
    /** Fixed mode: the target is zUserMm millimetres away. */
    bool SetMeasureModeFixed(float zUserMm, uint32_t* ackValue = nullptr);

    bool SetColorFormat(ColorFormat fmt, uint32_t* ackValue = nullptr);
    bool SetSampleTimeMs(uint16_t ms, uint32_t* ackValue = nullptr);
    bool SetBrightness(uint8_t percent, uint32_t* ackValue = nullptr);
    bool SetUsbOverride(bool enable, uint32_t* ackValue = nullptr);
    bool SetStatusLed(bool enable, uint8_t brightness, uint32_t* ackValue = nullptr);
    bool SetZmm(float zUserMm, uint32_t* ackValue = nullptr);

    /** Reads one COLORE_CONFIG_* item. */
    bool GetConfig(uint8_t item, uint32_t& valueOut);

    /** Raw write (Classic CAN: up to 8 bytes). */
    bool Write(uint32_t address, const uint8_t* data, size_t len, int32_t periodMS = 0);

    /** Read latest frame for address from blackboard. Returns bytes copied, 0 on failure. */
    int Read(uint32_t address, uint8_t* data, size_t maxLen);

    void DumpReceiveStream(unsigned maxMessages = 32);

private:
    /** Like Read(), but only if the frame hasn't been read before. */
    int ReadNew(uint32_t address, uint8_t* data, size_t maxLen);

    bool SendCmdWithAck(uint8_t cmdByte,
                        const uint8_t* payload,
                        uint8_t len,
                        uint32_t* ackValue,
                        uint32_t timeoutMs = 200);

    std::shared_ptr<VMXPi> vmx_;
    uint8_t canID_;
    unsigned int canrxhandle_{0};
    bool can_rx_stream_open_{false};

    // Latest XYZ reading and when it arrived.
    bool have_xyz_{false};
    uint8_t last_xyz_[8] = {0};
    std::chrono::steady_clock::time_point last_xyz_time_{};
};

} // namespace studica_driver
