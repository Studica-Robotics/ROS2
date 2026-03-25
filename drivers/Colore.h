#ifndef COLORE_H
#define COLORE_H

#include <cstdint>
#include <memory>

struct VMXPi;
class VMXCAN;

/**
 * Host-side driver for Colore (color sensor board) over CAN.
 *
 * Wire format: CAN 2.0B (29-bit extended ID), classic data frames only — not CAN FD.
 * Payloads for config commands/ACKs are at most 8 bytes (matches VMXCANMessage).
 *
 * Firmware: Core/Inc/fdcan_cmd.h (COLORE_CLASSIC_CAN_ONLY = 1: FD negotiation ignored)
 * - Receive filter window: 0x04090000 .. 0x040905C0 (same rule as Parsec DEVICE+MANID .. +0x5C0).
 * - Command extended ID (on wire today): COLORE_BASE + COLORE_CMD_MAILBOX + cmd_byte
 *   (cmd_byte = 0x12 .. 0x1A). The Colore constructor canID is reserved for future multi-drop;
 *   it does not change these IDs until firmware adds node addressing.
 * - Config ACK extended ID: COLORE_BASE + COLORE_RSP_MAILBOX + COLORE_RSP_CONFIG_ACK (0x05).
 * - ACK payload (Classic CAN, 6 bytes): [cmd_byte, status, value_u32_le]
 *
 * Note: COLORE_* below are preprocessor macros (not C++ namespace members).
 *       Use COLORE_CAN_STARTUP — never studica_driver::COLORE_CAN_STARTUP (that breaks the preprocessor).
 *
 * Host env: COLORE_CAN_BITRATE — 1000 (default) or 500 / 250 to match VMX and Colore FDCAN.
 * Host env: COLORE_CAN_CHANNEL — 0 or 1 (VMX CAN port); must match wiring (Parsec uses PARSEC_CAN_CHANNEL).
 * HAL: uses VMXPi::can; if your SDK has getCAN() (Studica/FRC), a second bus is tried when it differs from .can.
 * Build with -DCOLORE_VMX_KAUAI_ONLY if VMXPi has no getCAN() (Kauai-only headers).
 */
namespace studica_driver {

#define COLORE_DEVICE_TYPE  0x04000000u
#define COLORE_MANID        0x00090000u
#define COLORE_BASE         (COLORE_DEVICE_TYPE + COLORE_MANID)  // 0x04090000

/** Offset from (BASE+canID) to command region (matches CAN_CMD_BASE_ID in firmware for canID=0). */
#define COLORE_CMD_MAILBOX  0x500u
/** Offset to response region (matches CAN_RSP_BASE_ID in firmware for canID=0). */
#define COLORE_RSP_MAILBOX  0x580u

/*
 * "API index" for GetAddress: use the firmware command byte (same as CAN_CMD_*), not 0,1,2,...
 * Example: GetAddress(COLORE_CMD_SET_COLOR_FORMAT) -> 0x04090513 for canID 0.
 */
#define COLORE_CMD_SET_MEASMODE     0x12u
#define COLORE_CMD_SET_COLOR_FORMAT 0x13u
#define COLORE_CMD_SET_SAMPLE_TIME  0x14u
#define COLORE_CMD_SET_BRIGHTNESS   0x15u
#define COLORE_CMD_SET_USB_OVERRIDE 0x16u
#define COLORE_CMD_SET_STATUS_LED   0x17u
#define COLORE_CMD_SET_CAN_ID       0x18u
#define COLORE_CMD_SET_Z_MM         0x19u
#define COLORE_CMD_GET_CONFIG       0x1Au

#define COLORE_RSP_CONFIG_ACK       0x05u

#define COLORE_ACK_OK      0x00u
#define COLORE_ACK_ERR_LEN 0x01u
#define COLORE_ACK_ERR_ARG 0x02u
#define COLORE_ACK_ERR_EE  0x03u

/** Optional: other firmware IDs (same naming as fdcan_cmd.h). */
#define COLORE_CAN_STARTUP       (COLORE_BASE + 0x3C0u)
#define COLORE_CAN_STARTUP_RESP  (COLORE_BASE + 0x440u)
#define COLORE_CAN_TEST_ID       (COLORE_BASE + 0x500u)
#define COLORE_CAN_ECHO_ID       (COLORE_BASE + 0x504u)

/* Classic CAN telemetry: payload DLC=8, int16-scaled XYZ.
 * [0..1]=seq_u16_le, [2..3]=x_i16_le, [4..5]=y_i16_le, [6..7]=z_i16_le */
#define COLORE_CAN_TELEM_XYZ     (COLORE_BASE + 0x540u)

class Colore {
public:
    enum class ColorFormat : uint8_t {
        RGB  = 0,
        RAW  = 1,
        SRGB = 2,
        HEX  = 3,
        XYZ  = 4
    };

    /**
     * @param canID Reserved for future multi-drop (currently ignored for TX/RX extended IDs).
     * @param vmx   VMXPi instance (e.g. std::make_shared<VMXPi>(true, 50)); IsOpen() must be true for CAN.
     */
    Colore(uint8_t canID, std::shared_ptr<VMXPi> vmx);
    ~Colore();

    /** CAN extended ID: COLORE_BASE + COLORE_CMD_MAILBOX + cmdByte (see COLORE_CMD_*). */
    uint32_t GetAddress(uint32_t cmdByte) const;

    /** Same IDs as GetAddress; canID argument is ignored until firmware supports node offsets. */
    static uint32_t GetAddressForCanID(uint8_t canID, uint32_t cmdByte);

    /** CONFIG_ACK receive ID (fixed). */
    uint32_t GetConfigAckAddress() const;
    static uint32_t GetConfigAckAddressForCanID(uint8_t canID);

    bool SetMeasureModeOff(uint32_t* ackValue = nullptr);
    bool SetMeasureModeAuto(bool ambient, uint32_t* ackValue = nullptr);
    bool SetMeasureModeFixed(float zUserMm, bool ambient, uint32_t* ackValue = nullptr);

    bool SetColorFormat(ColorFormat fmt, uint32_t* ackValue = nullptr);
    bool SetSampleTimeMs(uint16_t ms, uint32_t* ackValue = nullptr);
    bool SetBrightness(uint8_t percent, uint32_t* ackValue = nullptr);
    bool SetUsbOverride(bool enable, uint32_t* ackValue = nullptr);
    bool SetStatusLed(bool enable, uint8_t brightness, uint32_t* ackValue = nullptr);
    bool SetCanId(uint32_t canId, uint32_t* ackValue = nullptr);
    bool SetZmm(float zUserMm, uint32_t* ackValue = nullptr);

    /** item: 1=colorformat,2=sampletime,3=measmode_flags,4=brightness,5=usb_override,6=status_led_pack,7=can_id */
    bool GetConfig(uint8_t item, uint32_t& valueOut);

    uint8_t GetCanID() const { return canID_; }

    /** False if VMX OpenReceiveStream failed (no ACKs possible until fixed). */
    bool IsCanReceiveStreamOpen() const { return canrxhandle_ != 0u; }

    /** True if RX stream accepts multiple IDs (0x0409xxxx family mask or catch-all); ACK matched in software. */
    bool IsWideCanReceiveFilter() const { return can_rx_multi_id_; }

    /** Raw write (Classic CAN: up to 8 bytes typical). */
    bool Write(uint32_t address, const uint8_t* data, size_t len, int32_t periodMS = 0);

    void PullCANData();

    /** Pull then read latest frame for address from blackboard. Returns bytes copied, 0 on failure. */
    int Read(uint32_t address, uint8_t* data, size_t maxLen);

    /** Read from blackboard only (no PullCANData). */
    int ReadCached(uint32_t address, uint8_t* data, size_t maxLen);

    void DumpReceiveStream(unsigned maxMessages = 32);

    /** VMX CAN channel (0 or 1). Set env COLORE_CAN_CHANNEL=1 to use CAN1. */
    static unsigned GetCANChannel();

private:
    bool SendCmdWithAck(uint8_t cmdByte,
                        const uint8_t* payload,
                        uint8_t len,
                        uint32_t* ackValue,
                        uint32_t timeoutMs = 500);

    bool WaitConfigAck(uint8_t expectedCmdByte,
                       uint32_t& valueOut,
                       uint8_t& statusOut,
                       uint32_t timeoutMs);

    VMXCAN& vmxCan();

    std::shared_ptr<VMXPi> vmx_;
    /** VMXCAN used for Send/Retrieve/streams (may be &vmx_->can or &vmx_->getCAN() if that opened the RX stream). */
    VMXCAN* vmx_can_iface_{nullptr};
    uint8_t canID_;
    unsigned int canrxhandle_{0};
    /** Same as Parsec: COLORE_BASE + 0x1FFF0000 or catch-all → many IDs per stream. */
    bool can_rx_multi_id_{false};
    /** VMX-pi CAN port 0 or 1 (must match RetrieveAllCANData, see Parsec PARSEC_CAN_CHANNEL). */
    unsigned can_channel_{0};
};

} // namespace studica_driver

#endif // COLORE_H
