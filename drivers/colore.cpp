#include "colore.hpp"
#include "VMXPi.h"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <thread>

namespace studica_driver {

namespace {

int readBlackboard(VMXPi& vmx, unsigned int handle, uint32_t address,
                   uint8_t* data, size_t maxLen, bool& alreadyRetrieved)
{
    VMXCANTimestampedMessage msg;
    uint64_t sys_ts = 0u;
    const uint32_t mid = address & VMXCAN_29BIT_MESSAGE_ID_MASK;
    /* Some VMX builds key the blackboard with the extended-ID bit set. */
    if (!vmx.can.GetBlackboardEntry(handle, mid, msg, sys_ts, alreadyRetrieved, nullptr) &&
        !vmx.can.GetBlackboardEntry(handle, mid | 0x80000000u, msg, sys_ts, alreadyRetrieved, nullptr))
        return 0;
    uint8_t n = msg.dataSize;
    if (n > 8u) n = 8u;
    if (n > static_cast<uint8_t>(maxLen)) n = static_cast<uint8_t>(maxLen);
    msg.getData(data, n);
    return static_cast<int>(n);
}

} // namespace

Colore::Colore(uint8_t canID, std::shared_ptr<VMXPi> vmx)
    : vmx_(std::move(vmx)), canID_(canID)
{
    if (canID_ > 63u) {
        std::printf("Colore CAN ID %u is out of range (0..63).\n", canID_);
        std::fflush(stdout);
        return;
    }
    if (!vmx_ || !vmx_->IsOpen()) {
        std::printf("Colore: VMXPi not open.\n");
        std::fflush(stdout);
        return;
    }

    /* Filter: this device only (base and CAN ID; any API). No FIFO
     * flushes: sensors start after the Titan, and flushing TX could drop its
     * repeating enable frame. */
    VMXErrorCode vmxerr;
    if (!vmx_->can.OpenReceiveStream(canrxhandle_, COLORE_BASE + canID_, 0x1FFF003Fu, 100, &vmxerr)) {
        std::printf("Colore: Error opening CAN RX Stream (VMX error %d).\n", static_cast<int>(vmxerr));
        std::fflush(stdout);
        return;
    }
    if (!vmx_->can.EnableReceiveStreamBlackboard(canrxhandle_, true, &vmxerr))
        std::printf("Colore: Error enabling Blackboard.\n");
    if (!vmx_->can.SetMode(VMXCAN::VMXCAN_NORMAL, &vmxerr))
        std::printf("Colore: Error setting CAN mode to Normal.\n");

    can_rx_stream_open_ = true;
    std::printf("Colore driver started (CAN ID %u).\n", canID_);
    std::fflush(stdout);
}

Colore::~Colore()
{
    /* Parsec: do not CloseReceiveStream — VMX-pi can end up unable to receive on the next process start. */
}

uint32_t Colore::GetAddress(uint32_t index) const
{
    return GetAddressForCanID(canID_, index);
}

uint32_t Colore::GetAddressForCanID(uint8_t canID, uint32_t index)
{
    return COLORE_BASE + COLORE_OFFSET * index + canID;
}

bool Colore::Write(uint32_t address, const uint8_t* data, size_t len, int32_t periodMS)
{
    if (!can_rx_stream_open_ || len == 0u || len > 8u || data == nullptr)
        return false;

    VMXCANMessage msg;
    msg.messageID = address;
    msg.setData(data, static_cast<uint8_t>(len));
    return vmx_->can.SendMessage(msg, periodMS, nullptr);
}

int Colore::Read(uint32_t address, uint8_t* data, size_t maxLen)
{
    if (!can_rx_stream_open_ || data == nullptr || maxLen == 0u)
        return 0;
    bool already = false;
    return readBlackboard(*vmx_, canrxhandle_, address, data, maxLen, already);
}

int Colore::ReadNew(uint32_t address, uint8_t* data, size_t maxLen)
{
    if (!can_rx_stream_open_ || data == nullptr || maxLen == 0u)
        return 0;
    bool already = true;
    const int n = readBlackboard(*vmx_, canrxhandle_, address, data, maxLen, already);
    return already ? 0 : n;
}

void Colore::DumpReceiveStream(unsigned maxMessages)
{
    if (!can_rx_stream_open_)
        return;
    const unsigned cap = maxMessages > 32u ? 32u : maxMessages;
    VMXCANTimestampedMessage msgs[32];
    uint32_t read = 0u;
    (void)vmx_->can.ReadReceiveStream(canrxhandle_, msgs, cap, read, nullptr);
    for (uint32_t i = 0u; i < read; ++i) {
        std::printf("  RX id=0x%08X dlc=%u\n",
                    static_cast<unsigned>(msgs[i].messageID & VMXCAN_29BIT_MESSAGE_ID_MASK),
                    static_cast<unsigned>(msgs[i].dataSize));
    }
}

bool Colore::ReadXyz(uint16_t& seq, float& x, float& y, float& z, std::chrono::milliseconds maxAge)
{
    const auto now = std::chrono::steady_clock::now();
    uint8_t buf[8] = {0};
    if (ReadNew(GetAddress(COLORE_API_COLOR), buf, sizeof(buf)) == 8) {
        std::memcpy(last_xyz_, buf, sizeof(buf));
        last_xyz_time_ = now;
        have_xyz_ = true;
    }
    if (!have_xyz_ || now - last_xyz_time_ > maxAge)
        return false;

    auto u16 = [](const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); };
    seq = u16(&last_xyz_[0]);
    x = static_cast<int16_t>(u16(&last_xyz_[2])) / 10000.0f;
    y = static_cast<int16_t>(u16(&last_xyz_[4])) / 10000.0f;
    z = static_cast<int16_t>(u16(&last_xyz_[6])) / 10000.0f;
    return true;
}

bool Colore::SetMeasureModeOff(uint32_t* ackValue)
{
    uint8_t p[1] = {0u};
    return SendCmdWithAck(COLORE_CMD_SET_MEASMODE, p, 1u, ackValue);
}

bool Colore::SetMeasureModeFixed(float zUserMm, uint32_t* ackValue)
{
    uint8_t p[6] = {2u, 0u, 0u, 0u, 0u, 0u};
    std::memcpy(&p[2], &zUserMm, sizeof(float));
    return SendCmdWithAck(COLORE_CMD_SET_MEASMODE, p, 6u, ackValue);
}

bool Colore::SetColorFormat(ColorFormat fmt, uint32_t* ackValue)
{
    uint8_t p[1] = {static_cast<uint8_t>(fmt)};
    return SendCmdWithAck(COLORE_CMD_SET_COLOR_FORMAT, p, 1u, ackValue);
}

bool Colore::SetSampleTimeMs(uint16_t ms, uint32_t* ackValue)
{
    uint8_t p[2] = {
        static_cast<uint8_t>(ms & 0xFFu),
        static_cast<uint8_t>((ms >> 8) & 0xFFu)
    };
    return SendCmdWithAck(COLORE_CMD_SET_SAMPLE_TIME, p, 2u, ackValue);
}

bool Colore::SetBrightness(uint8_t percent, uint32_t* ackValue)
{
    uint8_t p[1] = {percent};
    return SendCmdWithAck(COLORE_CMD_SET_BRIGHTNESS, p, 1u, ackValue);
}

bool Colore::SetUsbOverride(bool enable, uint32_t* ackValue)
{
    uint8_t p[1] = {static_cast<uint8_t>(enable ? 1u : 0u)};
    return SendCmdWithAck(COLORE_CMD_SET_USB_OVERRIDE, p, 1u, ackValue);
}

bool Colore::SetStatusLed(bool enable, uint8_t brightness, uint32_t* ackValue)
{
    uint8_t p[2] = {static_cast<uint8_t>(enable ? 1u : 0u), brightness};
    return SendCmdWithAck(COLORE_CMD_SET_STATUS_LED, p, 2u, ackValue);
}

bool Colore::SetZmm(float zUserMm, uint32_t* ackValue)
{
    uint8_t p[4] = {0u, 0u, 0u, 0u};
    std::memcpy(p, &zUserMm, sizeof(float));
    return SendCmdWithAck(COLORE_CMD_SET_Z_MM, p, 4u, ackValue);
}

bool Colore::GetConfig(uint8_t item, uint32_t& valueOut)
{
    uint8_t p[1] = {item};
    return SendCmdWithAck(COLORE_CMD_GET_CONFIG, p, 1u, &valueOut);
}

bool Colore::SendCmdWithAck(uint8_t cmdByte,
                            const uint8_t* payload,
                            uint8_t len,
                            uint32_t* ackValue,
                            uint32_t timeoutMs)
{
    const uint32_t ackId = GetAddress(COLORE_API_CONFIG_ACK);
    uint8_t ack[8] = {0};
    ReadNew(ackId, ack, sizeof(ack));  // drop a stale ACK so it isn't taken for the answer

    if (!Write(GetAddress(cmdByte), payload, len, 0)) return false;

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    while (std::chrono::steady_clock::now() < deadline) {
        if (ReadNew(ackId, ack, sizeof(ack)) >= 6 && ack[0] == cmdByte) {
            if (ackValue) {
                *ackValue = static_cast<uint32_t>(ack[2])
                          | (static_cast<uint32_t>(ack[3]) << 8)
                          | (static_cast<uint32_t>(ack[4]) << 16)
                          | (static_cast<uint32_t>(ack[5]) << 24);
            }
            return ack[1] == COLORE_ACK_OK;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
}

} // namespace studica_driver
