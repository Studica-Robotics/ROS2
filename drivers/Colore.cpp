#include "Colore.h"
#include "VMXPi.h"
#include "VMXCAN.h"
#include "VMXErrors.h"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <thread>

namespace studica_driver {

namespace {

unsigned ReadEnvChannel(const char* name, unsigned defv)
{
    const char* s = std::getenv(name);
    if (!s || !*s) return defv;
    unsigned v = static_cast<unsigned>(std::atoi(s));
    return v;
}

/** Firmware today uses fixed extended IDs (no per-node offset on the wire). */
static uint32_t wire_cmd_id(uint32_t cmdByte)
{
    return COLORE_BASE + COLORE_CMD_MAILBOX + cmdByte;
}

static uint32_t wire_ack_id(void)
{
    return COLORE_BASE + COLORE_RSP_MAILBOX + COLORE_RSP_CONFIG_ACK;
}

/** Colore FDCAN nominal rate is ~1 Mbps (see Core/Src/fdcan.c + SystemClock_Config). VMX often defaults to 500k — must match. */
static VMXCAN::CANBusBitrate vmx_bitrate_from_env()
{
    const char* s = std::getenv("COLORE_CAN_BITRATE");
    int kbps = 1000;
    if (s && *s) kbps = std::atoi(s);
    if (kbps <= 250) return VMXCAN::CAN_BUS_BITRATE_250KBPS;
    if (kbps <= 500) return VMXCAN::CAN_BUS_BITRATE_500KBPS;
    return VMXCAN::CAN_BUS_BITRATE_1MBPS;
}

static void colore_log_can_init(const char* msg, int ec)
{
    std::fprintf(stderr, "%s VMXErrorCode=%d\n", msg, ec);
    std::fprintf(stdout, "%s VMXErrorCode=%d\n", msg, ec);
    std::fflush(stderr);
    std::fflush(stdout);
}

/** Try family / exact-ACK / catch-all filters; sweep maxMessages (VMX 4.x is picky). */
static bool colore_open_rx_stream(VMXCAN& bus,
                                  uint32_t baseId,
                                  uint32_t ackId,
                                  bool& multi_id,
                                  unsigned int& handle_out)
{
    static const unsigned kBufSizes[] = { 100u, 128u, 64u, 32u };
    VMXErrorCode ec = static_cast<VMXErrorCode>(0);
    for (unsigned bi = 0u; bi < sizeof(kBufSizes) / sizeof(kBufSizes[0]); ++bi) {
        const unsigned nbuf = kBufSizes[bi];
        VMXCANReceiveStreamHandle h = 0;
        ec = static_cast<VMXErrorCode>(0);
        if (bus.OpenReceiveStream(h, baseId, 0x1FFF0000u, nbuf, &ec)) {
            multi_id = true;
            handle_out = static_cast<unsigned int>(h);
            (void)bus.EnableReceiveStreamBlackboard(h, true, nullptr);
            return true;
        }
        h = 0;
        ec = static_cast<VMXErrorCode>(0);
        if (bus.OpenReceiveStream(h, ackId, VMXCAN_29BIT_MESSAGE_ID_MASK, nbuf, &ec)) {
            multi_id = false;
            handle_out = static_cast<unsigned int>(h);
            (void)bus.EnableReceiveStreamBlackboard(h, true, nullptr);
            return true;
        }
        h = 0;
        ec = static_cast<VMXErrorCode>(0);
        if (bus.OpenReceiveStream(h, 0u, 0u, nbuf, &ec)) {
            multi_id = true;
            handle_out = static_cast<unsigned int>(h);
            (void)bus.EnableReceiveStreamBlackboard(h, true, nullptr);
            return true;
        }
    }
    colore_log_can_init("Colore: OpenReceiveStream failed (all filters, buffer sizes 100/128/64/32), last", (int)ec);
    return false;
}

static void colore_post_rx_stream_setup(VMXCAN& bus, VMXCAN::CANBusBitrate bitrate)
{
    (void)bus.FlushRxFIFO(nullptr);
    (void)bus.FlushTxFIFO(nullptr);
    (void)bus.ResetBusBitrate(bitrate, nullptr);
    (void)bus.SetMode(VMXCAN::VMXCAN_NORMAL, nullptr);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

} // namespace

VMXCAN& Colore::vmxCan()
{
    if (vmx_ && vmx_can_iface_)
        return *vmx_can_iface_;
    return vmx_->can;
}

Colore::Colore(uint8_t canID, std::shared_ptr<VMXPi> vmx)
    : vmx_(std::move(vmx)), canID_(canID)
{
    (void)canID_;
    if (!vmx_ || !vmx_->IsOpen())
        return;

    vmx_can_iface_ = &vmx_->can;
    can_channel_ = ReadEnvChannel("COLORE_CAN_CHANNEL", 0u);
    if (can_channel_ > 1u)
        can_channel_ = 0u;

    const uint32_t ackId = wire_ack_id() & VMXCAN_29BIT_MESSAGE_ID_MASK;
    const uint32_t baseId = (COLORE_BASE)&VMXCAN_29BIT_MESSAGE_ID_MASK;
    const VMXCAN::CANBusBitrate bitrate = vmx_bitrate_from_env();

    VMXCAN& c0 = vmx_->can;
    VMXCAN* c1 = nullptr;
#ifndef COLORE_VMX_KAUAI_ONLY
    {
        VMXCAN& g = vmx_->getCAN();
        if (reinterpret_cast<uintptr_t>(&g) != reinterpret_cast<uintptr_t>(&c0))
            c1 = &g;
    }
#endif

    auto try_parsec_order = [this, baseId, ackId, bitrate](VMXCAN& bus, const char* tag) -> bool {
        canrxhandle_ = 0u;
        if (!colore_open_rx_stream(bus, baseId, ackId, can_rx_multi_id_, canrxhandle_))
            return false;
        vmx_can_iface_ = &bus;
        std::fprintf(stdout, "Colore: OpenReceiveStream OK [%s] handle=%u\n", tag, canrxhandle_);
        std::fflush(stdout);
        colore_post_rx_stream_setup(bus, bitrate);
        return true;
    };

    bool opened = try_parsec_order(c0, "vmx.can");

    if (!opened && c1 != nullptr) {
        opened = try_parsec_order(*c1, "vmx.getCAN()");
    }

    if (!opened) {
        (void)c0.Reset(nullptr);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        opened = try_parsec_order(c0, "vmx.can after Reset()");
    }

    if (!opened && c1 != nullptr) {
        (void)c1->Reset(nullptr);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        opened = try_parsec_order(*c1, "getCAN() after Reset()");
    }

    if (!opened) {
        (void)c0.SetMode(VMXCAN::VMXCAN_CONFIG, nullptr);
        opened = try_parsec_order(c0, "vmx.can in CONFIG");
    }

    if (!opened && c1 != nullptr) {
        (void)c1->SetMode(VMXCAN::VMXCAN_CONFIG, nullptr);
        opened = try_parsec_order(*c1, "getCAN() in CONFIG");
    }

    /* Some VMX builds want controller in NORMAL before a stream can attach. */
    if (!opened) {
        vmx_can_iface_ = &c0;
        (void)c0.FlushRxFIFO(nullptr);
        (void)c0.FlushTxFIFO(nullptr);
        (void)c0.ResetBusBitrate(bitrate, nullptr);
        (void)c0.SetMode(VMXCAN::VMXCAN_NORMAL, nullptr);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        canrxhandle_ = 0u;
        if (colore_open_rx_stream(c0, baseId, ackId, can_rx_multi_id_, canrxhandle_)) {
            opened = true;
            std::fprintf(stdout, "Colore: OpenReceiveStream OK [NORMAL then open, vmx.can] handle=%u\n", canrxhandle_);
            std::fflush(stdout);
        }
    }

    if (!opened && c1 != nullptr) {
        vmx_can_iface_ = c1;
        (void)c1->FlushRxFIFO(nullptr);
        (void)c1->FlushTxFIFO(nullptr);
        (void)c1->ResetBusBitrate(bitrate, nullptr);
        (void)c1->SetMode(VMXCAN::VMXCAN_NORMAL, nullptr);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        canrxhandle_ = 0u;
        if (colore_open_rx_stream(*c1, baseId, ackId, can_rx_multi_id_, canrxhandle_)) {
            opened = true;
            std::fprintf(stdout, "Colore: OpenReceiveStream OK [NORMAL then open, getCAN] handle=%u\n", canrxhandle_);
            std::fflush(stdout);
        }
    }

    if (!opened) {
        vmx_can_iface_ = &c0;
        std::fprintf(stderr,
            "Colore: CAN RX stream unavailable. VMXRemoteServer or another app may hold all HW filters.\n"
            "  Try: stop competing CAN users, reboot VMX, or run when no WPILib CAN code is active.\n");
        std::fprintf(stdout,
            "Colore: CAN RX stream unavailable. VMXRemoteServer or another app may hold all HW filters.\n");
        std::fflush(stderr);
        std::fflush(stdout);
        colore_post_rx_stream_setup(c0, bitrate);
    }
}

Colore::~Colore()
{
    /* Parsec: do not CloseReceiveStream — VMX-pi can end up unable to receive on the next process start. */
}

unsigned Colore::GetCANChannel()
{
    return ReadEnvChannel("COLORE_CAN_CHANNEL", 0u);
}

static void retrieve_all_can(VMXCAN& can, unsigned channel)
{
    /* Parsec PullCANData: first argument is CAN port index (0 or 1), not a timestamp. */
    (void)can.RetrieveAllCANData(static_cast<uint32_t>(channel), nullptr);
}

uint32_t Colore::GetAddress(uint32_t cmdByte) const
{
    return GetAddressForCanID(canID_, cmdByte);
}

uint32_t Colore::GetAddressForCanID(uint8_t canID, uint32_t cmdByte)
{
    (void)canID;
    return wire_cmd_id(cmdByte);
}

uint32_t Colore::GetConfigAckAddress() const
{
    return GetConfigAckAddressForCanID(canID_);
}

uint32_t Colore::GetConfigAckAddressForCanID(uint8_t canID)
{
    (void)canID;
    return wire_ack_id();
}

bool Colore::Write(uint32_t address, const uint8_t* data, size_t len, int32_t periodMS)
{
    if (!vmx_ || !vmx_->IsOpen() || len == 0u || len > 8u || data == nullptr)
        return false;

    VMXCANMessage msg;
    /* Parsec Write: full arbitration field (extended 29-bit in low bits). */
    msg.messageID = address;
    msg.setData(data, static_cast<uint8_t>(len));
    return vmxCan().SendMessage(msg, periodMS, nullptr);
}

void Colore::PullCANData()
{
    if (!vmx_ || !vmx_->IsOpen())
        return;
    retrieve_all_can(vmxCan(), can_channel_);
}

int Colore::Read(uint32_t address, uint8_t* data, size_t maxLen)
{
    PullCANData();
    return ReadCached(address, data, maxLen);
}

int Colore::ReadCached(uint32_t address, uint8_t* data, size_t maxLen)
{
    if (!vmx_ || !vmx_->IsOpen() || canrxhandle_ == 0u || data == nullptr || maxLen == 0u)
        return 0;

    VMXCANTimestampedMessage msg;
    uint64_t sys_ts = 0u;
    bool already = false;
    const uint32_t mid = address & VMXCAN_29BIT_MESSAGE_ID_MASK;
    VMXCANReceiveStreamHandle hh = static_cast<VMXCANReceiveStreamHandle>(canrxhandle_);
    VMXCAN& vcan = vmxCan();
    if (!vcan.GetBlackboardEntry(hh, mid, msg, sys_ts, already, nullptr)) {
        /* Parsec ReadCached: alternate key used by some VMX builds. */
        if (!vcan.GetBlackboardEntry(hh, mid | 0x80000000u, msg, sys_ts, already, nullptr))
            return 0;
    }

    uint8_t n = msg.dataSize;
    if (n > static_cast<uint8_t>(maxLen))
        n = static_cast<uint8_t>(maxLen);
    msg.getData(data, n);
    return static_cast<int>(n);
}

void Colore::DumpReceiveStream(unsigned maxMessages)
{
    if (!vmx_ || !vmx_->IsOpen() || canrxhandle_ == 0u)
        return;
    PullCANData();
    const unsigned cap = maxMessages > 32u ? 32u : maxMessages;
    VMXCANTimestampedMessage msgs[32];
    uint32_t read = 0u;
    (void)vmxCan().ReadReceiveStream(
        static_cast<VMXCANReceiveStreamHandle>(canrxhandle_),
        msgs, cap, read, nullptr);
    for (uint32_t i = 0u; i < read; ++i) {
        std::printf("  RX id=0x%08X dlc=%u\n",
                    static_cast<unsigned>(msgs[i].messageID & VMXCAN_29BIT_MESSAGE_ID_MASK),
                    static_cast<unsigned>(msgs[i].dataSize));
    }
}

bool Colore::SetMeasureModeOff(uint32_t* ackValue)
{
    uint8_t p[1] = {0u};
    return SendCmdWithAck(COLORE_CMD_SET_MEASMODE, p, 1u, ackValue);
}

bool Colore::SetMeasureModeAuto(bool ambient, uint32_t* ackValue)
{
    uint8_t p[2] = {1u, static_cast<uint8_t>(ambient ? 1u : 0u)};
    return SendCmdWithAck(COLORE_CMD_SET_MEASMODE, p, 2u, ackValue);
}

bool Colore::SetMeasureModeFixed(float zUserMm, bool ambient, uint32_t* ackValue)
{
    uint8_t p[6] = {2u, static_cast<uint8_t>(ambient ? 1u : 0u), 0u, 0u, 0u, 0u};
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

bool Colore::SetCanId(uint32_t canId, uint32_t* ackValue)
{
    uint8_t p[4] = {
        static_cast<uint8_t>(canId & 0xFFu),
        static_cast<uint8_t>((canId >> 8) & 0xFFu),
        static_cast<uint8_t>((canId >> 16) & 0xFFu),
        static_cast<uint8_t>((canId >> 24) & 0xFFu)
    };
    return SendCmdWithAck(COLORE_CMD_SET_CAN_ID, p, 4u, ackValue);
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
    const uint32_t txAddr = GetAddress(cmdByte);
    if (!Write(txAddr, payload, len, 0)) return false;

    uint32_t value = 0u;
    uint8_t status = COLORE_ACK_ERR_ARG;
    if (!WaitConfigAck(cmdByte, value, status, timeoutMs)) return false;
    if (ackValue) *ackValue = value;
    return status == COLORE_ACK_OK;
}

bool Colore::WaitConfigAck(uint8_t expectedCmdByte,
                           uint32_t& valueOut,
                           uint8_t& statusOut,
                           uint32_t timeoutMs)
{
    if (canrxhandle_ == 0u)
        return false;

    const VMXCANReceiveStreamHandle h = static_cast<VMXCANReceiveStreamHandle>(canrxhandle_);
    const uint32_t ackMid = wire_ack_id() & VMXCAN_29BIT_MESSAGE_ID_MASK;
    auto t0 = std::chrono::steady_clock::now();

    while (true) {
        VMXCAN& vcan = vmxCan();
        retrieve_all_can(vcan, can_channel_);

        VMXCANTimestampedMessage msgs[16];
        uint32_t read = 0u;
        (void)vcan.ReadReceiveStream(h, msgs, 16u, read, nullptr);
        for (uint32_t i = 0u; i < read; ++i) {
            const uint32_t rid = msgs[i].messageID & VMXCAN_29BIT_MESSAGE_ID_MASK;
            if (rid != ackMid)
                continue;
            if (msgs[i].dataSize < 6u || msgs[i].data[0] != expectedCmdByte)
                continue;
            statusOut = msgs[i].data[1];
            valueOut = static_cast<uint32_t>(msgs[i].data[2])
                     | (static_cast<uint32_t>(msgs[i].data[3]) << 8)
                     | (static_cast<uint32_t>(msgs[i].data[4]) << 16)
                     | (static_cast<uint32_t>(msgs[i].data[5]) << 24);
            return true;
        }

        VMXCANTimestampedMessage bb;
        uint64_t sys_ts = 0u;
        bool already = false;
        bool bbok = vcan.GetBlackboardEntry(h, ackMid, bb, sys_ts, already, nullptr);
        if (!bbok)
            bbok = vcan.GetBlackboardEntry(h, ackMid | 0x80000000u, bb, sys_ts, already, nullptr);
        if (bbok && bb.dataSize >= 6u && bb.data[0] == expectedCmdByte) {
            statusOut = bb.data[1];
            valueOut = static_cast<uint32_t>(bb.data[2])
                     | (static_cast<uint32_t>(bb.data[3]) << 8)
                     | (static_cast<uint32_t>(bb.data[4]) << 16)
                     | (static_cast<uint32_t>(bb.data[5]) << 24);
            return true;
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::steady_clock::now() - t0).count();
        if (elapsed >= static_cast<int64_t>(timeoutMs)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

} // namespace studica_driver
