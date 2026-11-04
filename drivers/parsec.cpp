#include "parsec.hpp"
#include "VMXPi.h"
#include <cstring>
#include <stdio.h>
#include <thread>
#include <vector>

namespace studica_driver {

namespace {

constexpr int kZonesPerBlock = 3;
// A burst takes a few ms; an unfinished frame older than this is dropped, so
// distances from before a bus dropout never come back as new.
constexpr auto kMaxPartialAge = std::chrono::milliseconds(100);
constexpr auto kReplyTimeout = std::chrono::milliseconds(200);
// SET_RESOLUTION stops and restarts the sensor before it answers.
constexpr auto kResolutionTimeout = std::chrono::milliseconds(1000);

int16_t rd16(const uint8_t* p)
{
    return static_cast<int16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

int blocksFor(int zones)
{
    return (zones + kZonesPerBlock - 1) / kZonesPerBlock;
}

int countBits(uint32_t v)
{
    int n = 0;
    for (; v; v &= v - 1) n++;
    return n;
}

} // namespace

Parsec::Parsec(uint8_t canID, std::shared_ptr<VMXPi> vmx)
    : vmx_(vmx), canID_(canID)
{
    if (canID_ > 63)
    {
        printf("Parsec CAN ID %u is out of range (0..63).\n", canID_);
        fflush(stdout);
        return;
    }

    VMXErrorCode vmxerr;
    if (!vmx_ || !vmx_->IsOpen())
    {
        printf("Parsec: VMXPi not open.\n");
        fflush(stdout);
        return;
    }

    /* Filter: this device only (base and CAN ID; any API). No FIFO
     * flushes: sensors start after the Titan, and flushing TX could drop its
     * repeating enable frame. */
    if (!vmx_->can.OpenReceiveStream(canrxhandle_, PARSEC_BASE + canID_, 0x1FFF003Fu, 100, &vmxerr))
    {
        printf("Parsec: Error opening CAN RX Stream.\n");
        fflush(stdout);
        return;
    }
    if (!vmx_->can.EnableReceiveStreamBlackboard(canrxhandle_, true, &vmxerr))
        printf("Parsec: Error enabling Blackboard.\n");
    if (!vmx_->can.SetMode(VMXCAN::VMXCAN_NORMAL, &vmxerr))
        printf("Parsec: Error setting CAN mode to Normal.\n");

    open_ = true;
    printf("Parsec driver started (CAN ID %u).\n", canID_);
    fflush(stdout);
}

Parsec::~Parsec()
{
    /* Do not close the receive stream here. Explicit CloseReceiveStream leaves VMX
     * in a state where the next run cannot receive CAN (second run finds no device).
     * Rely on process exit so the next run can open a fresh stream. */
}

uint32_t Parsec::GetAddress(uint32_t index) const
{
    return GetAddressForCanID(canID_, index);
}

uint32_t Parsec::GetAddressForCanID(uint8_t canID, uint32_t index)
{
    return PARSEC_BASE + canID + (PARSEC_OFFSET * index);
}

bool Parsec::Write(uint32_t address, const uint8_t* data, size_t len, int32_t periodMS)
{
    if (!open_ || !data) return false;
    if (len > 8) len = 8;  // VMX is classic CAN only
    VMXCANMessage msg;
    msg.dataSize = static_cast<int>(len);
    msg.setData(const_cast<uint8_t*>(data), static_cast<int>(len));
    msg.messageID = address;
    VMXErrorCode vmxerr;
    return vmx_->can.SendMessage(msg, periodMS, &vmxerr);
}

static int readBlackboard(VMXPi& vmx, unsigned int handle, uint32_t address,
                          uint8_t* data, size_t maxLen, bool& alreadyRetrieved)
{
    VMXCANTimestampedMessage blackboard_msg;
    uint64_t sys_timestamp;
    VMXErrorCode vmxerr;
    /* Try plain ID first, then with extended-ID bit (0x80000000) in case VMX keys by that */
    if (!vmx.can.GetBlackboardEntry(handle, address, blackboard_msg, sys_timestamp, alreadyRetrieved, &vmxerr) &&
        !vmx.can.GetBlackboardEntry(handle, address | 0x80000000u, blackboard_msg, sys_timestamp,
                                    alreadyRetrieved, &vmxerr))
        return 0;
    int n = blackboard_msg.dataSize;
    if (n <= 0 || n > 8) n = 8;
    size_t copyLen = (maxLen < (size_t)n) ? maxLen : (size_t)n;
    std::memcpy(data, blackboard_msg.data, copyLen);
    return (int)copyLen;
}

int Parsec::Read(uint32_t address, uint8_t* data, size_t maxLen)
{
    if (!open_ || !data || maxLen == 0) return 0;
    bool alreadyRetrieved = false;
    return readBlackboard(*vmx_, canrxhandle_, address, data, maxLen, alreadyRetrieved);
}

int Parsec::ReadNew(uint32_t address, uint8_t* data, size_t maxLen)
{
    if (!open_ || !data || maxLen == 0) return 0;
    bool alreadyRetrieved = true;
    int n = readBlackboard(*vmx_, canrxhandle_, address, data, maxLen, alreadyRetrieved);
    return alreadyRetrieved ? 0 : n;
}

void Parsec::DumpReceiveStream(unsigned maxMessages)
{
    if (!open_ || maxMessages == 0) return;
    std::vector<VMXCANTimestampedMessage> msgs(maxMessages);
    uint32_t numRead = 0;
    VMXErrorCode vmxerr;
    if (!vmx_->can.ReadReceiveStream(canrxhandle_, msgs.data(), maxMessages, numRead, &vmxerr) || numRead == 0) {
        printf("  (no CAN frames in stream)\n");
        return;
    }
    printf("  Raw CAN stream: %u frame(s)\n", numRead);
    for (uint32_t i = 0; i < numRead; i++) {
        uint32_t id = msgs[i].messageID;
        int n = msgs[i].dataSize;
        if (n <= 0 || n > 8) n = 8;
        bool ext = true;
        if (id & 0x80000000u) { ext = false; id &= 0x7FFFFFFFu; }
        printf("    [%u] ID=0x%08X (%s) len=%d  data:", i, id, ext ? "ext" : "std", n);
        for (int j = 0; j < n; j++) printf(" %02X", msgs[i].data[j]);
        printf("\n");
    }
}

bool Parsec::GetConfig(Config& out)
{
    const uint32_t replyId = GetAddress(PARSEC_CMD_RESP);
    uint8_t data[8] = {0};
    ReadNew(replyId, data, sizeof(data));  // drop a stale reply so it isn't taken for the answer

    const uint8_t req[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    if (!Write(GetAddress(PARSEC_GETCONFIG), req, 8)) return false;

    const auto deadline = std::chrono::steady_clock::now() + kReplyTimeout;
    while (ReadNew(replyId, data, sizeof(data)) < 8) {
        if (std::chrono::steady_clock::now() >= deadline) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    out.version = data[0];
    out.distance_enabled = data[1] != 0;
    out.can_fd = data[2] != 0;
    out.can_brs = data[3] != 0;
    out.can_id = data[4] & 0x3F;
    out.firmware[0] = data[5];
    out.firmware[1] = data[6];
    out.firmware[2] = data[7];
    return true;
}

int Parsec::SendCommand(uint32_t index, const uint8_t* payload, size_t len, std::chrono::milliseconds timeout)
{
    const uint32_t ackId = GetAddress(PARSEC_ACK);
    uint8_t ack[8] = {0};
    ReadNew(ackId, ack, sizeof(ack));  // drop a stale ACK

    if (!Write(GetAddress(index), payload, len)) return PARSEC_ACK_TIMEOUT;

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (ReadNew(ackId, ack, sizeof(ack)) >= 6 && ack[0] == index)
            return ack[1];
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return PARSEC_ACK_TIMEOUT;
}

int Parsec::SetOutputRate(uint8_t hz)
{
    return SendCommand(PARSEC_SET_OUTPUT_RATE, &hz, 1, kReplyTimeout);
}

int Parsec::SetResolution(uint8_t zones)
{
    return SendCommand(PARSEC_SET_RESOLUTION, &zones, 1, kResolutionTimeout);
}

int Parsec::SetRangingFrequency(uint8_t hz)
{
    return SendCommand(PARSEC_SET_FREQUENCY, &hz, 1, kReplyTimeout);
}

int Parsec::SetZoneMask(uint64_t mask)
{
    uint8_t payload[8];
    for (int i = 0; i < 8; i++) payload[i] = static_cast<uint8_t>(mask >> (8 * i));
    return SendCommand(PARSEC_SET_ZONE_MASK, payload, sizeof(payload), kReplyTimeout);
}

bool Parsec::FeedBlock(const uint8_t* d, std::chrono::steady_clock::time_point now)
{
    const uint8_t seq = d[0];
    const int block = d[1] & ~PARSEC_BLOCK_64_ZONES;
    const uint8_t zones = (d[1] & PARSEC_BLOCK_64_ZONES) ? 64 : 16;

    /* Also stops a stale frame being completed by a later one whose 5-bit
     * sequence number happens to match. */
    if (partial_active_ && now - partial_start_ > kMaxPartialAge)
        partial_active_ = false;

    if (!partial_active_ || partial_seq_ != seq || partial_zones_ != zones) {
        partial_active_ = true;
        partial_seq_ = seq;
        partial_zones_ = zones;
        partial_blocks_ = 0;
        partial_start_ = now;
        for (int i = 0; i < 64; i++) partial_fdist_[i] = -1;
    }

    const int expected = blocksFor(zones);
    if (block >= expected) return false;
    for (int i = 0; i < kZonesPerBlock; i++) {
        const int zone = block * kZonesPerBlock + i;
        if (zone < zones) partial_fdist_[zone] = rd16(d + 2 + 2 * i);
    }
    partial_blocks_ |= 1u << block;
    if (countBits(partial_blocks_) == expected) {
        partial_active_ = false;
        return true;
    }
    return false;
}

int Parsec::ReadDataStreamCAN2(uint8_t* seqOut, uint8_t* zonesOut, int16_t* fdist, int maxZones)
{
    if (!open_ || !fdist || maxZones <= 0 || maxZones > 64) return -1;

    /* Drain the receive stream (the VMX IO thread fills it). Do not call
     * RetrieveAllCANData here: that races the IO thread. */
    const uint32_t data0Id = GetAddress(PARSEC_DATA0);
    VMXCANTimestampedMessage msgs[32];
    int filled = 0;
    const auto now = std::chrono::steady_clock::now();
    for (;;) {
        uint32_t numRead = 0;
        VMXErrorCode vmxerr;
        if (!vmx_->can.ReadReceiveStream(canrxhandle_, msgs, 32, numRead, &vmxerr) || numRead == 0)
            break;
        for (uint32_t i = 0; i < numRead; i++) {
            if ((msgs[i].messageID & 0x7FFFFFFFu) != data0Id || msgs[i].dataSize < 8) continue;
            if (!FeedBlock(msgs[i].data, now)) continue;
            // A complete frame; a later one in this drain replaces it.
            filled = partial_zones_ < maxZones ? partial_zones_ : maxZones;
            for (int z = 0; z < filled; z++) fdist[z] = partial_fdist_[z];
            if (seqOut) *seqOut = partial_seq_;
            if (zonesOut) *zonesOut = partial_zones_;
        }
    }
    return filled;
}

} // namespace studica_driver
