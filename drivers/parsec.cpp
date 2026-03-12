#include "parsec.h"
#include "VMXPi.h"
#include <cstdlib>
#include <cstring>
#include <stdio.h>
#include <array>
#include <map>
#include <vector>

namespace studica_driver {

static unsigned s_canChannel = 0;

unsigned Parsec::GetCANChannel() { return s_canChannel; }

Parsec::Parsec(uint8_t canID, std::shared_ptr<VMXPi> vmx)
    : vmx_(vmx), canID_(canID)
{
    const char* chEnv = std::getenv("PARSEC_CAN_CHANNEL");
    s_canChannel = (chEnv && chEnv[0] == '1' && chEnv[1] == '\0') ? 1u : 0u;
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

    /* Filter: only accept Parsec IDs 0x0609xxxx (ignore Titan 0x020C0xxx and others) */
    if (!vmx_->can.OpenReceiveStream(canrxhandle_, PARSEC_BASE, 0x1FFF0000u, 100, &vmxerr))
        printf("Parsec: Error opening CAN RX Stream.\n");
    else
    {
        printf("Parsec: Opened CAN RX Stream (filter 0x0609xxxx), handle %d\n", canrxhandle_);
        if (vmx_->can.EnableReceiveStreamBlackboard(canrxhandle_, true, &vmxerr))
            printf("Parsec: Blackboard enabled.\n");
        else
            printf("Parsec: Error enabling Blackboard.\n");
    }

    if (!vmx_->can.FlushRxFIFO(&vmxerr))
        printf("Parsec: Error flushing CAN RX FIFO.\n");
    if (!vmx_->can.FlushTxFIFO(&vmxerr))
        printf("Parsec: Error flushing CAN TX FIFO.\n");
    if (!vmx_->can.SetMode(VMXCAN::VMXCAN_NORMAL, &vmxerr))
        printf("Parsec: Error setting CAN mode to Normal.\n");
    else
        printf("Parsec: CAN mode Normal.\n");

    vmx_->time.DelayMilliseconds(20);
    printf("Parsec driver started (CAN ID %u, VMX CAN channel %u).\n", canID_, s_canChannel);
    if (s_canChannel == 0)
        printf("  (Use: sudo PARSEC_CAN_CHANNEL=1 ./parsec_example for VMX CAN1)\n");
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
    return PARSEC_BASE + canID_ + (PARSEC_OFFSET * index);
}

uint32_t Parsec::GetAddressForCanID(uint8_t canID, uint32_t index)
{
    return PARSEC_BASE + canID + (PARSEC_OFFSET * index);
}

bool Parsec::Write(uint32_t address, const uint8_t* data, size_t len, int32_t periodMS)
{
    if (!vmx_ || !vmx_->IsOpen() || !data) return false;
    if (len > 8) len = 8;  // VMX classic CAN typically 8 bytes
    VMXCANMessage msg;
    msg.dataSize = static_cast<int>(len);
    msg.setData(const_cast<uint8_t*>(data), static_cast<int>(len));
    msg.messageID = address;
    VMXErrorCode vmxerr;
    return vmx_->can.SendMessage(msg, periodMS, &vmxerr);
}

void Parsec::PullCANData()
{
    if (!vmx_) return;
    VMXErrorCode vmxerr;
    (void)vmx_->can.RetrieveAllCANData(static_cast<uint32_t>(s_canChannel), &vmxerr);
}

int Parsec::Read(uint32_t address, uint8_t* data, size_t maxLen)
{
    if (!vmx_ || !data || maxLen == 0) return 0;
    PullCANData();
    return ReadCached(address, data, maxLen);
}

int Parsec::ReadCached(uint32_t address, uint8_t* data, size_t maxLen)
{
    if (!vmx_ || !data || maxLen == 0) return 0;
    VMXCANTimestampedMessage blackboard_msg;
    uint64_t sys_timestamp;
    bool already_retrieved;
    VMXErrorCode vmxerr;
    /* Try plain ID first, then with extended-ID bit (0x80000000) in case VMX keys by that */
    uint32_t key = address;
    if (!vmx_->can.GetBlackboardEntry(canrxhandle_, key, blackboard_msg, sys_timestamp, already_retrieved, &vmxerr))
    {
        key = address | 0x80000000u;
        if (!vmx_->can.GetBlackboardEntry(canrxhandle_, key, blackboard_msg, sys_timestamp, already_retrieved, &vmxerr))
            return 0;
    }
    int n = blackboard_msg.dataSize;
    if (n <= 0 || n > 64) n = 8;
    size_t copyLen = (maxLen < (size_t)n) ? maxLen : (size_t)n;
    std::memcpy(data, blackboard_msg.data, copyLen);
    return (int)copyLen;
}

void Parsec::DumpReceiveStream(unsigned maxMessages)
{
    if (!vmx_ || maxMessages == 0) return;
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
        if (n <= 0 || n > 64) n = 8;
        bool ext = true;
        if (id & 0x80000000u) { ext = false; id &= 0x7FFFFFFFu; }
        printf("    [%u] ID=0x%08X (%s) len=%d  data:", i, id, ext ? "ext" : "std", n);
        for (int j = 0; j < n && j < 8; j++) printf(" %02X", msgs[i].data[j]);
        if (n > 8) printf(" ...");
        printf("\n");
    }
}

bool Parsec::RequestConfig()
{
    uint8_t req[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    return Write(GetAddress(PARSEC_GETCONFIG), req, 8, 0);
}

bool Parsec::GetConfigResponse(uint8_t* configOut, unsigned maxLen)
{
    if (!configOut || maxLen < 8) return false;
    return Read(GetAddress(PARSEC_CMD_RESP), configOut, maxLen) >= 8;
}

int Parsec::ReadDataChunk(uint32_t dataIndex, uint8_t* dataOut, unsigned maxLen)
{
    if (dataIndex > 3 || !dataOut || maxLen < 2) return 0;
    uint32_t id = GetAddress(PARSEC_DATA0 + dataIndex);
    int n = Read(id, dataOut, maxLen);
    if (n < 2) return 0;
    // Payload: [0]=seq, [1]=zones or block_idx (CAN2), [2..] = int16 LE per zone. Classic CAN = 8 bytes; FD can send up to 34 per chunk.
    return n;
}

int Parsec::ReadDataStreamCAN2(uint8_t* seqOut, uint8_t* zonesOut, int16_t* fdist, int maxZones)
{
    if (!vmx_ || !fdist || maxZones <= 0 || maxZones > 64) return -1;

    const uint32_t data0Id = GetAddress(PARSEC_DATA0);
    std::map<uint8_t, std::map<uint8_t, std::array<int16_t, 3>>> bySeq;
    auto rd16 = [](const uint8_t* p) {
        return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
    };

    /* Multiple pull+read rounds so we collect more DATA0 frames (64 zones = 22 frames; VMX FIFO may drop if burst) */
    std::vector<VMXCANTimestampedMessage> msgs(128);
    for (int round = 0; round < 3; round++) {
        PullCANData();
        uint32_t numRead = 0;
        VMXErrorCode vmxerr;
        if (vmx_->can.ReadReceiveStream(canrxhandle_, msgs.data(), static_cast<unsigned>(msgs.size()), numRead, &vmxerr)) {
            for (uint32_t i = 0; i < numRead; i++) {
                uint32_t id = msgs[i].messageID & 0x7FFFFFFFu;
                if (id != data0Id) continue;
                int n = msgs[i].dataSize;
                if (n < 8) continue;
                const uint8_t* d = msgs[i].data;
                uint8_t seq = d[0], blk = d[1];
                bySeq[seq][blk] = {{ rd16(d + 2), rd16(d + 4), rd16(d + 6) }};
            }
        }
        if (round < 2) vmx_->time.DelayMilliseconds(12);
    }

    if (bySeq.empty()) return 0;

    // Choose seq with most blocks (prefer one that has block 0)
    uint8_t bestSeq = 0;
    size_t bestCount = 0;
    for (const auto& kv : bySeq) {
        size_t cnt = kv.second.size();
        if (cnt > bestCount || (cnt == bestCount && kv.second.count(0))) {
            bestCount = cnt;
            bestSeq = kv.first;
        }
    }
    const auto& blocks = bySeq[bestSeq];
    if (blocks.empty()) return 0;

    unsigned int maxBlk = 0;
    for (const auto& kv : blocks)
        if (kv.first > maxBlk) maxBlk = kv.first;

    const int blocksNeeded = (maxZones + 2) / 3;
    int zonesFilled = 0;
    for (int b = 0; b < blocksNeeded && (b * 3) < maxZones; b++) {
        auto it = blocks.find(static_cast<uint8_t>(b));
        const bool have = (it != blocks.end());
        for (int i = 0; i < 3; i++) {
            int zi = b * 3 + i;
            if (zi >= maxZones) break;
            fdist[zi] = have ? it->second[i] : -1;
            zonesFilled = zi + 1;
        }
    }
    /* Effective zone count = last index with non-padding (-1) + 1, so 16 zones shows 16 not 18 */
    int effectiveZones = 0;
    for (int i = 0; i < zonesFilled; i++)
        if (fdist[i] != -1) effectiveZones = i + 1;

    if (seqOut) *seqOut = bestSeq;
    if (zonesOut) *zonesOut = (uint8_t)(effectiveZones > 0 ? effectiveZones : zonesFilled);
    return zonesFilled;
}

void Parsec::ParseConfigApiIds(const uint8_t* config, uint32_t* meta, uint32_t* data0, uint32_t* data1,
                               uint32_t* data2, uint32_t* data3, uint32_t* getconfig, uint32_t* cmdResp)
{
    auto rd32 = [](const uint8_t* p) {
        return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    };
    if (meta)     *meta     = rd32(config + 8);
    if (data0)    *data0    = rd32(config + 12);
    if (data1)    *data1    = rd32(config + 16);
    if (data2)    *data2    = rd32(config + 20);
    if (data3)    *data3    = rd32(config + 24);
    if (getconfig)*getconfig= rd32(config + 28);
    if (cmdResp)  *cmdResp  = rd32(config + 32);
}

uint64_t Parsec::ParseConfigZoneMask(const uint8_t* config)
{
    auto rd64 = [](const uint8_t* p) {
        return (uint64_t)p[0] | ((uint64_t)p[1] << 8) | ((uint64_t)p[2] << 16) | ((uint64_t)p[3] << 24)
             | ((uint64_t)p[4] << 32) | ((uint64_t)p[5] << 40) | ((uint64_t)p[6] << 48) | ((uint64_t)p[7] << 56);
    };
    return rd64(config + 36);
}

} // namespace studica_driver
