#include "navx3.hpp"
#include "VMXPi.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>

// Frame layouts and scales from StudicaLib (NavxProtocol.cpp), which was
// verified against NavX3 firmware on hardware.

namespace studica_driver {

namespace {

// Fixed-point scales used by the firmware's classic CAN frames.
constexpr double kQuatScale = 32767.0;
constexpr double kAngleScale = 32767.0;
constexpr double kGyroScale = 32767.0 / (4000.0 * M_PI / 180.0);
constexpr double kAccelScale = 32767.0 / (32 * 9.80665);
constexpr double kMagScale = 32767.0 / 800.0;
constexpr double kTempScale = 32767.0 / 100.0;
constexpr double kGyroBiasScale8 = 127.0 / 0.5;

constexpr auto kReplyTimeout = std::chrono::milliseconds(200);
// The IMU self test takes ~3.5 s; the NavX3 streams nothing meanwhile.
constexpr auto kSelfTestTimeout = std::chrono::milliseconds(5000);

// Streaming groups, in NAVX3_MSG_* bit order.
const uint32_t kDataApis[10] = {
    NAVX3_API_YAW, NAVX3_API_PITCH_ROLL, NAVX3_API_ANGLE, NAVX3_API_QUAT6D, NAVX3_API_QUAT9D,
    NAVX3_API_ANGULAR_VELOCITY, NAVX3_API_LINEAR_ACCEL, NAVX3_API_COMPASS, NAVX3_API_ALGO_STATES,
    NAVX3_API_TEMPERATURE,
};

int16_t i16(const uint8_t* d, size_t at)
{
    return static_cast<int16_t>(d[at] | (d[at + 1] << 8));
}

uint16_t u16(const uint8_t* d, size_t at)
{
    return static_cast<uint16_t>(d[at] | (d[at + 1] << 8));
}

uint32_t u32(const uint8_t* d, size_t at)
{
    return static_cast<uint32_t>(d[at]) | (static_cast<uint32_t>(d[at + 1]) << 8) |
           (static_cast<uint32_t>(d[at + 2]) << 16) | (static_cast<uint32_t>(d[at + 3]) << 24);
}

// Angles are sent as a signed whole part plus an unsigned fraction.
double angle(const uint8_t* d, size_t at)
{
    return i16(d, at) + u16(d, at + 2) / kAngleScale;
}

void vector(const uint8_t* d, double scale, double* out, int n)
{
    for (int i = 0; i < n; i++) out[i] = i16(d, 2 * i) / scale;
}

/** Decodes one streaming frame into `s`. Returns false if it is too short. */
bool decode(uint32_t api, const uint8_t* d, int len, Navx3Data& s)
{
    switch (api) {
        case NAVX3_API_YAW:
            if (len < 4) return false;
            s.yaw_deg = angle(d, 0);
            return true;
        case NAVX3_API_PITCH_ROLL:
            if (len < 8) return false;
            s.pitch_deg = angle(d, 0);
            s.roll_deg = angle(d, 4);
            return true;
        case NAVX3_API_ANGLE:
            if (len < 4) return false;
            s.angle_deg = angle(d, 0);
            return true;
        case NAVX3_API_QUAT6D:
            if (len < 8) return false;
            vector(d, kQuatScale, s.quat6, 4);
            return true;
        case NAVX3_API_QUAT9D:
            if (len < 8) return false;
            vector(d, kQuatScale, s.quat9, 4);
            return true;
        case NAVX3_API_ANGULAR_VELOCITY:
            if (len < 6) return false;
            vector(d, kGyroScale, s.angular_vel_rad_per_sec, 3);
            return true;
        case NAVX3_API_LINEAR_ACCEL:
            if (len < 6) return false;
            vector(d, kAccelScale, s.linear_accel_mps2, 3);
            return true;
        case NAVX3_API_COMPASS:
            if (len < 8) return false;
            vector(d, kMagScale, s.mag_micro_tesla, 4);
            return true;
        case NAVX3_API_ALGO_STATES:
            if (len < 8) return false;
            for (int i = 0; i < 3; i++) s.gyro_bias[i] = static_cast<int8_t>(d[i]) / kGyroBiasScale8;
            s.at_rest = (d[3] & 0x01) != 0;
            // The firmware sends a "magnetic disturbance detected" flag.
            s.mag_undisturbed = (d[3] & 0x02) == 0;
            return true;
        case NAVX3_API_TEMPERATURE:
            if (len < 2) return false;
            s.temperature_c = i16(d, 0) / kTempScale;
            return true;
        default:
            return false;
    }
}

} // namespace

Navx3::Navx3(uint8_t canID, std::shared_ptr<VMXPi> vmx)
    : vmx_(vmx), canID_(canID)
{
    if (canID_ > 63) {
        printf("NavX3 CAN ID %u is out of range (0..63).\n", canID_);
        fflush(stdout);
        return;
    }
    if (!vmx_ || !vmx_->IsOpen()) {
        printf("NavX3: VMXPi not open.\n");
        fflush(stdout);
        return;
    }

    /* Filter: this device only (base and CAN ID; any API). No FIFO
     * flushes: sensors start after the Titan, and flushing TX could drop its
     * repeating enable frame. */
    VMXErrorCode vmxerr;
    if (!vmx_->can.OpenReceiveStream(canrxhandle_, NAVX3_BASE + canID_, 0x1FFF003Fu, 100, &vmxerr)) {
        printf("NavX3: Error opening CAN RX Stream (VMX error %d).\n", static_cast<int>(vmxerr));
        fflush(stdout);
        return;
    }
    if (!vmx_->can.EnableReceiveStreamBlackboard(canrxhandle_, true, &vmxerr))
        printf("NavX3: Error enabling Blackboard.\n");
    if (!vmx_->can.SetMode(VMXCAN::VMXCAN_NORMAL, &vmxerr))
        printf("NavX3: Error setting CAN mode to Normal.\n");

    open_ = true;
    printf("NavX3 driver started (CAN ID %u).\n", canID_);
    fflush(stdout);
}

Navx3::~Navx3()
{
    /* Do not close the receive stream (see Parsec): the next run may not receive CAN. */
}

uint32_t Navx3::GetAddress(uint32_t index) const
{
    return NAVX3_BASE + NAVX3_OFFSET * index + canID_;
}

bool Navx3::Write(uint32_t address, const uint8_t* data, size_t len)
{
    if (!open_ || data == nullptr || len == 0 || len > 8) return false;
    VMXCANMessage msg;
    msg.messageID = address;
    msg.dataSize = static_cast<int>(len);
    msg.setData(const_cast<uint8_t*>(data), static_cast<int>(len));
    VMXErrorCode vmxerr;
    return vmx_->can.SendMessage(msg, 0, &vmxerr);
}

int Navx3::ReadNew(uint32_t address, uint8_t* data, size_t maxLen)
{
    if (!open_ || data == nullptr || maxLen == 0) return 0;
    VMXCANTimestampedMessage msg;
    uint64_t sys_timestamp = 0;
    bool already_retrieved = true;
    VMXErrorCode vmxerr;
    /* Try plain ID first, then with extended-ID bit (0x80000000) in case VMX keys by that */
    if (!vmx_->can.GetBlackboardEntry(canrxhandle_, address, msg, sys_timestamp, already_retrieved, &vmxerr) &&
        !vmx_->can.GetBlackboardEntry(canrxhandle_, address | 0x80000000u, msg, sys_timestamp,
                                      already_retrieved, &vmxerr))
        return 0;
    if (already_retrieved) return 0;
    int n = msg.dataSize;
    if (n <= 0 || n > 8) n = 8;
    const size_t copyLen = maxLen < static_cast<size_t>(n) ? maxLen : static_cast<size_t>(n);
    std::memcpy(data, msg.data, copyLen);
    return static_cast<int>(copyLen);
}

bool Navx3::Read(Navx3Data& out, std::chrono::milliseconds maxAge)
{
    const auto now = std::chrono::steady_clock::now();
    for (int i = 0; i < 10; i++) {
        uint8_t d[8] = {0};
        const int n = ReadNew(GetAddress(kDataApis[i]), d, sizeof(d));
        if (n > 0 && decode(kDataApis[i], d, n, sample_))
            stamps_[i] = now;
    }
    out = sample_;
    out.valid_mask = 0;
    for (int i = 0; i < 10; i++) {
        if (stamps_[i] != std::chrono::steady_clock::time_point{} && now - stamps_[i] <= maxAge)
            out.valid_mask |= 1u << i;
    }
    return out.valid_mask != 0;
}

int Navx3::Request(uint32_t api, const uint8_t* payload, size_t len, uint32_t replyApi,
                   uint8_t* reply, std::chrono::milliseconds timeout)
{
    const uint32_t replyId = GetAddress(replyApi);
    uint8_t stale[8];
    ReadNew(replyId, stale, sizeof(stale));  // drop a stale reply so it isn't taken for the answer
    if (!Write(GetAddress(api), payload, len)) return 0;

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        const int n = ReadNew(replyId, reply, 8);
        if (n > 0) return n;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return 0;
}

int Navx3::Command(uint32_t api, const uint8_t* payload, size_t len, uint32_t replyApi)
{
    uint8_t reply[8] = {0};
    if (Request(api, payload, len, replyApi, reply, kReplyTimeout) < 1) return NAVX3_TIMEOUT;
    return reply[0];  // status byte, 0 = success
}

int Navx3::ResetYaw()
{
    const uint8_t empty[1] = {0};
    return Command(NAVX3_API_RESET_YAW, empty, 1, NAVX3_API_RESET_YAW_RESP);
}

int Navx3::SetOutputRate(uint16_t hz)
{
    // The reply is a status byte then the applied rate, which may be lower than asked.
    const uint8_t payload[2] = {static_cast<uint8_t>(hz & 0xFF), static_cast<uint8_t>(hz >> 8)};
    return Command(NAVX3_API_SET_ODR_HZ, payload, 2, NAVX3_API_SET_ODR_HZ_RESP);
}

int Navx3::SetYawAxes(int axes)
{
    const uint8_t payload[1] = {static_cast<uint8_t>(axes)};
    return Command(NAVX3_API_YAW_MODE, payload, 1, NAVX3_API_YAW_MODE_RESP);
}

int Navx3::GetFirmwareVersion(int& major, int& minor, int& patch)
{
    const uint8_t empty[1] = {0};
    uint8_t reply[8] = {0};
    if (Request(NAVX3_API_FW_VERSION, empty, 1, NAVX3_API_FW_VERSION_RESP, reply, kReplyTimeout) < 3)
        return NAVX3_TIMEOUT;
    major = reply[0];
    minor = reply[1];
    patch = reply[2];
    return NAVX3_OK;
}

int Navx3::GetSensorUuid(uint32_t uuid[3])
{
    const uint8_t empty[1] = {0};
    uint8_t first[8] = {0};
    uint8_t second[8] = {0};
    if (Request(NAVX3_API_SENSOR_UID1, empty, 1, NAVX3_API_SENSOR_UID1_RESP, first, kReplyTimeout) < 8 ||
        Request(NAVX3_API_SENSOR_UID2, empty, 1, NAVX3_API_SENSOR_UID2_RESP, second, kReplyTimeout) < 4)
        return NAVX3_TIMEOUT;
    uuid[0] = u32(first, 0);
    uuid[1] = u32(first, 4);
    uuid[2] = u32(second, 0);
    return NAVX3_OK;
}

int Navx3::SelfTest(int32_t results[8])
{
    const uint8_t empty[1] = {0};
    uint8_t reply[8] = {0};
    if (Request(NAVX3_API_IMU_SELF_TEST, empty, 1, NAVX3_API_IMU_SELF_TEST_RESP, reply, kSelfTestTimeout) < 8)
        return NAVX3_TIMEOUT;
    for (int i = 0; i < 8; i++) results[i] = reply[i];
    return NAVX3_OK;
}

} // namespace studica_driver
