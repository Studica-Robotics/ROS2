#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>

struct VMXPi;

/**
 * Host-side driver for the NavX3 IMU over CAN.
 *
 * CAN address = BASE + (OFFSET * API index) + canID. The VMX-pi is classic CAN
 * only, so data arrives as one frame per message group (no CAN FD sample frame
 * or time sync).
 *
 * Note: NAVX3_* below are preprocessor macros (not C++ namespace members).
 */
namespace studica_driver {

#define NAVX3_BASE         0x040C0000u
#define NAVX3_OFFSET       0x40u

/* API indices for GetAddress(). Requests and their replies use separate IDs. */
#define NAVX3_API_SET_ODR_HZ            3
#define NAVX3_API_YAW                   5
#define NAVX3_API_QUAT6D                6
#define NAVX3_API_QUAT9D                7
#define NAVX3_API_ALGO_STATES           8
#define NAVX3_API_PITCH_ROLL            9
#define NAVX3_API_ANGULAR_VELOCITY      10
#define NAVX3_API_LINEAR_ACCEL          11
#define NAVX3_API_COMPASS               12
#define NAVX3_API_ANGLE                 13
#define NAVX3_API_SENSOR_UID1           14
#define NAVX3_API_SENSOR_UID2           15
#define NAVX3_API_IMU_SELF_TEST         17
#define NAVX3_API_TEMPERATURE           18
#define NAVX3_API_SENSOR_UID1_RESP      19
#define NAVX3_API_SENSOR_UID2_RESP      20
#define NAVX3_API_SET_ODR_HZ_RESP       24
#define NAVX3_API_IMU_SELF_TEST_RESP    27
#define NAVX3_API_FW_VERSION            28
#define NAVX3_API_FW_VERSION_RESP       29
#define NAVX3_API_RESET_YAW             30
#define NAVX3_API_RESET_YAW_RESP        31
#define NAVX3_API_YAW_MODE              34
#define NAVX3_API_YAW_MODE_RESP         35

/* Message groups, as bits of Navx3Data::valid_mask (same bits over USB). */
#define NAVX3_MSG_YAW               (1u << 0)
#define NAVX3_MSG_PITCH_ROLL        (1u << 1)
#define NAVX3_MSG_ANGLE             (1u << 2)
#define NAVX3_MSG_QUAT6D            (1u << 3)
#define NAVX3_MSG_QUAT9D            (1u << 4)
#define NAVX3_MSG_ANGULAR_VELOCITY  (1u << 5)
#define NAVX3_MSG_LINEAR_ACCEL      (1u << 6)
#define NAVX3_MSG_COMPASS           (1u << 7)
#define NAVX3_MSG_ALGO_STATES       (1u << 8)
#define NAVX3_MSG_TEMPERATURE       (1u << 9)

/** Command results: 0 is success, positive is the device's error status. */
#define NAVX3_OK                    0
#define NAVX3_TIMEOUT               -1

/** One NavX3 sample. Angles in degrees; quaternions w, x, y, z. */
struct Navx3Data {
    /** NAVX3_MSG_* bits whose fields are fresh. */
    uint32_t valid_mask = 0;
    double yaw_deg = 0, pitch_deg = 0, roll_deg = 0;
    /** Continuous (unwrapped) yaw. */
    double angle_deg = 0;
    /** 6-axis quaternion (no magnetometer). */
    double quat6[4] = {1, 0, 0, 0};
    /** 9-axis quaternion (with magnetometer). */
    double quat9[4] = {1, 0, 0, 0};
    double angular_vel_rad_per_sec[3] = {0, 0, 0};
    double linear_accel_mps2[3] = {0, 0, 0};
    /** Magnetic field x, y, z, then magnitude, in microtesla. */
    double mag_micro_tesla[4] = {0, 0, 0, 0};
    double gyro_bias[3] = {0, 0, 0};
    bool at_rest = false;
    bool mag_undisturbed = false;
    double temperature_c = 0;
};

class Navx3 {
public:
    /**
     * @param canID Device CAN ID (0..63), must match the device's setting.
     * @param vmx   VMXPi instance (e.g. std::make_shared<VMXPi>(true, 50)).
     */
    Navx3(uint8_t canID, std::shared_ptr<VMXPi> vmx);
    ~Navx3();

    /** CAN address for this device: NAVX3_BASE + NAVX3_OFFSET * index + canID */
    uint32_t GetAddress(uint32_t index) const;

    /** False if the CAN receive stream couldn't be opened. */
    bool IsOpen() const { return open_; }

    uint8_t GetCanID() const { return canID_; }

    /**
     * Takes in every frame that arrived since the last call and copies the
     * latest sample. A group counts as valid if it arrived within `maxAge`.
     * @return false if no group is valid
     */
    bool Read(Navx3Data& out, std::chrono::milliseconds maxAge);

    /*
     * Commands. Each waits for the reply and returns NAVX3_OK, the device's
     * error status, or NAVX3_TIMEOUT. Settings only change RAM.
     */
    int ResetYaw();
    /** Output data rate, 1-200 Hz. */
    int SetOutputRate(uint16_t hz);
    /** 6 (gyro and accelerometer) or 9 (with magnetometer) axes for yaw. */
    int SetYawAxes(int axes);
    int GetFirmwareVersion(int& major, int& minor, int& patch);
    int GetSensorUuid(uint32_t uuid[3]);
    /** Runs the IMU self test (~3.5 s, no data meanwhile). results: 8 codes. */
    int SelfTest(int32_t results[8]);

private:
    /** Newest frame on `address`, only if it hasn't been read before. Returns bytes copied. */
    int ReadNew(uint32_t address, uint8_t* data, size_t maxLen);
    bool Write(uint32_t address, const uint8_t* data, size_t len);
    /** Sends a request and waits for the reply. Returns reply bytes, 0 on timeout. */
    int Request(uint32_t api, const uint8_t* payload, size_t len, uint32_t replyApi,
                uint8_t* reply, std::chrono::milliseconds timeout);
    int Command(uint32_t api, const uint8_t* payload, size_t len, uint32_t replyApi);

    std::shared_ptr<VMXPi> vmx_;
    uint8_t canID_;
    unsigned int canrxhandle_{0};  // VMXCANReceiveStreamHandle
    bool open_{false};

    Navx3Data sample_;
    std::chrono::steady_clock::time_point stamps_[10] = {};
};

} // namespace studica_driver
