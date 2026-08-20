#pragma once

#include "titan_iface.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace studica_driver {

/**
 * Host driver for a Titan2 motor controller over USB CDC serial
 * Implements ITitanController, so titan_component drives CAN or USB through one interface.
 *
 * Protocol: line-based ASCII request/response - host writes "<cmd> <args>\r\n",
 * device replies with "KEY=value" lines (some ending in "EOR=").
 *
 */
class TitanUsb : public ITitanController {
public:
    /**
     * @param port      serial device, e.g. /dev/ttyACM0
     * @param motorFreq PWM frequency in Hz applied to all channels via SetupMotors
     *                  (ignored if 0 or out of range); matches the CAN motor_freq.
     * @param baud      CDC ignores the line rate, but termios needs a value.
     */
    explicit TitanUsb(const std::string& port, uint16_t motorFreq = 0, int baud = 115200);
    ~TitanUsb() override;

    TitanUsb(const TitanUsb&) = delete;
    TitanUsb& operator=(const TitanUsb&) = delete;

    bool IsOpen() const { return fd_ >= 0; }
    bool isUsb() const override { return true; }

    // --- ITitanController: enable / open-loop ---
    void Enable(bool enable) override;
    void SetSpeed(uint8_t motor, double speedCfg) override;
    void SetSpeedAll(double duty) override;
    void DisableMotor(uint8_t motor) override;
    void SetMotorStopMode(uint8_t mode) override;   // SetStopMode 0/1 (firmware command)

    // --- ITitanController: closed-loop ---
    void SetTargetVelocity(uint8_t motor, float velocityRpm) override;
    void SetTargetDistance(uint8_t motor, int32_t distanceCounts) override;
    void SetTargetAngle(uint8_t motor, double angleDeg) override;
    void SetPositionHold(uint8_t motor, bool hold) override;

    // --- ITitanController: pid ---
    void SetPIDType(uint8_t type) override;
    void SetMotorPIDType(uint8_t motor, uint8_t type) override;
    void SetSensitivity(uint8_t motor, uint8_t sensitivity) override;  // persists to EEPROM over USB
    void AutotuneAll() override;
    void AutotuneMotor(uint8_t motor) override;

    // --- ITitanController: encoder config ---
    void SetupEncoder(uint8_t encoder) override;
    void ConfigureEncoder(uint8_t motor, double cfg) override;
    void SetEncoderResolution(uint8_t channel, uint16_t cpr) override;  // persists to EEPROM over USB
    void ResetEncoder(uint8_t motor) override;                          // ResetEncoder <ch> (firmware command)

    // --- ITitanController: current limiting ---
    void SetCurrentLimit(uint8_t channel, float limitAmps) override;   // persists to EEPROM over USB
    void SetCurrentLimitMode(uint8_t channel, uint8_t mode) override;

    // --- ITitanController: inversion (host-side) ---
    void InvertMotor(uint8_t motor) override;
    void InvertMotorDirection(uint8_t motor) override;
    void InvertMotorRPM(uint8_t motor) override;
    void InvertEncoderDirection(uint8_t motor) override;

    // --- ITitanController: read back ---
    float GetRPM(uint8_t motor) override;
    int32_t GetEncoderCount(uint8_t motor) override;
    double GetEncoderDistance(uint8_t motor) override;
    bool TryGetTargetRPMFromAll(float targetRpm[4]) override;
    double GetCypherAngle(uint8_t port) override;
    bool GetLimitSwitch(uint8_t motor, uint8_t direction) override;    // ReadLimitSwitches (firmware command)
    float GetControllerTemp() override;                                // ReadTemp (firmware command)
    std::string GetFirmwareVersion() override;
    std::string GetHardwareVersion() override;
    std::string GetSerialNumber() override;
    uint8_t GetID() override;

    // --- ITitanController: read back with freshness (USB always reports fresh) ---
    bool GetEncoderDistanceFresh(uint8_t motor, double& distance, bool& is_fresh,
                                 uint64_t* out_timestamp_us) override;
    bool GetRPMFresh(uint8_t motor, float& rpm, bool& is_fresh,
                     uint64_t* out_timestamp_us) override;
    bool GetCypherAnglesFresh(double angles[4], bool& is_fresh,
                              uint64_t* out_timestamp_us) override;
    bool GetLimitSwitchesFresh(bool fwd[4], bool rev[4], bool& is_fresh,
                               uint64_t* out_timestamp_us) override;

    /** Send a raw command line (without trailing CRLF). Mainly for the example/tests. */
    bool SendCommand(const std::string& cmd);

private:
    static constexpr int kNumMotors = 4;
    /** Encoder/RPM and Cypher reads within this window reuse the last poll. */
    static constexpr int kCacheValidMs = 8;
    /** Firmware command-timeout window armed on Enable(true). The host must
     *  send a command more often than this (the resend at motor_update_rate_hz,
     *  default 50 Hz = 20 ms, gives a wide margin). */
    static constexpr int kUsbTimeoutMs = 250;

    // serial port
    int fd_{-1};
    std::string port_;
    uint16_t motor_freq_{0};

    // background reader: serial bytes -> complete lines
    std::thread reader_;
    std::atomic<bool> running_{false};
    std::string rx_partial_;                 // reader thread only
    std::mutex rx_mutex_;                     // guards rx_lines_
    std::deque<std::string> rx_lines_;

    std::mutex tx_mutex_;                     // serializes writes (line-atomic)
    std::mutex query_mutex_;                  // serializes request/response transactions

    // host-side per-motor state (mirrors the CAN driver's behaviour)
    std::array<double, kNumMotors> dist_per_tick_{{1.0, 1.0, 1.0, 1.0}};
    std::array<bool, kNumMotors> invert_motor_{{false, false, false, false}};
    std::array<bool, kNumMotors> invert_encoder_{{false, false, false, false}};
    std::array<bool, kNumMotors> invert_rpm_{{false, false, false, false}};

    // telemetry cache (raw values, before host-side inversion)
    std::mutex cache_mutex_;
    std::array<int32_t, kNumMotors> enc_count_raw_{{0, 0, 0, 0}};
    std::array<double, kNumMotors> rpm_raw_{{0.0, 0.0, 0.0, 0.0}};
    std::chrono::steady_clock::time_point counters_time_{};
    std::array<double, kNumMotors> cypher_deg_{{0.0, 0.0, 0.0, 0.0}};
    std::chrono::steady_clock::time_point cypher_time_{};
    std::array<bool, kNumMotors> ls_high_{{false, false, false, false}};
    std::array<bool, kNumMotors> ls_low_{{false, false, false, false}};
    std::chrono::steady_clock::time_point limits_time_{};

    bool openPort(int baud);
    void readerLoop();
    /** Actively brake all channels (0% duty); USBOverride 0 / DisableRobot
     *  don't zero the outputs on their own. */
    void stopAllMotors();

    /**
     * Send `cmd`, then collect reply lines until `complete(lines)` is true, an
     * "EOR=" line arrives, or `timeout_ms` elapses. Clears any stale lines
     * first so the returned lines belong to this command. Thread-safe.
     */
    std::vector<std::string> transact(
        const std::string& cmd, int timeout_ms,
        const std::function<bool(const std::vector<std::string>&)>& complete);

    void refreshCountersIfStale();           // ReadCounter -> enc_count_raw_/rpm_raw_
    void refreshCypherIfStale();             // ReadCypher  -> cypher_deg_
    void refreshLimitsIfStale();             // ReadLimitSwitches -> ls_high_/ls_low_
};

}  // namespace studica_driver
