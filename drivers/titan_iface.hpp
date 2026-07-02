#pragma once

#include <cstdint>
#include <string>

namespace studica_driver {

/**
 * Transport-agnostic interface for a Titan motor controller.
 *
 * Two implementations exist:
 *   - studica_driver::Titan    - controls the device over the VMX CAN bus.
 *   - studica_driver::TitanUsb - controls the device over USB CDC serial.
 *
 * The ROS2 titan_component holds an ITitanController and is otherwise
 * transport-unaware: it uses the CAN or USB implementation from params.yaml
 * and then drives it through this interface.
 *
 */
class ITitanController {
public:
    virtual ~ITitanController() = default;

    /** true if this controller talks to the device over USB, false for CAN.
     *  Lets callers branch on transport when needed (e.g. logging). All control
     *  operations are supported on both transports. */
    virtual bool isUsb() const = 0;

    // --- enable / safe state ---
    virtual void Enable(bool enable) = 0;

    // --- open-loop motor control ---
    virtual void SetSpeed(uint8_t motor, double speedCfg) = 0;
    virtual void SetSpeedAll(double duty) = 0;
    virtual void DisableMotor(uint8_t motor) = 0;
    virtual void SetMotorStopMode(uint8_t mode) = 0;

    // --- closed-loop control (Titan2 firmware) ---
    virtual void SetTargetVelocity(uint8_t motor, float velocityRpm) = 0;
    virtual void SetTargetDistance(uint8_t motor, int32_t distanceCounts) = 0;
    virtual void SetTargetAngle(uint8_t motor, double angleDeg) = 0;
    virtual void SetPositionHold(uint8_t motor, bool hold) = 0;

    // --- pid tuning (Titan2 firmware) ---
    virtual void SetPIDType(uint8_t type) = 0;
    virtual void SetMotorPIDType(uint8_t motor, uint8_t type) = 0;
    virtual void SetSensitivity(uint8_t motor, uint8_t sensitivity) = 0;
    virtual void AutotuneAll() = 0;
    virtual void AutotuneMotor(uint8_t motor) = 0;

    // --- encoder configuration ---
    virtual void SetupEncoder(uint8_t encoder) = 0;
    virtual void ConfigureEncoder(uint8_t motor, double cfg) = 0;
    virtual void SetEncoderResolution(uint8_t channel, uint16_t cpr) = 0;
    virtual void ResetEncoder(uint8_t motor) = 0;

    // --- current limiting ---
    virtual void SetCurrentLimit(uint8_t channel, float limitAmps) = 0;
    virtual void SetCurrentLimitMode(uint8_t channel, uint8_t mode) = 0;

    // --- motor direction / inversion (applied host-side) ---
    virtual void InvertMotor(uint8_t motor) = 0;
    virtual void InvertMotorDirection(uint8_t motor) = 0;
    virtual void InvertMotorRPM(uint8_t motor) = 0;
    virtual void InvertEncoderDirection(uint8_t motor) = 0;

    // --- read back ---
    virtual float GetRPM(uint8_t motor) = 0;
    virtual int32_t GetEncoderCount(uint8_t motor) = 0;
    virtual double GetEncoderDistance(uint8_t motor) = 0;
    virtual bool TryGetTargetRPMFromAll(float targetRpm[4]) = 0;
    virtual double GetCypherAngle(uint8_t port) = 0;
    virtual bool GetLimitSwitch(uint8_t motor, uint8_t direction) = 0;
    virtual float GetControllerTemp() = 0;
    virtual std::string GetFirmwareVersion() = 0;
    virtual std::string GetHardwareVersion() = 0;
    virtual std::string GetSerialNumber() = 0;
    virtual uint8_t GetID() = 0;

    // --- read back with freshness (CAN: VMX blackboard; USB: always fresh) ---
    virtual bool GetEncoderDistanceFresh(uint8_t motor, double& distance, bool& is_fresh,
                                         uint64_t* out_timestamp_us) = 0;
    virtual bool GetRPMFresh(uint8_t motor, float& rpm, bool& is_fresh,
                             uint64_t* out_timestamp_us) = 0;
    virtual bool GetCypherAnglesFresh(double angles[4], bool& is_fresh,
                                      uint64_t* out_timestamp_us) = 0;
    virtual bool GetLimitSwitchesFresh(bool fwd[4], bool rev[4], bool& is_fresh,
                                       uint64_t* out_timestamp_us) = 0;
};

}  // namespace studica_driver
