#ifndef IMU_H
#define IMU_H

#include <stdio.h>
#include <memory>
#include "VMXPi.h"

namespace studica_driver
{
class Imu
{
public:
    Imu();
    Imu(std::shared_ptr<VMXPi> vmx);
    ~Imu();

    float GetPitch();
    float GetYaw();
    float GetRoll();
    float GetCompassHeading();
    void ZeroYaw();
    bool IsCalibrating();
    bool IsConnected();
    int GetByteCount();
    int GetUpdateCount();
    int GetLastSensorTimestamp();
    float GetWorldLinearAccelX();
    float GetWorldLinearAccelY();
    float GetWorldLinearAccelZ();
    bool IsMoving();
    bool IsRotating();
    float GetBarometricPressure();
    float GetAltitude();
    bool IsAltitudeValid();
    float GetFusedHeading();
    bool IsMagneticDisturbance();
    bool IsMagnetometerCalibrated();
    float GetQuaternionW();
    float GetQuaternionX();
    float GetQuaternionY();
    float GetQuaternionZ();
    void ResetDisplacement();
    void UpdateDisplacement(float accel_x, float accel_y, float update_rate_hz, bool is_moving);
    float GetVelocityX();
    float GetVelocityY();
    float GetVelocityZ();
    float GetDisplacementX();
    float GetDisplacementY();
    float GetDisplacementZ();
    float GetAngle();
    float GetRate();
    void Reset();
    float GetRawGyroX();
    float GetRawGyroY();
    float GetRawGyroZ();
    float GetRawAccelX();
    float GetRawAccelY();
    float GetRawAccelZ();
    float GetRawMagX();
    float GetRawMagY();
    float GetRawMagZ();
    float GetPressure();
    float GetTempC();
    BoardYawAxis GetBoardYawAxis();
    std::string GetFirmwareVersion();
    bool RegisterCallback(IVMXTimestampedAHRSDataSubscriber* callback, void* callback_context);
    bool DeregisterCallback(IVMXTimestampedAHRSDataSubscriber* callback);
    bool BlockOnNewCurrentRegisterData(uint32_t timeout_ms, uint8_t *first_reg_addr_out, uint8_t *p_data_out, uint8_t requested_len, uint8_t *p_len_out);
    bool ReadConfigurationData(uint8_t first_reg_addr, uint8_t *p_data_out, uint8_t requested_len);
    int GetActualUpdateRate();
    int GetRequestedUpdateRate();
    void Stop();
private:
    std::shared_ptr<VMXPi> vmx_;
    VMXResourceHandle imu_res_handle_;
    void DisplayVMXError(VMXErrorCode vmxerr);
};
}

#endif // IMU_H
