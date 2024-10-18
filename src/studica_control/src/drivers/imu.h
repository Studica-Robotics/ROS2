#ifndef IMU_H
#define IMU_H

#include <stdio.h>
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
    int GetBoardYawAxis();
    int GetFirmwareVersion();
    void RegisterCallback(void* callback, void* callback_context);
    void DeregisterCallback(void* callback);
    void BlockOnNewCurrentRegisterData(int timeout_ms, int first_reg_addr_out, void* p_data_out, int requested_len, int p_len_out);
    bool ReadConfigurationData(int first_reg_addr, void* p_data_out, int requested_len);
    int GetActualUpdateRate();
    void Stop();
private:
    std::shared_ptr<VMXPi> vmx_;
    VMXResourceHandle imu_res_handle_;
    void DisplayVMXError(VMXErrorCode vmxerr);
};
}

#endif // IMU_H
