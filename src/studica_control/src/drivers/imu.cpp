#include "imu.h"
using namespace studica_driver;

Imu::Imu() : vmx_(std::make_shared<VMXPi>(true, 50)) {}

Imu::Imu(std::shared_ptr<VMXPi> vmx) : vmx_(vmx) {}

Imu::~Imu() {} // todo

float Imu::GetPitch() return vmx_->ahrs.GetPitch();

float Imu::GetYaw() return vmx_->ahrs.GetYaw();

float Imu::GetRoll() return vmx_->ahrs.GetRoll();

float Imu::GetCompassHeading() { return vmx_->ahrs.GetCompassHeading(); }

float Imu::ZeroYaw() { return vmx_->ahrs.ZeroYaw(); }

bool Imu::IsCalibrating() { return vmx_->ahrs.IsCalibrating(); }

bool Imu::IsConnected() { return vmx_->ahrs.IsConnected(); }

int Imu::GetByteCount() { return vmx_->ahrs.GetByteCount(); }

int Imu::GetUpdateCount() { return vmx_->ahrs.GetUpdateCount(); }

int Imu::GetLastSensorTimestamp() { return vmx_->ahrs.GetLastSensorTimestamp(); }

float Imu::GetWorldLinearAccelX() { return vmx_->ahrs.GetWorldLinearAccelX(); }

float Imu::GetWorldLinearAccelY() { return vmx_->ahrs.GetWorldLinearAccelY(); }

float Imu::GetWorldLinearAccelZ() { return vmx_->ahrs.GetWorldLinearAccelZ(); }

bool Imu::IsMoving() { return vmx_->ahrs.IsMoving(); }

bool Imu::IsRotating() { return vmx_->ahrs.IsRotating(); }

float Imu::GetBarometricPressure() { return vmx_->ahrs.GetBarometricPressure(); }

float Imu::GetAltitude() { return vmx_->ahrs.GetAltitude(); }

bool Imu::IsAltitudeValid() { return vmx_->ahrs.IsAltitudeValid(); }

float Imu::GetFusedHeading() { return vmx_->ahrs.GetFusedHeading(); }

bool Imu::IsMagneticDisturbance() { return vmx_->ahrs.IsMagneticDisturbance(); }

bool Imu::IsMagnetometerCalibrated() { return vmx_->ahrs.IsMagnetometerCalibrated(); }

float Imu::GetQuaternionW() { return vmx_->ahrs.GetQuaternionW(); }

float Imu::GetQuaternionX() { return vmx_->ahrs.GetQuaternionX(); }

float Imu::GetQuaternionY() { return vmx_->ahrs.GetQuaternionY(); }

float Imu::GetQuaternionZ() { return vmx_->ahrs.GetQuaternionZ(); }

void Imu::ResetDisplacement() { vmx_->ahrs.ResetDisplacement(); }

void Imu::UpdateDisplacement(float accel_x, float accel_y, float update_rate_hz, bool is_moving) { vmx_->ahrs.UpdateDisplacement(accel_x, accel_y, update_rate_hz, is_moving); }

float Imu::GetVelocityX() { return vmx_->ahrs.GetVelocityX(); }

float Imu::GetVelocityY() { return vmx_->ahrs.GetVelocityY(); }

float Imu::GetVelocityZ() { return vmx_->ahrs.GetVelocityZ(); }

float Imu::GetDisplacementX() { return vmx_->ahrs.GetDisplacementX(); }

float Imu::GetDisplacementY() { return vmx_->ahrs.GetDisplacementY(); }

float Imu::GetDisplacementZ() { return vmx_->ahrs.GetDisplacementZ(); }

float Imu::GetAngle() { return vmx_->ahrs.GetAngle(); }

float Imu::GetRate() { return vmx_->ahrs.GetRate(); }

void Imu::Reset() { vmx_->ahrs.Reset(); }

float Imu::GetRawGyroX() { return vmx_->ahrs.GetRawGyroX(); }

float Imu::GetRawGyroY() { return vmx_->ahrs.GetRawGyroY(); }

float Imu::GetRawGyroZ() { return vmx_->ahrs.GetRawGyroZ(); }

float Imu::GetRawAccelX() { return vmx_->ahrs.GetRawAccelX(); }

float Imu::GetRawAccelY() { return vmx_->ahrs.GetRawAccelY(); }

float Imu::GetRawAccelZ() { return vmx_->ahrs.GetRawAccelZ(); }

float Imu::GetRawMagX() { return vmx_->ahrs.GetRawMagX(); }

float Imu::GetRawMagY() { return vmx_->ahrs.GetRawMagY(); }

float Imu::GetRawMagZ() { return vmx_->ahrs.GetRawMagZ(); }

float Imu::GetPressure() { return vmx_->ahrs.GetPressure(); }

float Imu::GetTempC() { return vmx_->ahrs.GetTempC(); }

int Imu::GetBoardYawAxis() { return vmx_->ahrs.GetBoardYawAxis(); }

int Imu::GetFirmwareVersion() { return vmx_->ahrs.GetFirmwareVersion(); }

void Imu::RegisterCallback(void* callback, void* callback_context) { vmx_->ahrs.RegisterCallback(callback, callback_context); }

void Imu::DeregisterCallback(void* callback) { vmx_->ahrs.DeregisterCallback(callback); }

void Imu::BlockOnNewCurrentRegisterData(int timeout_ms, int first_reg_addr_out, void* p_data_out, int requested_len, int p_len_out) { vmx_->ahrs.BlockOnNewCurrentRegisterData(timeout_ms, first_reg_addr_out, p_data_out, requested_len, p_len_out); }

bool Imu::ReadConfigurationData(int first_reg_addr, void* p_data_out, int requested_len) { return vmx_->ahrs.ReadConfigurationData(first_reg_addr, p_data_out, requested_len); }

int Imu::GetActualUpdateRate() { return vmx_->ahrs.GetActualUpdateRate(); }

void Imu::Stop() { vmx_->ahrs.Stop(); }

void Imu::DisplayVMXError(VMXErrorCode vmxerr)
{
    const char *p_err_description = GetVMXErrorString(vmxerr);
    printf("VMXError %d:  %s\n", vmxerr, p_err_description);
}
