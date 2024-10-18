#include "servo.h"
using namespace studica_driver;

Servo::Servo(VMXChannelIndex port, ServoType type, int min, int max) 
    : Servo(port, type, min, max, std::make_shared<VMXPi>(true, 50)) {}

Servo::Servo(VMXChannelIndex port, ServoType type, int min, int max, std::shared_ptr<VMXPi> vmx) 
    : vmx_(vmx), port_(port), type_(type), min_(min), max_(max), prev_pwm_servo_value_(min - 1) {
    if (port >= 0 && port <= 21) {
        PWMGeneratorConfig pwmgen_cfg(50);  // 50Hz for servos
        pwmgen_cfg.SetMaxDutyCycleValue(5000);  // Set PWM precision for better accuracy
        VMXErrorCode vmxerr;
        bool success = vmx_->io.ActivateSinglechannelResource(VMXChannelInfo(port_, VMXChannelCapability::PWMGeneratorOutput), 
                                                              &pwmgen_cfg, pwm_res_handle_, &vmxerr);
        SetBounds(0.5, 1.5, 2.5);
        if (!success) {
            printf("Failed to initialize servo for port %d\n", port_);
            DisplayVMXError(vmxerr);
        } else {
            printf("Successfully initialized port %d as a servo output\n", port_);
        }
    } else {
        printf("Port %d is not a valid servo port!\n", port_);
    }
}

Servo::~Servo() {
    VMXErrorCode vmxerr;
    if (!vmx_->io.DeallocateResource(pwm_res_handle_, &vmxerr)) {
        printf("Failed to deallocate PWMGenerator Resource %d\n", port_);
        DisplayVMXError(vmxerr);
    } else {
        printf("Deallocated PWMGenerator Resource %d\n", port_);
    }
}

void Servo::SetBounds(double min, double center, double max) {
    min_us_ = static_cast<int>((min / 20) * 5000);
    center_us_ = static_cast<int>((center / 20) * 5000);
    max_us_ = static_cast<int>((max / 20) * 5000);
}

int Servo::Map(int value) {
    printf("Min: %d Max: %d Min_us: %d Max_us: %d\n", min_, max_, min_us_, max_us_);
    if (value < min_) value = min_;
    if (value > max_) value = max_;
    return static_cast<int>((value - min_) * (max_us_ - min_us_) / (max_ - min_) + min_us_);
}

void Servo::SetAngle(int angle) {
    if (prev_pwm_servo_value_ != angle) {
        VMXErrorCode vmxerr;
        bool success = vmx_->io.PWMGenerator_SetDutyCycle(pwm_res_handle_, port_, Map(angle), &vmxerr);
        prev_pwm_servo_value_ = angle;
        if (!success) { 
            printf("Failed to set duty cycle for servo on port %d\n", port_);
            DisplayVMXError(vmxerr);
        } else {
            printf("PWM Duty cycle set on port %d at %d\n", port_, angle);
        }
    }
}

void Servo::SetSpeed(int speed) {
    if (prev_pwm_servo_value_ != speed) {
        VMXErrorCode vmxerr;
        bool success = vmx_->io.PWMGenerator_SetDutyCycle(pwm_res_handle_, port_, Map(speed), &vmxerr);
        prev_pwm_servo_value_ = speed;
        if (!success) { 
            printf("Failed to set duty cycle for servo on port %d\n", port_);
            DisplayVMXError(vmxerr);
        } else {
            printf("PWM Duty cycle set on port %d at %d\n", port_, speed);
        }
    }
}

void Servo::DisplayVMXError(VMXErrorCode vmxerr) 
{
    const char* p_err_description = GetVMXErrorString(vmxerr);
    printf("VMXError %d: %s\n", vmxerr, p_err_description);
}
