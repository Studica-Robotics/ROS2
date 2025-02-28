#include "pwm.h"
using namespace studica_driver;

PWM::PWM(VMXChannelIndex port, PWMType type, int min, int max, std::shared_ptr<VMXPi> vmx) 
    : port_(port), type_(type), min_(min), max_(max), vmx_(vmx), prev_pwm_pwm_value_(min - 1) {
    if (port <= 21) {
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

PWM::~PWM() {
    VMXErrorCode vmxerr;
    if (!vmx_->io.DeallocateResource(pwm_res_handle_, &vmxerr)) {
        printf("Failed to deallocate PWMGenerator Resource %d\n", port_);
        DisplayVMXError(vmxerr);
    } else {
        printf("Deallocated PWMGenerator Resource %d\n", port_);
    }
}

void PWM::SetBounds(double min, double center, double max) {
    min_us_ = static_cast<int>((min / 20) * 5000);
    center_us_ = static_cast<int>((center / 20) * 5000);
    max_us_ = static_cast<int>((max / 20) * 5000);
}

int PWM::Map(int value) {
    printf("Min: %d Max: %d Min_us: %d Max_us: %d\n", min_, max_, min_us_, max_us_);
    if (value < min_) value = min_;
    if (value > max_) value = max_;
    return static_cast<int>((value - min_) * (max_us_ - min_us_) / (max_ - min_) + min_us_);
}

void PWM::DisplayVMXError(VMXErrorCode vmxerr) 
{
    const char* p_err_description = GetVMXErrorString(vmxerr);
    printf("VMXError %d: %s\n", vmxerr, p_err_description);
}
