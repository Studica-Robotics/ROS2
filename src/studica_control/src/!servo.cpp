#include "studica_control/servo!.h"
#include <stdio.h>

Servo::Servo(std::shared_ptr<VMXPi> vmx, VMXChannelIndex port, ServoType type, int min, int max) 
    : Device("servo_"), vmx_(vmx), port_(port), type_(type), min_(min), max_(max), prev_pwm_servo_value_(min - 1) {
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

void Servo::SetBounds(double min, double center, double max) {
    min_us_ = static_cast<int>((min / 20) * 5000);
    center_us_ = static_cast<int>((center / 20) * 5000);
    max_us_ = static_cast<int>((max / 20) * 5000);
}

int Servo::Map(int value) {
    cout << "Value: " << value << endl;
    cout << "Min: " << min_ << " Max: " << max_ << " Min_us: " << min_us_ << " Max_us: " << max_us_ << endl;
    if (value < min_) value = min_;
    if (value > max_) value = max_;
    return static_cast<int>((value - min_) * (max_us_ - min_us_) / (max_ - min_) + min_us_);
}

void Servo::SetAngle(int angle) {
    // if (type_ != ServoType::Standard) {
    //     printf("Cannot 'SetAngle'. Servo on port %d is not a standard servo\n", port_);
    //     return;
    // }
    
    if (prev_pwm_servo_value_ != angle) {
        VMXErrorCode vmxerr;
        bool success = vmx_->io.PWMGenerator_SetDutyCycle(pwm_res_handle_, port_, Map(angle), &vmxerr);
        // print the map value
        printf("Map value: %d\n", Map(angle));
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
    if (type_ != ServoType::Continuous) {
        printf("Cannot 'SetSpeed()'. Servo on port %d is not a continuous servo\n", port_);
        return;
    }
    if (prev_pwm_servo_value_ != speed) {
        VMXErrorCode vmxerr;
        bool success = vmx_->io.PWMGenerator_SetDutyCycle(pwm_res_handle_, 0, Map(speed), &vmxerr);
        prev_pwm_servo_value_ = speed;
        if (!success) {
            printf("Failed to set duty cycle for servo on port %d\n", port_);
            DisplayVMXError(vmxerr);
        } else {
            printf("PWM Duty cycle set on port %d at %d\n", port_, speed);
        }
    }
}

void Servo::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response)
{
    int param_angle = (int)strtol(params.c_str(), NULL, 10);
    SetAngle(param_angle);
    // if (params == "set_angle")
    // {
    //     SetAngle(angle);
    //     response->success = true;
    //     response->message = "Servo set to 90 degrees";
    // }
    // else if (params == "set_speed")
    // {
    //     SetSpeed(50);
    //     response->success = true;
    //     response->message = "Servo set to 50 speed";
    // }
    // else
    // {
    //     response->success = false;
    //     response->message = "Invalid command";
    // }
}

void Servo::DisplayVMXError(VMXErrorCode vmxerr) {
    const char* err_str = GetVMXErrorString(vmxerr);
    printf("VMX Error %d: %s\n", vmxerr, err_str);
}
