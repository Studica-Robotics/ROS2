#include "servo.h"
using namespace studica_driver;

Servo::Servo(VMXChannelIndex port, ServoType type, int min, int max, std::shared_ptr<VMXPi> vmx)
    : PWM(port, PWMType::Standard, min, max, vmx)
    , prev_pwm_servo_value_(min - 1)
{

    SetBounds(0.5, 1.5, 2.5);
    if (port <= 21)
    {
        printf("Studica Driver: Successfully initialized port %d as a servo output\n", port);
    }
    else
    {
        printf("Studica Driver: Port %d is not a valid servo port!\n", port);
    }
}

Servo::~Servo()
{
    printf("Studica Driver: Servo on port %d deconstructed.\n", port_);
}

void Servo::SetBounds(double min, double center, double max)
{
    PWM::SetBounds(min, center, max);
}

void Servo::SetAngle(int angle)
{
    if (prev_pwm_servo_value_ != angle)
    {
        VMXErrorCode vmxerr;
        bool success = vmx_->io.PWMGenerator_SetDutyCycle(pwm_res_handle_, port_, Map(angle), &vmxerr);
        if (!success)
        {
            printf("Studica Driver: Failed to set duty cycle for servo on port %d\n", port_);
            DisplayVMXError(vmxerr);
        }
        else
        {
            prev_pwm_servo_value_ = angle;
            printf("Studica Driver: PWM Duty cycle set on port %d at %d\n", port_, angle);
        }
    }
}

void Servo::SetSpeed(int speed)
{
    if (prev_pwm_servo_value_ != speed)
    {
        VMXErrorCode vmxerr;
        bool success = vmx_->io.PWMGenerator_SetDutyCycle(pwm_res_handle_, port_, Map(speed), &vmxerr);
        prev_pwm_servo_value_ = speed;
        if (!success)
        {
            printf("Studica Driver: Failed to set duty cycle for servo on port %d\n", port_);
            DisplayVMXError(vmxerr);
        }
        else
        {
            printf("Studica Driver: PWM Duty cycle set on port %d at %d\n", port_, speed);
        }
    }
}

float Servo::GetLastAngle()
{
    return prev_pwm_servo_value_;
}

void Servo::DisplayVMXError(VMXErrorCode vmxerr)
{
    const char* p_err_description = GetVMXErrorString(vmxerr);
    printf("Studica Driver: VMXError %d: %s\n", vmxerr, p_err_description);
}
