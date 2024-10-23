#ifndef PWM_H
#define PWM_H

#include <stdio.h>
#include <memory>
#include "VMXPi.h"

namespace studica_driver {

enum class PWMType {
    Standard,
    Continuous,
    Linear
};

class PWM {
public:
    PWM(VMXChannelIndex port, PWMType type, int min = -150, int max = 150, std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50));
    ~PWM();
    void SetBounds(double min, double center, double max);
    void SetAngle(int angle);
    void SetSpeed(int speed);

private:
    VMXChannelIndex port_;
    PWMType type_;
    int min_;
    int max_;
    std::shared_ptr<VMXPi> vmx_;
    VMXResourceHandle pwm_res_handle_;
    int min_us_;
    int max_us_;
    int center_us_;
    int prev_pwm_pwm_value_;

    int Map(int value);
    void DisplayVMXError(VMXErrorCode vmxerr);
};

}

#endif // PWM_H
