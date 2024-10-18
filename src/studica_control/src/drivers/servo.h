#ifndef SERVO_H
#define SERVO_H

#include <stdio.h>
#include "VMXPi.h"

namespace studica_driver {

enum class ServoType {
    Standard,
    Continuous,
    Linear
};

class Servo {
public:
    Servo(VMXChannelIndex port, ServoType type, int min = -150, int max = 150);
    Servo(VMXChannelIndex port, ServoType type, int min = -150, int max = 150, std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50));

    void SetBounds(double min, double center, double max);
    void SetAngle(int angle);
    void SetSpeed(int speed);

private:
    std::shared_ptr<VMXPi> vmx_;
    VMXChannelIndex port_;
    VMXResourceHandle pwm_res_handle_;
    int min_;
    int max_;
    int min_us_;
    int max_us_;
    int center_us_;
    int prev_pwm_servo_value_;
    ServoType type_;

    int Map(int value);
    void DisplayVMXError(VMXErrorCode vmxerr);
};

}

#endif // SERVO_H
