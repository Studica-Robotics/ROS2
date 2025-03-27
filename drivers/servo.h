#ifndef SERVO_H
#define SERVO_H

#include <stdio.h>
#include <memory>
#include "VMXPi.h"
#include "pwm.h"

namespace studica_driver {

enum class ServoType {
    Standard,
    Continuous,
    Linear
};

class Servo : public PWM {
public:
    Servo(VMXChannelIndex port, ServoType type, int min = -150, int max = 150, 
          std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50));
    ~Servo();
    void SetBounds(double min, double center, double max);
    void SetAngle(int angle);
    void SetSpeed(int speed);
    float GetLastAngle();

private:
    int prev_pwm_servo_value_;
};

}

#endif // SERVO_H
