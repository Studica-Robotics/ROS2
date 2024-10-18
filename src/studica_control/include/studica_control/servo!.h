#ifndef SERVO_H
#define SERVO_H

#include <stdio.h>
#include <memory>
#include "VMXPi.h"


#include <rclcpp/rclcpp.hpp>
#include <studica_control/srv/set_data.hpp>
#include <std_msgs/msg/string.hpp>
#include "studica_control/device.h"

enum class ServoType {
    Standard,
    Continuous,
    Linear
};

class Servo : public Device {
public:
    Servo(std::shared_ptr<VMXPi> vmx, VMXChannelIndex port, ServoType type, int min = -150, int max = 150);
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) override;
    
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

#endif // SERVO_H


