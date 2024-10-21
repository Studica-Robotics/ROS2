#ifndef SERVO_COMPONENT_H
#define SERVO_COMPONENT_H
#include <stdio.h>
#include <memory>
#include "VMXPi.h"
#include "rclcpp/rclcpp.hpp"
#include "servo.h"
#include "VMXManager.h"
// Messages
#include <studica_control/srv/set_data.hpp>
#include <std_msgs/msg/string.hpp>

namespace studica_control
{
    
class Servo : public rclcpp::Node {
public:
    // Servo(std::shared_ptr<VMXPi> vmx, VMXChannelIndex port, studica_driver::ServoType type, int min = -150, int max = 150);
    Servo(VMXChannelIndex port, studica_driver::ServoType type, int min = -150, int max = 150);
    explicit Servo(const rclcpp::NodeOptions &options);
    ~Servo() override;
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response);
    int Map(int value);
    void SetAngle(int angle);
    void SetSpeed(int speed);
    void SetBounds(double min, double center, double max);

private:
    std::shared_ptr<studica_driver::Servo> servo_;
    // vmxmanager_
    std::shared_ptr<VMXPi> vmx_;
    std::string name_;
    VMXChannelIndex port_;
    studica_driver::ServoType type_;
    void DisplayVMXError(VMXErrorCode vmxerr);
};

} // namespace studica_control

#endif // SERVO_COMPONENT_H