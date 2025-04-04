#ifndef SERVO_COMPONENT_H
#define SERVO_COMPONENT_H

#include <memory>
#include <stdio.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "servo.h"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {

class Servo : public rclcpp::Node {
public:
    explicit Servo(const rclcpp::NodeOptions &options);
    Servo(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex port, studica_driver::ServoType type, int min = -150, int max = 150);
    ~Servo();

private:
    void cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                      std::shared_ptr<studica_control::srv::SetData::Response> response);

    std::shared_ptr<VMXPi> vmx_;
    std::shared_ptr<studica_driver::Servo> servo_;

    std::string name_;
    VMXChannelIndex port_;
    studica_driver::ServoType type_;

    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;

    void DisplayVMXError(VMXErrorCode vmxerr);
};

} // namespace studica_control

#endif // SERVO_COMPONENT_H
