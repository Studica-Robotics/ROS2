#ifndef SERVO_COMPONENT_H
#define SERVO_COMPONENT_H
#include <stdio.h>
#include <memory>
#include "VMXPi.h"
#include "rclcpp/rclcpp.hpp"
#include "ultrasonic.h"
#include "VMXManager.h"
#include <studica_control/srv/set_data.hpp>
#include <std_msgs/msg/string.hpp>

namespace studica_control
{
    
class Ultrasonic : public rclcpp::Node {
public:
    // Servo(std::shared_ptr<VMXPi> vmx, VMXChannelIndex port, studica_driver::ServoType type, int min = -150, int max = 150);
    // Servo(VMXChannelIndex port, studica_driver::ServoType type, int min = -150, int max = 150);
    // explicit Servo(const rclcpp::NodeOptions &options);
    // ~Servo() override;
    explicit Ultrasonic(const rclcpp::NodeOptions &options);
    Ultrasonic(VMXChannelIndex ping, VMXChannelIndex echo);

private:
    std::shared_ptr<studica_driver::Ultrasonic> ultrasonic_;
    
    std::shared_ptr<VMXPi> vmx_;
    std::string name_;
    VMXChannelIndex ping_;
    VMXChannelIndex echo_;
    void DisplayVMXError(VMXErrorCode vmxerr);
};

} // namespace studica_control

#endif // SERVO_COMPONENT_H