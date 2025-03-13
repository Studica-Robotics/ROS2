#ifndef ULTRASONIC_COMPONENT_H
#define ULTRASONIC_COMPONENT_H

#include <memory>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/string.hpp"

#include "studica_control/srv/set_data.hpp"
#include "ultrasonic.h"
#include "VMXPi.h"

namespace studica_control {
    
class Ultrasonic : public rclcpp::Node {
public:
    explicit Ultrasonic(const rclcpp::NodeOptions &options);
    Ultrasonic(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex ping, VMXChannelIndex echo);
    ~Ultrasonic();
    void cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response);
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response);

private:
    std::shared_ptr<studica_driver::Ultrasonic> ultrasonic_;
    std::shared_ptr<VMXPi> vmx_;
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string name_;
    VMXChannelIndex ping_;
    VMXChannelIndex echo_;
    bool is_publishing_;
    void publish_range();
    void DisplayVMXError(VMXErrorCode vmxerr);
};

} // namespace studica_control

#endif // ULTRASONIC_COMPONENT_H
