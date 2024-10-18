#ifndef ENCODER_COMPONENT_H
#define ENCODER_COMPONENT_H
#include <stdio.h>
#include <memory>
#include "VMXPi.h"
#include "rclcpp/rclcpp.hpp"
#include "encoder.h"
// Messages
#include <studica_control/srv/set_data.hpp>
#include <std_msgs/msg/string.hpp>

namespace studica_control
{
    
class Encoder : public rclcpp::Node {
private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<studica_driver::Encoder> encoder_;
    std::shared_ptr<VMXPi> vmx_;
    std::string name_;
    VMXChannelIndex port_a_;
    VMXChannelIndex port_b_;
public:
    Encoder(const std::string &name, VMXChannelIndex port_a, VMXChannelIndex port_b, std::shared_ptr<VMXPi> vmx);
    explicit Encoder(const rclcpp::NodeOptions &options);
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response);
};

} // namespace studica_control

#endif // ENCODER_COMPONENT_H