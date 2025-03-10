#ifndef ENCODER_COMPONENT_H
#define ENCODER_COMPONENT_H

#include <memory>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "encoder.h"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {
    
class Encoder : public rclcpp::Node {
public:
    explicit Encoder(const rclcpp::NodeOptions &options);
    Encoder(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex port_a, VMXChannelIndex port_b);
    ~Encoder();
    void cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response);
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response);
    void DisplayVMXError(VMXErrorCode vmxerr);
    
private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<studica_driver::Encoder> encoder_;
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    std::shared_ptr<VMXPi> vmx_;
    std::string name_;
    VMXChannelIndex port_a_;
    VMXChannelIndex port_b_;
};

} // namespace studica_control

#endif // ENCODER_COMPONENT_H
