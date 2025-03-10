#ifndef SHARP_COMPONENT_H
#define SHARP_COMPONENT_H

#include <memory>
#include <stdio.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "sharp.h"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {
    
class Sharp : public rclcpp::Node {
public:
    Sharp(const std::string &name, VMXChannelIndex port, std::shared_ptr<VMXPi> vmx);
    explicit Sharp(const rclcpp::NodeOptions &options);
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response);
    void DisplayVMXError(VMXErrorCode vmxerr);

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<studica_driver::Sharp> sharp_;
    std::shared_ptr<VMXPi> vmx_;
    std::string name_;
    VMXChannelIndex port_;
};

} // namespace studica_control

#endif // SHARP_COMPONENT_H
