#ifndef ANALOG_COMPONENT_H
#define ANALOG_COMPONENT_H
#include <stdio.h>
#include <memory>
#include "VMXPi.h"
#include "rclcpp/rclcpp.hpp"
#include "analog.h"
// Messages
#include <studica_control/srv/set_data.hpp>
#include <std_msgs/msg/string.hpp>

namespace studica_control
{

class Analog : public rclcpp::Node {
public:
    explicit Analog(const rclcpp::NodeOptions &options);
    void Initialize(VMXChannelIndex channel);
    void DisplayVMXError(VMXErrorCode vmxerr);
private:
    std::shared_ptr<studica_driver::Analog> analog_;
    VMXChannelIndex channel_;
    std::shared_ptr<VMXPi> vmx_;
};

} // namespace studica_control

#endif // ANALOG_COMPONENT_H