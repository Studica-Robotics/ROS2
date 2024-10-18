#include <stdio.h>
#include <memory>
#include "VMXPi.h"
#include <rclcpp/rclcpp.hpp>
#include "encoder.h"
// Messages
#include <sensor_msgs/msg/imu.hpp>
#include <studica_control/srv/set_data.hpp>
#include <std_msgs/msg/string.hpp>

#include "studica_control/device.h"

namespace studica_control
{
    
class EncoderNode : public Device {
private:
    std::shared_ptr<Encoder> encoder_;
    std::shared_ptr<VMXPi> vmx_;
    std::string name_;
    VMXChannelIndex port_a_;
    VMXChannelIndex port_b_;
public:
    EncoderNode(const std::string &name, VMXChannelIndex port_a, VMXChannelIndex port_b, std::shared_ptr<VMXPi> vmx);
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response);
};

} // namespace studica_control