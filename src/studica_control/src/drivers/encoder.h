#ifndef ENCODER_H
#define ENCODER_H

#include <stdio.h>
#include <memory>
#include "VMXPi.h"
#include <rclcpp/rclcpp.hpp>
#include <studica_control/srv/set_data.hpp>
#include <std_msgs/msg/string.hpp>
#include "studica_control/device.h"

namespace studica_control {

class Encoder : public Device {
public:
    Encoder(std::shared_ptr<VMXPi> vmx, VMXChannelIndex port_a, VMXChannelIndex port_b);
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) override;

    int GetCount();
    std::string GetDirection();

private:
    std::shared_ptr<VMXPi> vmx_;
    VMXChannelIndex port_a_;
    VMXChannelIndex port_b_;
    VMXResourceHandle encoder_res_handle_;
    
    void DisplayVMXError(VMXErrorCode vmxerr);
};

}

#endif // ENCODER_H
