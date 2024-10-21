#ifndef PWM_H
#define PWM_H

#include <stdio.h>
#include <memory>
#include "VMXPi.h"


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <studica_control/srv/set_data.hpp>
#include <std_msgs/msg/string.hpp>
#include "studica_control/device.h"

class PWM : public Device {
public:
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) override;

    PWM(std::shared_ptr<VMXPi> vmx, VMXChannelIndex channel, uint16_t pin);

    bool set(bool state);

private:
    std::shared_ptr<VMXPi> vmx_;            // VMXPi shared pointer instance
    VMXChannelIndex channel_index_;          // Channel Index of the pin
    VMXResourceHandle dio_handle_;           // Handle to the DIO resource
    uint16_t pin_;                           // Pin mode (INPUT or OUTPUT)

    // Helper method to display error information
    void DisplayVMXError(VMXErrorCode vmxerr);
};

#endif // PWM_H

void PWM::DisplayVMXError(VMXErrorCode vmxerr) {
    const char *p_err_description = GetVMXErrorString(vmxerr);
    printf("VMXError %d:  %s\n", vmxerr, p_err_description);
}

void PWM::PWM(std::shared_ptr<VMXPi> vmx, VMXChannelIndex channel, uint16_t pin) {
    vmx_ = vmx;
    channel_index_ = channel;
    pin_ = pin;

    // Get the resource handle for the DIO channel
    if (!vmx_->io.GetResourceHandle(VMXResourceType::DIO, channel_index_, dio_handle_, &vmxerr)) {
        DisplayVMXError(vmxerr);
    }
}

void PWM::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (params == "on") {
        if (set(true)) {
            response->message = "PWM turned on";
        } else {
            response->message = "Failed to turn on PWM";
        }
    } else if (params == "off") {
        if (set(false)) {
            response->message = "PWM turned off";
        } else {
            response->message = "Failed to turn off PWM";
        }
    } else {
        response->message = "Invalid parameter";
    }
}