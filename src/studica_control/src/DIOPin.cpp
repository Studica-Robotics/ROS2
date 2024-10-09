#include "studica_control/DIOPin.h"
#include <studica_control/srv/control_imu.hpp>

// Constructor to initialize the DIOPin
DIOPin::DIOPin(std::shared_ptr<VMXPi> vmx, VMXChannelIndex channel, PinMode mode)
    : Device("dpio"), vmx_(vmx), channel_index_(channel), mode_(mode) {

    VMXErrorCode vmxerr;
    DIOConfig dio_config;

    // Configuring the pin based on the mode (OUTPUT or INPUT)
    if (mode == PinMode::OUTPUT) {
        dio_config.SetOutputMode(DIOConfig::OutputMode::PUSHPULL);
        if (!vmx_->io.ActivateSinglechannelResource(VMXChannelInfo(channel_index_, VMXChannelCapability::DigitalOutput), &dio_config, dio_handle_, &vmxerr)) {
            printf("Error activating digital output channel %d\n", channel_index_);
            DisplayVMXError(vmxerr);
        } else {
            printf("Output channel %d activated\n", channel_index_);
        }
    } else {
        if (!vmx_->io.ActivateSinglechannelResource(VMXChannelInfo(channel_index_, VMXChannelCapability::DigitalInput), &dio_config, dio_handle_, &vmxerr)) {
            printf("Error activating digital input channel %d\n", channel_index_);
            DisplayVMXError(vmxerr);
        } else {
            printf("Input channel %d activated\n", channel_index_);
        }
    }
}

// Method to read from input pin
bool DIOPin::get() {
    if (mode_ == PinMode::INPUT) {
        VMXErrorCode vmxerr;
        bool high = false;
        if (!vmx_->io.DIO_Get(dio_handle_, high, &vmxerr)) {
            printf("Error reading digital input\n");
            DisplayVMXError(vmxerr);
            return false;
        }
        return high;
    } else {
        printf("Pin is not configured as input.\n");
        return false;
    }
}

// Method to write to output pin
bool DIOPin::set(bool state) {
    if (mode_ == PinMode::OUTPUT) {
        VMXErrorCode vmxerr;
        if (!vmx_->io.DIO_Set(dio_handle_, state, &vmxerr)) {
            printf("Error setting digital output\n");
            DisplayVMXError(vmxerr);
        }
    } else {
        printf("Pin is not configured as output.\n");
    }
}

void DIOPin::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response)
{
    if (params == "set_hi")
    {
        if(set(true))
        {
            response->success = true;
            response->message = "Pin set to high";
        }
        else
        {
            response->success = false;
            response->message = "Error pin is not configured as output";
        }
    }
    else if (params == "set_lo")
    {
        if(set(false))
        {
            response->success = true;
            response->message = "Pin set to low";
        }
        else
        {
            response->success = false;
            response->message = "Error pin is not configured as output";
        }
    }
    else if (params == "get")
    {
        if (mode_ == PinMode::INPUT)
        {
            bool state = get();
            response->success = true;
            response->message = "Pin state: " + std::to_string(state);
        }
        else
        {
            response->success = false;
            response->message = "Error pin is not configured as input";
        }
    }
    else
    {
        response->success = false;
        response->message = "Unknown command";
    }
}

// Helper method to display VMX Error details
void DIOPin::DisplayVMXError(VMXErrorCode vmxerr) {
    const char* err_str = GetVMXErrorString(vmxerr);
    printf("VMX Error %d: %s\n", vmxerr, err_str);
}
