#include <stdio.h>
#include <string.h>
#include <memory>
#include "VMXPi.h"

class DIOPin {
public:
    enum class PinMode { INPUT, OUTPUT };

    DIOPin(std::shared_ptr<VMXPi> vmx, VMXChannelIndex channel, PinMode mode)
        : vmx_(vmx), channel_index_(channel), mode_(mode) {
        
        VMXErrorCode vmxerr;
        DIOConfig dio_config;

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
    bool get() {
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
    void set(bool state) {
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

private:
    std::shared_ptr<VMXPi> vmx_;
    VMXChannelIndex channel_index_;
    VMXResourceHandle dio_handle_;
    PinMode mode_;

    void DisplayVMXError(VMXErrorCode vmxerr) {
        const char* err_str = GetVMXErrorString(vmxerr);
        printf("VMX Error %d: %s\n", vmxerr, err_str);
    }
};
int main() {
    std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50); // Initialize VMXPi in realtime mode with 50 Hz update rate
    DIOPin input_pin(vmx, 1, DIOPin::PinMode::INPUT);  // Set channel 1 as input
    DIOPin output_pin(vmx, 2, DIOPin::PinMode::OUTPUT); // Set channel 2 as output

    // Read from input pin
    bool input_state = input_pin.get();
    printf("Input pin state: %d\n", input_state);

    // Set output pin
    output_pin.set(true);  // Set output pin to high
    output_pin.set(false); // Set output pin to low

    return 0;
}
