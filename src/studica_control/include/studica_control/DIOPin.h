#ifndef DIO_PIN_H
#define DIO_PIN_H

#include <stdio.h>
#include <memory>
#include "VMXPi.h"


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <studica_control/srv/set_data.hpp>
#include <std_msgs/msg/string.hpp>
#include "studica_control/device.h"

class DIOPin : public Device {
public:
    // Enum to define pin mode as Input or Output
    enum class PinMode { INPUT, OUTPUT };
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) override;

    // Constructor to initialize the DIOPin
    DIOPin(std::shared_ptr<VMXPi> vmx, VMXChannelIndex channel, PinMode mode);

    // Method to read from an input pin
    bool get();

    // Method to write to an output pin
    bool set(bool state);

private:
    std::shared_ptr<VMXPi> vmx_;            // VMXPi shared pointer instance
    VMXChannelIndex channel_index_;          // Channel Index of the pin
    VMXResourceHandle dio_handle_;           // Handle to the DIO resource
    PinMode mode_;                           // Pin mode (INPUT or OUTPUT)

    // Helper method to display error information
    void DisplayVMXError(VMXErrorCode vmxerr);
};

#endif // DIO_PIN_H
