#ifndef DIO_COMPONENT_H
#define DIO_COMPONENT_H
#include <stdio.h>
#include <memory>
#include "VMXPi.h"
#include "rclcpp/rclcpp.hpp"
#include "dio.h"
// Messages
#include <studica_control/srv/set_data.hpp>
#include <std_msgs/msg/string.hpp>

namespace studica_control
{

class DIO : public rclcpp::Node {
public:
    explicit DIO(const rclcpp::NodeOptions &options);
    void Initialize(VMXChannelIndex pin, PinMode pin_mode);
    void DisplayVMXError(VMXErrorCode vmxerr);
private:
    std::shared_ptr<studica_driver::DIO> dio_;
    VMXChannelIndex pin_;
    studica_driver::PinMode pin_mode_;
    std::shared_ptr<VMXPi> vmx_;
};

} // namespace studica_control

#endif // DIO_COMPONENT_H