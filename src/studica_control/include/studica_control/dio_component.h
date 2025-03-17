#ifndef DIO_COMPONENT_H
#define DIO_COMPONENT_H

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "dio.h"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {

class DIO : public rclcpp::Node {
public:
    explicit DIO(const rclcpp::NodeOptions &options);
    DIO(std::shared_ptr<VMXPi> vmx, VMXChannelIndex pin, studica_driver::PinMode pin_mode, std::string type);
    ~DIO();

private:
    std::shared_ptr<studica_driver::DIO> dio_; 
    std::shared_ptr<VMXPi> vmx_;                                       
    VMXChannelIndex pin_; // Pin index for the DIO component
    studica_driver::PinMode pin_mode_; // Pin mode for the DIO component
    int btn_pin;                                    
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
        std::shared_ptr<studica_control::srv::SetData::Response> response);
    void publish_dio_state();
    void DisplayVMXError(VMXErrorCode vmxerr);
};

} // namespace studica_control

#endif // DIO_COMPONENT_H
