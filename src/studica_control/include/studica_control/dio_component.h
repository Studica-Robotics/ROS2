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
    static std::vector<std::shared_ptr<rclcpp::Node>> initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);
    explicit DIO(const rclcpp::NodeOptions &options);
    DIO(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex pin, studica_driver::PinMode pin_mode, const std::string &topic);
    ~DIO();

private:
    std::shared_ptr<studica_driver::DIO> dio_; 
    std::shared_ptr<VMXPi> vmx_;                                       
    VMXChannelIndex pin_;
    studica_driver::PinMode pin_mode_;
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
