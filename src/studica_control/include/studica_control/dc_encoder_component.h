#ifndef DC_ENCODER_COMPONENT_H
#define DC_ENCODER_COMPONENT_H

#include "rclcpp/rclcpp.hpp"

#include "duty_cycle_encoder.h"
#include "studica_control/msg/duty_cycle_encoder_msg.hpp"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {

class DutyCycleEncoder : public rclcpp::Node {
public:
    static std::vector<std::shared_ptr<rclcpp::Node>> initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);
    explicit DutyCycleEncoder(const rclcpp::NodeOptions &options);
    DutyCycleEncoder(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex port, const std::string &topic);
    ~DutyCycleEncoder();

private:
    std::shared_ptr<studica_driver::DutyCycleEncoder> duty_cycle_encoder_;
    std::shared_ptr<VMXPi> vmx_;
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::Publisher<studica_control::msg::DutyCycleEncoderMsg>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    VMXChannelIndex port_;
    void cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
        std::shared_ptr<studica_control::srv::SetData::Response> response);
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response);
    void publish_data();
    void DisplayVMXError(VMXErrorCode vmxerr);
};

} // namespace studica_control

#endif // DC_ENCODER_COMPONENT_H
