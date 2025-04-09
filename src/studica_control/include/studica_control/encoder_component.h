#ifndef ENCODER_COMPONENT_H
#define ENCODER_COMPONENT_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "encoder.h"
#include "studica_control/msg/encoder_msg.hpp"
#include "studica_control/srv/set_data.hpp"
#include "VMXPi.h"

namespace studica_control {
    
class Encoder : public rclcpp::Node {
public:
    static std::vector<std::shared_ptr<rclcpp::Node>> initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx);
    explicit Encoder(const rclcpp::NodeOptions &options);
    Encoder(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex port_a, VMXChannelIndex port_b, const std::string &topic);
    ~Encoder();
    
private:
    std::shared_ptr<VMXPi> vmx_;
    std::shared_ptr<studica_driver::Encoder> encoder_;
    rclcpp::Service<studica_control::srv::SetData>::SharedPtr service_;
    rclcpp::Publisher<studica_control::msg::EncoderMsg>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    VMXChannelIndex port_a_;
    VMXChannelIndex port_b_;
    void cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response);
    void cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response);
    void publish_data();
    void DisplayVMXError(VMXErrorCode vmxerr);
};

} // namespace studica_control

#endif // ENCODER_COMPONENT_H
