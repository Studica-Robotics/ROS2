#include "studica_control/cobra_component.h"

namespace studica_control
{

Cobra::Cobra(const rclcpp::NodeOptions & options) : Node("cobra", options) {
    Cobra("cobra", 5.0, 0, std::make_shared<VMXPi>(true, 50));
}
Cobra::Cobra(const std::string &name, const float& vref, const int& muxch, std::shared_ptr<VMXPi> vmx)
    : rclcpp::Node(name), vmx_(vmx), name_(name), vref_(vref), muxch_(muxch) {
    cobra_ = std::make_shared<studica_driver::Cobra>(vref_, vmx_);
}


void Cobra::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (params == "get_raw") {
        response->success = true;
        response->message = std::to_string(cobra_->GetRawValue(muxch_));
    } else if (params == "get_volt") {
        response->success = true;
        response->message = std::to_string(cobra_->GetVoltage(muxch_));
    } else {
        response->success = false;
        response->message = "No such command '" + params + "'";
    }
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Cobra)
