#include "studica_control/cobra_component.h"

namespace studica_control {

Cobra::Cobra(const rclcpp::NodeOptions & options) : Node("cobra", options) {}

Cobra::Cobra(std::shared_ptr<VMXPi> vmx, const std::string &name, const float& vref)
    : Node(name), vmx_(vmx), name_(name), vref_(vref) {
    cobra_ = std::make_shared<studica_driver::Cobra>(vmx_, vref_);
    service_ = this->create_service<studica_control::srv::SetData>(
        "cobra_cmd",
        std::bind(&Cobra::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
}

Cobra::~Cobra() {}

void Cobra::cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    std::string params = request->params;
    cmd(params, response);
}

void Cobra::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (params == "get_raw") {
        response->success = true;
        response->message = std::to_string(cobra_->GetRawValue(1));
    } else if (params == "get_voltage") {
        response->success = true;
        response->message = std::to_string(cobra_->GetVoltage(1));
    } else {
        response->success = false;
        response->message = "No such command '" + params + "'";
    }
}

} // namespace studica_control

