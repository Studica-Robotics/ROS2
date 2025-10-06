#include "studica_control/dc_encoder_component.h"

namespace studica_control {

std::vector<std::shared_ptr<rclcpp::Node>> DutyCycleEncoder::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    std::vector<std::shared_ptr<rclcpp::Node>> dc_nodes;
    control->declare_parameter<std::vector<std::string>>("duty_cycle.sensors", std::vector<std::string>{});
    std::vector<std::string> sensor_ids = control->get_parameter("duty_cycle.sensors").as_string_array();
    
    for (const auto &sensor : sensor_ids) {
        std::string port_param = "duty_cycle." + sensor + ".port";
        std::string topic_param = "duty_cycle." + sensor + ".topic";

        control->declare_parameter<int>(port_param, -1);
        control->declare_parameter<std::string>(topic_param, "unknown");

        int port = control->get_parameter(port_param).as_int();
        std::string topic = control->get_parameter(topic_param).as_string();

        RCLCPP_INFO(control->get_logger(), "%s -> port: %d, topic: %s", sensor.c_str(), port, topic.c_str());

        auto dc = std::make_shared<DutyCycleEncoder>(vmx, sensor, port, topic);
        dc_nodes.push_back(dc);
    }
    return dc_nodes;
}

DutyCycleEncoder::DutyCycleEncoder(const rclcpp::NodeOptions &options) : Node("duty_cycle_encoder", options) {}

DutyCycleEncoder::DutyCycleEncoder(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex port, const std::string &topic) 
    : rclcpp::Node(name), vmx_(vmx), port_(port) {
    duty_cycle_encoder_ = std::make_shared<studica_driver::DutyCycleEncoder>(port_, vmx_);
    service_ = this->create_service<studica_control::srv::SetData>(
        "duty_cycle_encoder_cmd",
        std::bind(&DutyCycleEncoder::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    publisher_ = this->create_publisher<studica_control::msg::DutyCycleEncoderMsg>(topic, 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&DutyCycleEncoder::publish_data, this));
}

DutyCycleEncoder::~DutyCycleEncoder() {}

void DutyCycleEncoder::cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
    std::shared_ptr<studica_control::srv::SetData::Response> response) {
        std::string params = request->params;
        cmd(params, response);
    }

void DutyCycleEncoder::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (params == "get_absolute_position") {
        response->success = true;
        response->message = std::to_string(duty_cycle_encoder_->GetAbsolutePosition());
    } else if (params == "get_rollover_count") {
        response->success = true;
        response->message = std::to_string(duty_cycle_encoder_->GetRolloverCount());
    } else if (params == "get_total_rotation") {
        response->success = true;
        response->message = std::to_string(duty_cycle_encoder_->GetTotalRotation());
    } else {
        response->success = false;
        response->message = "No such command '" + params + "'";
    }
}

void DutyCycleEncoder::publish_data() {
    auto msg = studica_control::msg::DutyCycleEncoderMsg();
    msg.absolute_angle = duty_cycle_encoder_->GetAbsolutePosition();
    msg.rollover_count = duty_cycle_encoder_->GetRolloverCount();
    msg.total_rotation = duty_cycle_encoder_->GetTotalRotation();
    publisher_->publish(msg);
}

void DutyCycleEncoder::DisplayVMXError(VMXErrorCode vmxerr) {
    printf("VMX Error %d: %s\n", vmxerr, GetVMXErrorString(vmxerr));
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::DutyCycleEncoder)
