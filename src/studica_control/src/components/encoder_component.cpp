#include "studica_control/encoder_component.h"

namespace studica_control {

std::vector<std::shared_ptr<rclcpp::Node>> Encoder::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    std::vector<std::shared_ptr<rclcpp::Node>> encoder_nodes;
    control->declare_parameter<std::vector<std::string>>("encoder.sensors", std::vector<std::string>{});
    std::vector<std::string> sensor_ids = control->get_parameter("encoder.sensors").as_string_array();
    for (const auto &sensor : sensor_ids) {
        std::string port_a_param = "encoder." + sensor + ".port_a";
        std::string port_b_param = "encoder." + sensor + ".port_b";
        std::string topic_param = "encoder." + sensor + ".topic";

        control->declare_parameter<int>(port_a_param, -1);
        control->declare_parameter<int>(port_b_param, -1);
        control->declare_parameter<std::string>(topic_param, "unknown");

        int port_a = control->get_parameter(port_a_param).as_int();
        int port_b = control->get_parameter(port_b_param).as_int();
        std::string topic = control->get_parameter(topic_param).as_string();

        RCLCPP_INFO(control->get_logger(), "%s -> port_a: %d, port_b: %d, topic: %s", sensor.c_str(), port_a, port_b, topic.c_str());

        auto encoder = std::make_shared<Encoder>(vmx, sensor, port_a, port_b, topic);
        encoder_nodes.push_back(encoder);
    }
    return encoder_nodes;
}

Encoder::Encoder(const rclcpp::NodeOptions & options) : Node("encoder", options) {}

Encoder::Encoder(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex port_a, VMXChannelIndex port_b, const std::string &topic) 
    : Node(name), vmx_(vmx), port_a_(port_a), port_b_(port_b) {
    encoder_ = std::make_shared<studica_driver::Encoder>(port_a_, port_b_, vmx_);
    service_ = this->create_service<studica_control::srv::SetData>(
        "encoder_cmd",
        std::bind(&Encoder::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    publisher_ = this->create_publisher<studica_control::msg::EncoderMsg>(topic, 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&Encoder::publish_data, this));
}

Encoder::~Encoder() {}

void Encoder::cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    std::string params = request->params;
    cmd(params, response);
}

void Encoder::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (params == "get_count") {
        response->success = true;
        response->message = std::to_string(encoder_->GetCount());
    } else if (params == "get_direction") {
        response->success = true;
        response->message = encoder_->GetDirection();
    } else {
        response->success = false;
        response->message = "No such command '" + params + "'";
    }
}

void Encoder::publish_data() {
    auto msg = studica_control::msg::EncoderMsg();
    msg.encoder_count = encoder_->GetCount();
    msg.encoder_direction = encoder_->GetDirection();
    publisher_->publish(msg);
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Encoder)
