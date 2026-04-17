/*
 * encoder_component.cpp
 *
 * ros2 component for a quadrature encoder.
 * reads rotation count and direction from two digital input channels
 * on the vmx-pi board. supports multiple encoders by adding entries
 * to the sensors list in params.yaml.
 *
 * topic:   <topic> (studica_control/EncoderMsg)
 *            encoder_count     (int32)  — accumulated tick count since last reset
 *            encoder_direction (string) — "forward" or "reverse"
 *
 * service: encoder_cmd (studica_control/SetData)
 *   params field sets the command. available commands:
 *     get_count     — returns the current encoder count as a string
 *     get_direction — returns the current direction as a string
 */

#include "studica_control/encoder_component.h"

namespace studica_control {


// reads encoder parameters from params.yaml and creates one node per entry
// in the sensors list
std::vector<std::shared_ptr<rclcpp::Node>> Encoder::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    std::vector<std::shared_ptr<rclcpp::Node>> encoder_nodes;

    control->declare_parameter<std::vector<std::string>>("encoder.sensors", std::vector<std::string>{});
    std::vector<std::string> sensor_ids = control->get_parameter("encoder.sensors").as_string_array();

    for (const auto &sensor : sensor_ids) {
        std::string port_a_param = "encoder." + sensor + ".port_a";
        std::string port_b_param = "encoder." + sensor + ".port_b";
        std::string topic_param  = "encoder." + sensor + ".topic";

        control->declare_parameter<int>(port_a_param, -1);
        control->declare_parameter<int>(port_b_param, -1);
        control->declare_parameter<std::string>(topic_param, "unknown");

        int port_a       = control->get_parameter(port_a_param).as_int();
        int port_b       = control->get_parameter(port_b_param).as_int();
        std::string topic = control->get_parameter(topic_param).as_string();

        RCLCPP_INFO(control->get_logger(), "%s -> port_a: %d, port_b: %d, topic: %s",
                    sensor.c_str(), port_a, port_b, topic.c_str());

        auto encoder = std::make_shared<Encoder>(vmx, sensor, port_a, port_b, topic);
        encoder_nodes.push_back(encoder);
    }

    return encoder_nodes;
}


// composable node constructor — used when loading as a plugin
Encoder::Encoder(const rclcpp::NodeOptions & options) : Node("encoder", options) {}


// main constructor — connects to the encoder and sets up the publisher,
// service, and periodic timer
Encoder::Encoder(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex port_a, VMXChannelIndex port_b, const std::string &topic)
    : Node(name), vmx_(vmx), port_a_(port_a), port_b_(port_b) {

    encoder_ = std::make_shared<studica_driver::Encoder>(port_a_, port_b_, vmx_);

    // service for reading encoder values on demand
    service_ = this->create_service<studica_control::srv::SetData>(
        "encoder_cmd",
        std::bind(&Encoder::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    // publishes count and direction at 20hz
    publisher_ = this->create_publisher<studica_control::msg::EncoderMsg>(topic, 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&Encoder::publish_data, this));
}

Encoder::~Encoder() {}


void Encoder::cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request,
                           std::shared_ptr<studica_control::srv::SetData::Response> response) {
    cmd(request->params, response);
}


// dispatches incoming service commands
void Encoder::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (params == "get_count") {
        response->success = true;
        response->message = std::to_string(encoder_->GetCount());

    } else if (params == "get_direction") {
        response->success = true;
        response->message = encoder_->GetDirection();

    } else {
        response->success = false;
        response->message = "unknown command '" + params + "'";
    }
}


// reads count and direction from the encoder and publishes them
void Encoder::publish_data() {
    auto msg = studica_control::msg::EncoderMsg();
    msg.encoder_count     = encoder_->GetCount();
    msg.encoder_direction = encoder_->GetDirection();
    publisher_->publish(msg);
}


void Encoder::DisplayVMXError(VMXErrorCode vmxerr) {
    printf("vmx error %d: %s\n", vmxerr, GetVMXErrorString(vmxerr));
}


} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Encoder)
