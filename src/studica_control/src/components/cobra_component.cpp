/*
 * cobra_component.cpp
 *
 * ros2 component for the studica cobra reflectance sensor array.
 * publishes per-channel voltage on individual topics at 20hz.
 *
 * topics (publish): <name>/ch_0 ... <name>/ch_3 (std_msgs/Float32), 20hz
 * service: <name>/cobra_cmd (studica_control/SetData)
 *   initparams.n_encoder — channel (0–3)
 *   commands: get_raw, get_voltage
 */

#include "studica_control/cobra_component.hpp"

namespace studica_control {


std::vector<std::shared_ptr<rclcpp::Node>> Cobra::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    std::vector<std::shared_ptr<rclcpp::Node>> cobra_nodes;

    control->declare_parameter<std::vector<std::string>>("cobra.sensors", std::vector<std::string>{});
    std::vector<std::string> sensor_ids = control->get_parameter("cobra.sensors").as_string_array();

    for (const auto &sensor : sensor_ids) {
        std::string vref_param = "cobra." + sensor + ".vref";
        control->declare_parameter<float>(vref_param, 5.0f);
        float vref = control->get_parameter(vref_param).get_value<float>();

        RCLCPP_INFO(control->get_logger(), "%s -> vref: %.1f V", sensor.c_str(), vref);

        cobra_nodes.push_back(std::make_shared<Cobra>(vmx, sensor, vref));
    }

    return cobra_nodes;
}


Cobra::Cobra(const rclcpp::NodeOptions &options) : Node("cobra", options) {}


Cobra::Cobra(std::shared_ptr<VMXPi> vmx, const std::string &name, const float &vref)
    : Node(name), vmx_(vmx), vref_(vref) {

    cobra_ = std::make_shared<studica_driver::Cobra>(vmx_, vref_);

    service_ = this->create_service<studica_control::srv::SetData>(
        name + "/cobra_cmd",
        std::bind(&Cobra::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    for (int i = 0; i < 4; i++) {
        channel_pubs_[i] = this->create_publisher<std_msgs::msg::Float32>(
            name + "/ch_" + std::to_string(i), 10);
    }

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&Cobra::publish_data, this));

    RCLCPP_INFO(this->get_logger(), "cobra ready. topics: /%s/ch_0 ... /%s/ch_3",
                name.c_str(), name.c_str());
}

Cobra::~Cobra() {}


void Cobra::cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request,
                         std::shared_ptr<studica_control::srv::SetData::Response> response) {
    uint8_t channel = static_cast<uint8_t>(request->initparams.n_encoder);
    if (channel > 3) {
        response->success = false;
        response->message = "invalid channel " + std::to_string(channel) + " — must be 0–3";
        return;
    }

    if (request->params == "get_raw") {
        response->success = true;
        response->message = std::to_string(cobra_->GetRawValue(channel));

    } else if (request->params == "get_voltage") {
        response->success = true;
        response->message = std::to_string(cobra_->GetVoltage(channel));

    } else {
        response->success = false;
        response->message = "unknown command '" + request->params + "' — use 'get_raw' or 'get_voltage'";
    }
}


void Cobra::publish_data() {
    for (int i = 0; i < 4; i++) {
        std_msgs::msg::Float32 msg;
        msg.data = cobra_->GetVoltage(static_cast<uint8_t>(i));
        channel_pubs_[i]->publish(msg);
    }
}


} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Cobra)
