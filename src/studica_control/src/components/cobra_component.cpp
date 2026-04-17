/*
 * cobra_component.cpp
 *
 * ros2 component for the studica cobra reflectance sensor array.
 * the cobra is a 4-channel analog sensor that measures how much infrared
 * light is reflected off a surface. each channel outputs a voltage between
 * 0 and vref — low voltage over dark surfaces, high voltage over light ones.
 * this makes it well suited for line following and surface detection.
 *
 * topic:   <topic> (std_msgs/Float32MultiArray)
 *            voltage for all 4 channels, published at 20hz.
 *            data[0..3] = voltage in volts for channels 0–3
 *
 * service: cobra_cmd (studica_control/SetData)
 *   params field sets the command. available commands:
 *     get_raw <channel>     — returns the raw adc value for one channel (0–3)
 *     get_voltage <channel> — returns the voltage in volts for one channel (0–3)
 */

#include "studica_control/cobra_component.h"

namespace studica_control {


// reads cobra parameters from params.yaml and creates one node per entry
// in the sensors list
std::vector<std::shared_ptr<rclcpp::Node>> Cobra::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    std::vector<std::shared_ptr<rclcpp::Node>> cobra_nodes;

    control->declare_parameter<std::vector<std::string>>("cobra.sensors", std::vector<std::string>{});
    std::vector<std::string> sensor_ids = control->get_parameter("cobra.sensors").as_string_array();

    for (const auto &sensor : sensor_ids) {
        std::string vref_param  = "cobra." + sensor + ".vref";
        std::string topic_param = "cobra." + sensor + ".topic";

        control->declare_parameter<float>(vref_param, -1.0);
        control->declare_parameter<std::string>(topic_param, "unknown");

        float vref        = control->get_parameter(vref_param).get_value<float>();
        std::string topic = control->get_parameter(topic_param).as_string();

        RCLCPP_INFO(control->get_logger(), "%s -> vref: %f, topic: %s",
                    sensor.c_str(), vref, topic.c_str());

        auto cobra = std::make_shared<Cobra>(vmx, sensor, vref, topic);
        cobra_nodes.push_back(cobra);
    }

    return cobra_nodes;
}


// composable node constructor — used when loading as a plugin
Cobra::Cobra(const rclcpp::NodeOptions &options) : Node("cobra", options) {}


// main constructor — connects to the cobra sensor and sets up the publisher,
// service, and periodic timer
Cobra::Cobra(std::shared_ptr<VMXPi> vmx, const std::string &name, const float &vref, const std::string &topic)
    : Node(name), vmx_(vmx), vref_(vref) {

    cobra_ = std::make_shared<studica_driver::Cobra>(vmx_, vref_);

    // service for reading individual channel values on demand
    service_ = this->create_service<studica_control::srv::SetData>(
        "cobra_cmd",
        std::bind(&Cobra::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    // publishes all 4 channel voltages at 20hz
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(topic, 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&Cobra::publish_data, this));
}

Cobra::~Cobra() {}


void Cobra::cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request,
                         std::shared_ptr<studica_control::srv::SetData::Response> response) {
    cmd(request->params, response);
}


// dispatches incoming service commands. the params field should be
// "get_raw <channel>" or "get_voltage <channel>" where channel is 0–3.
void Cobra::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    try {
        if (params.rfind("get_raw", 0) == 0) {
            uint8_t channel = static_cast<uint8_t>(std::stoi(params.substr(8)));
            response->success = true;
            response->message = std::to_string(cobra_->GetRawValue(channel));

        } else if (params.rfind("get_voltage", 0) == 0) {
            uint8_t channel = static_cast<uint8_t>(std::stoi(params.substr(12)));
            response->success = true;
            response->message = std::to_string(cobra_->GetVoltage(channel));

        } else {
            response->success = false;
            response->message = "unknown command '" + params + "' — use 'get_raw <ch>' or 'get_voltage <ch>'";
        }
    } catch (const std::exception &e) {
        response->success = false;
        response->message = "invalid channel in command '" + params + "': " + std::string(e.what());
    }
}


// reads voltage from all 4 channels and publishes them as an array
void Cobra::publish_data() {
    std_msgs::msg::Float32MultiArray msg;
    msg.data.resize(4);

    for (uint8_t i = 0; i < 4; i++) {
        msg.data[i] = cobra_->GetVoltage(i);
    }

    publisher_->publish(msg);
}


} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Cobra)
