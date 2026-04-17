/*
 * dio_component.cpp
 *
 * ros2 component for a digital input/output pin on the vmx-pi board.
 * a pin can be set to either input mode (reads a signal) or output mode
 * (drives a signal). the mode is set in params.yaml and cannot change at
 * runtime. supports multiple pins by adding entries to the sensors list.
 *
 * topic:   <topic> (std_msgs/Bool)
 *            current pin state — true = high, false = low, published at 10hz
 *
 * service: dio_cmd (studica_control/SetData)
 *   params field sets the command. available commands:
 *     toggle — flip the pin state (output mode only)
 */

#include "studica_control/dio_component.h"

namespace studica_control {


// reads dio parameters from params.yaml and creates one node per entry
// in the sensors list. validates the type field and logs a warning for
// invalid values.
std::vector<std::shared_ptr<rclcpp::Node>> DIO::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    std::vector<std::shared_ptr<rclcpp::Node>> dio_nodes;

    control->declare_parameter<std::vector<std::string>>("dio.sensors", std::vector<std::string>{});
    std::vector<std::string> sensor_ids = control->get_parameter("dio.sensors").as_string_array();

    for (const auto &sensor : sensor_ids) {
        std::string pin_param   = "dio." + sensor + ".pin";
        std::string type_param  = "dio." + sensor + ".type";
        std::string topic_param = "dio." + sensor + ".topic";

        control->declare_parameter<int>(pin_param, -1);
        control->declare_parameter<std::string>(type_param, "");
        control->declare_parameter<std::string>(topic_param, "");

        int pin           = control->get_parameter(pin_param).as_int();
        std::string type  = control->get_parameter(type_param).as_string();
        std::string topic = control->get_parameter(topic_param).as_string();

        RCLCPP_INFO(control->get_logger(), "%s -> pin: %d, type: %s, topic: %s",
                    sensor.c_str(), pin, type.c_str(), topic.c_str());

        if (type == "input") {
            auto dio = std::make_shared<DIO>(vmx, sensor, pin, studica_driver::PinMode::INPUT, topic);
            dio_nodes.push_back(dio);
        } else if (type == "output") {
            auto dio = std::make_shared<DIO>(vmx, sensor, pin, studica_driver::PinMode::OUTPUT, topic);
            dio_nodes.push_back(dio);
        } else {
            RCLCPP_WARN(control->get_logger(), "invalid dio type '%s' — use 'input' or 'output'", type.c_str());
        }
    }

    return dio_nodes;
}


// composable node constructor — used when loading as a plugin
DIO::DIO(const rclcpp::NodeOptions &options) : rclcpp::Node("dio", options) {}


// main constructor — connects to the pin and sets up the publisher,
// service, and periodic timer
DIO::DIO(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex pin,
         studica_driver::PinMode pin_mode, const std::string &topic)
    : rclcpp::Node(name), vmx_(vmx), pin_(pin), pin_mode_(pin_mode) {

    dio_ = std::make_shared<studica_driver::DIO>(pin_, pin_mode_, vmx_);

    // service for sending commands to the pin
    service_ = this->create_service<studica_control::srv::SetData>(
        "dio_cmd",
        std::bind(&DIO::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    // publishes pin state at 10hz
    publisher_ = this->create_publisher<std_msgs::msg::Bool>(topic, 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&DIO::publish_dio_state, this));

    RCLCPP_INFO(this->get_logger(), "dio component ready.");
}

DIO::~DIO() {}


// handles incoming service commands
void DIO::cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                       std::shared_ptr<studica_control::srv::SetData::Response> response) {
    try {
        if (request->params == "toggle") {
            dio_->Set(!dio_->Get());
            response->success = true;
            response->message = "pin " + std::to_string(pin_) + " set to "
                                + std::string(dio_->Get() ? "high" : "low");
        } else {
            response->success = false;
            response->message = "unknown command '" + request->params + "'";
        }
    } catch (const std::exception &e) {
        response->success = false;
        response->message = "failed to set dio pin: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "failed to set dio pin: %s", e.what());
    }
}


// reads the current pin state and publishes it
void DIO::publish_dio_state() {
    try {
        auto message = std_msgs::msg::Bool();
        message.data = dio_->Get();
        publisher_->publish(message);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "failed to read dio state: %s", e.what());
    }
}


} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::DIO)
