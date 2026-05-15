/*
 * dio_component.cpp
 *
 * ros2 component for a digital input/output pin on the vmx-pi board.
 * mode is set in params.yaml and cannot change at runtime.
 *
 * topic (publishes):  <name>/state (std_msgs/Bool) — pin state, 10hz
 * topic (subscribes): <name>/cmd   (std_msgs/Bool) — set pin directly (output mode only)
 * service: <name>/dio_cmd (studica_control/SetData)
 *   toggle — flip output state (output mode only)
 */

#include "studica_control/dio_component.hpp"

namespace studica_control {


std::vector<std::shared_ptr<rclcpp::Node>> DIO::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    std::vector<std::shared_ptr<rclcpp::Node>> dio_nodes;

    control->declare_parameter<std::vector<std::string>>("dio.sensors", std::vector<std::string>{});
    std::vector<std::string> sensor_ids = control->get_parameter("dio.sensors").as_string_array();

    for (const auto &sensor : sensor_ids) {
        std::string pin_param  = "dio." + sensor + ".pin";
        std::string type_param = "dio." + sensor + ".type";

        control->declare_parameter<int>(pin_param, -1);
        control->declare_parameter<std::string>(type_param, "");

        int pin          = control->get_parameter(pin_param).as_int();
        std::string type = control->get_parameter(type_param).as_string();

        RCLCPP_INFO(control->get_logger(), "%s -> pin: %d, type: %s", sensor.c_str(), pin, type.c_str());

        if (type == "input") {
            dio_nodes.push_back(std::make_shared<DIO>(vmx, sensor, pin, studica_driver::PinMode::INPUT));
        } else if (type == "output") {
            dio_nodes.push_back(std::make_shared<DIO>(vmx, sensor, pin, studica_driver::PinMode::OUTPUT));
        } else {
            RCLCPP_ERROR(control->get_logger(),
                         "invalid dio type '%s' for '%s' — use 'input' or 'output'",
                         type.c_str(), sensor.c_str());
        }
    }

    return dio_nodes;
}


DIO::DIO(const rclcpp::NodeOptions &options) : rclcpp::Node("dio", options) {}


DIO::DIO(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex pin,
         studica_driver::PinMode pin_mode)
    : rclcpp::Node(name), vmx_(vmx), pin_(pin), pin_mode_(pin_mode) {

    dio_ = std::make_shared<studica_driver::DIO>(pin_, pin_mode_, vmx_);

    service_ = this->create_service<studica_control::srv::SetData>(
        name + "/dio_cmd",
        std::bind(&DIO::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    // output pins get a cmd subscriber for direct set
    if (pin_mode_ == studica_driver::PinMode::OUTPUT) {
        cmd_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            name + "/cmd", 10,
            [this](std_msgs::msg::Bool::SharedPtr msg) {
                dio_->Set(msg->data);
            });
        RCLCPP_INFO(this->get_logger(), "dio output ready on pin %d. cmd: /%s/cmd", pin_, name.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "dio input ready on pin %d.", pin_);
    }

    publisher_ = this->create_publisher<std_msgs::msg::Bool>(name + "/state", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&DIO::publish_dio_state, this));
}

DIO::~DIO() {}


void DIO::cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                       std::shared_ptr<studica_control::srv::SetData::Response> response) {
    try {
        if (request->params == "toggle") {
            if (pin_mode_ != studica_driver::PinMode::OUTPUT) {
                response->success = false;
                response->message = "toggle is only valid on output pins";
                return;
            }
            dio_->Set(!dio_->Get());
            response->success = true;
            response->message = "pin toggled to " + std::string(dio_->Get() ? "high" : "low");
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


void DIO::publish_dio_state() {
    try {
        std_msgs::msg::Bool msg;
        msg.data = dio_->Get();
        publisher_->publish(msg);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "failed to read dio state: %s", e.what());
    }
}


} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::DIO)
