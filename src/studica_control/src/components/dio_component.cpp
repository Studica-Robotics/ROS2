#include "studica_control/dio_component.h"

namespace studica_control {

std::vector<std::shared_ptr<rclcpp::Node>> DIO::initialize(rclcpp::Node *control, std::shared_ptr<VMXPi> vmx) {
    std::vector<std::shared_ptr<rclcpp::Node>> dio_nodes;
    control->declare_parameter<std::vector<std::string>>("dio.sensors", {});
    std::vector<std::string> sensor_ids = control->get_parameter("dio.sensors").as_string_array();
    for (const auto &sensor : sensor_ids) {
        std::string pin_param = "dio." + sensor + ".pin";
        std::string type_param = "dio." + sensor + ".type";
        std::string topic_param = "dio." + sensor + ".topic";

        control->declare_parameter<int>(pin_param, -1);
        control->declare_parameter<std::string>(type_param, "");
        control->declare_parameter<std::string>(topic_param, "");

        int pin = control->get_parameter(pin_param).as_int();
        std::string type = control->get_parameter(type_param).as_string();
        std::string topic = control->get_parameter(topic_param).as_string();

        RCLCPP_INFO(control->get_logger(), "%s -> pin: %d, type: %s, topic: %s", sensor.c_str(), pin, type.c_str(), topic.c_str());

        if (type == "input") {
            auto dio = std::make_shared<DIO>(vmx, sensor, pin, studica_driver::PinMode::INPUT, topic);
            dio_nodes.push_back(dio);
        } else if (type == "output") {
            auto dio = std::make_shared<DIO>(vmx, sensor, pin, studica_driver::PinMode::OUTPUT, topic);
            dio_nodes.push_back(dio);
        } else {
            RCLCPP_WARN(control->get_logger(), "Invalid dio type. Allowed values are 'input' or 'output'.");
        }
    }
    return dio_nodes;
}

DIO::DIO(const rclcpp::NodeOptions &options) : rclcpp::Node("dio", options) {}

DIO::DIO(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex pin, studica_driver::PinMode pin_mode, const std::string &topic) 
    : rclcpp::Node(name), vmx_(vmx), pin_(pin), pin_mode_(pin_mode), btn_pin(-1) {
    dio_ = std::make_shared<studica_driver::DIO>(pin_, pin_mode_, vmx_);
    
    btn_pin = pin_;
    RCLCPP_INFO(this->get_logger(), "DIO button pin %d.", btn_pin);
    
    service_ = this->create_service<studica_control::srv::SetData>(
        "dio_cmd",
        std::bind(&DIO::cmd_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
    publisher_ = this->create_publisher<std_msgs::msg::Bool>(topic, 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&DIO::publish_dio_state, this)
    );
    RCLCPP_INFO(this->get_logger(), "DIO component is ready.");
}

DIO::~DIO() {}

void DIO::cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                       std::shared_ptr<studica_control::srv::SetData::Response> response) {
    try {
        if (request->params == "toggle") {
            dio_->Set(!dio_->Get());
            response->success = true;
            response->message = "DIO pin " + std::to_string(pin_) + "set to " + std::string(dio_->Get() ? "True" : "False");
        } else {
            response->success = false;
            response->message = "Invalid command for non-button pin.";
        }
    } catch (const std::exception &e) {
        response->success = false;
        response->message = "Failed to set DIO pin: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "Failed to set DIO pin: %s", e.what());
    }
}

void DIO::publish_dio_state() {
    try {
        if (pin_ == btn_pin) {
            bool dio_value = dio_->Get();

            auto message = std_msgs::msg::Bool();
            message.data = dio_value;
            
            publisher_->publish(message);
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to retrieve DIO state: %s", e.what());
    }
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::DIO)
