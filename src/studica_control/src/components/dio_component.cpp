#include "studica_control/dio_component.h"

namespace studica_control {

DIO::DIO(const rclcpp::NodeOptions &options) : rclcpp::Node("dio", options) {}

DIO::DIO(std::shared_ptr<VMXPi> vmx, VMXChannelIndex pin, studica_driver::PinMode pin_mode, std::string type) 
    : rclcpp::Node("dio_component"), vmx_(vmx), pin_(pin), pin_mode_(pin_mode), btn_pin(-1) {
    dio_ = std::make_shared<studica_driver::DIO>(pin_, pin_mode_, vmx_);
    if (type == "button") {
        btn_pin = pin_;
        RCLCPP_INFO(this->get_logger(), "DIO button pin %d.", btn_pin);
    }
    service_ = this->create_service<studica_control::srv::SetData>(
        "dio_cmd",
        std::bind(&DIO::cmd_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
    publisher_ = this->create_publisher<std_msgs::msg::Bool>("dio_state", 10);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
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

            RCLCPP_INFO(this->get_logger(), "Published DIO state: %d", dio_value);
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
