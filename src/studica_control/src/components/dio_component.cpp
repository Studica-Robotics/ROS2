#include "studica_control/dio_component.h"
#include "std_msgs/msg/bool.hpp"
#include "studica_control/srv/set_data.hpp"

namespace studica_control
{

DIO::DIO(const rclcpp::NodeOptions &options) : rclcpp::Node("dio", options), is_publishing(false) {}

DIO::DIO(std::shared_ptr<VMXPi> vmx, VMXChannelIndex pin, studica_driver::PinMode pin_mode, std::string type) 
    : rclcpp::Node("dio_component"), vmx_(vmx), pin_(pin), pin_mode_(pin_mode), is_publishing(false), btn_pin(-1) {
    // Initialize DIO component
    dio_ = std::make_shared<studica_driver::DIO>(pin_, pin_mode_, vmx_);
    if (type == "button") {
        btn_pin = pin_;
        RCLCPP_INFO(this->get_logger(), "DIO button pin %d.", btn_pin);
    }

    publisher_ = this->create_publisher<std_msgs::msg::Bool>("dio_state", 10);
    service_ = this->create_service<studica_control::srv::SetData>(
        "dio_cmd",
        std::bind(&DIO::cmd_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Create a timer to publish the DIO state periodically
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), // Adjust the interval as needed
        std::bind(&DIO::publish_dio_state, this)
    );

    RCLCPP_INFO(this->get_logger(), "DIO component is ready.");
}

DIO::~DIO() {}

void DIO::publish_dio_state() {
    if (!is_publishing) {
        return; // Do nothing if publishing is not active
    }
    try {
        if (pin_ == btn_pin) {
            // Get the current DIO state
            bool dio_value = dio_->Get();

            // Publish the state
            auto message = std_msgs::msg::Bool();
            message.data = dio_value;
            publisher_->publish(message);

            RCLCPP_INFO(this->get_logger(), "Published DIO state: %d", dio_value);
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to retrieve DIO state: %s", e.what());
    }
}

void DIO::cmd_callback(const std::shared_ptr<studica_control::srv::SetData::Request> request,
                       std::shared_ptr<studica_control::srv::SetData::Response> response) {
    try {
        if (pin_ == btn_pin) {
            if (request->params == "start_pub") {
                is_publishing = true;
                response->success = true;
                response->message = "DIO state publishing started for button pin " + std::to_string(pin_);
                return;
            } else if (request->params == "stop_pub") {
                is_publishing = false;
                response->success = true;
                response->message = "DIO state publishing stopped for button pin " + std::to_string(pin_);
                return;
            }
        } else {
            if (request->params == "toggle") {
                dio_->Set(!dio_->Get());
                response->success = true;
                response->message = "DIO pin " + std::to_string(pin_) + "set to " + std::string(dio_->Get() ? "True" : "False");
                return;
            } else {
                response->success = false;
                response->message = "Invalid command for non-button pin.";
                return;
            }
            return;
        }
    } catch (const std::exception &e) {
        response->success = false;
        response->message = "Failed to set DIO pin: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "Failed to set DIO pin: %s", e.what());
    }
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::DIO)
