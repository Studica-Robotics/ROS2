#include "studica_control/ultrasonic_component.h"

namespace studica_control {

Ultrasonic::Ultrasonic(const rclcpp::NodeOptions &options) : Node("ultrasonic", options) {}

Ultrasonic::Ultrasonic(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex ping, VMXChannelIndex echo) 
    : Node(name), vmx_(vmx), ping_(ping), echo_(echo) {
    ultrasonic_ = std::make_shared<studica_driver::Ultrasonic>(ping_, echo_, vmx_);
    service_ = this->create_service<studica_control::srv::SetData>(
        "ultrasonic_cmd",
        std::bind(&Ultrasonic::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    publisher_ = this->create_publisher<sensor_msgs::msg::Range>("ultrasonic_range", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&Ultrasonic::publish_range, this));
}

Ultrasonic::~Ultrasonic() {}

void Ultrasonic::cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    std::string params = request->params;
    cmd(params, response);
}

void Ultrasonic::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (params == "get_distance_inches") {
        ultrasonic_->Ping();
        response->success = true;
        response->message = std::to_string(ultrasonic_->GetDistanceIN());
        ultrasonic_->Ping();
    } else if (params == "get_distance_millimeters") {
        ultrasonic_->Ping();
        response->success = true;
        response->message = std::to_string(ultrasonic_->GetDistanceMM());
        ultrasonic_->Ping();
    } else if (params == "get_distance") {
        ultrasonic_->Ping();
        response->success = true;
        response->message = std::to_string(ultrasonic_->GetDistanceMM());
        ultrasonic_->Ping();
    } else {
        response->success = false;
        response->message = "No such command '" + params + "'";
    }
}

void Ultrasonic::publish_range() {
    double min_range = 0.02, max_range = 4.0;

    ultrasonic_->Ping();
    double distance_m = ultrasonic_->GetDistanceMM() / 1000.0;

    if (distance_m < min_range || distance_m > max_range) distance_m = INFINITY;
    
    sensor_msgs::msg::Range msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "ultrasonic_sensor";
    msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    msg.field_of_view = 0.26;
    msg.min_range = min_range;
    msg.max_range = max_range;
    msg.range = distance_m;

    publisher_->publish(msg);
}

void Ultrasonic::DisplayVMXError(VMXErrorCode vmxerr) {
    const char *p_err_description = GetVMXErrorString(vmxerr);
    printf("VMXError %d: %s\n", vmxerr, p_err_description);
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Ultrasonic)
