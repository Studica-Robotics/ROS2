#include "studica_control/sharp_sensor_node.h"

SharpSensor::SharpSensor(std::shared_ptr<VMXPi> vmx, const std::string &name, std::shared_ptr<AnalogInput> analog_input)
    : Device("sharp_sensor_node"), vmx_(vmx), is_publishing_(false), analog_input_(analog_input) {
    
    // Init publishers 
    sharp_publisher_ = this->create_publisher<std_msgs::msg::String>("sharp/distance_" + name, 10);

    // Verify VMX connection
    if (vmx_->IsOpen()) {
        RCLCPP_INFO(this->get_logger(), "Sharp Sensor Connected");

        // Setup publishing timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&SharpSensor::Spin, this));  // Periodic execution of Spin

        // The AnalogInput has already been activated during the creation of the input
        if (!analog_input_->is_active()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to activate analog input for Sharp sensor.");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Error: Unable to open VMX Client.");
    }
}

// Publish analog data from Sharp Sensor
void SharpSensor::publish_analog_data() {
    float analog_voltage;

    // Get the average voltage using the AnalogInput object
    if (analog_input_->get_average_voltage(analog_voltage)) {
        double distance = std::pow(analog_voltage, -1.2045) * 27.726;
        RCLCPP_INFO(this->get_logger(), "[LOG] Analog Input Voltage(V): %f, Distance(cm): %f", analog_voltage, distance);
        // publish distance
        auto sharp_msg = std_msgs::msg::String();
        sharp_msg.data = "[PUB] Analog Input Voltage(V): " + std::to_string(analog_voltage) +
                        ", Distance(cm): " + std::to_string(distance);
        sharp_publisher_->publish(sharp_msg);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Error getting average voltage for analog input.");
    }
}

double SharpSensor::get_voltage() {
    float analog_voltage;
    if (analog_input_->get_average_voltage(analog_voltage)) {
        return analog_voltage;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Error getting average voltage for analog input.");
        return -1;
    }
}

// Main loop to manage publishing state
void SharpSensor::Spin() {
    if (is_publishing_) {
        publish_analog_data();  // Publish sensor data
    }
}

// Start publishing sensor data
void SharpSensor::start_publishing() {
    if (!is_publishing_) {
        is_publishing_ = true;
        RCLCPP_INFO(this->get_logger(), "Publishing started");
    } else {
        RCLCPP_INFO(this->get_logger(), "Already publishing");
    }
}

// Stop publishing sensor data
void SharpSensor::stop_publishing() {
    if (is_publishing_) {
        is_publishing_ = false;
        RCLCPP_INFO(this->get_logger(), "Publishing stopped");
    } else {
        RCLCPP_INFO(this->get_logger(), "Already stopped");
    }
}

// Override the cmd function to handle "start" and "stop" commands
void SharpSensor::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (params == "start") {
        start_publishing();
        response->success = true;
        response->message = "Publishing started";
    } else if (params == "stop") {
        stop_publishing();
        response->success = true;
        response->message = "Publishing stopped";
    } else {
        response->success = false;
        response->message = "Unknown command";
    }
}
