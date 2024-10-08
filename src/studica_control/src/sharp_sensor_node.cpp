#include "studica_control/sharp_sensor_node.h"

SharpSensor::SharpSensor(std::shared_ptr<VMXPi> vmx, const std::string &name, VMXChannelIndex ping) : Device("sharp_sensor_node"), is_publishing_(false), count_(0), vmx_(vmx) {
    // Init publishers
    sharp_publisher_ = this->create_publisher<std_msgs::msg::String>("sharp/distance_" + name, 10);

    // Verify VMX connection
    if (vmx_->IsOpen()) {
        RCLCPP_INFO(this->get_logger(), "Sharp Sensor Connected");

        // Setup publishing timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&SharpSensor::Spin, this));  // Periodic execution of Spin

        // input channel index
        ping_channel_index = ping;

        // Get full scale voltage of analog input
        VMXErrorCode vmxerr;
        float full_scale_voltage;
        if (vmx_->io.Accumulator_GetFullScaleVoltage(full_scale_voltage, &vmxerr)) {
            printf("Analog input voltage: %0.1f\n", full_scale_voltage);
        } else {
            printf("ERROR acquiring Analog Input Voltage.\n");
        }

        // Configure analog accumulator for the given ping channel
        AccumulatorConfig accum_config;
        accum_config.SetNumAverageBits(9);
        accumulator_res_handle = VMXResourceHandle();
        if (!vmx_->io.ActivateSinglechannelResource(
                VMXChannelInfo(ping_channel_index, VMXChannelCapability::AccumulatorInput),
                &accum_config, 
                accumulator_res_handle, 
                &vmxerr)) {
            printf("Error Activating Singlechannel Resource Accumulator for Channel index %d.\n", ping_channel_index);
        } else {
            printf("Analog Input Channel %d activated on Resource type %d, index %d\n", 
                   ping_channel_index,
                   EXTRACT_VMX_RESOURCE_TYPE(accumulator_res_handle),
                   EXTRACT_VMX_RESOURCE_INDEX(accumulator_res_handle));
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Error: Unable to open VMX Client.");
    }
}

// Publish analog data from Sharp Sensor
void SharpSensor::publish_analog_data() {
    // CONVERT VOLTAGE TO DISTANCE (example provided in the comment above)

    VMXErrorCode vmxerr;
    float analog_voltage;

    // Get the average voltage for the specified ping channel
    if (vmx_->io.Accumulator_GetAverageVoltage(accumulator_res_handle, analog_voltage, &vmxerr)) {
        std::cout << "Analog Input Voltage (V) for channel " << std::to_string(ping_channel_index) << ": " << analog_voltage << std::endl;
        
        // Publish the voltage
        auto sharp_msg = std_msgs::msg::String();
        sharp_msg.data = "Sharp Sensor Voltage for channel " + std::to_string(ping_channel_index) + ": " + std::to_string(analog_voltage);
        sharp_publisher_->publish(sharp_msg);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Error getting Average Voltage of analog accumulator for channel %d", ping_channel_index);
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
