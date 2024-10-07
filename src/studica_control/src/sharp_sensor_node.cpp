#include "studica_control/sharp_sensor_node.h"

void DisplayVMXError(VMXErrorCode vmxerr) {
    const char *p_err_description = GetVMXErrorString(vmxerr);
    printf("VMXError %d:  %s\n", vmxerr, p_err_description);
}

SharpSensor::SharpSensor() : Device("sharp_sensor_node"), is_publishing_(false), count_(0) {
    // Init publishers
    sharp_publisher_ = this->create_publisher<std_msgs::msg::String>("sharp_sensor/data", 10);
    publisher_ = this->create_publisher<std_msgs::msg::String>("sharp_sensor/message", 10);

    // Init VMX
    bool realtime = true;
    uint8_t update_rate_hz = 50;
    vmx = std::make_shared<VMXPi>(realtime, update_rate_hz);

    // Verify VMX connection
    if (vmx->IsOpen()) {
        RCLCPP_INFO(this->get_logger(), "Sharp Sensor Connected");

        // Setup publishing timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&SharpSensor::Spin, this));  // Periodic execution of Spin

        // Get full scale voltage of analog input
        VMXErrorCode vmxerr;
        float full_scale_voltage;
        if (vmx->io.Accumulator_GetFullScaleVoltage(full_scale_voltage, &vmxerr)) {
            printf("Analog input voltage: %0.1f\n", full_scale_voltage);
        } else {
            printf("ERROR acquiring Analog Input Voltage.\n");
            DisplayVMXError(vmxerr);
        }

        // Configure analog accumulator resources
        VMXChannelIndex first_anin_channel;
        uint8_t num_analog_inputs = vmx->io.GetNumChannelsByType(VMXChannelType::AnalogIn, first_anin_channel);
        accumulator_res_handles.resize(num_analog_inputs);  // Resize to hold resource handles

        for (uint8_t analog_in_chan_index = first_anin_channel; 
             analog_in_chan_index < first_anin_channel + num_analog_inputs; 
             analog_in_chan_index++) {
            VMXResourceIndex accum_res_index = analog_in_chan_index - first_anin_channel;
            AccumulatorConfig accum_config;
            accum_config.SetNumAverageBits(9);
            if (!vmx->io.ActivateSinglechannelResource(
                    VMXChannelInfo(analog_in_chan_index, VMXChannelCapability::AccumulatorInput),
                    &accum_config, 
                    accumulator_res_handles[accum_res_index], 
                    &vmxerr)) {
                printf("Error Activating Singlechannel Resource Accumulator for Channel index %d.\n", analog_in_chan_index);
                DisplayVMXError(vmxerr);
            } else {
                printf("Analog Input Channel %d activated on Resource type %d, index %d\n", 
                       analog_in_chan_index,
                       EXTRACT_VMX_RESOURCE_TYPE(accumulator_res_handles[accum_res_index]),
                       EXTRACT_VMX_RESOURCE_INDEX(accumulator_res_handles[accum_res_index]));
            }
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Error: Unable to open VMX Client.");
    }
}

void SharpSensor::publish_message() {
    auto message = std_msgs::msg::String();
    message.data = std::string("Message from ") + this->get_name() + std::string(": ") + std::to_string(count_++);
    publisher_->publish(message);
    std::cout << this->get_name() << ": " << count_ << std::endl;
}

// Publish analog data from Sharp Sensor
void SharpSensor::publish_analog_data() {
    VMXErrorCode vmxerr;
    float analog_voltage;
    std::ostringstream voltage_stream; // accum voltage readings

    // Loop through the accumulator resource handles to get the average voltage for each channel
    for (size_t j = 0; j < accumulator_res_handles.size(); j++) {
        if (vmx->io.Accumulator_GetAverageVoltage(accumulator_res_handles[j], analog_voltage, &vmxerr)) {
            voltage_stream << analog_voltage;  // appennd

            // Add a comma and space for separation except for the last element
            if (j < accumulator_res_handles.size() - 1) {
                voltage_stream << ", ";
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Error getting Average Voltage of analog accumulator %d", j);
            DisplayVMXError(vmxerr);
        }
    }

    // Print the formatted output once
    std::cout << "Analog Input Voltage (V): " << voltage_stream.str() << std::endl;

    // Publish the accumulated voltages
    auto sharp_msg = std_msgs::msg::String();
    sharp_msg.data = "Sharp Sensor Voltages: " + voltage_stream.str();
    sharp_publisher_->publish(sharp_msg);
}

// Main loop to manage publishing state
void SharpSensor::Spin() {
    if (is_publishing_) {
        publish_analog_data();  // Publish sensor data
        publish_message();      // Publish a test message
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
