#include "studica_control/ultrasonic.h"
static constexpr double kPingTime = 10 * 1e-6;

UltrasonicDriver::UltrasonicDriver(std::shared_ptr<VMXPi> vmx, VMXChannelIndex ping, VMXChannelIndex echo) : Device("ultrasonic_driver_node_"), vmx_(vmx) {
    is_reading_ = false;

    // Create a publisher for distance data
    distance_publisher_ = this->create_publisher<std_msgs::msg::Float32>("ultrasonic/distance", 10);

    // Check if VMXPi is open
    if (vmx_->IsOpen()) {
        RCLCPP_INFO(this->get_logger(), "Ultrasonic Sensor Connected");
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(55), 
            std::bind(&UltrasonicDriver::Spin, this));  
    } else {
        RCLCPP_ERROR(this->get_logger(), "Error: Unable to open VMX Client.");
    }

// setup

    ping_channel_index = ping; // Digital Output
    echo_channel_index = echo; // Timer Input
    // Activate the ping output channel
    uint32_t microseconds_per_inch = 148;
    
    VMXErrorCode vmxerr;

    double pulseLength = kPingTime;
    uint32_t num_microseconds = static_cast<uint32_t>(pulseLength / 1.0e-6);

    printf("Trigger Num Microseconds:  %d\n", num_microseconds);

    DIOConfig dio_config(DIOConfig::OutputMode::PUSHPULL);
    if (!vmx_->io.ActivateSinglechannelResource(VMXChannelInfo(ping_channel_index, VMXChannelCapability::DigitalOutput), &dio_config, 
                    ping_output_res_handle, &vmxerr)) {
            printf("Error Activating Singlechannel Resource DIO for Channel index %d.\n", ping_channel_index);
            DisplayVMXError(vmxerr);
    } else {
            printf("Ping (Digital Output) Channel %d activated on Resource type %d, index %d\n", ping_channel_index,
                            EXTRACT_VMX_RESOURCE_TYPE(ping_output_res_handle),
                            EXTRACT_VMX_RESOURCE_INDEX(ping_output_res_handle));
    }

    // Activate the echo input capture channel
    InputCaptureConfig inputcap_config;
    inputcap_config.SetSlaveMode(InputCaptureConfig::SLAVEMODE_RESET);
    inputcap_config.SetSlaveModeTriggerSource(InputCaptureConfig::TRIGGER_DYNAMIC);
    inputcap_config.SetCaptureChannelSource(InputCaptureConfig::CH1, InputCaptureConfig::CAPTURE_SIGNAL_DYNAMIC);
    inputcap_config.SetCaptureChannelSource(InputCaptureConfig::CH2, InputCaptureConfig::CAPTURE_SIGNAL_DYNAMIC);

    
    if (vmx_->io.ChannelSupportsCapability(echo_channel_index, VMXChannelCapability::InputCaptureInput2)) {
            // Second Timer Input:  Second Timer Channel must handle the rising edge
            inputcap_config.SetCaptureChannelActiveEdge(InputCaptureConfig::CH1, InputCaptureConfig::ACTIVE_FALLING);
            inputcap_config.SetCaptureChannelActiveEdge(InputCaptureConfig::CH2, InputCaptureConfig::ACTIVE_RISING);
    } else {
            // First Timer Input:  First Timer Channel must handle the rising edge
            inputcap_config.SetCaptureChannelActiveEdge(InputCaptureConfig::CH1, InputCaptureConfig::ACTIVE_RISING);
            inputcap_config.SetCaptureChannelActiveEdge(InputCaptureConfig::CH2, InputCaptureConfig::ACTIVE_FALLING);
    }

    /* Capture occurs on each and every input signal edge transition. */
    inputcap_config.SetCaptureChannelPrescaler(InputCaptureConfig::CH1, InputCaptureConfig::x1);
    inputcap_config.SetCaptureChannelPrescaler(InputCaptureConfig::CH2, InputCaptureConfig::x1);

    /* Digitally filter 2 successive samples when looking for edges  */
    uint8_t filter_number = inputcap_config.GetClosestCaptureCaptureFilterNumSamples(2);
    inputcap_config.SetCaptureChannelFilter(InputCaptureConfig::CH1, filter_number);
    inputcap_config.SetCaptureChannelFilter(InputCaptureConfig::CH2, filter_number);

    VMXChannelInfo echo_chaninfo(echo_channel_index, VMXChannelCapability::InputCaptureInput2);

    /* If the input is no longer present, do not automatically clear the Capture Channel Count */
    inputcap_config.SetStallAction(InputCaptureConfig::ACTION_NONE);

    if (!vmx_->io.ActivateSinglechannelResource(echo_chaninfo, &inputcap_config, echo_inputcap_res_handle, &vmxerr)) {
            printf("Error Activating Singlechannel Resource InputCapture for Channel index %d.\n", echo_channel_index);
            DisplayVMXError(vmxerr);
    } else {
            printf("Echo (Input Capture) Channel %d activated on Resource type %d, index %d\n", echo_channel_index,
                            EXTRACT_VMX_RESOURCE_TYPE(echo_inputcap_res_handle),
                            EXTRACT_VMX_RESOURCE_INDEX(echo_inputcap_res_handle));
    }
}


// Method to read distance and publish
void UltrasonicDriver::read_distance() {
    VMXErrorCode vmxerr;

    // loop
    if (!vmx_->io.DIO_Pulse(ping_output_res_handle, true, 10, &vmxerr)) {
        RCLCPP_ERROR(this->get_logger(), "Error triggering pulse.");
        DisplayVMXError(vmxerr);
    }

    vmx_->time.DelayMilliseconds(5); // Wait for echo to return
    uint32_t ch1_count = 0;
    uint32_t ch2_count = 0;

    if (!vmx_->io.InputCapture_GetChannelCounts(echo_inputcap_res_handle, ch1_count, ch2_count, &vmxerr)) {
        RCLCPP_ERROR(this->get_logger(), "Error retrieving input capture count.");
        DisplayVMXError(vmxerr);
    } else {
        double distance_inches = ch2_count / 148.0; // Convert to inches

        // Create and publish distance message
        auto distance_msg = std_msgs::msg::Float32();
        distance_msg.data = distance_inches;
        distance_publisher_->publish(distance_msg);

        RCLCPP_INFO(this->get_logger(), "Distance (inches): %.2f", distance_inches);
    }

    // vmx_->time.DelayMilliseconds(55); // Delay for next measurement
}

// Main loop where reading happens if enabled
void UltrasonicDriver::Spin() {
    if (is_reading_) {
        read_distance();
    }
}

// Start reading distances
void UltrasonicDriver::start_reading() {
    if (!is_reading_) {
        is_reading_ = true;
        RCLCPP_INFO(this->get_logger(), "Distance reading started");
    } else {
        RCLCPP_INFO(this->get_logger(), "Already reading distances");
    }
}

// Stop reading distances
void UltrasonicDriver::stop_reading() {
    if (is_reading_) {
        is_reading_ = false;
        RCLCPP_INFO(this->get_logger(), "Distance reading stopped");
    } else {
        RCLCPP_INFO(this->get_logger(), "Already stopped reading distances");
    }
}

// Override the cmd function to handle start/stop commands
void UltrasonicDriver::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (params == "start") {
        start_reading();
        response->success = true;
        response->message = "Distance reading started";
    } else if (params == "stop") {
        stop_reading();
        response->success = true;
        response->message = "Distance reading stopped";
    } else {
        response->success = false;
        response->message = "Unknown command";
    }
}

void UltrasonicDriver::DisplayVMXError(VMXErrorCode vmxerr) {
    const char *p_err_description = GetVMXErrorString(vmxerr);
    RCLCPP_ERROR(this->get_logger(), "VMXError %d: %s", vmxerr, p_err_description);
}
