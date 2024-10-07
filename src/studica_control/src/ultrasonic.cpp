#include "studica_control/ultrasonic.h"

UltrasonicDriver::UltrasonicDriver(std::shared_ptr<VMXPi> vmx, VMXChannelIndex ping, VMXChannelIndex echo) : Device("ultrasonic_driver_node_"), vmx_(vmx) {
    is_reading_ = false;

    // Create a publisher for distance data
    distance_publisher_ = this->create_publisher<std_msgs::msg::Float32>("ultrasonic/distance", 10);

    // Check if VMXPi is open
    if (vmx_->IsOpen()) {
        RCLCPP_INFO(this->get_logger(), "Ultrasonic Sensor Connected");
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000), 
            std::bind(&UltrasonicDriver::Spin, this));  
    } else {
        RCLCPP_ERROR(this->get_logger(), "Error: Unable to open VMX Client.");
    }

// setup

    ping_channel_index = ping; // Digital Output
    echo_channel_index = echo; // Timer Input
    // Activate the ping output channel
    DIOConfig dio_config(DIOConfig::OutputMode::PUSHPULL);
    VMXErrorCode vmxerr;

    if (!vmx_->io.ActivateSinglechannelResource(VMXChannelInfo(ping_channel_index, VMXChannelCapability::DigitalOutput), &dio_config, ping_output_res_handle, &vmxerr)) {
        RCLCPP_ERROR(this->get_logger(), "Error activating ping output channel.");
        DisplayVMXError(vmxerr);
        return;
    }

    // Activate the echo input capture channel
    InputCaptureConfig inputcap_config;
    inputcap_config.SetSlaveMode(InputCaptureConfig::SLAVEMODE_RESET);
    inputcap_config.SetSlaveModeTriggerSource(InputCaptureConfig::TRIGGER_DYNAMIC);
    inputcap_config.SetCaptureChannelSource(InputCaptureConfig::CH1, InputCaptureConfig::CAPTURE_SIGNAL_DYNAMIC);
    inputcap_config.SetCaptureChannelSource(InputCaptureConfig::CH2, InputCaptureConfig::CAPTURE_SIGNAL_DYNAMIC);

    if (!vmx_->io.ActivateSinglechannelResource(VMXChannelInfo(echo_channel_index, VMXChannelCapability::InputCaptureInput2), &inputcap_config, echo_inputcap_res_handle, &vmxerr)) {
        RCLCPP_ERROR(this->get_logger(), "Error activating echo input capture channel.");
        DisplayVMXError(vmxerr);
        return;
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

    vmx_->time.DelayMilliseconds(55); // Delay for next measurement
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
