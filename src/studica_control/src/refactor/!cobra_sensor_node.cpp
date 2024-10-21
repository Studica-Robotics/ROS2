#include "studica_control/cobra_sensor_node.h"

CobraSensor::CobraSensor(std::shared_ptr<VMXPi> vmx, std::shared_ptr<I2CHandler> i2c_handler, const std::string &name, const float &vRef, const int &mux_ch) 
    : Device(name), vmx_(vmx), vRef_(vRef), is_publishing_(false), i2c_handler_(i2c_handler), mux_ch_(mux_ch) {
    // init i2c values
    port = 1;
    mode = CONFIG_MODE_CONT;
    gain = CONFIG_PGA_2;
    sampleRate = CONFIG_RATE_1600HZ;
    multiplierVolts = 1.0F;

    cobra_publisher_ = this->create_publisher<std_msgs::msg::String>("cobra/sensor_" + name, 10);

    // activate I2C resource
    // if (!i2c_handler_->activateI2CResource(cobra_res_handle)) { // MODIFIED
    if (!i2c_handler_->activateI2CResource()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to activate I2C resource.");
        return;
    }

    // scan for I2C devices
    deviceAddress = i2c_handler_->scanI2CBus();
    if (deviceAddress == 0) {
        RCLCPP_ERROR(this->get_logger(), "No I2C devices found.");
        return;
    }

    // setup publishing timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&CobraSensor::Spin, this));

    IsConnected();
}

bool CobraSensor::IsConnected() {
    uint8_t partID = 0;
    
    if (!i2c_handler_->i2cTransaction(static_cast<uint8_t>(deviceAddress), nullptr, 0, &partID, 1)) {
        printf("Cobra is not Connected!");
        return false;
    } else {
        printf("Cobra is Connected!");
        return true;
    }
}

void CobraSensor::publish_cobra_data() {
    // Read cobra data from the Cobra sensor and publish it
    int raw_value = GetRawValue(mux_ch_);
    float voltage = GetVoltage(mux_ch_);

    auto message = std_msgs::msg::String();
    message.data = "ADC Value: " + std::to_string(raw_value) + ", Voltage(V): " + std::to_string(voltage);
    
    RCLCPP_INFO(this->get_logger(), message.data.c_str());
    cobra_publisher_->publish(message);
}

int CobraSensor::GetSingle(uint8_t channel) {
    if (channel > 3) {
        RCLCPP_ERROR(this->get_logger(), "Invalid multiplexer channel: %d", channel);
        return 0;
    }

    VMXErrorCode vmxerr;
    int config = CONFIG_OS_SINGLE | mode | sampleRate;
    config |= gain;

    // set the correct multiplexer setting for the selected channel
    switch (channel) {
        case 0:
            config |= CONFIG_MUX_SINGLE_0;
            break;
        case 1:
            config |= CONFIG_MUX_SINGLE_1;
            break;
        case 2:
            config |= CONFIG_MUX_SINGLE_2;
            break;
        case 3:
            config |= CONFIG_MUX_SINGLE_3;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Invalid multiplexer channel: %d", channel);
            return 0;
    }

    // Prepare the command to send to the ADC
    uint8_t tx_data[3];
    uint8_t rx_data[2];

    tx_data[0] = 0x0001;  // Configuration register
    tx_data[1] = config >> 8;  // MSB of config
    tx_data[2] = config & 0xFF;  // LSB of config

    // Send the configuration to the ADC
    if (!i2c_handler_->i2cTransaction(static_cast<uint8_t>(deviceAddress), tx_data, 3, nullptr, 0)) {
        RCLCPP_ERROR(this->get_logger(), "Error configuring the ADC on I2C bus.");
        return 0;
    }

    // Wait for the ADC to perform the conversion
    CobraSensor::Delay(0.01); // delay for 100ms

    // Read the conversion result from the ADC
    tx_data[0] = 0x00;  // Conversion result register
    if (!i2c_handler_->i2cTransaction(static_cast<uint8_t>(deviceAddress), tx_data, 2, rx_data, 2)) {
        RCLCPP_ERROR(this->get_logger(), "Error reading the ADC conversion result from I2C bus.");
        return 0;
    }

    return (rx_data[0] << 8) | rx_data[1]; // Combine the received bytes into an integer
}

int CobraSensor::GetRawValue(uint8_t channel) {
    return GetSingle(channel); 
}

float CobraSensor::GetVoltage(uint8_t channel) {
    float raw = GetSingle(channel); 
    float mV = vRef_ / 0x800;  //  <vRef_>V reference and 12-bit ADC
    return raw * mV;
}

void CobraSensor::Delay(double seconds){
    std::this_thread::sleep_for(std::chrono::milliseconds((int64_t)(seconds * 1000)));
}

void CobraSensor::Spin() {
    if (is_publishing_) {
        publish_cobra_data();  // Publish sensor data
    }
}

void CobraSensor::start_publishing() {
    if (!is_publishing_) {
        is_publishing_ = true;
        RCLCPP_INFO(this->get_logger(), "Publishing started");
    } else {
        RCLCPP_INFO(this->get_logger(), "Already publishing");
    }
}

void CobraSensor::stop_publishing() {
    if (is_publishing_) {
        is_publishing_ = false;
        RCLCPP_INFO(this->get_logger(), "Publishing stopped");
    } else {
        RCLCPP_INFO(this->get_logger(), "Already stopped");
    }
}

void CobraSensor::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) {
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