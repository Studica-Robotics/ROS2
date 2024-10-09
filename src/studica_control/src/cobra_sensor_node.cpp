#include "studica_control/cobra_sensor_node.h"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

CobraSensor::CobraSensor(std::shared_ptr<VMXPi> vmx, const std::string &name, const float &vRef, const int &mux_ch) : Device(name), vmx_(vmx), vRef_(vRef), is_publishing_(false) {
    // init publisher
    cobra_publisher_ = this->create_publisher<std_msgs::msg::String>("cobra/line_" + name, 10);
    
    // init i2c values
    port = 1;
    deviceAddress = 0x48;
    vRef_ = vRef; // user-defined
    mode = CONFIG_MODE_CONT;
    gain = CONFIG_PGA_2;
    sampleRate = CONFIG_RATE_1600HZ;
    multiplierVolts = 1.0F;
    mux_ch_ = mux_ch; // user-defined

    // verify vmx connection
    if (vmx_->IsOpen()) {
        // setup publishing timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&CobraSensor::Spin, this));

        VMXErrorCode vmxerr;
        VMXChannelInfo i2c_channels[2] = {
            {VMXChannelInfo(vmx_->getIO().GetSoleChannelIndex(VMXChannelCapability::I2C_SDA), VMXChannelCapability::I2C_SDA)},
            {VMXChannelInfo(vmx_->getIO().GetSoleChannelIndex(VMXChannelCapability::I2C_SCL), VMXChannelCapability::I2C_SCL)}
        }; // SDA:26, SCL:27

        I2CConfig i2c_cfg;
        if (!vmx_->io.ActivateDualchannelResource(i2c_channels[0], i2c_channels[1], &i2c_cfg, cobra_res_handle, &vmxerr)) {

            RCLCPP_ERROR(this->get_logger(), "Error Activating DualChannel Resource for Channel index %d and %d.\n", i2c_channels[0].index, i2c_channels[1].index);
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully Activated I2C Resource.");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Error: Unable to open VMX Client.");
    }
    IsConnected();
}

bool CobraSensor::IsConnected()
{
    uint8_t partID = 0;
    VMXErrorCode vmxerr;
    if(!vmx_->io.I2C_Transaction(cobra_res_handle, static_cast<uint8_t>(deviceAddress), nullptr, 0, &partID, 1, &vmxerr))
    {
        printf("Cobra is not Connected!");
        return 0;
    }
    else
    {
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
    
    RCLCPP_INFO(this->get_logger(), "Publishing: %s", message.data.c_str());
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
    if (!vmx_->io.I2C_Transaction(cobra_res_handle, static_cast<uint8_t>(deviceAddress), tx_data, 3, rx_data, 0, &vmxerr)) {
        RCLCPP_ERROR(this->get_logger(), "Error configuring the ADC on I2C bus.");
        return 0;
    }

    // Wait for the ADC to perform the conversion
    CobraSensor::Delay(0.01);  // Small delay for the conversion

    // Read the conversion result from the ADC
    tx_data[0] = 0x00;  // Conversion result register
    if (!vmx_->io.I2C_Transaction(cobra_res_handle, static_cast<uint8_t>(deviceAddress), tx_data, 2, rx_data, 2, &vmxerr)) {
        RCLCPP_ERROR(this->get_logger(), "Error reading the ADC conversion result from I2C bus.");
        return 0;
    }

    // Combine the two bytes received from the ADC into a single 12-bit result
    int result = ((rx_data[0] << 8) | rx_data[1]) >> 4;  // 12-bit result is in the upper 12 bits

    return result;
}

int CobraSensor::GetRawValue(uint8_t channel) {
    return GetSingle(channel); 
}

float CobraSensor::GetVoltage(uint8_t channel) {
    float raw = GetSingle(channel); 
    float mV = vRef_ / 0x800;  //  <vRef_>V reference and 12-bit ADC
    return raw * mV;
}

void CobraSensor::Delay(double seconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds((int64_t)(seconds * 1000)));
}

// Main loop to manage publishing state
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