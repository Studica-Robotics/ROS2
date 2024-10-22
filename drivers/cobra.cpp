#include "cobra.h"
using namespace studica_driver;

Cobra::Cobra(const float &vref, const uint8_t &muxch, std::shared_ptr<VMXPi> vmx)
    : vmx_(vmx), vref_(vref), muxch_(muxch) {
    
    // init cobra variables
    port = 1;
    mode = CONFIG_MODE_CONT;
    gain = CONFIG_PGA_2;
    sampleRate = CONFIG_RATE_1600HZ;
    multiplierVolts = 1.0F;

    if (!vmx_->IsOpen()) {
        std::cerr << "VMX is not open." << std::endl;
    } else {
        i2c_ = std::make_shared<studica_driver::I2C>(vmx_);
        deviceAddress = i2c_->scanI2CBus();
        if (deviceAddress == 0) {
            std::cout << "No I2C devices found" << std::endl;
        }
    }
}
Cobra::~Cobra() {}


int Cobra::GetSingle() {
    if (muxch_ > 3) {
        std::cout << "Invalid multiplexer ch: " << muxch_ << std::endl;
        return 0;
    }

    VMXErrorCode vmxerr;
    int config = CONFIG_OS_SINGLE | mode | sampleRate;
    config |= gain;

    // set the correct multiplexer setting for the selected muxch_
    switch (muxch_) {
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
            std::cout << "Invalid multiplexer muxch_: " << muxch_ << std::endl;
            return 0;
    }

    // prep to send to ADC
    uint8_t tx_data[3];
    uint8_t rx_data[2];

    tx_data[0] = 0x0001;  // Configuration register
    tx_data[1] = config >> 8;  // MSB of config
    tx_data[2] = config & 0xFF;  // LSB of config

    // send config to ADC
    if (!i2c_->i2cTransaction(static_cast<uint8_t>(deviceAddress), tx_data, 3, nullptr, 0)) {
        std::cout << "Error configuring the ADC on I2C bus." << std::endl;
        return 0;
    }

    // wait 100ms for ADC to convert
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // delay for 100ms

    // read ADC conversion result
    tx_data[0] = 0x00;
    if (!i2c_->i2cTransaction(static_cast<uint8_t>(deviceAddress), tx_data, 2, rx_data, 2)) {
        std::cout << "Error reading the ADC conversion result from I2C bus." << std::endl;
        return 0;
    }

    return (rx_data[0] << 8) | rx_data[1]; // Combine the received bytes into an integer
}

int Cobra::GetRawValue() {
    return GetSingle();
}

float Cobra::GetVoltage() {
    float raw = GetSingle(); 
    float mV = vref_ / 0x800;  //  <vRef_>V reference and 12-bit ADC
    return raw * mV;
}