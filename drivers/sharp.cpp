#include "sharp.h"
using namespace studica_driver;

Sharp::Sharp(VMXChannelIndex port, std::shared_ptr<VMXPi> vmx)
    : vmx_(vmx), port_(port) {
    if (!vmx_->IsOpen()) {
       std::cerr << "VMX is not open." << std::endl;
    } else {
        std::cout << "sharp init complete, creating analog_input_:" << std::endl;
        analog_input_ = std::make_shared<AnalogInput>(port_, vmx_);
    }
}
Sharp::~Sharp() {}

float Sharp::GetDistance() {
    float averageVoltage;
    if (analog_input_->GetAverageVoltage(averageVoltage)) {
        return pow(averageVoltage, -1.2045) * 27.726;
    }
    std::cerr << "Failed to retreive Average Voltage from AnalogInput." << std::endl;
    return -1;
}

float Sharp::GetVoltage() {
    float averageVoltage;
    if (analog_input_->GetAverageVoltage(averageVoltage)) {
        return averageVoltage;
    }
    std::cerr << "Failed to retreive Average Voltage from AnalogInput." << std::endl;
    return -1;
}