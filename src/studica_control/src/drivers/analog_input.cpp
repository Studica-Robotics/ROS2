#include "analog_input.h"
using namespace studica_driver;

AnalogInput::AnalogInput(VMXChannelIndex port, std::shared_ptr<VMXPi> vmx)
    : vmx_(vmx), port_(port), accumulator_res_handle_(VMXResourceHandle()) {
    
    VMXErrorCode vmxerr;
    if (!vmx_->IsOpen()) {
        std::cout << "VMX is not open." << full_scale_voltage << std::endl;
    } else {
        if (vmx_->io.Accumulator_GetFullScaleVoltage(full_scale_voltage, &vmxerr)) {
            std::cout << "Analog input voltage: " << full_scale_voltage << std::endl;
        } else {
            std::cerr << "ERROR acquiring Analog Input Voltage." << std::endl;
        }
    }

    AccumulatorConfig accum_config;
    accum_config.SetNumAverageBits(9);
    
    // Configure the analog accumulator for the specified channel
    if (!vmx_->io.ActivateSinglechannelResource(
            VMXChannelInfo(port_, VMXChannelCapability::AccumulatorInput),
            &accum_config, 
            accumulator_res_handle, 
            &vmxerr)) {
        std::cerr << "Error activating analog channel " << port_ << std::endl;
        active_ = false;
        return false;
    }

    std::cout << "Analog Input Channel " << port_ << " activated." << std::endl;
}

float AnalogInput::GetAverageVoltage(float &volt) {
    VMXErrorCode vmxerr;
    
    if (vmx_->io.Accumulator_GetAverageVoltage(accumulator_res_handle, volt, &vmxerr)) {
        return true;
    } else {
        std::cerr << "Error getting Average Voltage for channel " << channel_ << std::endl;
        return false;
    }
}
