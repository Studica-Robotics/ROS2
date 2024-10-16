#include "studica_control/analog_input.h"

AnalogInput::AnalogInput(std::shared_ptr<VMXPi> vmx, VMXChannelIndex channel)
    : vmx_(vmx), channel_(channel), accumulator_res_handle(VMXResourceHandle()), active_(false) {
    // Get full-scale voltage of analog input
    VMXErrorCode vmxerr;
    if (vmx_->io.Accumulator_GetFullScaleVoltage(full_scale_voltage, &vmxerr)) {
        std::cout << "Analog input voltage: " << full_scale_voltage << std::endl;
    } else {
        std::cerr << "ERROR acquiring Analog Input Voltage." << std::endl;
    }
}

bool AnalogInput::activate_channel() {
    VMXErrorCode vmxerr;
    AccumulatorConfig accum_config;
    accum_config.SetNumAverageBits(9);
    
    // Configure the analog accumulator for the specified channel
    if (!vmx_->io.ActivateSinglechannelResource(
            VMXChannelInfo(channel_, VMXChannelCapability::AccumulatorInput),
            &accum_config, 
            accumulator_res_handle, 
            &vmxerr)) {
        std::cerr << "Error activating analog channel " << channel_ << std::endl;
        active_ = false;
        return false;
    }

    std::cout << "Analog Input Channel " << channel_ << " activated." << std::endl;
    active_ = true;
    return true;
}

bool AnalogInput::get_average_voltage(float &voltage) {
    VMXErrorCode vmxerr;
    
    if (vmx_->io.Accumulator_GetAverageVoltage(accumulator_res_handle, voltage, &vmxerr)) {
        return true;
    } else {
        std::cerr << "Error getting Average Voltage for channel " << channel_ << std::endl;
        return false;
    }
}

bool AnalogInput::is_active() const {
    return active_;
}
