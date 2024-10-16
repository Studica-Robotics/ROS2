#ifndef ANALOG_INPUT_H
#define ANALOG_INPUT_H

#include "VMXPi.h"
#include <iostream>
#include <memory>

class AnalogInput {
public:
    AnalogInput(std::shared_ptr<VMXPi> vmx, VMXChannelIndex channel);
    bool activate_channel();
    bool get_average_voltage(float &voltage);
    bool is_active() const;

private:
    std::shared_ptr<VMXPi> vmx_;
    VMXChannelIndex channel_;
    VMXResourceHandle accumulator_res_handle;
    float full_scale_voltage;
    bool active_; 
};

#endif
