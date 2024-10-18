#ifndef ANALOG_INPUT_H
#define ANALOG_INPUT_H

#include <stdio.h>
#include "VMXPi.h"

namespace studica_driver
{

class AnalogInput
{
public:
    AnalogInput(VMXChannelIndex port, std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50));
    ~AnalogInput();

    float GetAverageVoltage();

private:
    std::shared_ptr<VMXPi> vmx_;
    VMXChannelIndex port_;
    VMXResourceHandle accumulator_res_handle_;
    void DisplayVMXError(VMXErrorCode vmxerr);
};

} // namespace studica_driver

#endif // ANALOG_INPUT_H