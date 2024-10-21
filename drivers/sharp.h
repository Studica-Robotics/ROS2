#ifndef SHARP_H
#define SHARP_H

#include <stdio.h>
#include <cmath>
#include "VMXPi.h"
#include "analog_input.h"

namespace studica_driver
{

class Sharp
{
public:
    Sharp(VMXChannelIndex port);
    Sharp(VMXChannelIndex port, std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50));
    ~Sharp();

    float GetDistance();

private:
    std::shared_ptr<VMXPi> vmx_;
    std::shared_ptr<AnalogInput> analog_input_;
    VMXChannelIndex port_;
    // void DisplayVMXError(VMXErrorCode vmxerr);
};

}

#endif // SHARP_H