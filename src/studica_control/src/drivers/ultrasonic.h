#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdio.h>
#include "VMXPi.h"

namespace studica_driver
{

class Ultrasonic
{
public:
    Ultrasonic(VMXChannelIndex ping, VMXChannelIndex echo);
    Ultrasonic(VMXChannelIndex ping, VMXChannelIndex echo, std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50));
    ~Ultrasonic();

    float Ping();
    float GetDistanceIN();
    float GetDistanceMM();

private:
    std::shared_ptr<VMXPi> vmx_;
    VMXChannelIndex ping_;
    VMXChannelIndex echo_;
    VMXResourceHandle ping_output_res_handle;
    VMXResourceHandle echo_inputcap_res_handle;
    void DisplayVMXError(VMXErrorCode vmxerr);
};

}

#endif // ULTRASONIC_H

