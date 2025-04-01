#ifndef DUTY_CYCLE_ENCODER_H
#define DUTY_CYCLE_ENCODER_H

#include <memory>
#include <stdio.h>

#include "VMXPi.h"

namespace studica_driver {

class DutyCycleEncoder {
public:
    DutyCycleEncoder(VMXChannelIndex port, std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50));
    ~DutyCycleEncoder();
    double GetAbsolutePosition();
    int GetRolloverCount();
    double GetTotalRotation();

private:
    VMXChannelIndex port_;
    std::shared_ptr<VMXPi> vmx_;
    
    VMXResourceHandle encoder_res_handle_;
    uint16_t low_ticks_ = 256;
    uint16_t high_ticks_ = 784;

    void DisplayVMXError(VMXErrorCode vmxerr);
};

} // namespace studica_driver

#endif // DUTY_CYCLE_ENCODER_H
