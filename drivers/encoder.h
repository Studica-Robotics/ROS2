#ifndef ENCODER_H
#define ENCODER_H

#include <stdio.h>
#include <memory>
#include "VMXPi.h"

namespace studica_driver {

class Encoder {
public:
    Encoder(VMXChannelIndex port_a, VMXChannelIndex port_b, std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50));
    ~Encoder();
    int GetCount();
    std::string GetDirection();

private:
    std::shared_ptr<VMXPi> vmx_;
    VMXChannelIndex port_a_;
    VMXChannelIndex port_b_;
    VMXResourceHandle encoder_res_handle_;
    
    void DisplayVMXError(VMXErrorCode vmxerr);
};

}

#endif // ENCODER_H
