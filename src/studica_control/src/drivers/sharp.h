#ifndef SHAPE_H
#define SHAPE_H

#include <stdio.h>
#include "VMXPi.h"

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
    VMXChannelIndex port_;
    VMXResourceHandle accumulator_res_handle;
    void DisplayVMXError(VMXErrorCode vmxerr);
};

}

#endif // SHAPE_H