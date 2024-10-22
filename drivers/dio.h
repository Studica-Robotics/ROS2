
#ifndef DIO_H
#define DIO_H

#include <stdio.h>
#include <memory>
#include "VMXPi.h"

namespace studica_driver
{

enum class PinMode { INPUT, OUTPUT };
class DIO {
public:
    DIO(VMXChannelIndex channel, PinMode mode);
    ~DIO();

    void Set(bool value);
    bool Get();
    void Toggle();
private:
    std::shared_ptr<VMXPi> vmx_;
    VMXChannelIndex channel_;
    VMXResourceHandle dio_res_handle_;
    PinMode mode_;
    void DisplayVMXError(VMXErrorCode vmxerr);
};

}

#endif // DIO_H
