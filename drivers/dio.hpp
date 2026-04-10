#pragma once

#include "VMXPi.h"
#include <memory>
#include <stdio.h>

namespace studica_driver
{

    enum class PinMode
    {
        INPUT,
        OUTPUT
    };
    class DIO
    {
        public:
            DIO(VMXChannelIndex channel, PinMode mode, std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50));
            ~DIO();

            void Set(bool value);
            bool Get();
            void Toggle();
            bool IsInitialized() const
            {
                return initialized_;
            }

        private:
            VMXChannelIndex channel_;
            PinMode mode_;
            std::shared_ptr<VMXPi> vmx_;
            VMXResourceHandle dio_res_handle_ = 0; // ensure deterministic value if activation fails
            bool initialized_ = false;
            void DisplayVMXError(VMXErrorCode vmxerr);
    };

} // namespace studica_driver
