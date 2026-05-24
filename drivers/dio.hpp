#pragma once

#include "VMXPi.h"
#include <functional>
#include <memory>
#include <stdio.h>

namespace studica_driver
{

    enum class PinMode
    {
        INPUT,
        OUTPUT
    };

    // callback signature: (pin_state_at_interrupt, edge_type)
    using InterruptCallback = std::function<void(bool, InterruptEdgeType)>;

    class DIO
    {
        public:
            DIO(VMXChannelIndex channel, PinMode mode, std::shared_ptr<VMXPi> vmx = std::make_shared<VMXPi>(true, 50));
            ~DIO();

            void Set(bool value);
            bool Get();
            void Toggle();

            // Attach a hardware interrupt to this pin (INPUT mode only).
            // edge: InterruptConfig::RISING or InterruptConfig::FALLING
            // callback fires in a VMX background thread — keep it short and non-blocking.
            // Returns true if the interrupt resource was activated successfully.
            bool EnableInterrupt(InterruptConfig::EdgeType edge, InterruptCallback callback);

            // Remove the interrupt — safe to call if never enabled.
            void DisableInterrupt();

            bool IsInitialized() const { return initialized_; }
            bool IsInterruptEnabled() const { return interrupt_enabled_; }

        private:
            VMXChannelIndex channel_;
            PinMode mode_;
            std::shared_ptr<VMXPi> vmx_;
            VMXResourceHandle dio_res_handle_       = 0;
            VMXResourceHandle interrupt_res_handle_ = 0;
            bool initialized_       = false;
            bool interrupt_enabled_ = false;

            InterruptCallback interrupt_callback_;

            // C-style trampoline required by the VMX interrupt API.
            // Casts param back to DIO* and calls interrupt_callback_.
            static void interrupt_trampoline(uint32_t io_interrupt_num,
                                             InterruptEdgeType edge,
                                             void *param,
                                             uint64_t timestamp_us);

            void DisplayVMXError(VMXErrorCode vmxerr);
    };

} // namespace studica_driver
