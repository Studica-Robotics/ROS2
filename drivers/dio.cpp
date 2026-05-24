#include "dio.hpp"

using namespace studica_driver;

DIO::DIO(VMXChannelIndex channel, PinMode mode, std::shared_ptr<VMXPi> vmx)
    : channel_(channel)
    , mode_(mode)
    , vmx_(vmx)
{
    if (!vmx_ || !vmx_->IsOpen())
    {
        printf("VMX HAL unavailable; skipping DIO initialisation on port %d\n", channel_);
        return;
    }

    VMXErrorCode vmxerr;
    if (mode == PinMode::OUTPUT)
    {
        DIOConfig dio_config(DIOConfig::PUSHPULL);
        if (!vmx_->io.ActivateSinglechannelResource(VMXChannelInfo(channel, VMXChannelCapability::DigitalOutput),
                                                    &dio_config, dio_res_handle_, &vmxerr))
        {
            printf("Failed to open DIO resource on port %d", channel);
            DisplayVMXError(vmxerr);
        }
        else
        {
            printf("DIO Channel %d activated on Resource type %d, index %d\n", channel,
                   EXTRACT_VMX_RESOURCE_TYPE(dio_res_handle_), EXTRACT_VMX_RESOURCE_INDEX(dio_res_handle_));
            initialized_ = true;
        }
    }
    else if (mode == PinMode::INPUT)
    {
        DIOConfig dio_config;
        dio_config.SetInputMode(DIOConfig::InputMode::PULLUP);
        if (!vmx_->io.ActivateSinglechannelResource(VMXChannelInfo(channel, VMXChannelCapability::DigitalInput),
                                                    &dio_config, dio_res_handle_, &vmxerr))
        {
            printf("Failed to open DIO resource on port %d", channel_);
            DisplayVMXError(vmxerr);
        }
        else
        {
            printf("DIO Channel %d activated on Resource type %d, index %d\n", channel,
                   EXTRACT_VMX_RESOURCE_TYPE(dio_res_handle_), EXTRACT_VMX_RESOURCE_INDEX(dio_res_handle_));
            initialized_ = true;
        }
    }
}

DIO::~DIO()
{
    DisableInterrupt();   // clean up interrupt resource before the DIO resource
    if (!initialized_)
        return;
    VMXErrorCode vmxerr;
    if (!vmx_->io.DeactivateResource(dio_res_handle_, &vmxerr))
    {
        printf("Failed to deactivate DIO resource on port %d", channel_);
        DisplayVMXError(vmxerr);
    }
}


// ---------------------------------------------------------------------------
// Interrupt support
// ---------------------------------------------------------------------------

bool DIO::EnableInterrupt(InterruptConfig::EdgeType edge, InterruptCallback callback)
{
    if (!initialized_)
    {
        printf("Cannot enable interrupt on uninitialized DIO port %d\n", channel_);
        return false;
    }
    if (mode_ != PinMode::INPUT)
    {
        printf("Interrupts are only supported on INPUT pins (port %d)\n", channel_);
        return false;
    }
    if (interrupt_enabled_)
    {
        printf("Interrupt already enabled on port %d; disable first\n", channel_);
        return false;
    }

    interrupt_callback_ = std::move(callback);

    VMXErrorCode vmxerr;
    // The VMX API requires InterruptConfig's C-style function pointer.
    // We store the std::function in interrupt_callback_ and use a static
    // trampoline that casts param (== this) back to DIO*.
    InterruptConfig int_config(edge, interrupt_trampoline, static_cast<void *>(this));

    if (!vmx_->io.ActivateSinglechannelResource(
            VMXChannelInfo(channel_, VMXChannelCapability::InterruptInput),
            &int_config, interrupt_res_handle_, &vmxerr))
    {
        printf("Failed to activate interrupt resource on port %d\n", channel_);
        DisplayVMXError(vmxerr);
        interrupt_callback_ = nullptr;
        return false;
    }

    interrupt_enabled_ = true;
    printf("Interrupt enabled on DIO port %d (%s edge)\n",
           channel_,
           (edge == InterruptConfig::RISING) ? "rising" : "falling");
    return true;
}

void DIO::DisableInterrupt()
{
    if (!interrupt_enabled_)
        return;
    VMXErrorCode vmxerr;
    vmx_->io.DeactivateResource(interrupt_res_handle_, &vmxerr);
    interrupt_enabled_ = false;
    interrupt_callback_ = nullptr;
}

// Static trampoline — called by the VMX HAL in its own interrupt thread.
// Reads the current pin state (after the edge) and forwards to the stored callback.
void DIO::interrupt_trampoline(uint32_t /*io_interrupt_num*/,
                                InterruptEdgeType edge,
                                void *param,
                                uint64_t /*timestamp_us*/)
{
    auto *self = static_cast<DIO *>(param);
    if (self && self->interrupt_callback_)
    {
        bool pin_state = self->Get();
        self->interrupt_callback_(pin_state, edge);
    }
}


// ---------------------------------------------------------------------------
// Set / Get / Toggle
// ---------------------------------------------------------------------------

void DIO::Set(bool value)
{
    if (!initialized_)
    {
        printf("Attempt to Set on uninitialized DIO port %d ignored.\n", channel_);
        return;
    }
    VMXErrorCode vmxerr;
    if (!vmx_->io.DIO_Set(dio_res_handle_, value, &vmxerr))
    {
        printf("Error setting DIO on port %d", channel_);
        DisplayVMXError(vmxerr);
    }
}

bool DIO::Get()
{
    if (!initialized_)
    {
        printf("Attempt to Get on uninitialized DIO port %d. Returning false.\n", channel_);
        return false;
    }
    VMXErrorCode vmxerr;
    bool value = false;
    if (!vmx_->io.DIO_Get(dio_res_handle_, value, &vmxerr))
    {
        printf("Error getting DIO on port %d", channel_);
        DisplayVMXError(vmxerr);
    }
    return value;
}

void DIO::Toggle()
{
    if (!initialized_)
    {
        printf("Attempt to Toggle on uninitialized DIO port %d ignored.\n", channel_);
        return;
    }
    if (mode_ != PinMode::OUTPUT)
    {
        printf("Toggle operation is only supported for OUTPUT mode on port %d\n", channel_);
        return;
    }

    VMXErrorCode vmxerr;
    bool current_value = false;

    if (!vmx_->io.DIO_Get(dio_res_handle_, current_value, &vmxerr))
    {
        printf("Error getting current DIO state on port %d\n", channel_);
        DisplayVMXError(vmxerr);
        return;
    }

    bool new_value = !current_value;

    if (!vmx_->io.DIO_Set(dio_res_handle_, new_value, &vmxerr))
    {
        printf("Error toggling DIO state on port %d\n", channel_);
        DisplayVMXError(vmxerr);
    }
    else
    {
        printf("DIO port %d toggled to %s\n", channel_, new_value ? "HIGH" : "LOW");
    }
}

void DIO::DisplayVMXError(VMXErrorCode vmxerr)
{
    const char *p_err_description = GetVMXErrorString(vmxerr);
    printf("VMXError %d: %s\n", vmxerr, p_err_description);
}
