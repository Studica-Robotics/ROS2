#include "dio.h"

using namespace studica_driver;

DIO::DIO(VMXChannelIndex channel, PinMode mode, std::shared_ptr<VMXPi> vmx) 
    : channel_(channel), mode_(mode), vmx_(vmx) {
    VMXErrorCode vmxerr;
    if (mode == PinMode::OUTPUT) {
        DIOConfig dio_config;
        if (!vmx_->io.ActivateSinglechannelResource(VMXChannelInfo(channel, VMXChannelCapability::DigitalOutput), &dio_config, dio_res_handle_, &vmxerr)) {
            printf("Failed to open DIO resource on port %d", channel);
            DisplayVMXError(vmxerr);
        } else {
            printf("DIO Channel %d activated on Resource type %d, index %d\n", channel,
                EXTRACT_VMX_RESOURCE_TYPE(dio_res_handle_),
                EXTRACT_VMX_RESOURCE_INDEX(dio_res_handle_));
            initialized_ = true;
        }
    } else if (mode == PinMode::INPUT) {
        DIOConfig dio_config; 
        if (!vmx_->io.ActivateSinglechannelResource(VMXChannelInfo(channel, VMXChannelCapability::DigitalInput), &dio_config, dio_res_handle_, &vmxerr)) {
            printf("Failed to open DIO resource on port %d", channel_);
            DisplayVMXError(vmxerr);
        } else {
            printf("DIO Channel %d activated on Resource type %d, index %d\n", channel,
                EXTRACT_VMX_RESOURCE_TYPE(dio_res_handle_),
                EXTRACT_VMX_RESOURCE_INDEX(dio_res_handle_));
            initialized_ = true;
        }
    }
}
DIO::~DIO() {
    if (!initialized_) return; // nothing to deactivate
    VMXErrorCode vmxerr;
    if (!vmx_->io.DeactivateResource(dio_res_handle_, &vmxerr)) {
        printf("Failed to deactivate DIO resource on port %d", channel_);
        DisplayVMXError(vmxerr);
    }
}

void DIO::Set(bool value) {
    if (!initialized_) {
        printf("Attempt to Set on uninitialized DIO port %d ignored.\n", channel_);
        return;
    }
    VMXErrorCode vmxerr;
    if (!vmx_->io.DIO_Set(dio_res_handle_, value, &vmxerr)) {
        printf("Error setting DIO on port %d", channel_);
        DisplayVMXError(vmxerr);
    }
}

bool DIO::Get() {
    if (!initialized_) {
        printf("Attempt to Get on uninitialized DIO port %d. Returning false.\n", channel_);
        return false;
    }
    VMXErrorCode vmxerr;
    bool value = false;
    if (!vmx_->io.DIO_Get(dio_res_handle_, value, &vmxerr)) {
        printf("Error getting DIO on port %d", channel_);
        DisplayVMXError(vmxerr);
    }
    return value;
}

void DIO::Toggle() {
    if (!initialized_) {
        printf("Attempt to Toggle on uninitialized DIO port %d ignored.\n", channel_);
        return;
    }
    if (mode_ != PinMode::OUTPUT) {
        printf("Toggle operation is only supported for OUTPUT mode on port %d\n", channel_);
        return;
    }

    VMXErrorCode vmxerr;
    bool current_value = false;

    if (!vmx_->io.DIO_Get(dio_res_handle_, current_value, &vmxerr)) {
        printf("Error getting current DIO state on port %d\n", channel_);
        DisplayVMXError(vmxerr);
        return;
    }

    bool new_value = !current_value;

    if (!vmx_->io.DIO_Set(dio_res_handle_, new_value, &vmxerr)) {
        printf("Error toggling DIO state on port %d\n", channel_);
        DisplayVMXError(vmxerr);
    } else {
        printf("DIO port %d toggled to %s\n", channel_, new_value ? "HIGH" : "LOW");
    }
}

void DIO::DisplayVMXError(VMXErrorCode vmxerr) {
    const char *p_err_description = GetVMXErrorString(vmxerr);
    printf("VMXError %d: %s\n", vmxerr, p_err_description);
}
