#include "duty_cycle_encoder.h"
using namespace studica_driver;

DutyCycleEncoder::DutyCycleEncoder(VMXChannelIndex port, std::shared_ptr<VMXPi> vmx) 
    : port_(port), vmx_(vmx) {
    VMXErrorCode vmxerr;
    VMXChannelInfo duty_cycle_encoder_cap_channels[1] = {
        { VMXChannelInfo(port_, VMXChannelCapability::PWMCaptureInput) }
    };

    PWMCaptureConfig pwmcap_cfg(1, PWMCaptureConfig::PWMCaptureTimeout::x2);
    pwmcap_cfg.SetVirtualCounterMode(InputCaptureConfig::VC_MODE_DUTYCYCLE_ENCODER);
    pwmcap_cfg.SetVirtualCounterSource(InputCaptureConfig::VC_SOURCE_CH2);
    pwmcap_cfg.SetVirtualCounterDutyCycleEncoderLowTicks(low_ticks_);
    pwmcap_cfg.SetVirtualCounterDutyCycleEncoderHighTicks(high_ticks_);

    if (!vmx_->io.ActivateSinglechannelResource(duty_cycle_encoder_cap_channels[0], &pwmcap_cfg, encoder_res_handle_, &vmxerr)) {
        DisplayVMXError(vmxerr);
    }
}

DutyCycleEncoder::~DutyCycleEncoder() {
    VMXErrorCode vmxerr;
    vmx_->io.DeallocateResource(encoder_res_handle_, &vmxerr);
}

int DutyCycleEncoder::GetAbsolutePosition() {
    VMXErrorCode vmxerr;
    uint32_t duty_cycle_us;
    uint32_t frequency_us;
    if (vmx_->io.PWMCapture_GetCount(encoder_res_handle_, frequency_us, duty_cycle_us, &vmxerr)) {
        return static_cast<int>(360 * (static_cast<float>(duty_cycle_us) / 1024.0f));
    }
    DisplayVMXError(vmxerr);
    return -1;
}

int DutyCycleEncoder::GetRolloverCount() {
    VMXErrorCode vmxerr;
    int32_t rollover_count;
    if (vmx_->io.InputCapture_GetCount(encoder_res_handle_, rollover_count, &vmxerr)) {
        return rollover_count;
    }
    DisplayVMXError(vmxerr);
    return -1;
}

int DutyCycleEncoder::GetTotalRotation() {
    return GetRolloverCount() * 360 + GetAbsolutePosition();
}

void DutyCycleEncoder::DisplayVMXError(VMXErrorCode vmxerr) {
    printf("VMXError %d: %s\n", vmxerr, GetVMXErrorString(vmxerr));
}
