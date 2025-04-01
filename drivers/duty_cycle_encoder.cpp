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

double DutyCycleEncoder::GetAbsolutePosition() {
    VMXErrorCode vmxerr;
    uint32_t chan1_counts;
    uint32_t chan2_counts;
    if (vmx_->io.InputCapture_GetChannelCounts(encoder_res_handle_, chan1_counts, chan2_counts, &vmxerr)) {
        if (chan1_counts == 0 || chan2_counts > chan1_counts) return -2.0;
        double duty_cycle = (static_cast<double>(chan2_counts) * 360.0) / (chan1_counts - 7);
        double cycle_to_remove = (7.0 * 360.0) / chan1_counts;
        duty_cycle -= cycle_to_remove;
        if (duty_cycle < 0.0) duty_cycle = 0.0;
        if (duty_cycle > ((4095.0 * 360.0) / chan1_counts)) duty_cycle = (4095.0 * 360.0) / chan1_counts;
        return duty_cycle;
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

double DutyCycleEncoder::GetTotalRotation() {
    return GetRolloverCount() * 360 + GetAbsolutePosition();
}

void DutyCycleEncoder::DisplayVMXError(VMXErrorCode vmxerr) {
    printf("VMXError %d: %s\n", vmxerr, GetVMXErrorString(vmxerr));
}
