#include "ultrasonic.h"
using namespace studica_driver;

Ultrasonic::Ultrasonic(VMXChannelIndex ping, VMXChannelIndex echo, std::shared_ptr<VMXPi> vmx) 
    : ping_(ping), echo_(echo), vmx_(vmx) { 
    VMXErrorCode vmxerr;
    if (ping_ >= 8 && ping_ <= 21) {
        if (echo_ >= 8 && echo_ <= 21) {
            DIOConfig dio_config(DIOConfig::OutputMode::PUSHPULL);
            if (!vmx_->io.ActivateSinglechannelResource(VMXChannelInfo(ping_, VMXChannelCapability::DigitalOutput), &dio_config, ping_output_res_handle, &vmxerr)) {
                printf("Failed to open ping output resource on port %d", ping_);
                DisplayVMXError(vmxerr);
            } else {
                printf("Ping (Digital Output) Channel %d activated on Resource type %d, index %d\n", ping_,
                    EXTRACT_VMX_RESOURCE_TYPE(ping_output_res_handle),
                    EXTRACT_VMX_RESOURCE_INDEX(ping_output_res_handle));
            }
            
            // Configure echo
            InputCaptureConfig inputcap_config;
            // Slave mode reset is used
            inputcap_config.SetSlaveMode(InputCaptureConfig::SLAVEMODE_RESET);
            // Rising edge input is dynamic
            inputcap_config.SetSlaveModeTriggerSource(InputCaptureConfig::TRIGGER_DYNAMIC);
            // The dynamically-selected VMXChannel should also be routed to both timer channels
            inputcap_config.SetCaptureChannelSource(InputCaptureConfig::CH1, InputCaptureConfig::CAPTURE_SIGNAL_DYNAMIC);
            inputcap_config.SetCaptureChannelSource(InputCaptureConfig::CH2, InputCaptureConfig::CAPTURE_SIGNAL_DYNAMIC);
            // Check to see which physical pin is driving the timer
            if (vmx_->io.ChannelSupportsCapability(echo_, VMXChannelCapability::InputCaptureInput2)) {
                // Second Timer Input:  Second Timer Channel must handle the rising edge
                inputcap_config.SetCaptureChannelActiveEdge(InputCaptureConfig::CH1, InputCaptureConfig::ACTIVE_FALLING);
                inputcap_config.SetCaptureChannelActiveEdge(InputCaptureConfig::CH2, InputCaptureConfig::ACTIVE_RISING);
            } else {
                // First Timer Input:  First Timer Channel must handle the rising edge
                inputcap_config.SetCaptureChannelActiveEdge(InputCaptureConfig::CH1, InputCaptureConfig::ACTIVE_RISING);
                inputcap_config.SetCaptureChannelActiveEdge(InputCaptureConfig::CH2, InputCaptureConfig::ACTIVE_FALLING);
            }
            // Capture should occur on each and every input signal edge transition
            inputcap_config.SetCaptureChannelPrescaler(InputCaptureConfig::CH1, InputCaptureConfig::x1);
            inputcap_config.SetCaptureChannelPrescaler(InputCaptureConfig::CH2, InputCaptureConfig::x1);

            // Digitally filter 2 successive samples when looking for edges
            uint8_t filter_number = inputcap_config.GetClosestCaptureCaptureFilterNumSamples(2);
            inputcap_config.SetCaptureChannelFilter(InputCaptureConfig::CH1, filter_number);
            inputcap_config.SetCaptureChannelFilter(InputCaptureConfig::CH2, filter_number);

            VMXChannelInfo echo_chaninfo(echo_, VMXChannelCapability::InputCaptureInput2);

            // If the input is no longer present, do not automatically clear the Capture Channel Count
            inputcap_config.SetStallAction(InputCaptureConfig::ACTION_NONE);

            if (!vmx_->io.ActivateSinglechannelResource(echo_chaninfo, &inputcap_config, echo_inputcap_res_handle, &vmxerr)) {
                printf("Failed to open echo input resource on port %d", echo_);
                DisplayVMXError(vmxerr);
            } else {
                printf("Echo (Input Capture) Channel %d activated on Resource type %d, index %d\n", echo_,
                    EXTRACT_VMX_RESOURCE_TYPE(echo_inputcap_res_handle),
                    EXTRACT_VMX_RESOURCE_INDEX(echo_inputcap_res_handle));
            }
        } else {
            printf("Invalid echo port %d", echo_);
        }
    } else {
        printf("Invalid ping port %d", ping_);
    }
    
}

Ultrasonic::~Ultrasonic() {
    VMXErrorCode vmxerr;
    if (!vmx_->io.DeactivateResource(ping_output_res_handle, &vmxerr)) {
        printf("Failed to deactivate ping output resource on port %d", ping_);
        DisplayVMXError(vmxerr);
    }
    if (!vmx_->io.DeactivateResource(echo_inputcap_res_handle, &vmxerr)) {
        printf("Failed to deactivate echo input resource on port %d", echo_);
        DisplayVMXError(vmxerr);
    }
}

void Ultrasonic::Ping() {
    VMXErrorCode vmxerr;

    if (!vmx_->io.DIO_Pulse(ping_output_res_handle, true, 10, &vmxerr)) {
        printf("Error triggering pulse on port %d", ping_);
        DisplayVMXError(vmxerr);
    }

    vmx_->time.DelayMilliseconds(5); // Wait for echo to return
}

float Ultrasonic::GetDistanceIN() {
    uint32_t microseconds_per_inch = 148;
    return get_count() / microseconds_per_inch;
}

float Ultrasonic::GetDistanceMM() {
    uint32_t microseconds_per_mm = 58;
    return get_count() / microseconds_per_mm;
}

// Method to read distance and publish
float Ultrasonic::get_count() {
    VMXErrorCode vmxerr;
    uint32_t ch1_count = 0;
    uint32_t ch2_count = 0;

    if (!vmx_->io.InputCapture_GetChannelCounts(echo_inputcap_res_handle, ch1_count, ch2_count, &vmxerr)) {
        printf("Failed to get echo count on port %d", echo_);
        DisplayVMXError(vmxerr);
    } else {
        return ch2_count;
    }
    return -1;
}

void Ultrasonic::DisplayVMXError(VMXErrorCode vmxerr) {
    const char *p_err_description = GetVMXErrorString(vmxerr);
    printf("VMXError %d: %s\n", vmxerr, p_err_description);
}