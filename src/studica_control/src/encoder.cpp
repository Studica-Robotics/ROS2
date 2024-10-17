#include "studica_control/encoder.h"
using namespace studica_control;

Encoder::Encoder(std::shared_ptr<VMXPi> vmx, VMXChannelIndex port_a, VMXChannelIndex port_b)
    : Device("encoder_"), vmx_(vmx), port_a_(port_a), port_b_(port_b), encoder_res_handle_(CREATE_VMX_RESOURCE_HANDLE(VMXResourceType::Undefined, INVALID_VMX_RESOURCE_INDEX)) {

    VMXChannelInfo enc_channels[2] = {
        { VMXChannelInfo(port_a_, VMXChannelCapability::EncoderAInput) },
        { VMXChannelInfo(port_b_, VMXChannelCapability::EncoderBInput) }
    };
    
    EncoderConfig encoder_cfg(EncoderConfig::EncoderEdge::x4);
    VMXErrorCode vmxerr;
    
    if (!vmx_->io.ActivateDualchannelResource(enc_channels[0], enc_channels[1], &encoder_cfg, encoder_res_handle_, &vmxerr)) {
        printf("Failed to activate Encoder Resource for channels %d and %d\n", port_a_, port_b_);
        DisplayVMXError(vmxerr);
        vmx_->io.DeallocateResource(encoder_res_handle_, &vmxerr);
    } else {
        printf("Successfully activated Encoder Resource on channels %d and %d\n", port_a_, port_b_);
    }
}

void Encoder::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (params == "get_count") {
        int count = GetCount();
        response->message = "Encoder count: " + std::to_string(count);
    } else if (params == "get_direction") {
        std::string direction = GetDirection();
        response->message = "Encoder direction: " + direction;
    } else {
        response->message = "Invalid command";
    }
}

int Encoder::GetCount() {
    int32_t counter = 0;
    VMXErrorCode vmxerr;

    if (vmx_->io.Encoder_GetCount(encoder_res_handle_, counter, &vmxerr)) {
        printf("Encoder count: %d\n", counter);
        return counter;
    } else {
        printf("Error retrieving Encoder count.\n");
        DisplayVMXError(vmxerr);
        return -1;
    }
}

std::string Encoder::GetDirection() {
    VMXIO::EncoderDirection encoder_direction;
    VMXErrorCode vmxerr;

    if (vmx_->io.Encoder_GetDirection(encoder_res_handle_, encoder_direction, &vmxerr)) {
        return (encoder_direction == VMXIO::EncoderForward) ? "Forward" : "Reverse";
    } else {
        printf("Error retrieving Encoder direction.\n");
        DisplayVMXError(vmxerr);
        return "Error";
    }
}

void Encoder::DisplayVMXError(VMXErrorCode vmxerr) {
    const char* p_err_description = GetVMXErrorString(vmxerr);
    printf("VMXError %d: %s\n", vmxerr, p_err_description);
}
