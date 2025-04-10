#include "encoder.h"
using namespace studica_driver;

Encoder::Encoder(VMXChannelIndex port_a, VMXChannelIndex port_b, std::shared_ptr<VMXPi> vmx)
    : vmx_(vmx), port_a_(port_a), port_b_(port_b), encoder_res_handle_(CREATE_VMX_RESOURCE_HANDLE(VMXResourceType::Undefined, INVALID_VMX_RESOURCE_INDEX)) 
{
    if (!vmx_->IsOpen()) {
        printf("Error:  Unable to open VMX Client.\n");
        printf("\n");
        printf("        - Is pigpio (or the system resources it requires) in use by another process?\n");
        printf("        - Does this application have root privileges?\n");
        return;
    }

    VMXChannelInfo enc_channels[2] = {
        { VMXChannelInfo(port_a_, VMXChannelCapability::EncoderAInput) },
        { VMXChannelInfo(port_b_, VMXChannelCapability::EncoderBInput) }
    };
    printf("Encoder channels: %d, %d\n", port_a_, port_b_);
    
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

Encoder::~Encoder() 
{
    VMXErrorCode vmxerr;
    if (!vmx_->io.DeallocateResource(encoder_res_handle_, &vmxerr)) {
        printf("Failed to deallocate Encoder Resource\n");
        DisplayVMXError(vmxerr);
    } else {
        printf("Successfully deallocated Encoder Resource\n");
    }
}

int Encoder::GetCount() 
{
    int32_t counter = 0;
    VMXErrorCode vmxerr;

    if (vmx_->io.Encoder_GetCount(encoder_res_handle_, counter, &vmxerr)) {
        return counter;
    } else {
        printf("Error retrieving Encoder count.\n");
        DisplayVMXError(vmxerr);
        return -1;
    }
}

std::string Encoder::GetDirection() 
{
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

void Encoder::DisplayVMXError(VMXErrorCode vmxerr) 
{
    const char* p_err_description = GetVMXErrorString(vmxerr);
    printf("VMXError %d: %s\n", vmxerr, p_err_description);
}
