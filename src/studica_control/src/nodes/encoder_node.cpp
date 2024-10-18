#include "encoder_node.h"

using namespace studica_control;

void EncoderNode::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (params == "get_count") {
        response->success = true;
        response->message = std::to_string(encoder_->GetCount());
    } else if (params == "get_direction") {
        response->success = true;
        response->message = encoder_->GetDirection();
    } else {
        response->success = false;
        response->message = "No such command '" + params + "'";
    }
}

EncoderNode::EncoderNode(const std::string &name, VMXChannelIndex port_a, VMXChannelIndex port_b, std::shared_ptr<VMXPi> vmx)
    : Device(name), vmx_(vmx), name_(name), port_a_(port_a), port_b_(port_b) {
    encoder_ = std::make_shared<Encoder>(port_a_, port_b_, vmx_);
}