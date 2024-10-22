#include "studica_control/titan_component.h"

namespace studica_control
{

Titan::Titan(const rclcpp::NodeOptions & options) : Node("titan_", options) {
    // Titan("titan", 5.0, 0, std::make_shared<VMXPi>(true, 50));
}

Titan::Titan(const std::string &name, const uint8_t &canID, const uint16_t &motorFreq, const float &distPerTick, const float &speed)
    : Node("titan_"), canID_(canID), motorFreq_(motorFreq), distPerTick_(distPerTick), speed_(speed)  {
    auto& vmx_manager = studica_driver::VMXManager::getInstance();
    vmx_ = vmx_manager.getVMX();
    titan_ = std::make_shared<studica_driver::Titan>(name, canID_, motorFreq_, distPerTick_, speed_, vmx_);
}
Titan::~Titan() {}


void Titan::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (params == "start") {
        titan_->Enable(true);
        response->success = true;
        response->message = "Titan started";
    } else if (params == "setup_enc") {
        titan_->SetupEncoder(request->initparams.n_encoder);
        response->success = true;
        response->message = "Titan encoder setup complete";
    } else if (params == "stop") {
        titan_->SetSpeed(request->initparams.n_encoder, 0.0);
        response->success = true;
        response->message = "Titan stopped";
    } else if (params == "reset") {
        titan_->ResetEncoder(request->initparams.n_encoder);
        response->success = true;
        response->message = "Titan reset";
    } else if (params == "set_speed") {
        response->success = true;
        speed_ = request->initparams.speed;
        titan_->SetSpeed(request->initparams.n_encoder, speed_);
        response->message = "Encoder " + std::to_string(request->initparams.n_encoder) + " speed set to " + std::to_string(speed_);
    } else if (params == "inv_enc_dir") {
        titan_->InvertEncoderDirection(request->initparams.n_encoder);
        titan_->SetSpeed(request->initparams.n_encoder, speed_);
        response->success = true;
        response->message = "Encoder " + std::to_string(request->initparams.n_encoder) + " direction has been inverted";
    } else if (params == "inv_motor_dir") {
        titan_->InvertMotorDirection(request->initparams.n_encoder);
        titan_->SetSpeed(request->initparams.n_encoder, speed_);
        response->success = true;
        response->message = "Motor " + std::to_string(request->initparams.n_encoder) + " direction has been inverted";
    } else if (params == "inv_motor_rpm") {
        titan_->InvertMotorRPM(request->initparams.n_encoder);
        titan_->SetSpeed(request->initparams.n_encoder, speed_);
        response->success = true;
        response->message = "Motor " + std::to_string(request->initparams.n_encoder) + " RPM has been inverted";
    } else if (params == "get_enc_dist") {
        response->success = true;
        response->message = std::to_string(titan_->GetEncoderDistance(request->initparams.n_encoder));
    } else if (params == "get_rpm") {
        response->success = true;
        response->message = std::to_string(titan_->GetRPM(request->initparams.n_encoder));
    } else if (params == "get_enc_cnt") {
        response->success = true;
        response->message = std::to_string(titan_->GetEncoderCount(request->initparams.n_encoder));
    } else {
        response->success = false;
        response->message = "No such command '" + params + "'";
    }
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Titan)
