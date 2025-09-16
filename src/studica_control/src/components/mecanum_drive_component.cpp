#include "studica_control/mecanum_drive_component.h"

namespace studica_control {

std::shared_ptr<rclcpp::Node> MecanumDrive::initialize(rclcpp::Node *control, std::shared_ptr<MecanumOdometry> odom, std::shared_ptr<VMXPi> vmx) {
    control->declare_parameter<int>("mecanum_drive_component.can_id", -1);
    control->declare_parameter<int>("mecanum_drive_component.motor_freq", -1);
    control->declare_parameter<int>("mecanum_drive_component.ticks_per_rotation", -1);
    control->declare_parameter<float>("mecanum_drive_component.wheel_radius", -1.0);
    control->declare_parameter<float>("mecanum_drive_component.wheelbase", -1.0);
    control->declare_parameter<float>("mecanum_drive_component.width", -1.0);
    control->declare_parameter<int>("mecanum_drive_component.front_left", -1);
    control->declare_parameter<int>("mecanum_drive_component.front_right", -1);
    control->declare_parameter<int>("mecanum_drive_component.rear_left", -1);
    control->declare_parameter<int>("mecanum_drive_component.rear_right", -1);
    control->declare_parameter<bool>("mecanum_drive_component.invert_front_left", false);
    control->declare_parameter<bool>("mecanum_drive_component.invert_front_right", false);
    control->declare_parameter<bool>("mecanum_drive_component.invert_rear_left", false);
    control->declare_parameter<bool>("mecanum_drive_component.invert_rear_right", false);
    control->declare_parameter<std::string>("mecanum_drive_component.topic", "");
    
    int can_id = control->get_parameter("mecanum_drive_component.can_id").as_int();
    int motor_freq = control->get_parameter("mecanum_drive_component.motor_freq").as_int();
    int ticks_per_rotation = control->get_parameter("mecanum_drive_component.ticks_per_rotation").as_int();
    float wheel_radius = control->get_parameter("mecanum_drive_component.wheel_radius").get_value<float>();
    float wheelbase = control->get_parameter("mecanum_drive_component.wheelbase").get_value<float>();
    float width = control->get_parameter("mecanum_drive_component.width").get_value<float>();
    int fl = control->get_parameter("mecanum_drive_component.front_left").as_int();
    int fr = control->get_parameter("mecanum_drive_component.front_right").as_int();
    int rl = control->get_parameter("mecanum_drive_component.rear_left").as_int();
    int rr = control->get_parameter("mecanum_drive_component.rear_right").as_int();
    bool invert_fl = control->get_parameter("mecanum_drive_component.invert_front_left").as_bool();
    bool invert_fr = control->get_parameter("mecanum_drive_component.invert_front_right").as_bool();
    bool invert_rl = control->get_parameter("mecanum_drive_component.invert_rear_left").as_bool();
    bool invert_rr = control->get_parameter("mecanum_drive_component.invert_rear_right").as_bool();
    std::string topic = control->get_parameter("mecanum_drive_component.topic").as_string();

    auto mecanum_drive = std::make_shared<MecanumDrive>(vmx, odom, topic, can_id, motor_freq, ticks_per_rotation, wheel_radius, wheelbase, width, fl, fr, rl, rr, invert_fl, invert_fr, invert_rl, invert_rr);
    return mecanum_drive;
}

MecanumDrive::MecanumDrive(const rclcpp::NodeOptions & options) : Node("mecanum_drive", options) {}

MecanumDrive::MecanumDrive(
    std::shared_ptr<VMXPi> vmx,
    std::shared_ptr<studica_control::MecanumOdometry> odom,
    const std::string &name, 
    const uint8_t &can_id, 
    const uint16_t &motor_freq, 
    const float &ticks_per_rotation, 
    const float &wheel_radius, 
    const float &wheelbase, 
    const float &width,
    const uint8_t &front_left,
    const uint8_t &front_right,
    const uint8_t &rear_left,
    const uint8_t &rear_right,
    const bool &invert_front_left,
    const bool &invert_front_right,
    const bool &invert_rear_left,
    const bool &invert_rear_right)
    : Node("mecanum_drive"),
      vmx_(vmx),
      odom_(odom),
      fl_inverted_(invert_front_left),
      fr_inverted_(invert_front_right),
      rl_inverted_(invert_rear_left),
      rr_inverted_(invert_rear_right),
      name_(name),
      can_id_(can_id),
      motor_freq_(motor_freq),
      ticks_per_rotation_(ticks_per_rotation),
      wheel_radius_(wheel_radius),
      wheelbase_(wheelbase),
      width_(width),
      fl_(front_left),
      fr_(front_right),
      rl_(rear_left),
      rr_(rear_right) {

    dist_per_tick_ = 2 * M_PI * wheel_radius_ / ticks_per_rotation_;
    titan_ = std::make_shared<studica_driver::Titan>(can_id_, motor_freq_, dist_per_tick_, vmx_);
    service_ = this->create_service<studica_control::srv::SetData>(
        "titan_cmd",
        std::bind(&MecanumDrive::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    titan_->ConfigureEncoder(fl_, dist_per_tick_);
    titan_->ConfigureEncoder(fr_, dist_per_tick_);
    titan_->ConfigureEncoder(rl_, dist_per_tick_);
    titan_->ConfigureEncoder(rr_, dist_per_tick_);

    titan_->ResetEncoder(fl_);
    titan_->ResetEncoder(fr_);
    titan_->ResetEncoder(rl_);
    titan_->ResetEncoder(rr_);

    if (invert_front_left) titan_->InvertMotor(fl_);
    if (invert_front_right) titan_->InvertMotor(fr_);
    if (invert_rear_left) titan_->InvertMotor(rl_);
    if (invert_rear_right) titan_->InvertMotor(rr_);

    titan_->Enable(true);

    odom_->setWheelParams(wheelbase_, width_);
    odom_->init(this->now());
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&MecanumDrive::publish_odometry, this));

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        10,
        std::bind(&MecanumDrive::cmd_vel_callback, this, std::placeholders::_1)
    );
    fl_enc_sub_ = this->create_subscription<studica_control::msg::EncoderMsg>(
        "fl_enc",
        10,
        std::bind(&MecanumDrive::fl_enc_callback, this, std::placeholders::_1)
    );
    fr_enc_sub_ = this->create_subscription<studica_control::msg::EncoderMsg>(
        "fr_enc",
        10,
        std::bind(&MecanumDrive::fr_enc_callback, this, std::placeholders::_1)
    );
    rl_enc_sub_ = this->create_subscription<studica_control::msg::EncoderMsg>(
        "rl_enc",
        10,
        std::bind(&MecanumDrive::rl_enc_callback, this, std::placeholders::_1)
    );
    rr_enc_sub_ = this->create_subscription<studica_control::msg::EncoderMsg>(
        "rr_enc",
        10,
        std::bind(&MecanumDrive::rr_enc_callback, this, std::placeholders::_1)
    );
}

double MecanumDrive::enc_distance(const studica_control::msg::EncoderMsg::SharedPtr msg, bool inverted) {
    double distance = msg->encoder_count * dist_per_tick_;
    if (inverted) {
        distance *= -1;
    }
    return distance;
}

void MecanumDrive::fl_enc_callback(const studica_control::msg::EncoderMsg::SharedPtr msg) {

    fl_enc_dist_ = enc_distance(msg, fl_inverted_);
}
void MecanumDrive::fr_enc_callback(const studica_control::msg::EncoderMsg::SharedPtr msg) {
    fr_enc_dist_ = enc_distance(msg, fr_inverted_);
}
void MecanumDrive::rl_enc_callback(const studica_control::msg::EncoderMsg::SharedPtr msg) {
    rl_enc_dist_ = enc_distance(msg, rl_inverted_);
}
void MecanumDrive::rr_enc_callback(const studica_control::msg::EncoderMsg::SharedPtr msg) {
    rr_enc_dist_ = enc_distance(msg, rr_inverted_);
}

MecanumDrive::~MecanumDrive() {}

void MecanumDrive::cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    std::string params = request->params;
    cmd(params, request, response);
}

void MecanumDrive::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double x_vel = msg->linear.x;
    double y_vel = msg->linear.y;
    double angular = msg->angular.z;

    double front_left = x_vel - y_vel - angular * (wheelbase_ + width_);
    double front_right = x_vel + y_vel + angular * (wheelbase_ + width_);
    double rear_left = x_vel + y_vel - angular * (wheelbase_ + width_);
    double rear_right = x_vel - y_vel + angular * (wheelbase_ + width_);

    titan_->SetSpeed(fl_, front_left);
    titan_->SetSpeed(fr_, front_right);
    titan_->SetSpeed(rl_, rear_left);
    titan_->SetSpeed(rr_, rear_right);
}

void MecanumDrive::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    if (params == "enable") {
        titan_->Enable(true);
        response->success = true;
        response->message = "Titan enabled";
    } else if (params == "disable") {
        titan_->Enable(false);
        response->success = true;
        response->message = "Titan disabled";
    } else if (params == "start") {
        titan_->Enable(true);
        response->success = true;
        response->message = "Titan started";
    } else if (params == "setup_encoder") {
        titan_->SetupEncoder(request->initparams.n_encoder);
        response->success = true;
        response->message = "Titan encoder setup complete";
    } else if (params == "configure_encoder") {
        titan_->ConfigureEncoder(request->initparams.n_encoder, request->initparams.dist_per_tick);
        response->success = true;
        response->message = "Titan encoder configured";
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
        float speed = request->initparams.speed;
        RCLCPP_INFO(this->get_logger(), "Setting speed to %f", speed);
        titan_->SetSpeed(request->initparams.n_encoder, speed);
        response->message = "Encoder " + std::to_string(request->initparams.n_encoder) + " speed set to " + std::to_string(request->initparams.speed);
    } else if (params == "get_encoder_distance") {
        response->success = true;
        response->message = std::to_string(titan_->GetEncoderDistance(request->initparams.n_encoder));
    } else if (params == "get_rpm") {
        response->success = true;
        response->message = std::to_string(titan_->GetRPM(request->initparams.n_encoder));
    } else if (params == "get_encoder_count") {
        response->success = true;
        response->message = std::to_string(titan_->GetEncoderCount(request->initparams.n_encoder));
    } else {
        response->success = false;
        response->message = "No such command '" + params + "'";
    }
}

void MecanumDrive::publish_odometry() {
    double front_left = fl_enc_dist_;
    double front_right = fr_enc_dist_; 
    double rear_left = rl_enc_dist_; 
    double rear_right = rr_enc_dist_; 

    auto current_time = this->now();

    odom_->updateAndPublish(front_left, front_right, rear_left, rear_right, current_time);
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::MecanumDrive)
