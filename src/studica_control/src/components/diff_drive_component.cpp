#include "studica_control/diff_drive_component.h"

namespace studica_control {

std::shared_ptr<rclcpp::Node> DiffDrive::initialize(rclcpp::Node *control, std::shared_ptr<DiffOdometry> odom, std::shared_ptr<VMXPi> vmx) {
    control->declare_parameter<std::string>("diff_drive_component.name", "");
    control->declare_parameter<int>("diff_drive_component.can_id", -1);
    control->declare_parameter<int>("diff_drive_component.motor_freq", -1);
    control->declare_parameter<int>("diff_drive_component.ticks_per_rotation", -1);
    control->declare_parameter<float>("diff_drive_component.wheel_radius", -1.0);
    control->declare_parameter<float>("diff_drive_component.wheelbase", -1.0);
    control->declare_parameter<float>("diff_drive_component.width", -1.0);
    control->declare_parameter<int>("diff_drive_component.front_left", -1);
    control->declare_parameter<int>("diff_drive_component.front_right", -1);
    control->declare_parameter<int>("diff_drive_component.rear_left", -1);
    control->declare_parameter<int>("diff_drive_component.rear_right", -1);
    control->declare_parameter<bool>("diff_drive_component.invert_front_left", false);
    control->declare_parameter<bool>("diff_drive_component.invert_front_right", false);
    control->declare_parameter<bool>("diff_drive_component.invert_rear_left", false);
    control->declare_parameter<bool>("diff_drive_component.invert_rear_right", false);

    std::string name = control->get_parameter("diff_drive_component.name").as_string();
    int can_id = control->get_parameter("diff_drive_component.can_id").as_int();
    int motor_freq = control->get_parameter("diff_drive_component.motor_freq").as_int();
    int ticks_per_rotation = control->get_parameter("diff_drive_component.ticks_per_rotation").as_int();
    float wheel_radius = control->get_parameter("diff_drive_component.wheel_radius").get_value<float>();
    float wheelbase = control->get_parameter("diff_drive_component.wheelbase").get_value<float>();
    float width = control->get_parameter("diff_drive_component.width").get_value<float>();
    int fl = control->get_parameter("diff_drive_component.front_left").as_int();
    int fr = control->get_parameter("diff_drive_component.front_right").as_int();
    int rl = control->get_parameter("diff_drive_component.rear_left").as_int();
    int rr = control->get_parameter("diff_drive_component.rear_right").as_int();
    bool invert_fl = control->get_parameter("diff_drive_component.invert_front_left").as_bool();
    bool invert_fr = control->get_parameter("diff_drive_component.invert_front_right").as_bool();
    bool invert_rl = control->get_parameter("diff_drive_component.invert_rear_left").as_bool();
    bool invert_rr = control->get_parameter("diff_drive_component.invert_rear_right").as_bool();

    RCLCPP_INFO(control->get_logger(), "%s -> fl: %d, fr: %d, rl: %d, rr: %d", name.c_str(), fl, fr, rl, rr);

    auto diff_drive_node = std::make_shared<DiffDrive>(vmx, odom, name, can_id, motor_freq, ticks_per_rotation, wheel_radius, wheelbase, width, fl, fr, rl, rr, invert_fl, invert_fr, invert_rl, invert_rr);
    return diff_drive_node;
}

DiffDrive::DiffDrive(const rclcpp::NodeOptions & options) : Node("diff_drive", options) {}

DiffDrive::DiffDrive(
    std::shared_ptr<VMXPi> vmx,
    std::shared_ptr<studica_control::DiffOdometry> odom,
    const std::string &name,
    const uint8_t &can_id,
    const uint16_t &motor_freq,
    const uint16_t &ticks_per_rotation,
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
    : Node(name),
      vmx_(vmx),
      odom_(odom),
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
        std::bind(&DiffDrive::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

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

    odom_->setWheelParams(wheelbase_);
    odom_->init(this->now());

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&DiffDrive::publish_odometry, this));

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10,
        std::bind(&DiffDrive::cmd_vel_callback, this, std::placeholders::_1)
    );
}

DiffDrive::~DiffDrive() {}

void DiffDrive::cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    std::string params = request->params;
    cmd(params, request, response);
}

void DiffDrive::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double linear = msg->linear.x;
    double angular = msg->angular.z;

    // For differential drive, wheelbase is the distance between left and right wheels
    double left_command = linear - angular * wheelbase_ / 2.0;
    double right_command = linear + angular * wheelbase_ / 2.0;

    // Set speed for all 4 motors (front and rear left/right)
    titan_->SetSpeed(fl_, left_command);
    titan_->SetSpeed(rl_, left_command);
    titan_->SetSpeed(fr_, right_command);
    titan_->SetSpeed(rr_, right_command);
}

void DiffDrive::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response) {
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

void DiffDrive::publish_odometry() {
    // Get encoder distances from all 4 motors
    double fl_encoder = titan_->GetEncoderDistance(fl_);
    double fr_encoder = titan_->GetEncoderDistance(fr_);
    double rl_encoder = titan_->GetEncoderDistance(rl_);
    double rr_encoder = titan_->GetEncoderDistance(rr_);

    // Average left and right sides for differential drive
    double left_encoder = (fl_encoder + rl_encoder) / 2.0;
    double right_encoder = (fr_encoder + rr_encoder) / 2.0;

    auto current_time = this->now();

    odom_->updateAndPublish(left_encoder, right_encoder, current_time);
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::DiffDrive)
