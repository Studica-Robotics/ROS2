#include "studica_control/mecanum_drive_component.h"

namespace studica_control {

MecanumDrive::MecanumDrive(const rclcpp::NodeOptions & options) : Node("mecanum_drive_", options) {}

MecanumDrive::MecanumDrive(
    std::shared_ptr<VMXPi> vmx, 
    const std::string &name, 
    const uint8_t &canID, 
    const uint16_t &motorFreq, 
    const float &ticksPerRotation, 
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
    : Node("titan_"), 
      vmx_(vmx), 
      name_(name), 
      canID_(canID), 
      motorFreq_(motorFreq), 
      ticksPerRotation_(ticksPerRotation), 
      wheel_radius_(wheel_radius), 
      wheelbase_(wheelbase), 
      width_(width),
      fl_(front_left),
      fr_(front_right),
      rl_(rear_left),
      rr_(rear_right) {

    distPerTick_ = 2 * M_PI * wheel_radius_ / ticksPerRotation_;
    titan_ = std::make_shared<studica_driver::Titan>(name, canID_, motorFreq_, distPerTick_, vmx_);
    service_ = this->create_service<studica_control::srv::SetData>(
        "titan_cmd",
        std::bind(&MecanumDrive::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    titan_->ConfigureEncoder(fl_, distPerTick_);
    titan_->ConfigureEncoder(fr_, distPerTick_);
    titan_->ConfigureEncoder(rl_, distPerTick_);
    titan_->ConfigureEncoder(rr_, distPerTick_);

    titan_->ResetEncoder(fl_);
    titan_->ResetEncoder(fr_);
    titan_->ResetEncoder(rl_);
    titan_->ResetEncoder(rr_);

    if (invert_front_left) titan_->InvertMotor(fl_);
    if (invert_front_right) titan_->InvertMotor(fr_);
    if (invert_rear_left) titan_->InvertMotor(rl_);
    if (invert_rear_right) titan_->InvertMotor(rr_);

    titan_->Enable(true);

    odom_ = std::make_unique<MecanumOdometry>();
    odom_->setWheelParams(wheelbase_, width_);
    
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&MecanumDrive::publish_odometry, this));

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        10,
        std::bind(&MecanumDrive::cmd_vel_callback, this, std::placeholders::_1)
    );
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
    } else if (params == "setup_enc") {
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
    }
    else if (params == "get_enc_dist") {
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

void MecanumDrive::publish_odometry() {
    double front_left = titan_->GetEncoderDistance(fl_);
    double front_right = titan_->GetEncoderDistance(fr_);
    double rear_left = titan_->GetEncoderDistance(rl_);
    double rear_right = titan_->GetEncoderDistance(rr_);

    auto current_time = this->now();

    odom_->update(front_left, front_right, rear_left, rear_right, current_time);

    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = odom_->getX();
    odom_msg.pose.pose.position.y = odom_->getY();
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, odom_->getHeading());
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_publisher_->publish(odom_msg);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = current_time;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_footprint";

    tf.transform.translation.x = odom_->getX();
    tf.transform.translation.y = odom_->getY();
    tf.transform.translation.z = 0.0;

    tf.transform.rotation = odom_msg.pose.pose.orientation;
    
    tf_broadcaster_->sendTransform(tf);
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::MecanumDrive)
