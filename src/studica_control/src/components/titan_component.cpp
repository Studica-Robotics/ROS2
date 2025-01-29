#include "studica_control/titan_component.h"

namespace studica_control {

Titan::Titan(const rclcpp::NodeOptions & options) : Node("titan_", options) {}

Titan::Titan(std::shared_ptr<VMXPi> vmx, const std::string &name, const uint8_t &canID, const uint16_t &motorFreq, const float &distPerTick, const float &speed)
    : Node("titan_"), vmx_(vmx), name_(name), canID_(canID), motorFreq_(motorFreq), distPerTick_(distPerTick), speed_(speed)  {
    titan_ = std::make_shared<studica_driver::Titan>(name, canID_, motorFreq_, distPerTick_, speed_, vmx_);
    service_ = this->create_service<studica_control::srv::SetData>(
        "titan_cmd",
        std::bind(&Titan::cmd_callback, this, std::placeholders::_1, std::placeholders::_2));

    titan_->ConfigureEncoder(2, distPerTick_);
    titan_->ConfigureEncoder(3, distPerTick_);

    titan_->ResetEncoder(2);
    titan_->ResetEncoder(3);

    titan_->Enable(true);

    odom_ = std::make_unique<Odometry>();
    odom_->setWheelParams(wheel_separation_, wheel_radius_);
    odom_->init(this->now());
    
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&Titan::publish_odometry, this));

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        10,
        std::bind(&Titan::cmd_vel_callback, this, std::placeholders::_1)
    );
}
Titan::~Titan() {}

void Titan::cmd_callback(std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response) {
    std::string params = request->params;
    cmd(params, request, response);
}

void Titan::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double linear = msg->linear.x;
    double angular = msg->angular.z;

    left_command_ = linear - angular * wheel_separation_ / 2.0;
    right_command_ = linear + angular * wheel_separation_ / 2.0;

    titan_->SetSpeed(2, left_command_);
    titan_->SetSpeed(3, -1.0 * right_command_);
}

void Titan::cmd(std::string params, std::shared_ptr<studica_control::srv::SetData::Request> request, std::shared_ptr<studica_control::srv::SetData::Response> response) {
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

void Titan::publish_odometry() {
    double left_encoder = titan_->GetEncoderDistance(2);
    double right_encoder = -1.0 * titan_->GetEncoderDistance(3);

    auto current_time = this->now();

    odom_->update(left_encoder, right_encoder, current_time);

    // double delta_left = left_encoder - last_left_encoder_;
    // double delta_right = right_encoder - last_right_encoder_;

    // last_left_encoder_ = left_encoder;
    // last_right_encoder_ = right_encoder;

    // double delta_distance = (delta_left + delta_right) / 2.0;
    // double delta_theta = (delta_right - delta_left) / wheel_seperation_;

    // theta_ += delta_theta;
    // x_ += delta_distance * cos(theta_);
    // y_ += delta_distance * sin(theta_);

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

    odom_msg.twist.twist.linear.x = odom_->getLinear();
    odom_msg.twist.twist.angular.z = odom_->getAngular();

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
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::Titan)
