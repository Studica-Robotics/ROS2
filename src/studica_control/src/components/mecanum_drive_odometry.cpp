#include "studica_control/mecanum_drive_odometry.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.hpp>

namespace studica_control {

std::shared_ptr<MecanumOdometry> MecanumOdometry::initialize(rclcpp::Node *control) {
    control->declare_parameter<std::string>("mecanum_drive_odometry.name", "");
    control->declare_parameter<bool>("mecanum_drive_odometry.use_imu", true);
    control->declare_parameter<std::string>("mecanum_drive_odometry.imu_topic", "");
    control->declare_parameter<std::string>("mecanum_drive_odometry.topic", "unknown");
    std::string name = control->get_parameter("mecanum_drive_odometry.name").as_string();
    bool use_imu = control->get_parameter("mecanum_drive_odometry.use_imu").as_bool();
    std::string imu_topic = control->get_parameter("mecanum_drive_odometry.imu_topic").as_string();
    std::string topic = control->get_parameter("mecanum_drive_odometry.topic").as_string();

    auto mecanum_odom = std::make_shared<MecanumOdometry>(name, use_imu, imu_topic, topic);
    return mecanum_odom;
}

MecanumOdometry::MecanumOdometry(const rclcpp::NodeOptions & options) : Node("mecanum_odometry", options) {}

MecanumOdometry::MecanumOdometry(const std::string &name, bool use_imu, const std::string &imu_topic, const std::string &topic)
: Node(name),
  timestamp_(0.0),
  use_imu_(use_imu),
  imu_topic_(imu_topic),
  topic_(topic),
  length_x_(0.0),
  length_y_(0.0),
  x_(0.0),
  y_(0.0),
  theta_(0.0),
  prev_front_left_(0.0),
  prev_front_right_(0.0),
  prev_rear_left_(0.0),
  prev_rear_right_(0.0) {}

MecanumOdometry::~MecanumOdometry() {}

void MecanumOdometry::init(const rclcpp::Time &time) {
    timestamp_ = time;
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_, 10, std::bind(&MecanumOdometry::imuCallback, this, std::placeholders::_1));
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(topic_, 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void MecanumOdometry::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(imu_data_mutex_);
    imu_data_ = *msg;
    use_imu_ = true;
}

bool MecanumOdometry::updateAndPublish(
    const double front_left, const double front_right, 
    const double rear_left, const double rear_right, const rclcpp::Time &time) {

    const double dt = time.seconds() - timestamp_.seconds();
    if (dt < 0.0001) return false;

    double delta_front_left = front_left - prev_front_left_;
    double delta_front_right = front_right - prev_front_right_;
    double delta_rear_left = rear_left - prev_rear_left_;
    double delta_rear_right = rear_right - prev_rear_right_;

    prev_front_left_ = front_left;
    prev_front_right_ = front_right;
    prev_rear_left_ = rear_left;
    prev_rear_right_ = rear_right;

    double delta_x = 0.25 * (delta_front_left + delta_front_right + delta_rear_left + delta_rear_right);
    double delta_y = 0.25 * (-delta_front_left + delta_front_right + delta_rear_left - delta_rear_right);
    double delta_theta;
    double avg_theta;
    
    if (use_imu_) {
        std::lock_guard<std::mutex> lk(imu_data_mutex_);
      // extract yaw from IMU quaternion
        double imu_yaw = tf2::getYaw(imu_data_.orientation);
        // shortest angular difference
        double raw = imu_yaw - prev_imu_yaw_;
        delta_theta = std::atan2(std::sin(raw), std::cos(raw));
        // mid‐point heading
        avg_theta    = prev_imu_yaw_ + delta_theta * 0.5;
        prev_imu_yaw_ = imu_yaw;
    } else {
        delta_theta = (-delta_front_left + delta_front_right - delta_rear_left + delta_rear_right) / (2.0 * (length_x_ + length_y_));
        avg_theta = theta_ + delta_theta / 2.0;
    }   

    double cos_theta = cos(avg_theta);
    double sin_theta = sin(avg_theta);

    double delta_x_global = delta_x * cos_theta - delta_y * sin_theta;
    double delta_y_global = delta_x * sin_theta + delta_y * cos_theta;

    x_ += delta_x_global;
    y_ += delta_y_global;
    theta_ += delta_theta;

    theta_ = fmod(theta_ + M_PI, 2.0 * M_PI);
    if (theta_ < 0) {
        theta_ += 2.0 * M_PI;
    }
    theta_ -= M_PI;

    publishOdometry();

    return true;   
}

void MecanumOdometry::publishOdometry() {
    auto current_time = this->now();
    
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    tf2::Quaternion q;

    if (use_imu_) {
        std::lock_guard<std::mutex> lock(imu_data_mutex_);
        odom_msg.pose.pose.orientation = imu_data_.orientation;
        odom_msg.twist.twist.angular = imu_data_.angular_velocity;
    } else {
        q.setRPY(0, 0, theta_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
    }
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    
    odom_publisher_->publish(odom_msg);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = current_time;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_footprint";

    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;

    tf.transform.rotation = odom_msg.pose.pose.orientation;
    
    tf_broadcaster_->sendTransform(tf);
}

void MecanumOdometry::setWheelParams(const double length_x, const double length_y) {
    length_x_ = length_x;
    length_y_ = length_y;
}

} // namespace studica_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(studica_control::MecanumOdometry)
