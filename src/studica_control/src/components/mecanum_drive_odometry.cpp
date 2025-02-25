#include "studica_control/mecanum_drive_odometry.h"

namespace studica_control {

MecanumOdometry::MecanumOdometry()
:   timestamp_(0.0),
    length_x_(0.0),
    length_y_(0.0),
    x_(0.0),
    y_(0.0),
    theta_(0.0),
    prev_front_left_(0.0),
    prev_front_right_(0.0),
    prev_rear_left_(0.0),
    prev_rear_right_(0.0) {
        
    }  

bool MecanumOdometry::update(
    const double front_left, const double front_right, 
    const double rear_left, const double rear_right, const rclcpp::Time &dt) {

    double delta_front_left = front_left - prev_front_left_;
    double delta_front_right = front_right - prev_front_right_;
    double delta_rear_left = rear_left - prev_rear_left_;
    double delta_rear_right = rear_right - prev_rear_right_;

    prev_front_left_ = front_left;
    prev_front_right_ = front_right;
    prev_rear_left_ = rear_left;
    prev_rear_right_ = rear_right;
    
    if (dt.seconds() < 0.0001) return false;

    double delta_x = 0.25 * (delta_front_left + delta_front_right + delta_rear_left + delta_rear_right);
    double delta_y = 0.25 * (-delta_front_left + delta_front_right + delta_rear_left - delta_rear_right);
    double delta_theta = (-delta_front_left + delta_front_right - delta_rear_left + delta_rear_right) / (2.0 * (length_x_ + length_y_));

    double avg_theta = theta_ + delta_theta / 2.0;

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

    return true;   
}

void MecanumOdometry::setWheelParams(const double length_x, const double length_y) {
    length_x_ = length_x;
    length_y_ = length_y;
}

}
            