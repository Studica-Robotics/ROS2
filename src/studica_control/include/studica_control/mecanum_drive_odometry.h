#ifndef MECANUM_DRIVE_ODOMETRY_H
#define MECANUM_DRIVE_ODOMETRY_H

#include <cmath>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/time.hpp"

# define PLANAR_POINT_DIM 3

namespace studica_control {

class MecanumOdometry {
    public:
        MecanumOdometry();

        bool update(
            const double front_left, const double front_right,
            const double rear_left, const double rear_right, const rclcpp::Time &dt);
        
        double getX() const { return x_; }
        double getY() const { return y_; }
        double getHeading() const { return theta_; }

        void setWheelParams(const double length_x, const double length_y);

    private:
        rclcpp::Time timestamp_;

        double length_x_;
        double length_y_;

        double x_;
        double y_;
        double theta_;
        
        double prev_front_left_;
        double prev_front_right_;
        double prev_rear_left_;
        double prev_rear_right_;
};

}

#endif
