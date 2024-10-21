#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/detail/set_bool__struct.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <memory>

class ServerNode : public rclcpp::Node {
public:
  ServerNode() : Node("service_move_right") {

    srv_ = create_service<std_srvs::srv::SetBool>(
        "move_right", std::bind(&ServerNode::moving_callback, this,
                                std::placeholders::_1, std::placeholders::_2));
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  void moving_callback(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Requested /move_right Service: %d",
                request->data);
    auto message = geometry_msgs::msg::Twist();
    if (request->data == true) {
      message.linear.x = 0.25;
      message.angular.z = -0.25;
      response->message = "Robot Moving";
    } else {
      message.linear.x = 0.0;
      message.angular.z = 0.0;
      response->message = "Robot Stopped";
    }
    publisher_->publish(message);
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "/move_right Service Response: %s",
                response->message.c_str());
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerNode>());
  rclcpp::shutdown();
  return 0;
}

// End of Code