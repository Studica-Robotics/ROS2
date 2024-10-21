#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "std_srvs/srv/detail/empty__struct.hpp"
#include "std_srvs/srv/detail/set_bool__struct.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>

using namespace std::chrono_literals;

class ServiceClient : public rclcpp::Node {
private:
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool service_done_ = false; // inspired from action client c++ code

  void timer_callback() {
    while (!client_->wait_for_service(1s)) {
      if (rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Client interrupted while waiting for service. Terminating...");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service Unavailable. Waiting for Service...");
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    // set request variables here, if any
    request->data = true; // comment this line if using Empty() message

    service_done_ = false; // inspired from action client c++ code
    auto result_future = client_->async_send_request(
        request, std::bind(&ServiceClient::response_callback, this,
                           std::placeholders::_1));
  }

  void response_callback(
      rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
      // uncomment below line if using Empty() message
      // RCLCPP_INFO(this->get_logger(), "Result: success");
      // comment below line if using Empty() message
      RCLCPP_INFO(this->get_logger(), "Result: success: %i, message: %s",
                  future.get()->success, future.get()->message.c_str());
      service_done_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  }

public:
  ServiceClient() : Node("service_client") {
    client_ = this->create_client<std_srvs::srv::SetBool>("move_right");
    timer_ = this->create_wall_timer(
        1s, std::bind(&ServiceClient::timer_callback, this));
    // use below line if you do not want to use
    // "using namespace std::chrono_literals;"
    // timer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(1000),
    //     std::bind(&ServiceClient::timer_callback, this));
  }

  bool is_service_done() const {
    // inspired from action client c++ code
    return this->service_done_;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // inspired from action client c++ code
  auto service_client = std::make_shared<ServiceClient>();
  while (!service_client->is_service_done()) {
    rclcpp::spin_some(service_client);
  }

  rclcpp::shutdown();
  return 0;
}

// End of Code