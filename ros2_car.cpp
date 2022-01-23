#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "car_pkg/msg/car_msg.h"
#include "car_pkg/msg/intersection_msg.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("car_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<car_pkg::msg::car_msg>("car_data", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.vehicle_speed = 32.4;
      message.heading = 1;
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%f' '%d'", message.vehicle_speed, message.heading);
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

void subscriber(const car_pkg::msg::intersection_msg::ConstPtr& msg)
{
  RCLCPP_INFO(this->get_logger(), "Intersection State: %d", msg->intersection_state);
  RCLCPP_INFO(this->get_logger(), "Intersection Next State: %d", msg->intersection_next_state);
  RCLCPP_INFO(this->get_logger(), "Intersection Switch Time: %f", msg->intersection_switch_time);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeHandle n;
  rclcpp::Subscriber sub = n.subscribe("intersection_data", 1, subscriber);

  rclcpp::spin(std::make_shared<MinimalPublisher>());

  rclcpp::shutdown();
  return 0;
}