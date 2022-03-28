#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "car_pkg/msg/car_msg.h"
#include "car_pkg/msg/intersection_msg.h"

using namespace std::chrono_literals;

struct {
  uint32_t position_x;
  uint32_t position_y;
  uint32_t heading;
  float vehicle_speed;
  uint8_t priority;
} CarMsg;

struct {
  uint32_t position_x;
  uint32_t position_y;
  uint8_t num_directions;
  uint32_t directions[32];
  uint8_t intersection_state;
  uint8_t intersection_next_state;
  float intersection_switch_time;
} IntersectionMsg;


class IntersectionMessager : public rclcpp::Node
{
  public:
    IntersectionMessager()
    : Node("intersection_publisher")
    {
      publisher_ = this->create_publisher<car_pkg::msg::intersection_msg>("intersection_data", 10);
    }

    void publish(struct IntersectionMsg IntersectionMsg)
    {
      auto message = car_pkg::msg::intersection_msg();
      message.position_x = IntersectionMsg.position_x;
      message.position_y = IntersectionMsg.position_y;
      message.num_directions = IntersectionMsg.num_directions;
      message.directions = IntersectionMsg.directions;
      message.intersection_state = IntersectionMsg.intersection_state;
      message.intersection_next_state = IntersectionMsg.intersection_next_state;
      message.intersection_switch_time = IntersectionMsg.intersection_switch_time;

      // RCLCPP_INFO(this->get_logger(), "Publishing: '%f' '%d'", message.vehicle_speed, message.heading);
      publisher_->publish(message);
    }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    private:
};

void subscriber(const car_pkg::msg::intersection_msg::ConstPtr& msg)
{
  struct CarMsg CMsg;
  CMsg.position_x = msg->position_x;
  CMsg.position_y = msg->position_y;
  CMsg.heading = msg->heading;
  CMsg.vehicle_speed = msg->vehicle_speed;
  CMsg.priority = msg->priority;

  //TODO: CALL CALLBACK AND PASS IN STRUCTURE

  // RCLCPP_INFO(this->get_logger(), "Intersection State: %d", msg->intersection_state);
  // RCLCPP_INFO(this->get_logger(), "Intersection Next State: %d", msg->intersection_next_state);
  // RCLCPP_INFO(this->get_logger(), "Intersection Switch Time: %f", msg->intersection_switch_time);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeHandle n;
  rclcpp::Subscriber sub = n.subscribe("car_data", 1, subscriber);

  rclcpp::spin(std::make_shared<IntersectionMessager>());

  rclcpp::shutdown();
  return 0;
}