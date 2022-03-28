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
    : Node("car_publisher")
    {
      publisher_ = this->create_publisher<car_pkg::msg::car_msg>("car_data", 10);
    }

    void publish(struct CarMsg CarMsg)
    {
      auto message = car_pkg::msg::car_msg();
      message.position_x = CarMsg.position_x;
      message.position_y = CarMsg.position_y;
      message.heading = CarMsg.heading;
      message.vehicle_speed = CarMsg.vehicle_speed;
      message.priority = CarMsg.priority;
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%f' '%d'", message.vehicle_speed, message.heading);
      publisher_->publish(message);
    }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    private:
};

void subscriber(const car_pkg::msg::intersection_msg::ConstPtr& msg)
{
  struct IntersectionMsg IMsg;
  IMsg.position_x = msg->position_x;
  IMsg.position_y = msg->position_y;
  IMsg.num_directions = msg->num_directions;
  IMsg.directions[32] = msg->directions;
  IMsg.intersection_state = msg->intersection_state;
  IMsg.intersection_next_state = msg->intersection_next_state;
  IMsg.intersection_switch_time = msg->intersection_switch_time;

  //TODO: CALL CALLBACK AND PASS IN STRUCTURE

  // RCLCPP_INFO(this->get_logger(), "Intersection State: %d", msg->intersection_state);
  // RCLCPP_INFO(this->get_logger(), "Intersection Next State: %d", msg->intersection_next_state);
  // RCLCPP_INFO(this->get_logger(), "Intersection Switch Time: %f", msg->intersection_switch_time);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeHandle n;
  rclcpp::Subscriber sub = n.subscribe("intersection_data", 1, subscriber);

  rclcpp::spin(std::make_shared<IntersectionMessager>());

  rclcpp::shutdown();
  return 0;
}