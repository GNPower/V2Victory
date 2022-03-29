#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "car_pkg/msg/carmsg.hpp"
#include "car_pkg/msg/intersectionmsg.hpp"

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
  uint8_t directions[4];
  uint8_t intersection_state;
  uint8_t intersection_next_state;
  float intersection_switch_time;
} IntersectionMsg;


class CarMessager : public rclcpp::Node
{
  public:
    CarMessager()
    : Node("car_messager")
    {
      publisher_ = this->create_publisher<car_pkg::msg::carmsg>("car_data", 10);
      subscription_ = this->create_subscription<std_msgs::msg::intersectionmsg>("intersection_data", 10, std::bind(&CarMessager::intersection_callback, this, _1));
    }

    void publish(struct CarMsg CarMsg)
    {
      auto message = car_pkg::msg::carmsg();
      message.position_x = CarMsg.position_x;
      message.position_y = CarMsg.position_y;
      message.heading = CarMsg.heading;
      message.vehicle_speed = CarMsg.vehicle_speed;
      message.priority = CarMsg.priority;
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%f' '%d'", message.vehicle_speed, message.heading);
      publisher_->publish(message);
    }

  private:
    void intersection_callback(const car_pkg::msg::carmsg & msg) const
    {
      struct IntersectionMsg IMsg;
      IMsg.position_x = msg->position_x;
      IMsg.position_y = msg->position_y;
      IMsg.num_directions = msg->num_directions;
      uint32_t directions = msg->directions;
      IMsg.directions[0] = (directions >>  0) & 0xFF;
      IMsg.directions[1] = (directions >>  8) & 0xFF;
      IMsg.directions[2] = (directions >> 16) & 0xFF;
      IMsg.directions[3] = (directions >> 24) & 0xFF;
      IMsg.intersection_state = msg->intersection_state;
      IMsg.intersection_next_state = msg->intersection_next_state;
      IMsg.intersection_switch_time = msg->intersection_switch_time;

      // RCLCPP_INFO(this->get_logger(), "Intersection State: %d", msg->intersection_state);
      // RCLCPP_INFO(this->get_logger(), "Intersection Next State: %d", msg->intersection_next_state);
      // RCLCPP_INFO(this->get_logger(), "Intersection Switch Time: %f", msg->intersection_switch_time);

      //TODO: CALL CALLBACK AND PASS IN STRUCTURE
    }

    rclcpp::Publisher<car_pkg::msg::carmsg>::SharedPtr publisher_;
    rclcpp::Subscription<car_pkg::msg::intersectionmsg>::SharedPtr subscription_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<CarMessager>());

  rclcpp::shutdown();
  return 0;
}